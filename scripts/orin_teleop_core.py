"""Core logic and preflight checks for arm_teleop_rx."""
import os, sys, time, json, math, socket, struct, signal, subprocess, urllib.request
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

ARM_NAMES_LONG = [f"idx{i}_{'left' if i<20 else 'right'}_arm_joint{i-12 if i<20 else i-19}" for i in range(13, 27)]
HAND_NAMES = [f"{s}_{f}" for s in ("left", "right") for f in ("thumb_0", "thumb_1", "index", "middle", "ring", "pinky")]
# Elbow J4 bound = AimDK +-0.03 since 17/11 (was +-0.10: the robot stands at +-0.035 after SetAction, so the first
# command bent the elbow by itself). Same as teleop/robot/aimdk_map.py ARM_LOWER_14 / ARM_UPPER_14.
LIMS_MIN = [-2.91, -0.46, -2.91, -2.00, -2.94, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.03, -2.94, -0.30, -0.20]
LIMS_MAX = [ 2.91,  1.5359, 2.91, -0.03,  2.94,  0.30,  0.20,  2.91,  0.46,   2.91, 2.00,  2.94,  0.30,  0.20]
VENDOR_MIN = [-2.91, -0.46, -2.91, -2.00, -2.94, -0.45, -0.35, -2.91, -1.60, -2.91, 0.03, -2.94, -0.45, -0.35]   # AimDK
VENDOR_MAX = [ 2.91,  1.60, 2.91, -0.03,  2.94,  0.45,  0.35,  2.91,  0.46, 2.91, 2.00,  2.94,  0.45,  0.35]


def start_limits(q14):
    """Bounds for a session that starts at the measured pose: ours, widened (never past AimDK) to include it, so the
    first command never pulls a joint that stands outside our tighter bounds -> (lo, hi) arrays."""
    q = np.clip(np.asarray(q14, dtype=np.float64), VENDOR_MIN, VENDOR_MAX)
    return np.minimum(LIMS_MIN, q), np.maximum(LIMS_MAX, q)
HOME = [0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0]
DATA_COLLECTION = [float(HOME[0] + np.deg2rad(50.0))] + HOME[1:7] + [float(HOME[7] + np.deg2rad(50.0))] + HOME[8:]
SPEED_LEVELS = {1: (0.40, 1.00), 2: (0.60, 1.50), 3: (0.80, 2.00), 4: (1.00, 2.50), 5: (1.50, 4.00), 6: (2.00, 6.28)}

class PlayoutBuffer:
    """Jitter buffer for laptop arm targets. Samples are placed on the SENDER's even grid (seq / wire_hz), mapped to
    local time with the smallest (arrival - seq*dt) seen recently, and played back `delay` s later. Packets that
    arrive in bursts (2 in one 10 ms tick, none in the next) or on an irregular tick are therefore still replayed at
    the even spacing the laptop produced, instead of being re-timed by their arrival."""
    def __init__(self, wire_hz=100, delay=0.030, maxlen=32, window=300):
        self.dt, self.delay, self.maxlen = 1.0 / float(wire_hz), float(delay), int(maxlen)
        self.buf, self.offs = [], deque(maxlen=int(window))

    def __len__(self): return len(self.buf)

    def reset(self): self.buf, _ = [], self.offs.clear()

    def keep_last(self): self.buf = self.buf[-1:]

    def push(self, arrival, q, seq):
        seq = int(seq)
        if self.buf:
            last = self.buf[-1][0]
            if seq <= last:
                if last - seq < 1000: return          # duplicate / out of order
                self.reset()                          # sender restarted its counter
            elif seq - last > 200: self.reset()       # long gap: old samples are meaningless
        self.offs.append(arrival - seq * self.dt)
        self.buf.append((seq, q))
        if len(self.buf) > self.maxlen: self.buf = self.buf[-self.maxlen:]

    def target(self, now):
        """-> (q, extrapolating). extrapolating = playout time is past the newest sample (buffer underrun)."""
        if not self.buf: return None, False
        if len(self.buf) == 1: return self.buf[0][1], False
        off = min(self.offs)
        s_play = (now - self.delay - off) / self.dt   # playout position on the sender's sample grid
        if s_play <= self.buf[0][0]: return self.buf[0][1], False
        if s_play >= self.buf[-1][0]: return self.buf[-1][1], True
        for i in range(len(self.buf) - 1, 0, -1):
            s0, q0 = self.buf[i - 1]
            s1, q1 = self.buf[i]
            if s0 <= s_play <= s1: return q0 + ((s_play - s0) / (s1 - s0)) * (q1 - q0), False
        return self.buf[-1][1], False

class JointSmoother:
    """Per-joint smoothing of the published command: two cascaded first-order low-pass stages on every joint
    separately, advanced by the TRUE time since the last publish. Whatever arrives from the jitter buffer (a burst,
    an underrun followed by a catch-up, a late tick), each joint's published position has no velocity step: it
    trails a ramp by 2 * tau and converges on the target without overshoot (both stages are critically damped)."""
    def __init__(self, tau_s):
        """tau_s: one time constant for every joint, or 14 of them (JOINT_TAU_MS: per-joint smoothing, 0 = none)."""
        tv = np.maximum(0.0, np.asarray(tau_s, dtype=np.float64))
        self.tauv = tv if tv.ndim else None
        self.tau, self.y1 = float(tv.max()), None               # scalar (largest) for the start-up print

    def reset(self, q): self.y1 = np.array(q, dtype=np.float64)

    def step(self, cur, tgt, period, hold=False):
        """cur = last published position (second stage). -> next position before the speed guard."""
        if self.tau <= 0.0: return tgt
        if self.y1 is None: self.y1 = np.array(cur, dtype=np.float64)
        if self.tauv is not None:                                # per joint; tau 0 -> a = 1 (passes straight through)
            a = np.where(self.tauv > 0.0, 1.0 - np.exp(-period / np.maximum(self.tauv, 1e-9)), 1.0)
        else: a = 1.0 - math.exp(-period / self.tau)
        if not hold: self.y1 = self.y1 + a * (tgt - self.y1)     # hold: freeze stage 1, stage 2 glides onto it
        return cur + a * (self.y1 - cur)


class OneEuroFilter:
    """Speed-adaptive low-pass per joint (Casiez et al., CHI 2012, "1 euro filter"): the cut-off frequency rises with
    the (filtered) speed, fc = min_cutoff + beta * |v|. Slow / still -> strong smoothing (removes jitter and small
    steps); fast -> little lag. Same step(cur, tgt, period, hold) interface as JointSmoother, so the pacer can use
    either (PACER_FILTER=oneeuro). Defaults chosen with tools/filter_report.py on a real 16/11 trajectory."""
    def __init__(self, min_cutoff=1.5, beta=0.5, d_cutoff=5.0):
        self.min_cutoff, self.beta, self.d_cutoff = float(min_cutoff), float(beta), float(d_cutoff)
        self.x, self.dx = None, None
        self.tau = 1.0 / (2 * math.pi * self.min_cutoff)          # nominal lag at rest, for the start-up print

    @staticmethod
    def _alpha(cutoff, period):
        r = 2 * math.pi * cutoff * period
        return r / (r + 1.0)

    def reset(self, q): self.x, self.dx = np.array(q, dtype=np.float64), np.zeros(len(q))

    def step(self, cur, tgt, period, hold=False):
        tgt = np.asarray(tgt, dtype=np.float64)
        if self.x is None: self.reset(cur)
        dx = (tgt - self.x) / max(period, 1e-4)
        self.dx = self.dx + self._alpha(self.d_cutoff, period) * (dx - self.dx)
        fc = self.min_cutoff + self.beta * np.abs(self.dx)
        r = 2 * math.pi * fc * period
        self.x = self.x + (r / (r + 1.0)) * (tgt - self.x)
        return self.x.copy()


class KalmanJointFilter:
    """Per-joint constant-velocity Kalman filter on the target stream (state: position, velocity; measurement: the
    jitter-buffer target). q = process noise (how fast the operator may change velocity, rad^2/s^3), r = measurement
    noise variance (rad^2: steps / jitter left by the network and the controller tracking). Because it tracks the
    velocity, a steadily moving target is followed with less lag than a low-pass of the same smoothness; the price
    is a small overshoot when the target stops abruptly - hence an optional low-pass stage after it (lpf_ms).
    Same step(cur, tgt, period, hold) interface as JointSmoother (PACER_FILTER=kalman)."""
    def __init__(self, q=40.0, r=4e-5, lpf_ms=0.0):
        self.q, self.r = float(q), float(r)
        self.post = JointSmoother(lpf_ms / 1000.0) if lpf_ms > 0 else None
        self.x = None
        self.tau = (lpf_ms / 1000.0) if lpf_ms > 0 else 0.0

    def reset(self, q):
        self.x = np.array(q, dtype=np.float64); n = len(self.x)
        self.v = np.zeros(n); self.p11 = np.full(n, self.r); self.p12 = np.zeros(n); self.p22 = np.full(n, 1e-2)
        if self.post: self.post.reset(q)

    def step(self, cur, tgt, period, hold=False):
        if self.x is None: self.reset(cur)
        dt = max(period, 1e-4)
        # predict (constant velocity, white-acceleration process noise)
        self.x = self.x + self.v * dt
        p11 = self.p11 + dt * (2 * self.p12 + dt * self.p22) + self.q * dt ** 3 / 3.0
        p12 = self.p12 + dt * self.p22 + self.q * dt ** 2 / 2.0
        p22 = self.p22 + self.q * dt
        # update with the target as the measurement
        y = np.asarray(tgt, dtype=np.float64) - self.x
        s = p11 + self.r
        k1, k2 = p11 / s, p12 / s
        self.x = self.x + k1 * y; self.v = self.v + k2 * y
        self.p11, self.p12, self.p22 = (1 - k1) * p11, (1 - k1) * p12, p22 - k2 * p12
        out = self.x.copy()
        return self.post.step(cur, out, period, hold) if self.post else out


VENDOR_VMAX, VENDOR_AMAX = 3.0, 6.28      # arm_joint_command limits from the vendor guide (rad/s, rad/s^2)


def accel_guard(cur, v_prev, tgt, tgt_prev, period, max_vel, max_acc, step_abs):
    """Last stage of the published arm command, per the vendor guide: speed <= min(level, 3 rad/s) and acceleration
    <= max_acc (6.28 rad/s^2) in BOTH directions, without ever running past the target.
    Per joint: the wanted speed is the error over one period, capped by the speed limit and by the braking speed
    v_brake = the largest speed from which slowing down by max_acc*dt per tick still stops within the remaining
    error (discrete form, same as SafetyLayer). The speed then changes by at most max_acc*dt per tick.
    Up to 1.0 rad/s (speed levels 1-4) the target's own velocity (tgt - tgt_prev, already low-passed) is fed
    forward, so a moving target is followed without the v^2 / (2 * max_acc) lag of a pure braking curve. Above that
    the feed-forward is off: in simulation (16/11) it made a jittery stream at 2 rad/s overshoot by up to 0.26 rad,
    while without it there is no overshoot at the price of ~100 ms more lag at full speed.
    Simulated (bursty network, 50 ms buffer, 10 ms LPF): level 3 -> peak accel 6.3 rad/s^2 (LPF alone: 41), no
    overshoot, tracking error 16 mrad; level 6 -> 6.3 rad/s^2 (LPF alone: 45), no overshoot, error 90 mrad.
    Effect: no velocity steps reach the MC (late packet bursts, buffer underruns, a target that stops).
    (Versions tried 16/11: a two-sided limit WITHOUT the braking term overshot ~0.3 rad in simulation; a speeding-up-
    only limit let the braking spikes through.)  -> (step, limited?)"""
    ma = np.asarray(max_acc, dtype=np.float64)                   # scalar, or 14 values (JOINT_AMAX; <= 0 = no limit)
    if np.all(ma <= 0.0) or period <= 0.0: return None, False
    max_acc = np.where(ma > 0.0, ma, 1e6)
    e = tgt - cur
    ae = np.abs(e)
    a_dt = max_acc * period
    v_brake = a_dt * (np.sqrt(0.25 + 2.0 * ae / (a_dt * period)) - 0.5)
    vmax = np.minimum(np.asarray(max_vel, dtype=np.float64), VENDOR_VMAX)     # scalar, or 14 values (JOINT_VMAX)
    ff = (vmax <= 1.0) if (tgt_prev is not None and vmax.ndim == 0) else False   # feed-forward: speed levels 1-4 only, never per joint (overshoots, sim 17/11)
    v_tgt = np.where(ff, np.clip((tgt - tgt_prev) / period, -vmax, vmax), 0.0) if tgt_prev is not None else np.zeros_like(e)
    v_des = np.clip(v_tgt + np.sign(e) * np.minimum(ae / period, v_brake), -vmax, vmax)
    toward = np.sign(v_des) == np.sign(e)                        # never faster toward the target than stopping on it allows
    v_des = np.where(toward & (np.abs(v_des) > v_brake + np.abs(v_tgt)), np.sign(v_des) * (v_brake + np.abs(v_tgt)), v_des)
    dv = np.clip(v_des - v_prev, -a_dt, a_dt)
    limited = bool(np.any(np.abs(v_des - v_prev) > a_dt + 1e-12))
    step = (v_prev + dv) * period
    m = float(np.max(np.abs(step)))
    if m > step_abs > 0.0: step = step * (step_abs / m)        # hard per-message ceiling (MC rejects > 0.02 rad)
    return step, limited


def per_joint_env(name, default=""):
    """Env var with 1 number (every joint), 7 (one arm, J1..J7, used for both arms) or 14 (left J1..J7, right
    J1..J7) -> array(14); unset -> default; empty / 'off' -> None."""
    s = os.environ.get(name, default).strip()
    if not s or s.lower() in ("off", "none", "0"): return None
    v = [float(x) for x in s.replace(";", ",").replace(" ", ",").split(",") if x.strip()]
    if len(v) == 1: v = v * 14
    if len(v) == 7: v = v + v
    if len(v) != 14: raise ValueError(f"{name}: need 1, 7 or 14 numbers, got {len(v)}")
    return np.array(v, dtype=np.float64)


class JointRateLimiter:
    """Per-joint speed cap (rad/s, JOINT_VMAX), applied to the playout target BEFORE the smoothing stage, each joint
    on its own (a capped joint lags a little; the others keep 1:1). Because the cap sits before the low-pass, the
    corners where a joint enters/leaves its cap are rounded by the smoother: the published speed has no step, at
    the start nor at the stop (a cap placed after the low-pass stops a capped joint dead = a jerk).
    Why per joint: 402 shakes in proportion to joint speed (17/11), and the robot traces show it unevenly: wrist
    J6/J7 shake 20x their (smooth) command, shoulder/elbow J1-J5 about 5x (trace_recorder_20261116_191840).
    0 or less = no cap on that joint."""
    def __init__(self, vmax):
        v = np.asarray(vmax, dtype=np.float64)
        self.vmax = np.where(v > 0.0, np.minimum(v, VENDOR_VMAX), np.inf)
        self.r = None

    def reset(self, q): self.r = np.array(q, dtype=np.float64)

    def step(self, tgt, period, hold=False):
        if self.r is None: self.r = np.array(tgt, dtype=np.float64)
        if not hold:
            cap = self.vmax * period
            self.r = self.r + np.clip(np.asarray(tgt, dtype=np.float64) - self.r, -cap, cap)
        return self.r.copy()


def rpc_get_action(host="192.168.100.100", port=56322):
    try:
        req = urllib.request.Request(f"http://{host}:{port}/rpc/aimdk.protocol.McActionService/GetAction", data=b"{}", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            data = json.loads(resp.read().decode())
            info = data.get("info", {}) if isinstance(data, dict) else {}
            return info.get("current_action", "") or info.get("current_ext_action", "") or data.get("action", "") or data.get("ext_action", "") or str(data)
    except Exception: return ""


def rpc_set_action(action_id: int, host="192.168.100.100", port=56322, timeout=5.0) -> bool:
    """Send SetAction RPC to MC. Returns True if the call succeeded (HTTP 200).
    NOTE: MC replies 'called successfully' even when it ignores the request (e.g. wrong robot state).
    Always verify with rpc_get_action() afterwards.
    """
    payload = json.dumps({"action": action_id}).encode()
    try:
        req = urllib.request.Request(
            f"http://{host}:{port}/rpc/aimdk.protocol.McActionService/SetAction",
            data=payload,
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            resp.read()
            return True
    except Exception:
        return False

def check_topic_rate_preflight(node: Node, topic: str, duration_sec: float = 1.5) -> float:
    count = [0]
    qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    sub = node.create_subscription(JointState, topic, lambda msg: count.__setitem__(0, count[0] + 1), qos)
    t0 = time.time()
    while time.time() - t0 < duration_sec: rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return count[0] / duration_sec

def check_other_publishers(node: Node, topic: str) -> list[str]:
    return [p.node_name for p in node.get_publishers_info_by_topic(topic) if p.node_name != node.get_name()]

def extract_field_by_name(msg: JointState, field: str, names: list):
    """Velocity / effort of the named joints, or None when the message does not carry that field for them."""
    vals = getattr(msg, field, None)
    if vals is None or len(vals) == 0 or len(vals) != len(msg.name): return None
    m = dict(zip(msg.name, vals))
    return [float(m[k]) for k in names] if all(k in m for k in names) else None

def extract_q14_from_msg(msg: JointState) -> list:
    name_map = dict(zip(msg.name, msg.position))
    if all(k in name_map for k in ARM_NAMES_LONG): return [float(name_map[k]) for k in ARM_NAMES_LONG]
    idx_keys = [k for k in sorted(name_map.keys()) if "idx" in k]
    if len(idx_keys) >= 14: return [float(name_map[k]) for k in idx_keys[:14]]
    if len(msg.position) >= 14: return [float(x) for x in msg.position[:14]]
    return []


# ---------------------------------------------------------------------------------------------------------------
# Pacer: a separate, single-threaded, very light process that owns the arm command timing.
# Why: on the Orin (load ~30 on 12 cores, robot processes niced -5..-20) the receiver process uses ~46 % of a core,
# far above the CPU share of a nice-0 task, so its 100 Hz tick is scheduled late (6-9 % of ticks > 18 ms apart).
# The MC low-passes the position steps it receives, so every late command makes the joint speed sag and then
# surge: a 5-50 Hz ripple that was not in the command. A task that only needs a few % of a core is always behind
# the run-queue's virtual time and is scheduled as soon as it wakes - no privilege or system setting needed.
# The pacer does nothing but: read laptop packets -> jitter buffer -> speed guard -> publish (pre-serialised).
# All decisions (session lock, engage, stale, faults, hands, feedback, logging) stay in the parent node.
# ---------------------------------------------------------------------------------------------------------------
SHM_N = 72                                    # float64 slots, single writer per slot (no locks)
P_HB, P_MODE, P_SYNC_N, P_RESET_N, P_RUN, P_MAXVEL, P_STEPABS, P_SYNC_Q = 0, 1, 2, 3, 4, 5, 6, 8       # parent -> pacer
C_HB, C_PUB, C_EXTRA, C_DEPTH, C_TICKS, C_G12, C_G18, C_GMAX, C_WORK, C_FAULT, C_SEQ, C_Q, C_V = 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 40, 54
C_ALIM = 68                                   # publishes where the acceleration guard had to cut the velocity change
C_PRIO, C_L30 = 69, 70                        # pacer priority (1 = SCHED_FIFO, 0.5 = nice, 0 = normal); gaps > 30 ms
MODE_SILENT, MODE_HOLD, MODE_TRACK = 0, 1, 2
ARM_CMD_TOPIC = "/motion/control/arm_joint_command"


def _arm_cmd_template(vendor_fmt=False):
    """Serialised JointState with fixed names; returns (bytearray, offset of positions, offset of velocities).
    Per tick only the stamp and 28 doubles are overwritten: no Python->C message conversion in the 100 Hz loop."""
    from rclpy.serialization import serialize_message
    msg = JointState(); msg.name = list(ARM_NAMES_LONG)
    msg.position = [1.0e300 + i for i in range(14)]; msg.velocity = [-1.0e300 - i for i in range(14)]
    if vendor_fmt: msg.effort = [0.0] * 14     # the vendor's arm.py sends velocity = 0 and effort = 0 for all joints
    raw = bytearray(serialize_message(msg))
    op, ov = raw.find(struct.pack("<14d", *msg.position)), raw.find(struct.pack("<14d", *msg.velocity))
    if raw[1] != 1 or op < 0 or ov < 0: raise RuntimeError("unexpected JointState serialisation")
    return raw, op, ov


def pacer_main(argv):
    from multiprocessing import shared_memory
    fd, fwd_port, shm_name, allow_ip = int(argv[0]), int(argv[1]), argv[2], argv[3]
    wire_hz, delay_s, live = int(argv[4]), float(argv[5]) / 1000.0, argv[6] == "1"
    vendor_fmt = len(argv) > 7 and argv[7] == "1"
    # Publish rate, independent of the laptop's 100 Hz packet rate (the jitter buffer is interpolated at publish
    # time). The MC appears to latch the newest command once per 10 ms cycle of its own clock; publishing faster
    # than that gives every MC cycle a fresh point of the smooth trajectory instead of 0, 1 or 2 new commands.
    pub_hz = int(argv[8]) if len(argv) > 8 else wire_hz
    smooth = JointSmoother((float(argv[9]) if len(argv) > 9 else 0.0) / 1000.0)   # per-joint smoothing, ms per stage
    if os.environ.get("PACER_FILTER", "").strip().lower() == "kalman":       # constant-velocity Kalman (+ optional low-pass)
        smooth = KalmanJointFilter(float(os.environ.get("KALMAN_Q", "40")), float(os.environ.get("KALMAN_R", "4e-5")),
                                   float(os.environ.get("KALMAN_LPF_MS", "0")))
    if os.environ.get("PACER_FILTER", "").strip().lower() == "oneeuro":      # speed-adaptive alternative to the 2-stage LPF
        smooth = OneEuroFilter(float(os.environ.get("ONEEURO_MINCUTOFF", "1.5")), float(os.environ.get("ONEEURO_BETA", "0.5")))
    max_acc = min(6.28, max(0.0, float(argv[10]))) if len(argv) > 10 else 0.0     # acceleration guard, 0 = off
    # Per-joint limits (17/11; 1, 7 (J1..J7 both arms) or 14 values; 'off' disables):
    #   JOINT_VMAX   speed cap per joint (rad/s)          JOINT_AMAX  acceleration limit per joint (rad/s^2, <= 6.28 x2)
    #   JOINT_TAU_MS smoothing per joint (replaces PACER_SMOOTH_MS)
    # With JOINT_AMAX the speed caps go into the acceleration guard (it brakes toward the TRUE target: speed changes
    # by <= amax*dt, no overshoot on ramps/steps). Without it they go into JointRateLimiter before the smoother.
    # Simulated on real 17/11 hand trajectories (fast config as the input): fast = shoulder 2.0-2.2 rad/s,
    # accel 18-30 rad/s^2; JOINT_VMAX=1.2,1.5,1.2,1.5,2,2,2 JOINT_AMAX=8 -> shoulder 1.2 rad/s, accel <= 8,
    # jerk -60..-70 %, lag 50-160 ms per joint (smooth config LPF60/speed 3: 120-210 ms).
    # Config D (17/11, operator: "slight shake, slight lag, very good") was 1.2,1.5,1.2,1.5,2,2,2 / 6. Default since
    # 17/11 evening = D rebalanced: 1.5,1,2,0.8,2,2,2 / 6 (operator 19:10 after an A/B on the robot: "runs smooth,
    # make it the default"; the jerks seen 18:30-19:00 came with robot feedback stalls, not with these limits). Hand vibration per rad/s (sweep at HOME): J1 0.85 mm, J2 0.95,
    # J4 0.80, J3 0.25, J5 0.09 -> J2 and the elbow are capped lower, J1 / J3 higher. Replay of real sessions: vibration
    # peaks -3..-11 %, hand lag +12 % on big swings, -18 % on small moves (not yet confirmed on the robot).
    # JOINT_VMAX=1.2,1.5,1.2,1.5,2,2,2 = config D again; JOINT_VMAX=off JOINT_AMAX=off
    # restores the former level-only speed cap.
    jvmax, jamax = per_joint_env("JOINT_VMAX", "1.5,1,2,0.8,2,2,2"), per_joint_env("JOINT_AMAX", "6")
    jtau = per_joint_env("JOINT_TAU_MS")
    if jamax is not None: max_acc = np.clip(jamax, 0.0, 12.0)
    jlim = JointRateLimiter(jvmax) if (jvmax is not None and jamax is None) else None
    if jtau is not None:
        if isinstance(smooth, JointSmoother): smooth = JointSmoother(np.clip(jtau, 0.0, 80.0) / 1000.0)
        else: print("[pacer] WARNING: JOINT_TAU_MS ignored (PACER_FILTER chooses another filter)", flush=True)
    v_prev, late_n, late_warn_t, tgt_prev = np.zeros(14), 0, 0.0, None
    # Real-time priority for this tiny process. Real robot 16/11 at normal priority: 326 instead of 400 publishes/s,
    # 8.6 % of ticks > 2x late, gaps up to 138 ms - each gap = the MC gets nothing, then one big step = a visible jerk.
    # SCHED_FIFO needs an rtprio limit: start the receiver from a shell after
    #   sudo prlimit --pid $$ --rtprio=30:30 --nice=40:40
    prio = "normal priority"
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(30)); prio = "SCHED_FIFO 30 (real-time)"
    except Exception:
        try: os.nice(-10); prio = "nice -10"
        except Exception: pass
    # Optional: pin the pacer to one core that the robot's own heavy processes do not use (PACER_CPU=11).
    pin = os.environ.get("PACER_CPU", "").strip()
    if pin:
        try: os.sched_setaffinity(0, {int(c) for c in pin.split(",")}); prio += f", CPU {pin}"
        except Exception as e: prio += f", CPU pin failed ({e})"
    signal.signal(signal.SIGINT, signal.SIG_IGN)       # Ctrl+C reaches both processes; the parent tells us to stop
    shm = shared_memory.SharedMemory(name=shm_name)
    _prio_val = 1.0 if prio.startswith("SCHED_FIFO") else (0.5 if prio.startswith("nice") else 0.0)
    try:                                               # the parent owns the segment: do not unlink it when we exit
        from multiprocessing import resource_tracker
        resource_tracker.unregister(shm._name, "shared_memory")
    except Exception: pass
    S = np.ndarray((SHM_N,), dtype=np.float64, buffer=shm.buf)
    S[C_PRIO] = _prio_val                              # reported to the laptop (pacer health in the feedback packet)
    sock = socket.socket(fileno=fd); sock.setblocking(False)
    fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); fwd_addr = ("127.0.0.1", fwd_port)
    rclpy.init()
    node = Node("arm_teleop_pacer")
    qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(JointState, ARM_CMD_TOPIC, qos) if live else None
    raw, off_p, off_v = _arm_cmd_template(vendor_fmt)
    zero14 = [0.0] * 14
    lmin, lmax = np.array(LIMS_MIN, dtype=np.float64), np.array(LIMS_MAX, dtype=np.float64)
    vmin, vmax = np.array(VENDOR_MIN, dtype=np.float64), np.array(VENDOR_MAX, dtype=np.float64)
    buf, lock, cur, rx = PlayoutBuffer(wire_hz, delay_s), None, None, 0
    while True:                                          # packets queued before we started are stale: drop them
        try: sock.recvfrom(2048)
        except (BlockingIOError, OSError): break
    sync_n, reset_n = S[P_SYNC_N], S[P_RESET_N]
    dt = 1.0 / pub_hz
    next_t = last = time.monotonic()
    import gc; gc.collect(); gc.freeze()               # startup objects never rescanned: shorter GC pauses in the loop
    print(f"[pacer] arm command pacer running: {pub_hz} Hz, playout {delay_s*1000:.0f} ms, pid {os.getpid()}, "
          f"message format: {'vendor (velocity=0, effort=0)' if vendor_fmt else 'position + velocity'}, "
          f"{'one-euro filter min_cutoff ' + str(smooth.min_cutoff) + ' Hz beta ' + str(smooth.beta) if isinstance(smooth, OneEuroFilter) else ('Kalman q ' + str(smooth.q) + ' r ' + str(smooth.r) + ' + low-pass ' + format(smooth.tau * 1000, '.0f') + ' ms x2') if isinstance(smooth, KalmanJointFilter) else 'per-joint smoothing ' + format(smooth.tau * 1000, '.0f') + ' ms x2'}, accel guard {float(np.max(max_acc)):.2f} rad/s^2 (both directions, "
          f"speed <= {min(float(S[P_MAXVEL]), VENDOR_VMAX):.2f} rad/s), {prio}", flush=True)
    if jvmax is not None or jtau is not None or jamax is not None:
        f = lambda a: "/".join(format(x, "g") for x in a[7:]) if a is not None else "-"
        print(f"[pacer] per-joint limits (right arm J1..J7; left same unless 14 values): speed cap rad/s {f(jvmax)}, "
              f"accel rad/s^2 {f(jamax)}, smoothing ms {f(jtau)}", flush=True)
        if jvmax is not None and jamax is None and jtau is None and smooth.tau <= 0.0:
            print("[pacer] WARNING: JOINT_VMAX without JOINT_AMAX or smoothing = speed steps where a joint hits its cap "
                  "(not smooth). Add JOINT_AMAX=8", flush=True)
        if jvmax is not None and np.any(jvmax > float(S[P_MAXVEL]) + 1e-9):
            print(f"[pacer] NOTE: some JOINT_VMAX > speed level cap {float(S[P_MAXVEL]):.2f} rad/s - the level cap still "
                  f"applies to them (raise --speed for the per-joint caps to be the only speed limit)", flush=True)
    if not prio.startswith("SCHED_FIFO"):
        print("[pacer] WARNING: not real-time - on a loaded Orin commands come in gaps (up to 138 ms on 16/11) = jerks. "
              "Stop, run 'sudo prlimit --pid $$ --rtprio=30:30 --nice=40:40' in this shell, start again.", flush=True)
    ppid, why, hb_hold = os.getppid(), "parent asked to stop", False
    try:
        while True:
            now = time.monotonic()
            if S[P_RUN] < 0.5: break                             # parent asked us to stop
            # Parent silent: hold the arm after 0.5 s (no new targets followed), quit only after 3 s or when the parent
            # process is really gone. Before 17/11 a 0.5 s stall of the parent's tick (e.g. starting the recorder)
            # killed the pacer = FAULT "pacer process stopped" in the middle of a session.
            hb_age = now - S[P_HB]
            if os.getppid() != ppid: why = "parent process gone"; break
            if hb_age > 3.0: why = f"no heartbeat from the parent for {hb_age:.1f} s"; break
            if (hb_age > 0.5) != hb_hold:
                hb_hold = hb_age > 0.5
                print(f"[pacer] parent silent {hb_age * 1000:.0f} ms -> {'HOLD arm' if hb_hold else 'resumed'}", flush=True)
            while True:                                          # 1. laptop packets -> parent (all) + jitter buffer
                try: data, addr = sock.recvfrom(2048)
                except (BlockingIOError, OSError): break
                try: fwd.sendto(addr[0].encode() + b"|" + data, fwd_addr)
                except OSError: pass
                if allow_ip != "0.0.0.0" and addr[0] != allow_ip: continue
                try:
                    p = json.loads(data); q = p["q"]
                    if len(q) != 14: continue
                    qa = np.asarray(q, dtype=np.float64)
                    if not np.all(np.isfinite(qa)): continue
                    sid = p.get("session_id", "default")
                    if lock is None: lock = sid
                    elif sid != lock: continue                   # foreign session: the parent goes HOLDING
                    seq = int(p.get("seq", rx)); rx += 1
                    buf.push(now, np.clip(qa, vmin, vmax), seq); S[C_SEQ] = seq   # session bounds applied at playout
                except Exception: continue
            if S[P_RESET_N] != reset_n:                          # 2. events from the parent
                reset_n = S[P_RESET_N]; buf.reset(); lock = None
            if S[P_SYNC_N] != sync_n:                            # engage: restart from the measured pose
                n1 = S[P_SYNC_N]; q0 = S[P_SYNC_Q:P_SYNC_Q + 14].copy()
                if S[P_SYNC_N] == n1:
                    sync_n, cur = n1, q0; buf.keep_last(); smooth.reset(q0); v_prev = np.zeros(14); tgt_prev = None
                    lmin, lmax = start_limits(q0)        # robot stays where it stands at engage
                    if jlim is not None: jlim.reset(q0)
            mode = int(S[P_MODE])
            period = min(0.040, max(0.002, now - last))
            if now - last > 1.2 * dt: S[C_G12] += 1            # counters are relative to the publish period
            if now - last > 1.8 * dt: S[C_G18] += 1
            if now - last > 0.030:                               # vendor: adjacent commands <= 0.03 s apart
                late_n += 1; S[C_L30] += 1
                if now - late_warn_t > 5.0:
                    print(f"[pacer] WARNING: {late_n} gap(s) > 30 ms between commands (last {(now - last) * 1000:.0f} ms) - "
                          f"Orin overloaded? see real_robot_command.md (priority)", flush=True)
                    late_warn_t, late_n = now, 0
            S[C_GMAX] = max(S[C_GMAX], (now - last) * 1000.0)
            last = now
            if mode != MODE_SILENT and cur is not None and S[C_FAULT] < 0.5:      # 3. playout -> guard -> publish
                track = mode == MODE_TRACK and not hb_hold
                tgt, extr = buf.target(now) if track else (cur, False)
                if tgt is None: tgt = cur
                tgt = np.clip(tgt, lmin, lmax)                   # bounds of this session (widened at engage)
                if jlim is not None: tgt = jlim.step(tgt, period, hold=not track)
                tgt = smooth.step(cur, tgt, period, hold=not track)
                vcap = S[P_MAXVEL] if jvmax is None or jlim is not None else np.minimum(jvmax, S[P_MAXVEL])
                guarded, limited = accel_guard(cur, v_prev, tgt, tgt_prev, period, vcap, max_acc, S[P_STEPABS])
                tgt_prev = tgt
                if guarded is not None:
                    step = guarded                               # speed + acceleration limits, both directions
                    if limited: S[C_ALIM] += 1
                else:                                            # PACER_MAX_ACC=0: former speed-only guard
                    err = tgt - cur
                    d = float(np.max(np.abs(err)))
                    # speed limit over the true period, but never more than 2 nominal ticks' worth: after a scheduling gap
                    # the command continues at normal speed instead of jumping (16 mrad in one message after 100+ ms
                    # gaps = 5-6 rad/s spikes seen on the robot 16/11); it catches up over the next ticks
                    cap = min(S[P_STEPABS], S[P_MAXVEL] * min(period, 2.0 * dt))
                    step = err * (cap / d) if d > cap else err
                # HARD per-message ceiling AFTER the bound clip: a joint standing outside the bounds is walked in at
                # <= step_abs per message, never jumped. (17/11 12:22-13:24: the clip jumped an elbow 0.035 -> 0.10 in
                # one message = 26 rad/s; the acceleration guard took that as the joint's speed and drove the elbow
                # to 1.88 rad in 0.5 s at ~7 rad/s - the MC then stopped servoing that elbow.)
                nxt = cur + np.clip(np.clip(cur + step, lmin, lmax) - cur, -S[P_STEPABS], S[P_STEPABS])
                if not np.all(np.isfinite(nxt)): S[C_FAULT] = 1.0
                else:
                    v_act, v_own = (nxt - cur) / period, step / period
                    cur = nxt
                    vlim = np.minimum(np.minimum(jvmax if jvmax is not None else S[P_MAXVEL], S[P_MAXVEL]), VENDOR_VMAX)
                    # speed state = the guard's OWN speed (a displacement forced by the bound clip is not motion the guard
                    # must brake), or less when a bound stopped it; never above the cap. Also the published velocity.
                    vel = np.clip(np.where(np.abs(v_act) < np.abs(v_own), v_act, v_own), -vlim, vlim)
                    v_prev = vel
                    if pub is not None:
                        t_ns = time.time_ns()
                        struct.pack_into("<iI", raw, 4, t_ns // 1000000000, t_ns % 1000000000)
                        struct.pack_into("<14d", raw, off_p, *cur); struct.pack_into("<14d", raw, off_v, *(zero14 if vendor_fmt else vel))
                        pub.publish(bytes(raw))
                    S[C_Q:C_Q + 14] = cur; S[C_V:C_V + 14] = vel; S[C_EXTRA] = 1.0 if extr else 0.0
                    S[C_PUB] += 1
            else:
                v_prev, tgt_prev = np.zeros(14), None            # not publishing: the next command starts from rest
            S[C_DEPTH] = len(buf); S[C_TICKS] += 1
            S[C_HB] = time.monotonic(); S[C_WORK] = (S[C_HB] - now) * 1000.0
            next_t += dt
            slack = next_t - time.monotonic()
            if slack > 0.0: time.sleep(slack)
            elif slack < -dt: next_t = time.monotonic(); time.sleep(0.0005)
    except Exception:
        import traceback; why = "crashed:\n" + traceback.format_exc()
    finally:
        print(f"[pacer] stopped: {why}", flush=True)
        try: node.destroy_node(); rclpy.shutdown()
        except Exception: pass
        shm.close()


class PacerLink:
    """Parent side of the pacer. Starts the child, hands it the UDP socket, and is the parent's packet source."""
    def __init__(self, sock, allow_ip, wire_hz, playout_delay, live, max_vel, max_step_abs, vendor_fmt=False, pub_hz=None, smooth_ms=0.0,
                 max_acc=0.0):
        from multiprocessing import shared_memory
        self.shm = shared_memory.SharedMemory(create=True, size=SHM_N * 8)
        self.S = np.ndarray((SHM_N,), dtype=np.float64, buffer=self.shm.buf); self.S[:] = 0.0
        self.S[P_RUN], self.S[P_HB], self.S[P_MAXVEL], self.S[P_STEPABS] = 1.0, time.monotonic(), max_vel, max_step_abs
        self.fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.fwd.bind(("127.0.0.1", 0)); self.fwd.setblocking(False)
        fd = sock.fileno(); os.set_inheritable(fd, True)
        cmd = [sys.executable, os.path.abspath(__file__), "--pacer", str(fd), str(self.fwd.getsockname()[1]), self.shm.name,
               allow_ip, str(wire_hz), str(playout_delay * 1000.0), "1" if live else "0", "1" if vendor_fmt else "0", str(int(pub_hz or wire_hz)), str(float(smooth_ms)), str(float(max_acc))]
        self.proc = subprocess.Popen(cmd, pass_fds=[fd])
        self._pub_seen = 0.0
        t0 = time.monotonic()
        while self.S[C_HB] == 0.0:                               # wait until the child loop is alive
            if self.proc.poll() is not None or time.monotonic() - t0 > 8.0:
                self.stop(); raise RuntimeError("pacer process did not start")
            self.S[P_HB] = time.monotonic(); time.sleep(0.02)

    def recvfrom(self, n):
        data, _ = self.fwd.recvfrom(n + 64)
        ip, _, payload = data.partition(b"|")
        return payload, (ip.decode("ascii", "replace"), 0)

    def heartbeat(self): self.S[P_HB] = time.monotonic()
    def alive(self): return self.proc.poll() is None and (time.monotonic() - self.S[C_HB]) < 0.5
    def set_mode(self, m): self.S[P_MODE] = float(m)
    def reset_buffer(self): self.S[P_RESET_N] += 1
    def sync(self, q14):
        self.S[P_SYNC_Q:P_SYNC_Q + 14] = np.asarray(q14, dtype=np.float64); self.S[P_SYNC_N] += 1
    def new_pub_count(self):
        n = self.S[C_PUB]; d = int(n - self._pub_seen); self._pub_seen = n
        return max(0, d)
    def stats(self): return [float(self.S[i]) for i in (C_TICKS, C_G12, C_G18, C_GMAX, C_WORK, C_ALIM)]

    def health(self):
        """For the laptop: real-time priority obtained, and publish gaps > 30 ms since start (vendor limit)."""
        return {"rt": bool(self.S[C_PRIO] >= 1.0), "prio": float(self.S[C_PRIO]), "gaps30": int(self.S[C_L30]),
                "gmax_ms": round(float(self.S[C_GMAX]), 1)}

    def stop(self):
        try: self.S[P_RUN] = 0.0; self.S[P_MODE] = 0.0
        except Exception: pass
        try: self.proc.wait(timeout=1.5)
        except Exception:
            try: self.proc.kill()
            except Exception: pass
        for f in (self.shm.close, self.shm.unlink, self.fwd.close):
            try: f()
            except Exception: pass


# ---------------------------------------------------------------------------------------------------------------
# Episode recorder (data collection). Runs on the Orin because that is where the camera topics live; the laptop only
# says which episode should be recording (field "rec" in every command packet), so a lost packet cannot desync it.
# Layout:  <root>/<task>/<YYYY-MM-DD>/<episode>/robot/   raw rosbag (camera + joint/hand state and command)
#                                               /meta.json  what, when, which topics, how big
#                                               /ros2bag.log
# ---------------------------------------------------------------------------------------------------------------
REC_TOPICS_DEFAULT = ["/aima/hal/rgbd_camera/head_front/color", "/aima/hal/rgbd_camera/head_front/depth",
                      "/motion/control/arm_joint_state", "/motion/control/arm_joint_command",
                      "/motion/control/hand_joint_state", "/motion/control/hand_joint_command"]
REC_MIN_FREE_GB, REC_STOP_FREE_GB, REC_MAX_SECS = 5.0, 2.0, 600.0


def safe_name(text, default="default", max_len=48):
    """Folder-safe name: letters, digits, '_' and '-' only (no path separators, no dots)."""
    out = "".join(ch if (ch.isalnum() and ch.isascii()) or ch in "_-" else "_" for ch in str(text or ""))[:max_len].strip("_")
    return out or default


class EpisodeRecorder:
    def __init__(self, root=None, topics=None, info=None):
        import threading
        self.root = os.path.abspath(root or os.environ.get("ORIN_DATA_DIR", "./data"))
        env_topics = [t.strip() for t in os.environ.get("ORIN_REC_TOPICS", "").split(",") if t.strip()]
        self.topics = list(topics or env_topics or REC_TOPICS_DEFAULT)
        self.info = dict(info or {})
        self.want_key, self.key, self.proc, self.dir = None, None, None, None
        self.state, self.msg, self.t0, self.mb, self.last = "idle", "", 0.0, 0.0, ""
        self._lock, self._run = threading.Lock(), True
        self._thread = threading.Thread(target=self._loop, name="episode_recorder", daemon=True)
        self._thread.start()

    def request(self, rec):
        """rec = {"task", "date", "ep"} while an episode should be recording, else None. Cheap; call every tick."""
        key = None
        if isinstance(rec, dict) and rec.get("ep"):
            key = (safe_name(rec.get("task")), safe_name(rec.get("date"), time.strftime("%Y-%m-%d")), safe_name(rec.get("ep"), "ep"))
        with self._lock: self.want_key = key

    def status(self):
        secs = (time.monotonic() - self.t0) if self.state in ("starting", "recording") else 0.0
        return {"state": self.state, "ep": "/".join(self.key) if self.key else self.last, "secs": round(secs, 1),
                "mb": round(self.mb, 1), "msg": self.msg}

    def close(self):
        self._run = False
        with self._lock: self.want_key = None
        self._stop()

    def _loop(self):
        while self._run:
            with self._lock: want = self.want_key
            try:
                if want != self.key:
                    if self.key is not None: self._stop()
                    if want is not None and self._run: self._start(want)
                elif self.key is not None: self._watch()
            except Exception as e:
                self.state, self.msg = "error", f"recorder: {e}"
            time.sleep(0.1)

    def _free_gb(self):
        import shutil
        os.makedirs(self.root, exist_ok=True)
        return shutil.disk_usage(self.root).free / 1e9

    def _start(self, key):
        self.key, self.mb, self.msg = key, 0.0, ""
        free = self._free_gb()
        if free < REC_MIN_FREE_GB:
            self.state, self.msg = "error", f"only {free:.1f} GB free on the robot (need {REC_MIN_FREE_GB:.0f})"; return
        self.dir = os.path.join(self.root, *key)
        os.makedirs(self.dir, exist_ok=True)
        bag, n = os.path.join(self.dir, "robot"), 1
        while os.path.exists(bag): n += 1; bag = os.path.join(self.dir, f"robot_{n}")   # ros2 bag refuses an existing dir
        self.bag = bag
        self.meta = dict(self.info, task=key[0], date=key[1], episode=key[2], topics=self.topics, bag=os.path.basename(bag),
                         start_unix=time.time(), start_iso=time.strftime("%Y-%m-%dT%H:%M:%S%z"), free_gb_at_start=round(free, 1))
        self._write_meta()
        try:
            self.log = open(os.path.join(self.dir, "ros2bag.log"), "ab")
            self.proc = subprocess.Popen(["ros2", "bag", "record", "-o", bag] + self.topics, stdout=self.log,
                                         stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        except Exception as e:
            self.proc, self.state, self.msg = None, "error", f"cannot start 'ros2 bag record': {e}"; return
        self.state, self.t0 = "starting", time.monotonic()
        print(f"[rec] START {'/'.join(key)} -> {bag}", flush=True)

    def _bag_mb(self):
        total = 0
        try:
            for f in os.listdir(self.bag): total += os.path.getsize(os.path.join(self.bag, f))
        except OSError: pass
        return total / 1e6

    def _watch(self):
        if self.proc is None: return
        if self.proc.poll() is not None:
            self.state, self.msg, self.proc = "error", f"'ros2 bag record' exited with code {self.proc.returncode} (see ros2bag.log)", None; return
        self.mb = self._bag_mb()
        if self.state == "starting" and self.mb > 0.0: self.state = "recording"
        secs = time.monotonic() - self.t0
        if secs > REC_MAX_SECS or self._free_gb() < REC_STOP_FREE_GB:
            self.msg = "stopped automatically: " + ("maximum episode length" if secs > REC_MAX_SECS else "robot disk almost full")
            with self._lock: self.want_key = None

    def _stop(self):
        key, self.last = self.key, "/".join(self.key) if self.key else self.last
        if self.proc is not None:
            try: os.killpg(self.proc.pid, signal.SIGINT)
            except Exception: pass
            try: self.proc.wait(timeout=10.0)
            except Exception:
                try: os.killpg(self.proc.pid, signal.SIGKILL)
                except Exception: pass
            self.mb = self._bag_mb()
            self.meta.update(stop_unix=time.time(), duration_s=round(time.monotonic() - self.t0, 2), size_mb=round(self.mb, 1), note=self.msg)
            self._write_meta()
            try: self.log.close()
            except Exception: pass
            print(f"[rec] STOP  {'/'.join(key)}  {self.meta['duration_s']} s, {self.mb:.0f} MB", flush=True)
        self.proc, self.key = None, None
        if self.state != "error": self.state = "idle"

    def _write_meta(self):
        try:
            with open(os.path.join(self.dir, "meta.json"), "w") as f: json.dump(self.meta, f, indent=2)
        except Exception as e: self.msg = f"meta.json: {e}"


# ---------------------------------------------------------------------------------------------------------------
# Camera streamer (data collection). A separate process that exists only while an episode is being recorded: it
# subscribes one colour camera, shrinks and JPEG-compresses each frame and sends it to the laptop over TCP
# (frame header: magic, sequence, image stamp, send time, width, height, length). Nothing is stored on the robot.
# ---------------------------------------------------------------------------------------------------------------
CAM_MAGIC, CAM_STATUS = b"A2I2", b"A2ST"
# wire format per frame: magic(4) | header length (uint16) | JSON header | payload length (uint32) | payload
# The JSON header carries every field of the original sensor_msgs/Image (stamp, frame_id, height, width, encoding,
# is_bigendian, step) plus what was sent (cam name, fmt jpg/png, out_w, out_h, seq, t_send).


def _codecs():
    """-> (name, jpeg(rgb, width, q), png16(depth_u16, width)) using OpenCV if it imports, else Pillow; or (None,..)."""
    # If numpy >= 2.0 and cv2 was compiled for numpy 1.x, importing cv2 emits C-API error and fails.
    # Safely skip cv2 if numpy >= 2 to prevent noisy tracebacks and use Pillow directly.
    allow_cv2 = True
    try:
        np_major = int(np.__version__.split(".")[0])
        if np_major >= 2:
            allow_cv2 = False
    except Exception:
        pass
    if allow_cv2:
        try:
            import cv2
            def size(w0, h0, width): return (width, int(round(h0 * width / w0))) if (width and w0 > width) else (w0, h0)
            def jpeg(rgb, width, q):
                w, h = size(rgb.shape[1], rgb.shape[0], width)
                if (w, h) != (rgb.shape[1], rgb.shape[0]): rgb = cv2.resize(rgb, (w, h), interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [int(cv2.IMWRITE_JPEG_QUALITY), int(q)])
                return (buf.tobytes(), w, h) if ok else (None, 0, 0)
            def png16(d, width):
                w, h = size(d.shape[1], d.shape[0], width)
                if (w, h) != (d.shape[1], d.shape[0]): d = cv2.resize(d, (w, h), interpolation=cv2.INTER_NEAREST)   # never blend depths
                ok, buf = cv2.imencode(".png", d, [int(cv2.IMWRITE_PNG_COMPRESSION), 1])
                return (buf.tobytes(), w, h) if ok else (None, 0, 0)
            return "opencv", jpeg, png16
        except Exception: pass
    try:
        import io
        from PIL import Image as PILImage
        def jpeg(rgb, width, q):
            im = PILImage.fromarray(rgb)
            if width and im.width > width: im = im.resize((width, int(round(im.height * width / im.width))), PILImage.BILINEAR)
            b = io.BytesIO(); im.save(b, format="JPEG", quality=int(q))
            return b.getvalue(), im.width, im.height
        def png16(d, width):
            if width and d.shape[1] > width:
                h = int(round(d.shape[0] * width / d.shape[1]))
                yi = (np.arange(h) * d.shape[0] / h).astype(int); xi = (np.arange(width) * d.shape[1] / width).astype(int)
                d = d[yi][:, xi]                                                    # nearest neighbour: never blend depths
            im = PILImage.fromarray(np.ascontiguousarray(d).astype("<u2"), mode="I;16")
            b = io.BytesIO(); im.save(b, format="PNG", compress_level=1)
            return b.getvalue(), d.shape[1], d.shape[0]
        return "pillow", jpeg, png16
    except Exception: return None, None, None


def _image_to_rgb(msg):
    """sensor_msgs/Image -> (HxWx3 uint8 RGB, "") or (None, reason) for an encoding this streamer does not handle."""
    enc, h, w = (msg.encoding or "").lower(), int(msg.height), int(msg.width)
    ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(enc)
    if ch is None: return None, f"unsupported colour encoding '{msg.encoding}'"
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    step = int(msg.step) or w * ch
    if a.size < h * step: return None, "image buffer shorter than height x step"
    a = a[:h * step].reshape(h, step)[:, :w * ch].reshape(h, w, ch)
    if ch == 1: a = np.repeat(a, 3, axis=2)
    elif enc in ("bgr8", "bgra8"): a = a[:, :, 2::-1]
    else: a = a[:, :, :3]
    return np.ascontiguousarray(a), ""


def _image_to_depth16(msg):
    """sensor_msgs/Image depth -> (HxW uint16, note) or (None, reason). 16UC1/mono16 are kept bit-exact; 32FC1
    (metres) is converted to uint16 millimetres and the note says so."""
    enc, h, w = (msg.encoding or "").lower(), int(msg.height), int(msg.width)
    raw = bytes(msg.data)
    if enc in ("16uc1", "mono16"):
        step = int(msg.step) or w * 2
        if len(raw) < h * step: return None, "depth buffer shorter than height x step"
        a = np.frombuffer(raw, dtype=np.uint8)[:h * step].reshape(h, step)[:, :w * 2]
        d = np.ascontiguousarray(a).view(">u2" if msg.is_bigendian else "<u2").reshape(h, w)
        return d.astype(np.uint16), "as_published"
    if enc == "32fc1":
        step = int(msg.step) or w * 4
        if len(raw) < h * step: return None, "depth buffer shorter than height x step"
        a = np.frombuffer(raw, dtype=np.uint8)[:h * step].reshape(h, step)[:, :w * 4]
        f = np.ascontiguousarray(a).view(">f4" if msg.is_bigendian else "<f4").reshape(h, w)
        return np.clip(np.nan_to_num(f, nan=0.0, posinf=0.0) * 1000.0, 0, 65535).astype(np.uint16), "converted_from_32FC1_metres_to_uint16_mm"
    return None, f"unsupported depth encoding '{msg.encoding}'"


def camera_main(argv):
    from sensor_msgs.msg import Image
    host, port, streams = argv[0], int(argv[1]), json.loads(argv[2])
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    sock = None
    def connect():
        nonlocal sock
        try:
            s = socket.create_connection((host, port), timeout=2.0); s.settimeout(2.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1); sock = s
        except OSError: sock = None
    def send(magic, header, payload=b""):
        nonlocal sock
        if sock is None: connect()
        if sock is None: return False
        hb = json.dumps(header).encode()
        try: sock.sendall(magic + struct.pack("<H", len(hb)) + hb + struct.pack("<I", len(payload)) + payload); return True
        except OSError: sock = None; return False
    def status(text): send(CAM_STATUS, {"msg": str(text)[:300], "t_send": time.time()})
    connect()
    name, jpeg, png16 = _codecs()
    if name is None:
        status("no image encoder on the robot (neither OpenCV nor Pillow imports)"); print("[cam] no image encoder", flush=True); return
    rclpy.init()
    node = Node("arm_teleop_camera")
    qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    state = {}
    def make_cb(cfg):
        st = state[cfg["name"]] = {"seq": 0, "last": 0.0, "warned": "", "frames": 0}
        min_dt = (1.0 / float(cfg.get("fps", 15))) * 0.9
        def cb(msg):
            now = time.monotonic()
            if now - st["last"] < min_dt: return
            st["last"] = now
            if cfg.get("kind") == "depth":
                arr, note = _image_to_depth16(msg)
                out = png16(arr, int(cfg.get("width", 0))) if arr is not None else None
                fmt = "png"
            else:
                arr, note = _image_to_rgb(msg)
                out = jpeg(arr, int(cfg.get("width", 0)), int(cfg.get("q", 85))) if arr is not None else None
                fmt = "jpg"
            if out is None or not out[0]:
                why = note or "encode failed"
                if why != st["warned"]: st["warned"] = why; status(f"{cfg['name']}: {why}")
                return
            st["seq"] += 1
            hdr = {"cam": cfg["name"], "kind": cfg.get("kind", "color"), "fmt": fmt, "seq": st["seq"], "t_send": time.time(),
                   "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, "frame_id": msg.header.frame_id,
                   "height": int(msg.height), "width": int(msg.width), "encoding": msg.encoding,
                   "is_bigendian": int(msg.is_bigendian), "step": int(msg.step), "out_w": out[1], "out_h": out[2], "note": note}
            if send(CAM_MAGIC, hdr, out[0]): st["frames"] += 1
        return cb
    for cfg in streams:
        node.create_subscription(Image, cfg["topic"], make_cb(cfg), qos)
    print(f"[cam] streaming {[c['name'] + '=' + c['topic'] for c in streams]} -> {host}:{port} ({name})", flush=True)
    ppid, t0, told = os.getppid(), time.monotonic(), False
    try:
        while rclpy.ok() and os.getppid() == ppid:          # the parent terminates us when the episode ends
            rclpy.spin_once(node, timeout_sec=0.2)
            if not told and time.monotonic() - t0 > 4.0:
                told = True
                dead = [c["topic"] for c in streams if state[c["name"]]["frames"] == 0 and not state[c["name"]]["warned"]]
                if dead: status("no image received after 4 s on: " + ", ".join(dead))
    finally:
        try: node.destroy_node(); rclpy.shutdown()
        except Exception: pass


class CameraStreamer:
    """Parent side: runs camera_main as a child process only while the laptop asks for camera frames."""
    def __init__(self, host):
        self.host, self.proc, self.key = host, None, None

    @staticmethod
    def _clean(cam):
        """Validate the laptop's request -> (port, [stream dicts]) or None. Nothing from the packet reaches a shell."""
        if not isinstance(cam, dict): return None
        out = []
        for st in (cam.get("streams") or [])[:4]:
            topic, name = str(st.get("topic", "")), safe_name(st.get("name"), "cam", 24)
            if not (topic.startswith("/") and len(topic) < 120 and all(ch.isalnum() or ch in "/_" for ch in topic)): continue
            out.append({"name": name, "topic": topic, "kind": "depth" if st.get("kind") == "depth" else "color",
                        "width": int(np.clip(int(st.get("width", 640)), 0, 1920)), "q": int(np.clip(int(st.get("q", 85)), 30, 95)),
                        "fps": float(np.clip(float(st.get("fps", 15)), 1.0, 30.0))})
        return (int(np.clip(int(cam.get("port", 5007)), 1024, 65535)), out) if out else None

    def request(self, cam):
        try: req = self._clean(cam)
        except Exception: req = None
        key = json.dumps(req, sort_keys=True) if req else None
        if key == self.key and (self.proc is None or self.proc.poll() is None): return
        self.stop()
        self.key = key
        if req is not None:
            try:
                self.proc = subprocess.Popen([sys.executable, os.path.abspath(__file__), "--camera", self.host, str(req[0]), json.dumps(req[1])],
                                             stdin=subprocess.DEVNULL)
            except Exception as e:
                print(f"[cam] cannot start: {e}", flush=True); self.proc = None

    def stop(self):
        if self.proc is not None:
            try: self.proc.terminate(); self.proc.wait(timeout=2.0)
            except Exception:
                try: self.proc.kill()
                except Exception: pass
        self.proc, self.key = None, None


if __name__ == "__main__" and len(sys.argv) > 2 and sys.argv[1] == "--pacer":
    pacer_main(sys.argv[2:])
if __name__ == "__main__" and len(sys.argv) > 2 and sys.argv[1] == "--camera":
    camera_main(sys.argv[2:])
