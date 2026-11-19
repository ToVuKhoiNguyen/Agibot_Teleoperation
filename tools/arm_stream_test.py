#!/usr/bin/env python3
"""tools/arm_stream_test.py - Which property of the command stream makes the real arm vibrate?

Runs ON THE ORIN, standalone (no laptop, no arm_teleop_rx). Like the vendor's arm.py it moves only the two
elbows (joint 4 of each arm) around HOME, but in several phases that differ in ONE property at a time:
publish rate (50 / 100 Hz), speed (0.1 .. 0.8 rad/s) and the JointState velocity field (0 like the vendor / true).
Watch the arm in each phase and note which phases vibrate; everything is also logged for analysis.

  Preconditions: motion_player disabled, SetAction 10 then 12 (action 402), arm_teleop_rx NOT running,
                 workspace around both forearms clear (elbows swing +-0.3 rad = about +-8 cm at the hands).
  Run:           python3 arm_stream_test.py
                 python3 arm_stream_test.py --phases 50:0.1:0,50:0.8:0      # rate:speed_rad_s:velocity_field(0|1)
                 rate = 20/25/50/100/200/400/500 (Hz, free-running timer) or sync / sync3 / sync5 / sync7 =
                 exactly ONE command per MC state message, sent 0 / 3 / 5 / 7 ms after it (locked to the MC's own clock).
                 python3 arm_stream_test.py --compare      # 100, 200, 400 Hz and the sync variants at 0.6 rad/s
  Result:        a table "ripple % of speed" per phase is printed at the end (lower = smoother).
  Stop:          Ctrl+C (the MC holds the last pose).
  Log:           arm_stream_test_<time>.jsonl in the current directory.
"""
import sys, time, json, math, argparse, urllib.request
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

NAMES = [f"idx{i}_{'left' if i < 20 else 'right'}_arm_joint{i - 12 if i < 20 else i - 19}" for i in range(13, 27)]
HOME = [0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0]
DEFAULT_PHASES = "50:0.1:0,50:0.4:0,50:0.8:0,100:0.8:0,100:0.8:1,100:0.4:1"
COMPARE_PHASES = "200:0.6:0,100:0.6:0,400:0.6:0,sync:0.6:0,sync3:0.6:0,sync5:0.6:0,sync7:0.6:0,200:0.6:0"
RATES = (20, 25, 50, 100, 200, 400, 500)
ACC = 2.0            # rad/s^2 for the elbow sweep
MAX_STEP = 0.0165    # rad per message (the MC rejects > 0.02)


def get_action(host="192.168.100.100", port=56322):
    try:
        req = urllib.request.Request(f"http://{host}:{port}/rpc/aimdk.protocol.McActionService/GetAction", data=b"{}",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            return resp.read().decode()
    except Exception as e:
        return f"(no answer: {e})"


class Tester(Node):
    def __init__(self, log_path):
        super().__init__("arm_stream_test")
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(JointState, "/motion/control/arm_joint_command", qos)
        self.create_subscription(JointState, "/motion/control/arm_joint_state", self.on_state, qos)
        self.create_subscription(JointState, "/motion/control/arm_joint_command", self.on_cmd, qos)
        self.meas, self.seen, self.sent, self.phase = None, 0, 0, "init"
        self.n_state, self.state_stamp = 0, 0.0            # MC state messages received / stamp of the newest
        self.rows = {}                                     # phase -> [(stamp, v_cmd, qL, qR, vL, vR)] for the summary
        self.v_cmd, self.gaps = 0.0, {}                    # commanded elbow speed now; phase -> publish gaps (s)
        self._t_last_send = None
        self.log = open(log_path, "w", buffering=1 << 16)

    def on_cmd(self, msg): self.seen += 1

    def on_state(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(n in m for n in NAMES): q = [float(m[n]) for n in NAMES]
        elif len(msg.position) >= 14: q = [float(x) for x in msg.position[:14]]
        else: return
        self.meas = q
        self.n_state += 1; self.state_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rec = {"k": "S", "t": time.monotonic(), "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
               "ph": self.phase, "q": [round(x, 5) for x in q]}
        if len(msg.velocity) >= 14 and len(msg.name) == len(msg.velocity):
            v = dict(zip(msg.name, msg.velocity)); rec["v"] = [round(float(v.get(n, 0.0)), 4) for n in NAMES]
        if len(msg.effort) >= 14 and len(msg.name) == len(msg.effort):
            e = dict(zip(msg.name, msg.effort)); rec["e"] = [round(float(e.get(n, 0.0)), 3) for n in NAMES]
        self.log.write(json.dumps(rec) + "\n")
        if ":" in self.phase:
            vv = rec.get("v")
            self.rows.setdefault(self.phase, []).append((rec["stamp"], self.v_cmd, q[3], q[10],
                                                         vv[3] if vv else None, vv[10] if vv else None))

    def send(self, q, vel, vel_field):
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = NAMES, [float(x) for x in q]
        msg.velocity = [float(x) for x in vel] if vel_field else [0.0] * 14
        msg.effort = [0.0] * 14
        self.pub.publish(msg); self.sent += 1
        now = time.monotonic()
        if ":" in self.phase and self._t_last_send is not None: self.gaps.setdefault(self.phase, []).append(now - self._t_last_send)
        self._t_last_send = now
        self.log.write(json.dumps({"k": "C", "t": time.monotonic(), "ph": self.phase, "q": [round(float(x), 5) for x in q]}) + "\n")

    def pump(self, until):
        """Handle incoming messages until the given monotonic time (keeps the absolute publish schedule)."""
        while True:
            left = until - time.monotonic()
            if left <= 0.0005: break
            rclpy.spin_once(self, timeout_sec=left)   # sleeps until a message arrives or the deadline


def summarize(node):
    """Ripple per phase from the MC's own state stream: deviation of the elbow speed from its local trend while the
    command is in steady cruise, as a percentage of the cruise speed. Needs no laptop tools."""
    def rms(x): return math.sqrt(sum(a * a for a in x) / len(x)) if x else float("nan")
    print("\n" + "=" * 78 + "\n  RESULT - ripple of the measured elbow speed in steady cruise (lower = smoother)\n" + "=" * 78)
    print(f"  {'phase':24s} {'cruise s':>8s} {'ripple rad/s':>12s} {'% of speed':>10s} {'publish gap p50/p95 ms':>24s}")
    for ph, rows in node.rows.items():
        sp = float(ph.split(":")[2]); dev = []; n = 0
        for col_q, col_v in ((2, 4), (3, 5)):
            t = [r[0] for r in rows]; vc = [r[1] for r in rows]
            if rows[0][col_v] is not None: v = [abs(r[col_v]) for r in rows]
            else: v = [0.0] + [abs((rows[i][col_q] - rows[i - 1][col_q]) / max(1e-4, t[i] - t[i - 1])) for i in range(1, len(rows))]
            W = 8                                             # +-80 ms window: local trend = mean, cruise = command at speed throughout
            for i in range(W, len(rows) - W):
                if min(abs(x) for x in vc[i - W:i + W + 1]) < 0.98 * sp: continue
                dev.append(v[i] - sum(v[i - W:i + W + 1]) / (2 * W + 1)); n += 1
        g = sorted(node.gaps.get(ph, [0.0]))
        r = rms(dev)
        print(f"  {ph:24s} {n / 200.0:8.1f} {r:12.4f} {100.0 * r / sp:9.1f}% {g[len(g) // 2] * 1000:12.1f} / {g[int(len(g) * 0.95)] * 1000:.1f}")
    print("  (send this table back; the full log is in the .jsonl file)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phases", default=DEFAULT_PHASES, help="comma list of rate_hz:speed_rad_s:velocity_field(0|1)")
    ap.add_argument("--compare", action="store_true", help="compare publish strategies at 0.6 rad/s: " + COMPARE_PHASES)
    ap.add_argument("--amp", type=float, default=0.3, help="elbow sweep amplitude around HOME, rad (max 0.35)")
    ap.add_argument("--secs", type=float, default=9.0, help="seconds per phase")
    ap.add_argument("--mc-host", default="192.168.100.100"); ap.add_argument("--mc-port", type=int, default=56322)
    args = ap.parse_args()
    amp = max(0.05, min(0.35, args.amp))
    phases = []
    for p in (COMPARE_PHASES if args.compare else args.phases).split(","):
        hz, sp, vf = p.split(":"); sp = float(sp)
        if hz.startswith("sync"): sync_ms, hz = float(hz[4:] or 0), 100          # one command per MC state message
        else: sync_ms, hz = None, int(hz)
        if hz not in RATES or not (0.02 <= sp <= 0.8) or (sync_ms is not None and not 0 <= sync_ms <= 9):
            sys.exit(f"bad phase '{p}' (rate {'/'.join(map(str, RATES))} Hz or sync[0-9], speed 0.02..0.8 rad/s)")
        if sp / hz > MAX_STEP: sys.exit(f"phase '{p}': step {sp / hz * 1000:.1f} mrad per message is above {MAX_STEP * 1000:.1f}")
        phases.append((p.split(":")[0], hz, sp, vf == "1", sync_ms))

    act = get_action(args.mc_host, args.mc_port)
    if not any(k in act for k in ("RL_LOCOMOTION_ARM_EXT_JOINT_SERVO", "402")):
        sys.exit(f"FATAL: robot action is not 402 (ARM_EXT_JOINT_SERVO): {act[:160]}\nRun SetAction.py (10, then 12) first.")
    rclpy.init()
    node = Tester(time.strftime("arm_stream_test_%Y%m%d_%H%M%S.jsonl"))
    try:
        print("[test] checking that nobody else is publishing arm commands (1.5 s)...")
        node.pump(time.monotonic() + 1.5)
        if node.seen > 0: sys.exit(f"FATAL: {node.seen} arm commands seen from another publisher. Stop arm_teleop_rx / arm.py first.")
        t0 = time.monotonic()
        while node.meas is None:
            node.pump(time.monotonic() + 0.1)
            if time.monotonic() - t0 > 5.0: sys.exit("FATAL: no /motion/control/arm_joint_state received.")
        start = list(node.meas)
        dist = max(abs(a - b) for a, b in zip(start, HOME))
        dur = max(3.0, dist / 0.3)
        print(f"[test] moving smoothly to HOME in {dur:.1f} s (largest joint offset {dist:.2f} rad)...")
        node.phase, dt, t_next, n = "homing", 0.02, time.monotonic(), int(dur / 0.02)
        for i in range(1, n + 1):
            a = 0.5 * (1.0 - math.cos(math.pi * i / n))
            node.send([s + a * (h - s) for s, h in zip(start, HOME)], [0.0] * 14, False)
            t_next += dt; node.pump(t_next)

        x, v = 0.0, 0.0                      # elbow offset from HOME and its speed (same sweep on both arms)
        for k, (label, hz, sp, vf, sync_ms) in enumerate(phases, 1):
            dt = 1.0 / hz
            node.phase = f"{k}:{label if sync_ms is not None else str(hz) + 'Hz'}:{sp}:{'vel' if vf else 'vel0'}"
            node._t_last_send = None
            print("\n" + "=" * 72 + f"\n  PHASE {k}/{len(phases)}:  {label if sync_ms is not None else str(hz) + ' Hz'}   speed {sp} rad/s   velocity field: {'true value' if vf else '0 (vendor)'}"
                  f"   step {sp * dt * 1000:.1f} mrad\n" + "=" * 72, flush=True)
            goal, t_end, t_next = amp, time.monotonic() + args.secs, time.monotonic()
            last_stamp = node.state_stamp
            while True:
                if sync_ms is not None:
                    # wait for the next MC state message, then advance by exactly the MC time that passed (its stamps),
                    # so the MC receives one evenly spaced position per cycle whatever this process' wake-up jitter is
                    n0, t_wait = node.n_state, time.monotonic()
                    while node.n_state == n0 and time.monotonic() - t_wait < 0.1: rclpy.spin_once(node, timeout_sec=0.02)
                    dt = min(0.025, max(0.005, node.state_stamp - last_stamp)) if node.n_state != n0 else 0.01
                    last_stamp = node.state_stamp
                    if sync_ms > 0: time.sleep(sync_ms / 1000.0)
                e = goal - x
                if abs(e) < 1e-4 and abs(v) < 1e-3:
                    if time.monotonic() >= t_end and abs(goal) < 1e-9: break       # phase over and back at centre
                    goal = 0.0 if time.monotonic() >= t_end else (-amp if goal >= 0 else amp)
                    continue
                v_des = math.copysign(min(sp, math.sqrt(2.0 * 0.8 * ACC * abs(e))), e)
                v += max(-ACC * dt, min(ACC * dt, v_des - v))
                step = v * dt
                if abs(step) > abs(e) and step * e > 0: step, v = e, 0.0
                x += step
                q = list(HOME); q[3] = HOME[3] - x; q[10] = HOME[10] + x
                vel = [0.0] * 14; vel[3], vel[10] = -v, v
                node.v_cmd = v
                node.send(q, vel, vf)
                if sync_ms is None: t_next += dt; node.pump(t_next)
            print(f"  phase {k} done - did the arm vibrate? (note it down)", flush=True)
            hold_until = time.monotonic() + 2.0                                   # hold still between phases
            node.phase, node.v_cmd = "pause", 0.0
            while time.monotonic() < hold_until:
                node.send(HOME, [0.0] * 14, False); t_next = time.monotonic() + 0.02; node.pump(t_next)
        print(f"\n[test] finished. Log: {node.log.name}  (sent {node.sent} commands)")
        summarize(node)
    except KeyboardInterrupt:
        print("\n[test] stopped by Ctrl+C - the MC holds the last pose.")
        try: summarize(node)
        except Exception: pass
    finally:
        try: node.log.close()
        except Exception: pass
        try: node.destroy_node(); rclpy.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()
