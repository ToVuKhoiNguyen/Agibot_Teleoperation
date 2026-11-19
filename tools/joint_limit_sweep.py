#!/usr/bin/env python3
"""tools/joint_limit_sweep.py - Measure how much each arm joint shakes at each speed / acceleration (402 mode).

Runs ON THE ORIN, standalone (no laptop, no arm_teleop_rx), like tools/arm_stream_test.py. Moves ONE joint of ONE arm
at a time around HOME (the other joints and the other arm hold still), at several cruise speeds and accelerations,
publishing at 400 Hz like the teleop pacer. The MC's measured state is logged; on the laptop
tools/joint_sweep_report.py turns it into hand vibration in mm per joint / speed / accel and proposes the per-joint
limits (JOINT_VMAX / JOINT_AMAX of arm_teleop_rx).

  Preconditions: motion_player disabled, SetAction 10 then 12 (action 402), arm_teleop_rx NOT running,
                 workspace around the chosen arm clear (shoulder joints swing the hand up to ~25 cm).
  Run:           python3 joint_limit_sweep.py --dry                 # print the plan, move nothing
                 python3 joint_limit_sweep.py                       # right arm, J1..J7, speeds x accels (~8 min)
                 python3 joint_limit_sweep.py --joints 1,4 --speeds 0.8,1.2,1.6,2.0 --accels 6
  Stop:          Ctrl+C at any time (the MC holds the last pose).
  Log:           joint_sweep_<time>.jsonl in the current directory -> scp to the laptop logs/.
"""
import os, sys, time, json, math, argparse, urllib.request
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState

NAMES = [f"idx{i}_{'left' if i < 20 else 'right'}_arm_joint{i - 12 if i < 20 else i - 19}" for i in range(13, 27)]
HOME = [0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0]
LIMS_MIN = [-2.91, -0.46, -2.91, -2.00, -2.94, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.03, -2.94, -0.30, -0.20]
LIMS_MAX = [2.91, 1.5359, 2.91, -0.03, 2.94, 0.30, 0.20, 2.91, 0.46, 2.91, 2.00, 2.94, 0.30, 0.20]
# sweep amplitude per joint J1..J7 (rad around HOME): shoulder pitch / roll / yaw, elbow, forearm yaw, wrist pitch / roll
AMP = [0.45, 0.30, 0.45, 0.40, 0.60, 0.25, 0.17]
PUB_HZ = 400
MAX_STEP = 0.0165    # rad per message (the MC rejects > 0.02)


def get_action(host, port):
    try:
        req = urllib.request.Request(f"http://{host}:{port}/rpc/aimdk.protocol.McActionService/GetAction", data=b"{}",
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            return resp.read().decode()
    except Exception as e:
        return f"(no answer: {e})"


def reachable_cruise(amp, sp, acc):
    """Seconds spent at full speed per one-way stroke of 2*amp (0 = the speed is never reached)."""
    d_acc = sp * sp / acc                                     # accelerate + brake distance
    return max(0.0, (2.0 * amp - d_acc) / sp)


class Sweeper(Node):
    def __init__(self, log_path):
        super().__init__("joint_limit_sweep")
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(JointState, "/motion/control/arm_joint_command", qos)
        self.create_subscription(JointState, "/motion/control/arm_joint_state", self.on_state, qos)
        self.create_subscription(JointState, "/motion/control/arm_joint_command", lambda m: setattr(self, "seen", self.seen + 1), qos)
        self.meas, self.seen, self.sent, self.phase, self.v_cmd = None, 0, 0, "init", 0.0
        self.log = open(log_path, "w", buffering=1 << 16)

    def on_state(self, msg):
        m = dict(zip(msg.name, msg.position))
        if all(n in m for n in NAMES): q = [float(m[n]) for n in NAMES]
        elif len(msg.position) >= 14: q = [float(x) for x in msg.position[:14]]
        else: return
        self.meas = q
        self.log.write(json.dumps({"k": "S", "t": round(time.monotonic(), 5),
                                   "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                   "ph": self.phase, "vc": round(self.v_cmd, 4), "q": [round(x, 5) for x in q]}) + "\n")

    def send(self, q):
        msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.name, msg.position = NAMES, [float(x) for x in q]
        msg.velocity, msg.effort = [0.0] * 14, [0.0] * 14
        self.pub.publish(msg); self.sent += 1

    def pump(self, until):
        while True:
            left = until - time.monotonic()
            if left <= 0.0003: break
            rclpy.spin_once(self, timeout_sec=left)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=["right", "left"], default="right")
    ap.add_argument("--joints", default="1,2,3,4,5,6,7", help="joints to sweep, 1..7")
    ap.add_argument("--speeds", default="0.4,0.8,1.2,1.6,2.0", help="cruise speeds rad/s (max 2.5)")
    ap.add_argument("--accels", default="6", help="accelerations rad/s^2 (max 12); e.g. 4,6,8")
    ap.add_argument("--strokes", type=int, default=4, help="one-way strokes per phase (each 2*amp)")
    ap.add_argument("--dry", action="store_true", help="print the plan only")
    ap.add_argument("--mc-host", default="192.168.100.100"); ap.add_argument("--mc-port", type=int, default=56322)
    a = ap.parse_args()
    off = 0 if a.arm == "left" else 7
    joints = [int(x) for x in a.joints.split(",") if x.strip()]
    speeds = [min(2.5, max(0.05, float(x))) for x in a.speeds.split(",") if x.strip()]
    accels = [min(12.0, max(0.5, float(x))) for x in a.accels.split(",") if x.strip()]
    if any(not 1 <= j <= 7 for j in joints): sys.exit("joints are 1..7")
    plan = []
    for j in joints:
        i = off + j - 1
        amp = min(AMP[j - 1], HOME[i] - LIMS_MIN[i] - 0.03, LIMS_MAX[i] - HOME[i] - 0.03)
        for acc in accels:
            for sp in speeds:
                cr = reachable_cruise(amp, sp, acc)
                if sp / PUB_HZ > MAX_STEP: continue
                plan.append((j, i, amp, sp, acc, cr))
    secs = sum(a.strokes * (2 * amp / sp + sp / acc) + 1.5 for _, _, amp, sp, acc, _ in plan)
    print(f"[sweep] {a.arm} arm, {len(plan)} phases, about {secs / 60:.1f} min (+ homing)")
    for j, i, amp, sp, acc, cr in plan:
        note = "" if cr >= 0.15 else "  (speed barely reached: amplitude too small - result marked short)"
        print(f"  J{j}  +-{amp:.2f} rad around {HOME[i]:+.2f}   speed {sp:.1f} rad/s   accel {acc:.0f} rad/s^2   cruise {cr:.2f} s/stroke{note}")
    if a.dry: return

    act = get_action(a.mc_host, a.mc_port)
    if not any(k in act for k in ("RL_LOCOMOTION_ARM_EXT_JOINT_SERVO", "402")):
        sys.exit(f"FATAL: robot action is not 402: {act[:160]}\nRun SetAction.py (10, then 12) first.")
    try: os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(30)); prio = "SCHED_FIFO 30"
    except Exception: prio = "normal priority (run 'sudo prlimit --pid $$ --rtprio=30:30' first for even timing)"
    rclpy.init()
    node = Sweeper(time.strftime("joint_sweep_%Y%m%d_%H%M%S.jsonl"))
    node.log.write(json.dumps({"k": "CONFIG", "arm": a.arm, "pub_hz": PUB_HZ, "home": HOME, "prio": prio,
                               "plan": [dict(j=j, amp=amp, sp=sp, acc=acc, cruise=cr) for j, _, amp, sp, acc, cr in plan]}) + "\n")
    print(f"[sweep] {prio}")
    try:
        print("[sweep] checking that nobody else is publishing arm commands (1.5 s)...")
        node.pump(time.monotonic() + 1.5)
        if node.seen > 0: sys.exit(f"FATAL: {node.seen} arm commands from another publisher. Stop arm_teleop_rx first.")
        t0 = time.monotonic()
        while node.meas is None:
            node.pump(time.monotonic() + 0.1)
            if time.monotonic() - t0 > 5.0: sys.exit("FATAL: no /motion/control/arm_joint_state received.")
        start = list(node.meas)
        dist = max(abs(s - h) for s, h in zip(start, HOME))
        dur = max(3.0, dist / 0.3)
        print(f"[sweep] moving smoothly to HOME in {dur:.1f} s...")
        node.phase, dt, t_next, n = "homing", 1.0 / PUB_HZ, time.monotonic(), int(dur * PUB_HZ)
        for k in range(1, n + 1):
            f = 0.5 * (1.0 - math.cos(math.pi * k / n))
            node.send([s + f * (h - s) for s, h in zip(start, HOME)]); t_next += dt; node.pump(t_next)
        # every joint must actually be servoed: 17/11 an elbow stayed limp after an MC step rejection (stuck at 0.72
        # while commanded 0.5) and the whole sweep measured a floppy arm. Hold HOME 1.5 s, then compare.
        t_hold = time.monotonic() + 1.5
        while time.monotonic() < t_hold:
            node.send(HOME); t_next += dt; node.pump(t_next)
        bad = [(f"{'L' if k < 7 else 'R'}J{k % 7 + 1}", node.meas[k], HOME[k]) for k in range(14) if abs(node.meas[k] - HOME[k]) > 0.08]
        if bad:
            print("[sweep] FATAL: joint(s) not following the command (measured vs HOME): " +
                  ", ".join(f"{n} {m:+.3f} vs {h:+.3f}" for n, m, h in bad) +
                  "\n        The MC is not servoing them. Run SetAction.py 10, then 12 again (power-cycle the robot if that does "
                  "not fix it), then re-run this test.")
            return
        print("[sweep] all 14 joints follow the command (within 0.08 rad of HOME)")
        cmd = list(HOME)
        for k, (j, i, amp, sp, acc, cr) in enumerate(plan, 1):
            node.phase = f"J{j}:{sp:g}:{acc:g}"
            print(f"\n[sweep] phase {k}/{len(plan)}: J{j} speed {sp:g} rad/s accel {acc:g} rad/s^2", flush=True)
            x, v, strokes, goal = 0.0, 0.0, 0, amp                  # offset from HOME, speed, strokes done
            t_next = time.monotonic()
            while True:
                e = goal - x
                if abs(e) < 1e-5 and abs(v) < 1e-4:
                    strokes += 1
                    if goal == 0.0: break                          # back at HOME: phase over
                    goal = 0.0 if strokes >= a.strokes else (-amp if goal > 0 else amp)
                    continue
                v_des = math.copysign(min(sp, math.sqrt(2.0 * acc * abs(e))), e)
                v += max(-acc * dt, min(acc * dt, v_des - v))
                step = v * dt
                if abs(step) >= abs(e): step, v = e, 0.0
                x += step
                cmd[i] = HOME[i] + x
                node.v_cmd = v
                node.send(cmd); t_next += dt; node.pump(t_next)
            node.phase, node.v_cmd = "pause", 0.0
            hold = time.monotonic() + 1.5
            while time.monotonic() < hold:
                node.send(HOME); t_next += dt; node.pump(t_next)
        print(f"\n[sweep] finished. Log: {node.log.name}  ({node.sent} commands). scp it to the laptop logs/.")
    except KeyboardInterrupt:
        print(f"\n[sweep] stopped by Ctrl+C - the MC holds the last pose. Partial log: {node.log.name}")
    finally:
        try: node.log.close()
        except Exception: pass
        try: node.destroy_node(); rclpy.shutdown()
        except Exception: pass


if __name__ == "__main__":
    main()
