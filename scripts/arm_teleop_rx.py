#!/usr/bin/env python3
"""scripts/arm_teleop_rx.py - Audited 50 Hz Arm & Hand Teleoperation Receiver for Orin."""
import os, sys, time, json, math, socket, hashlib, argparse, urllib.request
import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
except ImportError:
    rclpy, Node, JointState = None, object, None

ARM_NAMES_LONG = [f"idx{i}_{'left' if i<20 else 'right'}_arm_joint{i-12 if i<20 else i-19}" for i in range(13, 27)]
HAND_NAMES = [f"{s}_{f}" for s in ("left", "right") for f in ("thumb_0", "thumb_1", "index", "middle", "ring", "pinky")]
LIMS_MIN = [-2.91, -0.46, -2.91, -2.00, -2.94, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.10, -2.94, -0.30, -0.20]
LIMS_MAX = [ 2.91,  1.5359, 2.91, -0.10,  2.94,  0.30,  0.20,  2.91,  0.46,   2.91, 2.00,  2.94,  0.30,  0.20]
HOME = [0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0]
SPEED_LEVELS = {1: (0.40, 1.00), 2: (0.60, 1.50), 3: (0.80, 2.00), 4: (1.00, 2.50)}

def rpc_get_action(host="192.168.100.100", port=56322):
    try:
        req = urllib.request.Request(f"http://{host}:{port}/rpc/aimdk.protocol.McActionService/GetAction", data=b"{}", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            data = json.loads(resp.read().decode())
            info = data.get("info", {}) if isinstance(data, dict) else {}
            return info.get("current_action", "") or info.get("current_ext_action", "") or data.get("action", "") or data.get("ext_action", "") or str(data)
    except Exception: return ""

class ArmTeleopRx(Node):
    def __init__(self, mode_live=True, allow_ip="192.168.2.75", port=5005, tx_state_port=5006,
                 speed_level=3, deadband_rad=0.003, enable_thumb_swing=True):
        super().__init__("arm_teleop_rx")
        self.is_live, self.allow_ip, self.tx_state_port = mode_live, allow_ip, tx_state_port
        self.deadband_rad, self.enable_thumb_swing = deadband_rad, enable_thumb_swing
        self.speed_level = speed_level if speed_level in SPEED_LEVELS else 3
        self.max_vel, self.max_acc = SPEED_LEVELS[self.speed_level]
        self.max_step = self.max_vel * 0.010  # 100 Hz step cap (e.g. 0.008 rad at speed 3)
        self.curr_cmd_q, self.curr_cmd_v = None, np.zeros(14)
        self.target_q, self.target_h, self.target_engaged = None, [0.0]*12, [False, False]
        self.meas_h, self.hand_enabled = None, True
        self.session_lock, self.foreign_sender_count = None, 0
        self.last_pkt_time, self.rx_count, self.last_tick_t = 0.0, 0, time.monotonic()
        self.fault_latched, self.holding = False, False
        self.fault_reason = ""
        self.has_published_engaged = False
        self.our_arm_pub_count = 0
        self.total_arm_cmd_count = 0
        self.dual_check_timer_t = time.monotonic()
        self.pkt_buffer = []  # [(arrival_mono_t, q_14)] for playout interpolation
        self.playout_delay = 0.030  # 30 ms jitter/drop buffer
        with open(__file__, "rb") as f: self.file_sha = hashlib.sha256(f.read()).hexdigest()
        self.meas_q = None
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(JointState, "/motion/control/arm_joint_command", qos) if self.is_live else None
        self.hand_cmd_pub = self.create_publisher(JointState, "/motion/control/hand_joint_command", qos) if self.is_live else None
        self.state_sub = self.create_subscription(JointState, "/motion/control/arm_joint_state", self._on_state, qos)
        self.hand_state_sub = self.create_subscription(JointState, "/motion/control/hand_joint_state", self._on_hand_state, qos)
        self.arm_cmd_monitor_sub = self.create_subscription(JointState, "/motion/control/arm_joint_command", self._on_arm_cmd_seen, qos)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.bind_ip = os.environ.get("ROBOT_BIND_IP", "127.0.0.1" if allow_ip in ("127.0.0.1", "localhost") else "0.0.0.0")
        self.sock.bind((self.bind_ip, port))
        self.sock.setblocking(False)
        self.running = True
        self.timer = self.create_timer(0.010, self._control_tick)  # 100 Hz timer
        print(f"[arm_teleop_rx] SHA256: {self.file_sha}")
        print(f"[arm_teleop_rx] 100Hz. LIVE={self.is_live} | lvl={self.speed_level} (v={self.max_vel:.2f}, step<={self.max_step:.4f}) | thumb={self.enable_thumb_swing}")

    def _on_arm_cmd_seen(self, msg: JointState):
        self.total_arm_cmd_count += 1

    def _on_hand_state(self, msg: JointState):
        fid = getattr(getattr(msg, "header", None), "frame_id", "")
        self.hand_enabled = (not fid or fid == "AgiHand")
        if not self.hand_enabled: return
        n2p = dict(zip(msg.name, msg.position))
        if all(k in n2p for k in HAND_NAMES): self.meas_h = [float(n2p[k]) for k in HAND_NAMES]
        elif len(msg.position) >= 12: self.meas_h = [float(x) for x in msg.position[:12]]

    def _on_state(self, msg: JointState):
        name_map = dict(zip(msg.name, msg.position))
        if all(k in name_map for k in ARM_NAMES_LONG): self.meas_q = [name_map[k] for k in ARM_NAMES_LONG]
        elif any("idx13" in k for k in name_map): self.meas_q = [name_map[k] for k in sorted(name_map.keys()) if "idx" in k][:14]
        elif len(msg.position) >= 14: self.meas_q = list(msg.position[:14])
        if self.meas_q and self.allow_ip not in ("0.0.0.0",):
            try:
                pkt = {"t": time.time(), "q": [float(round(x, 5)) for x in self.meas_q],
                       "speed_level": self.speed_level, "robot_fault": bool(self.fault_latched),
                       "fault_reason": str(self.fault_reason)}
                if self.meas_h is not None: pkt["hand"] = [float(round(x, 1)) for x in self.meas_h]
                self.sock.sendto(json.dumps(pkt).encode(), (self.allow_ip, self.tx_state_port))
            except Exception: pass

    def _drain_socket(self):
        newest_pkt = None
        while True:
            try: data, addr = self.sock.recvfrom(2048)
            except (BlockingIOError, socket.error): break
            if self.allow_ip != "0.0.0.0" and addr[0] != self.allow_ip:
                self.foreign_sender_count += 1; continue
            try: p = json.loads(data.decode("utf-8"))
            except Exception: continue
            q = p.get("q", [])
            if len(q) != 14 or any(math.isnan(x) or math.isinf(x) for x in q): continue
            sender_id = (addr[0], p.get("session_id", "default"))
            if self.session_lock is None: self.session_lock = sender_id
            elif self.session_lock != sender_id:
                self.foreign_sender_count += 1; self.holding = True; continue
            newest_pkt = p
        if newest_pkt is not None:
            self.target_q = [float(np.clip(newest_pkt["q"][i], LIMS_MIN[i], LIMS_MAX[i])) for i in range(14)]
            self.pkt_buffer.append((time.monotonic(), np.array(self.target_q, dtype=np.float64)))
            if len(self.pkt_buffer) > 10: self.pkt_buffer = self.pkt_buffer[-10:]
            h = newest_pkt.get("hand")
            if h and len(h) == 12: self.target_h = [float(np.clip(x, 0.0, 800.0)) for x in h]
            eng = newest_pkt.get("engaged")
            if eng and len(eng) >= 2: self.target_engaged = [bool(eng[0]), bool(eng[1])]
            self.last_pkt_time, self.rx_count = time.monotonic(), self.rx_count + 1

    def _get_playout_target(self, now):
        if not self.pkt_buffer: return np.array(self.target_q) if self.target_q else np.array(self.curr_cmd_q)
        if len(self.pkt_buffer) == 1: return self.pkt_buffer[0][1]
        t_play = now - self.playout_delay
        if t_play <= self.pkt_buffer[0][0]: return self.pkt_buffer[0][1]
        if t_play >= self.pkt_buffer[-1][0]: return self.pkt_buffer[-1][1]
        for i in range(len(self.pkt_buffer) - 1):
            t0, q0 = self.pkt_buffer[i]
            t1, q1 = self.pkt_buffer[i+1]
            if t0 <= t_play <= t1:
                return q0 + ((t_play - t0) / (t1 - t0) if t1 > t0 else 1.0) * (q1 - q0)
        return self.pkt_buffer[-1][1]

    def _final_line_guard_and_publish(self, target_pos, dt):
        if self.fault_latched or self.holding: return
        err = np.asarray(target_pos, dtype=np.float64) - np.asarray(self.curr_cmd_q, dtype=np.float64)
        D = float(np.max(np.abs(err)))
        step = err * (self.max_step / D) if D > self.max_step else err
        next_cmd_q = np.asarray(self.curr_cmd_q) + step
        if np.any(np.isnan(next_cmd_q)) or np.any(np.isinf(next_cmd_q)):
            self.fault_latched, self.fault_reason = True, "NaN/inf in command"
            return
        self.curr_cmd_v = step / dt
        self.curr_cmd_q = [float(np.clip(next_cmd_q[i], LIMS_MIN[i], LIMS_MAX[i])) for i in range(14)]
        if self.is_live and self.cmd_pub:
            msg = JointState(); msg.header.stamp = self.get_clock().now().to_msg()
            msg.name, msg.position, msg.velocity = ARM_NAMES_LONG, list(self.curr_cmd_q), list(self.curr_cmd_v)
            self.cmd_pub.publish(msg)
            self.our_arm_pub_count += 1

    def _control_tick(self):
        now = time.monotonic()
        dt = min(0.040, max(0.010, now - self.last_tick_t))
        self.last_tick_t = now
        self._drain_socket()
        meas = list(self.meas_q) if self.meas_q else None

        # Runtime dual publisher check every 0.3 s
        if self.is_live and (now - self.dual_check_timer_t >= 0.30):
            elapsed = now - self.dual_check_timer_t
            if self.our_arm_pub_count > 0:
                other_msgs = self.total_arm_cmd_count - self.our_arm_pub_count
                if other_msgs > 0.10 * self.our_arm_pub_count:
                    self.fault_latched = True
                    self.fault_reason = "DUAL PUBLISHER: another node is publishing on /motion/control/arm_joint_command"
                    print(f"\n[FAULT] DUAL PUBLISHER DETECTED! Total: {self.total_arm_cmd_count}, Ours: {self.our_arm_pub_count}")
            self.dual_check_timer_t, self.our_arm_pub_count, self.total_arm_cmd_count = now, 0, 0

        # Do not publish or move on its own before receiving valid ENGAGED packet
        is_any_engaged = any(self.target_engaged)
        is_fresh = (now - self.last_pkt_time <= 0.400) and not self.holding
        if not (self.has_published_engaged or (is_any_engaged and is_fresh)):
            if meas: self.curr_cmd_q = list(meas)
            return

        if not self.has_published_engaged:
            self.curr_cmd_q = list(meas) if meas else list(HOME)
            self.curr_cmd_v = np.zeros(14)
            self.has_published_engaged = True
            print("[ENGAGE] First command synchronized to measured pose. Teleop live active.")

        target = self._get_playout_target(now) if (is_fresh and self.target_q) else np.array(self.curr_cmd_q)
        self._final_line_guard_and_publish(target, dt)

        if self.is_live and self.hand_cmd_pub and self.hand_enabled:
            is_hand_fresh = is_fresh and not self.holding and not self.fault_latched
            if not is_hand_fresh: cmd_h = np.zeros(12)
            else:
                cmd_h = np.array(self.target_h, dtype=np.float64)
                if not self.target_engaged[0]: cmd_h[:6] = 0.0
                if not self.target_engaged[1]: cmd_h[6:] = 0.0
                if not self.enable_thumb_swing: cmd_h[0] = 0.0; cmd_h[6] = 0.0
                cmd_h = np.clip(cmd_h, 0.0, 800.0)
            hmsg = JointState(); hmsg.header.stamp = self.get_clock().now().to_msg()
            hmsg.header.frame_id = "AgiHand"; hmsg.name = HAND_NAMES
            hmsg.position = [float(x) for x in cmd_h]; hmsg.effort = [1140.0] * 12
            self.hand_cmd_pub.publish(hmsg)

    def shutdown_hold(self): self.holding, self.running = True, False

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

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dry", action="store_true", help="Run dry-run mode (default: live)")
    p.add_argument("--speed", type=int, default=3, choices=[1, 2, 3, 4], help="Speed level (1..4)")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--tx-state-port", type=int, default=5006)
    p.add_argument("--allow-ip", type=str, default="192.168.2.75")
    p.add_argument("--no-thumb-swing", action="store_true")
    args, _ = p.parse_known_args()
    live_active = not args.dry
    if live_active:
        print("="*60 + f"\n  WARNING: REAL ROBOT TELEOPERATION (IP: {args.allow_ip}, Speed: {args.speed})\n  Starting in 3s... (Ctrl+C to abort)\n" + "="*60)
        try:
            for i in range(3, 0, -1): print(f"  {i}...", flush=True); time.sleep(1.0)
        except KeyboardInterrupt: print("\nAborted."); sys.exit(0)
        curr_act = rpc_get_action()
        if not any(k in curr_act for k in ("RL_LOCOMOTION_ARM_EXT_JOINT_SERVO", "402")):
            print(f"FATAL: Robot action is '{curr_act}'. Must set to RL_LOCOMOTION_ARM_EXT_JOINT_SERVO (12)!"); sys.exit(1)
    rclpy.init()
    node = ArmTeleopRx(mode_live=live_active, allow_ip=args.allow_ip, port=args.port, tx_state_port=args.tx_state_port, speed_level=args.speed, enable_thumb_swing=not args.no_thumb_swing)
    if live_active:
        arm_pubs = check_other_publishers(node, "/motion/control/arm_joint_command")
        hand_pubs = check_other_publishers(node, "/motion/control/hand_joint_command")
        print(f"[PUB_CHECK] publishers: arm={arm_pubs or 'clean'}, hand={hand_pubs or 'clean'}")
        arm_rate = check_topic_rate_preflight(node, "/motion/control/arm_joint_command", 1.5)
        hand_rate = check_topic_rate_preflight(node, "/motion/control/hand_joint_command", 1.5)
        print(f"[PREFLIGHT] arm: {arm_rate:.1f} msg/s, hand: {hand_rate:.1f} msg/s")
        if arm_rate > 0.0 or hand_rate > 0.0:
            print(f"FATAL: Active publisher detected on robot command topic! arm={arm_pubs}, hand={hand_pubs}"); sys.exit(1)
    try: rclpy.spin(node)
    except (KeyboardInterrupt, Exception): pass
    finally:
        node.shutdown_hold()
        try: node.destroy_node()
        except Exception: pass
        rclpy.shutdown()

if __name__ == "__main__": main()
