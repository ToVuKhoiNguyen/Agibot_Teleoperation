#!/usr/bin/env python3
"""Agibot A2 Ultra - Safe Arm Teleoperation UDP Receiver Node for Orin (Self-Contained with Auto-Homing)"""
import os, sys, time, json, math, socket, threading, argparse, urllib.request

def rpc_set_action(action_name, host="192.168.100.100", port=56322):
    url = f"http://{host}:{port}/rpc/aimdk.protocol.McActionService/SetAction"
    payload = {
        "header": {"timestamp": {"seconds": int(time.time()), "nanos": 0}, "control_source": "ControlSource_SAFE"},
        "command": {"action": "McAction_USE_EXT_CMD", "ext_action": action_name}
    }
    try:
        req = urllib.request.Request(url, data=json.dumps(payload).encode("utf-8"), headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except Exception as e:
        print(f"[arm_teleop_rx] RPC SetAction '{action_name}' warning: {e}")
        return None
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
except ImportError:
    rclpy, Node, JointState = None, object, None

ARM_NAMES_LONG = [
    "idx13_left_arm_joint1", "idx14_left_arm_joint2", "idx15_left_arm_joint3",
    "idx16_left_arm_joint4", "idx17_left_arm_joint5", "idx18_left_arm_joint6",
    "idx19_left_arm_joint7", "idx20_right_arm_joint1", "idx21_right_arm_joint2",
    "idx22_right_arm_joint3", "idx23_right_arm_joint4", "idx24_right_arm_joint5",
    "idx25_right_arm_joint6", "idx26_right_arm_joint7"
]
LIMS_MIN = [-2.91, -0.46, -2.91, -2.00, -2.94, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.10, -2.94, -0.30, -0.20]
LIMS_MAX = [ 2.91,  1.5359, 2.91, -0.10,  2.94,  0.30,  0.20,  2.91,  0.46,   2.91, 2.00,  2.94,  0.30,  0.20]
HOME = [0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0]
for _i in range(14):
    assert LIMS_MIN[_i] <= HOME[_i] <= LIMS_MAX[_i], f"Joint {_i} HOME exceeds limits"

class ArmTeleopRx(Node):
    def __init__(self, mode_live=False, allow_ip="127.0.0.1", port=5005, tx_state_port=5006, max_step_rad=0.035, deadband_rad=0.003, max_acc=8.0, omega=16.0):
        super().__init__("arm_teleop_rx")
        self.is_live, self.allow_ip, self.tx_state_port = mode_live, allow_ip, tx_state_port
        self.max_step_rad, self.deadband_rad, self.max_acc, self.omega = max_step_rad, deadband_rad, max_acc, omega
        self.max_vel = self.max_step_rad / 0.02
        self.curr_cmd_v = [0.0] * 14
        self.token = os.environ.get("TELEOP_TOKEN", "")
        self.meas_q, self.meas_lock = None, threading.Lock()
        
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(JointState, "/motion/control/arm_joint_command", qos) if self.is_live else None
        self.state_sub = self.create_subscription(JointState, "/motion/control/arm_joint_state", self._on_state, qos)
        
        self.anchored = False
        self.homing_active = True
        self.homing_dropped_udp = 0
        self.target_q, self.curr_cmd_q, self.last_pkt_time, self.rx_count = None, None, 0.0, 0
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.1)
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.timer = self.create_timer(0.02, self._control_tick)
        print(f"[arm_teleop_rx] Ready at 50 Hz. LIVE={self.is_live} | allow_ip={self.allow_ip}:{port} | max_vel={self.max_vel:.2f} max_acc={self.max_acc:.1f} deadband={self.deadband_rad}")

    def _on_state(self, msg: JointState):
        name_map = {n: p for n, p in zip(msg.name, msg.position)}
        with self.meas_lock:
            if all(k in name_map for k in ARM_NAMES_LONG): self.meas_q = [name_map[k] for k in ARM_NAMES_LONG]
            elif any("idx13" in k for k in name_map): self.meas_q = [name_map[k] for k in sorted(name_map.keys()) if "idx" in k][:14]
            elif len(msg.position) >= 14: self.meas_q = list(msg.position[:14])
        if self.meas_q and self.allow_ip not in ("0.0.0.0", "127.0.0.1"):
            try:
                pkt = {"t": time.time(), "q": [float(round(x, 5)) for x in self.meas_q]}
                self.sock.sendto(json.dumps(pkt).encode("utf-8"), (self.allow_ip, self.tx_state_port))
            except Exception: pass

    def _rx_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
                if self.allow_ip != "0.0.0.0" and addr[0] != self.allow_ip:
                    print(f"[arm_teleop_rx] [CẢNH BÁO] Từ chối gói tin từ IP lạ: {addr[0]} (cần {self.allow_ip})")
                    continue
                if self.homing_active:
                    self.homing_dropped_udp += 1
                    if self.homing_dropped_udp % 50 == 1:
                        print(f"[arm_teleop_rx] Đang Homing, tạm bỏ qua gói tin từ {addr[0]} (đã bỏ {self.homing_dropped_udp})")
                    continue
                pkt = json.loads(data.decode("utf-8"))
                if self.token and pkt.get("token") != self.token:
                    print(f"[arm_teleop_rx] [LỖI TOKEN] Token không khớp! Nhận '{pkt.get('token')}' != '{self.token}'")
                    continue
                q = pkt.get("q", [])
                if len(q) != 14 or any(math.isnan(x) or math.isinf(x) for x in q):
                    print(f"[arm_teleop_rx] [LỖI GÓI] Góc khớp không hợp lệ: len={len(q)}")
                    continue
                self.target_q = [max(LIMS_MIN[i], min(LIMS_MAX[i], float(q[i]))) for i in range(14)]
                self.last_pkt_time = time.time()
                self.rx_count += 1
                if self.rx_count == 1:
                    print(f"\n>>> [arm_teleop_rx] ĐÃ NHẬN GÓI ĐẦU TIÊN TỪ {addr[0]}:{addr[1]}! <<<")
                if self.rx_count % 50 == 1:
                    print(f"[arm_teleop_rx] RX #{self.rx_count:05d} | OK từ {addr[0]} | Target J4={self.target_q[3]:.3f}, J11={self.target_q[10]:.3f}")
            except (socket.timeout, Exception): pass

    def _publish_raw_q(self, q):
        if self.is_live and self.cmd_pub:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name, msg.position = ARM_NAMES_LONG, list(q)
            msg.velocity, msg.effort = [0.0] * 14, [0.0] * 14
            self.cmd_pub.publish(msg)

    def _control_tick(self):
        if self.homing_active: return
        with self.meas_lock: meas = list(self.meas_q) if self.meas_q else None
        if not meas: return
        if not self.anchored: self.curr_cmd_q, self.curr_cmd_v = list(meas), [0.0]*14; self.anchored = True
        now = time.time()
        des_q = list(self.target_q) if (self.target_q and now - self.last_pkt_time <= 0.400) else list(self.curr_cmd_q)
        for i in range(14):
            err = des_q[i] - self.curr_cmd_q[i]
            if abs(err) < self.deadband_rad: err = 0.0
            a = max(-self.max_acc, min(self.max_acc, (self.omega ** 2) * err - 2.0 * self.omega * self.curr_cmd_v[i]))
            self.curr_cmd_v[i] = max(-self.max_vel, min(self.max_vel, self.curr_cmd_v[i] + a * 0.02))
            if err == 0.0 and abs(self.curr_cmd_v[i]) < 0.02: self.curr_cmd_v[i] = 0.0
            self.curr_cmd_q[i] = max(LIMS_MIN[i], min(LIMS_MAX[i], self.curr_cmd_q[i] + self.curr_cmd_v[i] * 0.02))
        self._publish_raw_q(self.curr_cmd_q)

    def auto_home(self, prompt_fn=None):
        """Automatically moves robot arms from current pose to HOME smoothly."""
        t0 = time.time()
        while time.time() - t0 < 3.5:
            with self.meas_lock:
                if self.meas_q is not None: break
            time.sleep(0.02)
        with self.meas_lock: meas = list(self.meas_q) if self.meas_q else None
        if not meas or len(meas) != 14 or any(math.isnan(x) or math.isinf(x) for x in meas):
            print("FATAL: State not alive within 3.5 s. Refusing homing."); return False

        deltas = [HOME[i] - meas[i] for i in range(14)]
        max_d = max(abs(d) for d in deltas)
        dur = max(2.5, max_d / 0.15) if max_d > 1e-4 else 0.5
        print(f"[HOMING] Per-joint deltas: {[round(d, 4) for d in deltas]}")
        print(f"[HOMING] max|delta| = {max_d:.4f} rad | Est duration = {dur:.2f} s")
        if max_d > 1.0:
            print(f"FATAL: max|delta| ({max_d:.3f} rad) > 1.0 rad. Move arms manually closer to home."); return False

        if not self.is_live:
            print("[HOMING] --dry mode: Plan computed successfully. 0 commands published.")
            self.homing_active = False; return True

        print(f"[HOMING] Auto-homing starting ({dur:.1f}s move to HOME)...")

        self.curr_cmd_q = list(meas)
        self._publish_raw_q(self.curr_cmd_q)
        t_start, fault_cnt = time.time(), 0
        while True:
            now = time.time()
            tau = min(1.0, (now - t_start) / dur) if dur > 0 else 1.0
            s = tau * tau * (3.0 - 2.0 * tau)  # Smoothstep
            target_step = [meas[i] + deltas[i] * s for i in range(14)]
            self.curr_cmd_q = [max(LIMS_MIN[i], min(LIMS_MAX[i], target_step[i])) for i in range(14)]
            self._publish_raw_q(self.curr_cmd_q)
            with self.meas_lock: curr_meas = list(self.meas_q) if self.meas_q else None
            if curr_meas and max(abs(curr_meas[i] - self.curr_cmd_q[i]) for i in range(14)) > 0.05:
                fault_cnt += 1
                if fault_cnt >= 10:
                    print("FATAL FAULT: Joint deviation > 0.05 rad for >200 ms!"); return False
            else: fault_cnt = 0
            if tau >= 1.0: break
            time.sleep(0.02)

        t_settle = time.time()
        while time.time() - t_settle < 0.5:
            self._publish_raw_q(HOME)
            with self.meas_lock: curr_meas = list(self.meas_q) if self.meas_q else None
            if curr_meas and max(abs(curr_meas[i] - HOME[i]) for i in range(14)) >= 0.02: t_settle = time.time()
            time.sleep(0.02)

        self.curr_cmd_q = list(HOME)
        self.anchored = True
        self.homing_active = False
        print(f"[HOMING] Dropped {self.homing_dropped_udp} UDP packets during homing.\nHOMED")
        return True

    def shutdown_hold(self):
        self.running = False
        if self.is_live and self.cmd_pub and self.curr_cmd_q and rclpy and rclpy.ok():
            t_end = time.time() + 2.0
            while time.time() < t_end and rclpy.ok():
                try: self._publish_raw_q(self.curr_cmd_q)
                except Exception: break
                time.sleep(0.02)

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dry", action="store_true", default=False)
    p.add_argument("--live", action="store_true")
    p.add_argument("--i-understand-hardware-risk", action="store_true")
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--tx-state-port", type=int, default=5006)
    p.add_argument("--allow-ip", type=str, default="127.0.0.1")
    p.add_argument("--max-step", type=float, default=0.035)
    p.add_argument("--deadband", type=float, default=0.003)
    p.add_argument("--max-acc", type=float, default=8.0)
    p.add_argument("--omega", type=float, default=16.0)
    args, _ = p.parse_known_args()

    live_active = False
    if args.live:
        token = os.environ.get("TELEOP_TOKEN", "")
        if not (args.i_understand_hardware_risk and args.allow_ip and token):
            print("FATAL: --live requires 5 gates!"); sys.exit(1)
        live_active = True
        print("[arm_teleop_rx] Tự động cấu hình SetAction -> 10 (Stand) -> 12 (Ext Servo)...")
        rpc_set_action("McAction_RL_JOINT_DEFAULT")
        time.sleep(1.0)
        rpc_set_action("McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO")
        time.sleep(1.0)
        
    rclpy.init()
    node = ArmTeleopRx(mode_live=live_active, allow_ip=args.allow_ip, port=args.port, tx_state_port=args.tx_state_port, max_step_rad=args.max_step, deadband_rad=args.deadband, max_acc=args.max_acc, omega=args.omega)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if not node.auto_home():
        node.shutdown_hold()
        node.running = False
        try: node.destroy_node()
        except Exception: pass
        sys.exit(1)

    try:
        while rclpy.ok(): time.sleep(0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.shutdown_hold()
        node.running = False
        try: node.destroy_node()
        except Exception: pass

if __name__ == "__main__":
    main()
