#!/usr/bin/env python3
"""Agibot A2 Ultra - Safe Arm Teleoperation UDP Receiver Node for Orin (Precision Servo Interpolator)"""
import os, sys, time, json, math, socket, threading, argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from arm_homing import HOME, check_home_limits, execute_homing

ARM_NAMES_LONG = [
    "idx13_left_arm_joint1", "idx14_left_arm_joint2", "idx15_left_arm_joint3",
    "idx16_left_arm_joint4", "idx17_left_arm_joint5", "idx18_left_arm_joint6",
    "idx19_left_arm_joint7", "idx20_right_arm_joint1", "idx21_right_arm_joint2",
    "idx22_right_arm_joint3", "idx23_right_arm_joint4", "idx24_right_arm_joint5",
    "idx25_right_arm_joint6", "idx26_right_arm_joint7"
]
LIMS_MIN = [-2.91, -0.46, -2.91, -2.00, -2.94, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.10, -2.94, -0.30, -0.20]
LIMS_MAX = [ 2.91,  1.5359, 2.91, -0.10,  2.94,  0.30,  0.20,  2.91,  0.46,   2.91, 2.00,  2.94,  0.30,  0.20]
check_home_limits(LIMS_MIN, LIMS_MAX)

class ArmTeleopRx(Node):
    def __init__(self, mode_live=False, allow_ip="127.0.0.1", port=5005, tx_state_port=5006, max_step_rad=0.01):
        super().__init__("arm_teleop_rx")
        self.is_live = mode_live
        self.allow_ip = allow_ip
        self.tx_state_port = tx_state_port
        self.max_step_rad = max_step_rad
        self.token = os.environ.get("TELEOP_TOKEN", "")
        self.lim_min = LIMS_MIN
        self.lim_max = LIMS_MAX
        self.meas_q = None
        self.meas_lock = threading.Lock()
        
        qos_best_effort = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(JointState, "/motion/control/arm_joint_command", qos_best_effort) if self.is_live else None
        self.state_sub = self.create_subscription(JointState, "/motion/control/arm_joint_state", self._on_state, qos_best_effort)
        
        self.anchored = False
        self.homing_active = True
        self.homing_dropped_udp = 0
        self.target_q = None
        self.curr_cmd_q = None
        self.last_pkt_time = 0.0
        self.rx_count = 0
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.1)
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.timer = self.create_timer(0.02, self._control_tick)
        print(f"[arm_teleop_rx] Ready at 50 Hz. LIVE={self.is_live} | allow_ip={self.allow_ip}:{port}")

    def _on_state(self, msg: JointState):
        name_map = {n: p for n, p in zip(msg.name, msg.position)}
        with self.meas_lock:
            if all(k in name_map for k in ARM_NAMES_LONG):
                self.meas_q = [name_map[k] for k in ARM_NAMES_LONG]
            elif any("idx13" in k for k in name_map):
                self.meas_q = [name_map[k] for k in sorted(name_map.keys()) if "idx" in k][:14]
            elif len(msg.position) >= 14:
                self.meas_q = list(msg.position[:14])
        if self.meas_q and self.allow_ip not in ("0.0.0.0", "127.0.0.1"):
            try:
                pkt = {"t": time.time(), "q": [float(round(x, 5)) for x in self.meas_q]}
                self.sock.sendto(json.dumps(pkt).encode("utf-8"), (self.allow_ip, self.tx_state_port))
            except Exception: pass

    def _rx_loop(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(2048)
                if self.allow_ip != "0.0.0.0" and addr[0] != self.allow_ip: continue
                if self.homing_active:
                    self.homing_dropped_udp += 1
                    continue
                pkt = json.loads(data.decode("utf-8"))
                if self.token and pkt.get("token") != self.token: continue
                q = pkt.get("q", [])
                if len(q) != 14 or any(math.isnan(x) or math.isinf(x) for x in q): continue
                self.target_q = [max(LIMS_MIN[i], min(LIMS_MAX[i], float(q[i]))) for i in range(14)]
                self.last_pkt_time = time.time()
                self.rx_count += 1
                if self.rx_count % 50 == 1:
                    print(f"[arm_teleop_rx] RX #{self.rx_count} | Target J4={self.target_q[3]:.3f}, J11={self.target_q[10]:.3f}")
            except socket.timeout: pass
            except Exception: pass

    def _publish_raw_q(self, q):
        if self.is_live and self.cmd_pub:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ARM_NAMES_LONG
            msg.position = list(q)
            msg.velocity = [0.0] * 14
            msg.effort = [0.0] * 14
            self.cmd_pub.publish(msg)

    def _control_tick(self):
        if self.homing_active: return
        with self.meas_lock:
            meas = list(self.meas_q) if self.meas_q else None
        if not meas: return
        if not self.anchored:
            self.curr_cmd_q = list(meas)
            self.anchored = True
        now = time.time()
        des_q = list(self.target_q) if (self.target_q and now - self.last_pkt_time <= 0.400) else list(self.curr_cmd_q)
        diffs = [des_q[i] - self.curr_cmd_q[i] for i in range(14)]
        max_d = max(abs(d) for d in diffs)
        scale = min(1.0, self.max_step_rad / max_d) if max_d > 1e-6 else 1.0
        self.curr_cmd_q = [max(LIMS_MIN[i], min(LIMS_MAX[i], self.curr_cmd_q[i] + diffs[i] * scale)) for i in range(14)]
        self._publish_raw_q(self.curr_cmd_q)

    def shutdown_hold(self):
        self.running = False
        if self.is_live and self.cmd_pub and self.curr_cmd_q and rclpy.ok():
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
    p.add_argument("--max-step", type=float, default=0.01)
    args, _ = p.parse_known_args()

    live_active = False
    if args.live:
        token = os.environ.get("TELEOP_TOKEN", "")
        if not (args.i_understand_hardware_risk and args.allow_ip and token):
            print("FATAL: --live requires 5 gates!")
            sys.exit(1)
        live_active = True
        
    rclpy.init()
    node = ArmTeleopRx(mode_live=live_active, allow_ip=args.allow_ip, port=args.port, tx_state_port=args.tx_state_port, max_step_rad=args.max_step)
    
    # Run spin in background thread for state_sub callbacks
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    homed = execute_homing(node)
    if not homed:
        node.shutdown_hold()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        sys.exit(1)

    try:
        while rclpy.ok(): time.sleep(0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    finally:
        node.shutdown_hold()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()
