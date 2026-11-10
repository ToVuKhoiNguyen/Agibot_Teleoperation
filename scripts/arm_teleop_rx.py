#!/usr/bin/env python3
"""scripts/arm_teleop_rx.py
AGIBOT A2 Ultra - Pre-Live Audited 50 Hz Arm Teleoperation Receiver for Orin.
Guarantees single publish site, uniform vector scaling, accel cap, socket drain, and version handshake.
"""
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
LIMS_MIN = [-2.91, -0.46, -2.91, -2.00, -2.94, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.10, -2.94, -0.30, -0.20]
LIMS_MAX = [ 2.91,  1.5359, 2.91, -0.10,  2.94,  0.30,  0.20,  2.91,  0.46,   2.91, 2.00,  2.94,  0.30,  0.20]
HOME = [0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0]
PROFILES = {"slow": (0.2, 0.004), "normal": (0.3, 0.006), "fast": (0.5, 0.010)}
PROTO_VER = 2

def rpc_set_action(action_name, host="192.168.100.100", port=56322):
    url = f"http://{host}:{port}/rpc/aimdk.protocol.McActionService/SetAction"
    p = {"header": {"timestamp": {"seconds": int(time.time()), "nanos": 0}, "control_source": "ControlSource_SAFE"},
         "command": {"action": "McAction_USE_EXT_CMD", "ext_action": action_name}}
    try:
        r = urllib.request.Request(url, data=json.dumps(p).encode(), headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(r, timeout=1.5) as resp: return json.loads(resp.read().decode())
    except Exception: return None

class ArmTeleopRx(Node):
    def __init__(self, mode_live=False, allow_ip="127.0.0.1", port=5005, tx_state_port=5006,
                 speed_profile="normal", max_step_rad=None, deadband_rad=0.003, max_acc=1.40,
                 omega=12.0, accept_higher_speed=False):
        super().__init__("arm_teleop_rx")
        self.is_live, self.allow_ip, self.tx_state_port = mode_live, allow_ip, tx_state_port
        self.deadband_rad, self.max_acc, self.omega = deadband_rad, max_acc, omega
        self.profile = speed_profile if speed_profile in PROFILES else "normal"
        base_vel, _ = PROFILES[self.profile]
        
        cand_vel = (max_step_rad / 0.02) if max_step_rad is not None else base_vel
        self.max_vel = min(1.0 if accept_higher_speed else 0.5, cand_vel)
        self.max_step_rad = self.max_vel * 0.02
        
        self.curr_cmd_q = list(HOME)
        self.curr_cmd_v, self.curr_cmd_a = np.zeros(14), np.zeros(14)
        self.target_q = list(HOME)
        self.token = os.environ.get("TELEOP_TOKEN", "")
        self.session_lock, self.foreign_sender_count = None, 0
        self.last_pkt_time, self.rx_count = 0.0, 0
        self.last_tick_t = time.monotonic()
        self.fault_latched, self.holding, self.anchored = False, False, False
        self.fault_reason = ""
        
        with open(__file__, "rb") as f: self.file_sha = hashlib.sha256(f.read()).hexdigest()
        self.sender_build_id = "unknown"
        self.meas_q = None
        
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.cmd_pub = self.create_publisher(JointState, "/motion/control/arm_joint_command", qos) if self.is_live else None
        self.state_sub = self.create_subscription(JointState, "/motion/control/arm_joint_state", self._on_state, qos)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.running = True
        
        self.homing_active, self.homing_start_t, self.homing_dur = True, 0.0, 3.0
        self.homing_start_q = None
        self.timer = self.create_timer(0.02, self._control_tick)
        print(f"[arm_teleop_rx] SHA256: {self.file_sha}")
        print(f"[arm_teleop_rx] Ready 50 Hz. LIVE={self.is_live} | profile={self.profile} (vel={self.max_vel:.2f} rad/s) | IP={allow_ip}:{port}")

    def _on_state(self, msg: JointState):
        name_map = {n: p for n, p in zip(msg.name, msg.position)}
        if all(k in name_map for k in ARM_NAMES_LONG): self.meas_q = [name_map[k] for k in ARM_NAMES_LONG]
        elif any("idx13" in k for k in name_map): self.meas_q = [name_map[k] for k in sorted(name_map.keys()) if "idx" in k][:14]
        elif len(msg.position) >= 14: self.meas_q = list(msg.position[:14])
        if self.meas_q and self.allow_ip not in ("0.0.0.0", "127.0.0.1"):
            try:
                pkt = {"t": time.time(), "q": [float(round(x, 5)) for x in self.meas_q],
                       "rx_sha": self.file_sha[:12], "echo_build": self.sender_build_id, "profile": self.profile}
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
            if self.token and p.get("token") != self.token: continue
            q = p.get("q", [])
            if len(q) != 14 or any(math.isnan(x) or math.isinf(x) for x in q): continue
            
            sess_id = p.get("session_id", "default")
            sender_id = (addr[0], sess_id)
            if self.session_lock is None: self.session_lock = sender_id
            elif self.session_lock != sender_id:
                self.foreign_sender_count += 1; self.holding = True; continue
                
            self.sender_build_id = str(p.get("build_id", "unknown"))
            newest_pkt = p
            
        if newest_pkt is not None:
            self.target_q = [float(np.clip(newest_pkt["q"][i], LIMS_MIN[i], LIMS_MAX[i])) for i in range(14)]
            self.last_pkt_time = time.monotonic()
            self.rx_count += 1

    def _final_line_guard_and_publish(self, v_target_req, dt):
        """EXACTLY ONE PUBLISH SITE: applies uniform scaling, accel cap, and fault latching."""
        if self.fault_latched or self.holding:
            v_target = np.zeros(14)
        else:
            max_req_v = float(np.max(np.abs(v_target_req)))
            if max_req_v * dt > 0.020001:
                self.fault_latched = True
                self.fault_reason = f"Step {max_req_v*dt:.4f} > 0.02 rad"
                v_target = np.zeros(14)
            else:
                scale_v = min(1.0, self.max_vel / max_req_v) if max_req_v > 1e-6 else 1.0
                v_target = v_target_req * scale_v
        
        # Accel cap with uniform scaling (max_acc <= 1.5 rad/s^2)
        dv = v_target - self.curr_cmd_v
        max_dv = float(np.max(np.abs(dv)))
        if max_dv > self.max_acc * dt:
            dv = dv * ((self.max_acc * dt) / max_dv)
        self.curr_cmd_v = self.curr_cmd_v + dv
        
        max_v = float(np.max(np.abs(self.curr_cmd_v)))
        if max_v > self.max_vel:
            self.curr_cmd_v = self.curr_cmd_v * (self.max_vel / max_v)
            
        next_cmd_q = np.array(self.curr_cmd_q) + self.curr_cmd_v * dt
        if np.any(np.isnan(next_cmd_q)) or np.any(np.isinf(next_cmd_q)):
            self.fault_latched = True
            next_cmd_q = np.array(self.curr_cmd_q)
            self.curr_cmd_v = np.zeros(14)
            
        self.curr_cmd_q = [float(np.clip(next_cmd_q[i], LIMS_MIN[i], LIMS_MAX[i])) for i in range(14)]
        
        if self.is_live and self.cmd_pub:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ARM_NAMES_LONG
            msg.position = list(self.curr_cmd_q)
            msg.velocity = list(self.curr_cmd_v)
            self.cmd_pub.publish(msg)

    def _control_tick(self):
        now = time.monotonic()
        dt = min(0.040, max(0.010, now - self.last_tick_t))
        self.last_tick_t = now
        
        self._drain_socket()
        meas = list(self.meas_q) if self.meas_q else None
        
        if self.homing_active:
            if not meas: return
            if self.homing_start_q is None:
                self.homing_start_q, self.curr_cmd_q = list(meas), list(meas)
                self.homing_start_t = now
                deltas = [HOME[i] - meas[i] for i in range(14)]
                max_d = max(abs(d) for d in deltas)
                self.homing_dur = max(2.5, max_d / (self.max_vel * 0.8))
                print(f"[HOMING] Auto-homing starting ({self.homing_dur:.1f}s move to HOME)...")
            
            tau = min(1.0, (now - self.homing_start_t) / self.homing_dur)
            s = tau * tau * (3.0 - 2.0 * tau)
            tgt_step = np.array([self.homing_start_q[i] + (HOME[i] - self.homing_start_q[i]) * s for i in range(14)])
            v_homing = (tgt_step - np.array(self.curr_cmd_q)) / dt
            self._final_line_guard_and_publish(v_homing, dt)
            
            if tau >= 1.0 and np.max(np.abs(np.array(self.curr_cmd_q) - np.array(HOME))) < 0.005:
                self.homing_active, self.anchored = False, True
                self.curr_cmd_v = np.zeros(14)
                print("HOMED")
            return
            
        if not self.anchored:
            if meas: self.curr_cmd_q = list(meas); self.anchored = True
            return
            
        is_fresh = (now - self.last_pkt_time <= 0.400) and not self.holding
        target = np.array(self.target_q) if is_fresh else np.array(self.curr_cmd_q)
        
        err = target - np.array(self.curr_cmd_q)
        D = float(np.max(np.abs(err)))
        if D < self.deadband_rad:
            v_req = np.zeros(14)
        else:
            v_stop = -0.5 * self.max_acc * dt + math.sqrt(max(0.0, (0.5 * self.max_acc * dt)**2 + 2.0 * self.max_acc * D))
            V = min(self.max_vel, v_stop, D / dt)
            v_req = err * (V / D)
        self._final_line_guard_and_publish(v_req, dt)

    def shutdown_hold(self):
        self.holding = True
        self.running = False

def main():
    p = argparse.ArgumentParser()
    for a in ["--dry", "--live", "--i-understand-hardware-risk", "--i-accept-higher-speed"]: p.add_argument(a, action="store_true")
    p.add_argument("--speed-profile", type=str, default="normal", choices=["slow", "normal", "fast"])
    p.add_argument("--port", type=int, default=5005)
    p.add_argument("--tx-state-port", type=int, default=5006)
    p.add_argument("--allow-ip", type=str, default="127.0.0.1")
    p.add_argument("--max-step", type=float, default=None)
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
    node = ArmTeleopRx(
        mode_live=live_active, allow_ip=args.allow_ip, port=args.port, tx_state_port=args.tx_state_port,
        speed_profile=args.speed_profile, max_step_rad=args.max_step, accept_higher_speed=args.i_accept_higher_speed
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception): pass
    finally:
        node.shutdown_hold()
        try: node.destroy_node()
        except Exception: pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()
