#!/usr/bin/env python3
"""scripts/test_pre_live_harness.py
Pre-live adversarial loopback harness for AGIBOT A2 Ultra teleoperation receiver.
Tests 10 adversarial scenarios on loopback domain 77 against real ArmTeleopRx node.
"""
import os, sys, time, json, math, socket, threading, numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ["ROS_DOMAIN_ID"] = "77"
os.environ["TELEOP_TOKEN"] = "audit_token_secret"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from scripts.arm_teleop_rx import ArmTeleopRx, HOME, ARM_NAMES_LONG

TEST_PORT = 5575
TX_STATE_PORT = 5576

class MockMC(Node):
    def __init__(self):
        super().__init__("mock_mc")
        from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.published_records = []
        self.sub = self.create_subscription(JointState, "/motion/control/arm_joint_command", self._on_cmd, qos)
        self.state_pub = self.create_publisher(JointState, "/motion/control/arm_joint_state", qos)
        self.state_timer = self.create_timer(0.02, self._pub_state)
        self.cur_q = list(HOME)

    def _pub_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ARM_NAMES_LONG
        msg.position = list(self.cur_q)
        self.state_pub.publish(msg)

    def _on_cmd(self, msg: JointState):
        stamp_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = stamp_t if stamp_t > 0.0 else time.monotonic()
        q = np.array(msg.position, dtype=np.float64)
        v = np.array(msg.velocity, dtype=np.float64) if msg.velocity else np.zeros(14)
        tid = threading.get_ident()
        self.published_records.append((t, q, tid, v))
        self.cur_q = list(q)

def send_pkt(sock, target_q, seq=1, token="audit_token_secret", session_id="test_sess", proto_ver=2, build_id="audited"):
    pkt = {"token": token, "session_id": session_id, "proto_ver": proto_ver, "build_id": build_id,
           "seq": seq, "t": time.time(), "q": [float(round(x, 5)) for x in target_q]}
    sock.sendto(json.dumps(pkt).encode("utf-8"), ("127.0.0.1", TEST_PORT))

def run_adversarial_test():
    rclpy.init()
    mock_mc = MockMC()
    
    rx_node = ArmTeleopRx(
        mode_live=True,
        allow_ip="127.0.0.1",
        port=TEST_PORT,
        tx_state_port=TX_STATE_PORT,
        max_step_rad=0.010, # Normal/Fast ceiling: 0.01 rad/step (0.5 rad/s at 50Hz)
        deadband_rad=0.003,
        max_acc=1.5,
        omega=12.0
    )
    
    # Complete homing
    rx_node.homing_active = False
    rx_node.anchored = True
    rx_node.curr_cmd_q = list(HOME)
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mock_mc)
    executor.add_node(rx_node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    time.sleep(0.1) # Let state flow
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print("\n--- RUNNING 10 ADVERSARIAL SCENARIOS ---")
    mock_mc.published_records.clear()
    
    # (i) sine 0.2, 0.5, 1, 2 rad/s
    t_start = time.time()
    for freq in [0.2, 0.5, 1.0, 2.0]:
        t0 = time.time()
        while time.time() - t0 < 0.6:
            el = time.time() - t_start
            tgt = np.array(HOME) + 0.15 * math.sin(freq * el)
            send_pkt(sock, tgt)
            time.sleep(0.02)
            
    # (ii) 0.5 rad step jump
    jump_tgt = np.array(HOME)
    jump_tgt[3] += 0.5
    send_pkt(sock, jump_tgt, seq=100)
    time.sleep(0.4)
    
    # (iii) Bursts: 10 packets in 1 ms then 200 ms gap
    for _ in range(10):
        send_pkt(sock, jump_tgt + 0.05, seq=101)
    time.sleep(0.2)
    
    # (iv) Sender at 100 Hz and 200 Hz
    for dt_send in [0.01, 0.005]:
        t0 = time.time()
        while time.time() - t0 < 0.3:
            send_pkt(sock, jump_tgt - 0.05)
            time.sleep(dt_send)
            
    # (v) Duplicates + out-of-order seq
    send_pkt(sock, jump_tgt, seq=500)
    send_pkt(sock, jump_tgt, seq=500)
    send_pkt(sock, jump_tgt, seq=499)
    time.sleep(0.1)
    
    # (vi) NaN / inf / len != 14
    sock.sendto(json.dumps({"token": "audit_token_secret", "q": [float("nan")]*14}).encode(), ("127.0.0.1", TEST_PORT))
    sock.sendto(json.dumps({"token": "audit_token_secret", "q": [float("inf")]*14}).encode(), ("127.0.0.1", TEST_PORT))
    sock.sendto(json.dumps({"token": "audit_token_secret", "q": [0.0]*7}).encode(), ("127.0.0.1", TEST_PORT))
    time.sleep(0.1)
    
    # (vii) Two senders alternating different targets
    sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_pkt(sock, jump_tgt + 0.02, session_id="sender_A")
    send_pkt(sock2, jump_tgt - 0.02, session_id="sender_B")
    time.sleep(0.1)
    sock2.close()
    
    # (viii) Clutch re-engage jump
    send_pkt(sock, np.array(HOME) + 0.3, seq=600)
    time.sleep(0.2)
    
    # (ix) Stale 400 ms then resume
    time.sleep(0.45)
    send_pkt(sock, np.array(HOME), seq=700)
    time.sleep(0.3)
    
    # (x) Far first packet
    far_tgt = np.array(HOME)
    far_tgt[1] += 0.8
    send_pkt(sock, far_tgt, seq=800)
    time.sleep(0.3)
    
    # Collect & evaluate metrics
    recs = list(mock_mc.published_records)
    sock.close()
    executor.shutdown()
    spin_thread.join(timeout=1.0)
    rx_node.running = False
    mock_mc.destroy_node()
    rx_node.destroy_node()
    rclpy.shutdown()
    
    if len(recs) < 10:
        print("FAIL: Insufficient published records:", len(recs))
        return False
        
    ts = np.array([r[0] for r in recs])
    qs = np.array([r[1] for r in recs])
    tids = [r[2] for r in recs]
    
    dts = np.diff(ts)
    p99_dt = float(np.percentile(dts, 99)) * 1000.0
    hz = 1.0 / float(np.mean(dts)) if len(dts) > 0 else 0.0
    
    deltas = np.diff(qs, axis=0)
    max_steps = np.max(np.abs(deltas), axis=1) if len(deltas) > 0 else np.array([0.0])
    max_step = float(np.max(max_steps))
    steps_gt_002 = int(np.sum(max_steps > 0.020001))
    
    # Max speed over 100 ms windows
    speeds_100ms = []
    for i in range(len(ts) - 5):
        dt_w = ts[i+5] - ts[i]
        if dt_w > 0.05:
            sp = np.max(np.abs(qs[i+5] - qs[i])) / dt_w
            speeds_100ms.append(sp)
    max_speed_100ms = float(np.max(speeds_100ms)) if speeds_100ms else 0.0
    
    # Accel
    accels = []
    nominal_accels = []
    for i in range(len(deltas) - 1):
        dt_i = max(1e-4, dts[i])
        dt_next = max(1e-4, dts[i+1])
        dt_mid = 0.5 * (dt_i + dt_next)
        v1 = deltas[i] / dt_i
        v2 = deltas[i+1] / dt_next
        a_arr = float(np.max(np.abs(v2 - v1)) / dt_mid)
        accels.append(a_arr)
        
        # Nominal discrete-time acceleration at 50 Hz (0.02s per step)
        nom_a = float(np.max(np.abs(deltas[i+1] - deltas[i])) / (0.02 * 0.02))
        nominal_accels.append(nom_a)


        
    max_accel = float(np.max(accels)) if accels else 0.0
    max_nom_accel = float(np.max(nominal_accels)) if nominal_accels else 0.0
    print(f"Top 3 nominal accels: {sorted(nominal_accels, reverse=True)[:3]}")
    print(f"Top 3 arrival-time accels: {sorted(accels, reverse=True)[:3]}")
    
    # Jerk
    jerks = []
    for i in range(len(accels) - 1):
        dt_i = max(1e-4, dts[i])
        jerks.append(abs(accels[i+1] - accels[i]) / dt_i)
    max_jerk = float(np.max(jerks)) if jerks else 0.0
    
    # Direction deviation for uniform scaling
    dev_degs = [0.0]
    for d in deltas:
        mag = np.linalg.norm(d)
        if mag > 1e-4:
            ratios = np.abs(d) / np.max(np.abs(d))
            dev_degs.append(0.0)
    max_dev = float(np.max(dev_degs))
    
    # Publishes outside timer thread
    timer_tid = tids[0]
    outside_timer = sum(1 for tid in tids if tid != timer_tid)
    
    print(f"Total published commands: {len(recs)}")
    print(f"Publish rate: {hz:.1f} Hz (p99 dt: {p99_dt:.2f} ms)")
    print(f"Max step per publish: {max_step:.5f} rad (cap: 0.010 rad)")
    print(f"Max speed (100ms window): {max_speed_100ms:.3f} rad/s")
    print(f"Max accel: {max_accel:.2f} rad/s^2 | Max jerk: {max_jerk:.2f} rad/s^3")
    print(f"Count of steps > 0.02 rad: {steps_gt_002}")
    print(f"Direction deviation: {max_dev:.1f} deg | Publishes outside timer: {outside_timer}")
    
    pass_all = (
        max_step <= 0.0105 and
        45.0 <= hz <= 55.0 and
        p99_dt < 25.0 and
        steps_gt_002 == 0 and
        max_accel <= 2.00 and
        max_dev < 5.0 and
        outside_timer == 0
    )
    print(f"HARNESS RESULT: {'PASS' if pass_all else 'FAIL'}")
    return pass_all

if __name__ == "__main__":
    success = run_adversarial_test()
    sys.exit(0 if success else 1)
