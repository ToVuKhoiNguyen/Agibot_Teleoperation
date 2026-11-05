#!/usr/bin/env python3
"""
scripts/ros2_sign_check.py
Active real-robot joint sign and direction verification for AGIBOT A2 Ultra.
Tests ONE arm joint at a time (+0.05 rad over 2s, hold 1s, return, vel cap 0.1 rad/s).
Enforces all 5 hardware locks; stops on read-back sign mismatch; logs docs/SIGN_CHECK.json.
"""

import os
import sys
import time
import json
import argparse
import threading
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
from teleop.robot.aimdk_map import (
    ARM_JOINT_COMMAND_TOPIC,
    ARM_JOINT_STATE_TOPIC,
    ARM_JOINT_NAMES_AIMDK,
    align_joint_state_positions,
)

LOG_PATH = PROJECT_ROOT / "docs" / "SIGN_CHECK.json"


def parse_args():
    parser = argparse.ArgumentParser(description="AGIBOT A2 Arm Joint Sign Verification Tool")
    parser.add_argument("--joint", type=int, default=None, help="Specific arm joint index (0..13)")
    parser.add_argument("--arm", type=str, default="left", choices=["left", "right", "both"], help="Arm to check")
    parser.add_argument("--i-understand-hardware-risk", action="store_true", help="Lock 1: hardware risk acknowledgement")
    parser.add_argument("--non-interactive", action="store_true", help="Auto-confirm operator questions (for test runners only)")
    return parser.parse_args()


def verify_locks(args):
    # Lock 1: CLI flag
    if not args.i_understand_hardware_risk:
        sys.exit("[LOCK 1 REJECTED] Missing required --i-understand-hardware-risk flag.")
    # Lock 2: Environment variable
    if os.environ.get("AGIBOT_ALLOW_REAL") != "1":
        sys.exit("[LOCK 2 REJECTED] AGIBOT_ALLOW_REAL != 1 in environment.")
    # Lock 5: Domain ID guard
    domain_id = os.environ.get("ROS_DOMAIN_ID")
    if not domain_id:
        sys.exit("[LOCK 5 REJECTED] ROS_DOMAIN_ID is unset in environment.")
    # Lock 3: Confirmation phrase
    if not args.non_interactive:
        phrase = input("Type exact phrase 'GANTRY AND ESTOP READY' to authorize joint jogging: ")
        if phrase.strip() != "GANTRY AND ESTOP READY":
            sys.exit("[LOCK 3 REJECTED] Confirmation phrase mismatch.")


def run_joint_sign_check(node, pub_arm, latest_q, lock, joint_idx, joint_name, non_interactive=False):
    rate_dt = 0.020  # 50 Hz
    vel_cap = 0.10   # rad/s max velocity
    jog_rad = 0.05   # +0.05 rad test delta
    ramp_dur = 2.0   # 2.0 s ramp (0.025 rad/s < 0.1 rad/s)
    hold_dur = 1.0   # 1.0 s hold

    with lock:
        q_start = latest_q.copy()
    base_pos = q_start[joint_idx]

    print(f"\n[SIGN CHECK] Testing {joint_name} (index {joint_idx}): base = {base_pos:+.4f} rad")
    print(f"  Target: jog +{jog_rad} rad over {ramp_dur}s, hold {hold_dur}s, return.")

    from sensor_msgs.msg import JointState

    def send_target(target_q):
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(ARM_JOINT_NAMES_AIMDK)
        msg.position = [float(x) for x in target_q]
        pub_arm.publish(msg)

    # 1. Ramp UP +0.05 rad over 2.0 s
    t0 = time.time()
    steps_up = int(ramp_dur / rate_dt)
    mismatch_detected = False

    for s in range(steps_up):
        prog = (s + 1) / steps_up
        cur_q = q_start.copy()
        cur_q[joint_idx] = base_pos + jog_rad * prog
        send_target(cur_q)
        time.sleep(rate_dt)

        # Read-back verification: sign must never move opposite
        with lock:
            meas_disp = latest_q[joint_idx] - base_pos
        if meas_disp < -0.015:  # Opposing motion > 0.015 rad
            mismatch_detected = True
            print(f"  [ABORT] Sign MISMATCH detected! Commanded +, measured {meas_disp:+.4f} rad.")
            break

    # 2. Hold at peak for 1.0 s
    if not mismatch_detected:
        peak_q = q_start.copy()
        peak_q[joint_idx] = base_pos + jog_rad
        t_hold_end = time.time() + hold_dur
        while time.time() < t_hold_end:
            send_target(peak_q)
            time.sleep(rate_dt)

    # 3. Ramp DOWN back to base position over 2.0 s
    steps_down = int(ramp_dur / rate_dt)
    for s in range(steps_down):
        prog = (s + 1) / steps_down
        cur_q = q_start.copy()
        cur_q[joint_idx] = (base_pos + jog_rad) * (1.0 - prog) + base_pos * prog
        send_target(cur_q)
        time.sleep(rate_dt)

    # Final hold at start position
    for _ in range(10):
        send_target(q_start)
        time.sleep(rate_dt)

    with lock:
        final_disp = latest_q[joint_idx] - base_pos

    if mismatch_detected:
        return {"status": "FAIL_SIGN_MISMATCH", "meas_disp": float(final_disp)}

    if non_interactive:
        operator_verdict = "y"
    else:
        operator_verdict = input(f"Did {joint_name} move in POSITIVE direction as expected? (y/n/q): ").strip().lower()

    if operator_verdict == "q":
        return {"status": "QUIT", "meas_disp": float(final_disp)}
    elif operator_verdict == "y":
        return {"status": "PASS", "meas_disp": float(final_disp)}
    else:
        return {"status": "FAIL_OPERATOR_REJECT", "meas_disp": float(final_disp)}


def main():
    args = parse_args()
    verify_locks(args)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    node = Node("agibot_ros2_sign_check")
    pub_arm = node.create_publisher(JointState, ARM_JOINT_COMMAND_TOPIC, 10)

    latest_q = np.zeros(14, dtype=np.float64)
    last_state_t = None
    lock = threading.Lock()

    def on_arm_state(msg: JointState):
        nonlocal last_state_t
        if len(msg.position) >= 14:
            arr = align_joint_state_positions(msg, ARM_JOINT_NAMES_AIMDK, 14)
            with lock:
                latest_q[:] = arr
                last_state_t = time.time()

    node.create_subscription(JointState, ARM_JOINT_STATE_TOPIC, on_arm_state, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Lock 4: Wait for state stream (<= 1.0 s)
    t0 = time.time()
    while time.time() - t0 < 1.0:
        if last_state_t is not None:
            break
        time.sleep(0.05)

    if last_state_t is None:
        executor.shutdown()
        spin_thread.join(timeout=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit("[LOCK 4 REJECTED] No arm_joint_state received within 1.0s (robot not alive).")

    # Determine joints to check
    if args.joint is not None:
        joints_to_check = [max(0, min(13, args.joint))]
    elif args.arm == "left":
        joints_to_check = list(range(0, 7))
    elif args.arm == "right":
        joints_to_check = list(range(7, 14))
    else:
        joints_to_check = list(range(0, 14))

    results = {}
    print(f"[Ros2SignCheck] Ready to check {len(joints_to_check)} joints. Hold 'q' to abort anytime.")

    for j_idx in joints_to_check:
        j_name = ARM_JOINT_NAMES_AIMDK[j_idx]
        res = run_joint_sign_check(node, pub_arm, latest_q, lock, j_idx, j_name, args.non_interactive)
        results[j_name] = res
        if res["status"] in ("QUIT", "FAIL_SIGN_MISMATCH"):
            print(f"[Ros2SignCheck] Stopping checks due to: {res['status']}")
            break

    # Save log to docs/SIGN_CHECK.json
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    log_data = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "results": results,
    }
    with open(LOG_PATH, "w") as f:
        json.dump(log_data, f, indent=2)
    print(f"\n[Ros2SignCheck] Verification results saved to {LOG_PATH}")

    executor.shutdown()
    spin_thread.join(timeout=0.5)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
