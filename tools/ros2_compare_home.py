#!/usr/bin/env python3
"""
tools/ros2_compare_home.py
Compares live measured robot joint state against simulation 'home' keyframe posture.
Averages 2.0 s of telemetry; strictly READ-ONLY (no publishers created).
"""

import os
import sys
import time
import threading
from pathlib import Path

# Add project root to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from teleop.robot.aimdk_map import (
    ARM_JOINT_STATE_TOPIC,
    HAND_JOINT_STATE_TOPIC,
    ARM_JOINT_NAMES_AIMDK,
    HAND_JOINT_NAMES,
    align_joint_state_positions,
    HOME,
)

# Reference MuJoCo scene 'home' keyframe arm postures (rad)
SIM_HOME_ARM_Q14 = np.array(HOME, dtype=np.float64)

# Reference MuJoCo sim home hand postures (raw 0 = fully open)
SIM_HOME_HAND_RAW12 = np.zeros(12, dtype=np.float64)

ARM_TOLERANCE_RAD = 0.10   # Flag if |diff| > 0.1 rad
HAND_TOLERANCE_RAW = 150.0  # Flag if raw > 150


def main():
    domain_id = os.environ.get("ROS_DOMAIN_ID")
    if not domain_id:
        print("[FATAL] ROS_DOMAIN_ID environment variable is unset.")
        print("[FATAL] Refusing to run without an explicit domain ID (e.g. export ROS_DOMAIN_ID=77).")
        sys.exit(1)

    print(f"[Ros2CompareHome] Checking home posture on ROS_DOMAIN_ID={domain_id}...")
    if str(domain_id).strip() == "232":
        print("!" * 78)
        print("[WARNING] CAUTION: Connected to ROS_DOMAIN_ID=232 (Real Robot Production Domain)!")
        print("[WARNING] Listener mode is strictly READ-ONLY (zero publishers created).")
        print("!" * 78)

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from rclpy.executors import SingleThreadedExecutor
    except ImportError as e:
        print(f"[FATAL] ROS 2 (rclpy/sensor_msgs) import error: {e}")
        sys.exit(1)

    rclpy.init()
    node = Node("agibot_ros2_compare_home")

    arm_samples = []
    hand_samples = []
    lock = threading.Lock()

    def on_arm_state(msg: JointState):
        if len(msg.position) >= 14:
            arr = align_joint_state_positions(msg, ARM_JOINT_NAMES_AIMDK, 14)
            with lock:
                arm_samples.append(arr)

    def on_hand_state(msg: JointState):
        if len(msg.position) >= 12:
            arr = align_joint_state_positions(msg, HAND_JOINT_NAMES, 12)
            with lock:
                hand_samples.append(arr)

    node.create_subscription(JointState, ARM_JOINT_STATE_TOPIC, on_arm_state, 10)
    node.create_subscription(JointState, HAND_JOINT_STATE_TOPIC, on_hand_state, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("[Ros2CompareHome] Collecting 2.0 s of telemetry samples...")
    t_end = time.time() + 2.0
    while time.time() < t_end:
        time.sleep(0.05)

    executor.shutdown()
    spin_thread.join(timeout=0.5)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    with lock:
        n_arm = len(arm_samples)
        n_hand = len(hand_samples)

    if n_arm == 0:
        print(f"[ERROR] No arm samples received on {ARM_JOINT_STATE_TOPIC} within 2 s.")
        sys.exit(1)

    arm_mean = np.mean(arm_samples, axis=0)
    hand_mean = np.mean(hand_samples, axis=0) if n_hand > 0 else np.zeros(12)

    lines = [
        "",
        "=" * 76,
        f"AGIBOT A2 ULTRA - MEASURED VS SIM HOME COMPARISON (Domain: {domain_id})",
        f"Samples collected: {n_arm} arm frames, {n_hand} hand frames over 2.0 s",
        "=" * 76,
        f"{'Joint Name':<16} | {'Measured':>10} | {'Sim Home':>10} | {'Diff':>10} | {'Unit':<5} | Status",
        f"{'-'*16}-+-{'-'*10}-+-{'-'*10}-+-{'-'*10}-+-{'-'*5}-+---------",
    ]

    mismatches = 0
    # Arm comparisons
    for i in range(14):
        meas = float(arm_mean[i])
        ref = float(SIM_HOME_ARM_Q14[i])
        diff = meas - ref
        is_mismatch = abs(diff) > ARM_TOLERANCE_RAD
        if is_mismatch:
            mismatches += 1
            status = f"[MISMATCH >{ARM_TOLERANCE_RAD} rad]"
        else:
            status = "[MATCH]"
        lines.append(f"{ARM_JOINT_NAMES_AIMDK[i]:<16} | {meas:>10.4f} | {ref:>10.4f} | {diff:>+10.4f} | rad   | {status}")

    lines.append(f"{'-'*16}-+-{'-'*10}-+-{'-'*10}-+-{'-'*10}-+-{'-'*5}-+---------")

    # Hand comparisons
    for i in range(12):
        meas = float(hand_mean[i])
        ref = float(SIM_HOME_HAND_RAW12[i])
        diff = meas - ref
        is_mismatch = meas > HAND_TOLERANCE_RAW
        if is_mismatch:
            mismatches += 1
            status = f"[MISMATCH >{int(HAND_TOLERANCE_RAW)} raw]"
        else:
            status = "[MATCH]"
        lines.append(f"{HAND_JOINT_NAMES[i]:<16} | {meas:>10.1f} | {ref:>10.1f} | {diff:>+10.1f} | raw   | {status}")

    lines.append("=" * 76)
    if mismatches == 0:
        lines.append(">>> VERDICT: [PASS] ALL JOINTS MATCH SIM HOME KEYFRAME WITHIN TOLERANCE.")
    else:
        lines.append(f">>> VERDICT: [WARN] {mismatches} JOINTS EXCEED HOME TOLERANCE. VERIFY ROBOT GANTRY & POSE.")
    lines.append("=" * 76)
    print("\n".join(lines))

    if mismatches > 0:
        sys.exit(2)


if __name__ == "__main__":
    main()

