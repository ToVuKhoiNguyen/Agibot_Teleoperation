#!/usr/bin/env python3
"""
tools/ros2_listen.py
Strictly READ-ONLY ROS 2 telemetry listener for AGIBOT A2 Ultra.
Subscribes ONLY to arm_joint_state and hand_joint_state topics.
NO publishers, NO robot commands, NO network side-effects.
"""

import os
import sys
import time
from pathlib import Path

# Add project root to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from teleop.robot.aimdk_map import (
    ARM_JOINT_STATE_TOPIC,
    HAND_JOINT_STATE_TOPIC,
)


def main():
    domain_id = os.environ.get("ROS_DOMAIN_ID")
    if not domain_id:
        print("[FATAL] ROS_DOMAIN_ID environment variable is unset.")
        print("[FATAL] Refusing to run without an explicit domain ID (e.g. export ROS_DOMAIN_ID=77).")
        sys.exit(1)

    print(f"[Ros2Listen] Initializing listener on ROS_DOMAIN_ID={domain_id}...")
    if str(domain_id).strip() == "232":
        print("!" * 78)
        print("[WARNING] CAUTION: Connected to ROS_DOMAIN_ID=232 (Real Robot Production Domain)!")
        print("[WARNING] Listener mode is strictly READ-ONLY (zero publishers created).")
        print("!" * 78)

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError as e:
        print(f"[FATAL] ROS 2 (rclpy/sensor_msgs) import error: {e}")
        sys.exit(1)

    rclpy.init()
    node = Node("agibot_ros2_readonly_listener")

    stats = {
        "arm_count": 0,
        "arm_last_t": 0.0,
        "arm_names": [],
        "arm_pos": [],
        "arm_rate": 0.0,
        "hand_count": 0,
        "hand_last_t": 0.0,
        "hand_names": [],
        "hand_pos": [],
        "hand_effort": [],
        "hand_rate": 0.0,
    }

    t_window_start = time.time()
    arm_window_count = 0
    hand_window_count = 0

    def on_arm_state(msg: JointState):
        nonlocal arm_window_count
        stats["arm_count"] += 1
        arm_window_count += 1
        stats["arm_last_t"] = time.time()
        stats["arm_names"] = list(msg.name)
        stats["arm_pos"] = list(msg.position)

    def on_hand_state(msg: JointState):
        nonlocal hand_window_count
        stats["hand_count"] += 1
        hand_window_count += 1
        stats["hand_last_t"] = time.time()
        stats["hand_names"] = list(msg.name)
        stats["hand_pos"] = list(msg.position)
        stats["hand_effort"] = list(msg.effort)

    # Strictly READ-ONLY subscriptions (no publishers created)
    sub_arm = node.create_subscription(JointState, ARM_JOINT_STATE_TOPIC, on_arm_state, 10)
    sub_hand = node.create_subscription(JointState, HAND_JOINT_STATE_TOPIC, on_hand_state, 10)

    print(f"[Ros2Listen] Listening on:\n  - {ARM_JOINT_STATE_TOPIC}\n  - {HAND_JOINT_STATE_TOPIC}\n(Press Ctrl+C to stop)")

    import threading
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while True:
            time.sleep(1.0)
            now = time.time()
            dt = now - t_window_start
            if dt > 0:
                stats["arm_rate"] = arm_window_count / dt
                stats["hand_rate"] = hand_window_count / dt
            arm_window_count = 0
            hand_window_count = 0
            t_window_start = now

            # Print telemetry summary
            print("\n" + "=" * 65)
            print(f"ROS 2 TELEMETRY (Domain {domain_id}) | Time: {time.strftime('%H:%M:%S')}")
            print("-" * 65)
            # Arm summary
            arm_alive = (now - stats["arm_last_t"]) < 1.0 if stats["arm_last_t"] > 0 else False
            arm_status = f"{stats['arm_rate']:5.1f} Hz (Total: {stats['arm_count']})" if arm_alive else "NO DATA / STALE"
            print(f"ARM STATE  [{ARM_JOINT_STATE_TOPIC}]: {arm_status}")
            if stats["arm_pos"]:
                min_v, max_v = min(stats["arm_pos"]), max(stats["arm_pos"])
                print(f"  Count: {len(stats['arm_pos'])} joints | Range: [{min_v:+.3f} .. {max_v:+.3f}] rad")
                if stats["arm_names"]:
                    print(f"  Names ({len(stats['arm_names'])}): {', '.join(stats['arm_names'][:4])} ...")

            # Hand summary
            hand_alive = (now - stats["hand_last_t"]) < 1.0 if stats["hand_last_t"] > 0 else False
            hand_status = f"{stats['hand_rate']:5.1f} Hz (Total: {stats['hand_count']})" if hand_alive else "NO DATA / STALE"
            print(f"HAND STATE [{HAND_JOINT_STATE_TOPIC}]: {hand_status}")
            if stats["hand_pos"]:
                min_p, max_p = min(stats["hand_pos"]), max(stats["hand_pos"])
                print(f"  Count: {len(stats['hand_pos'])} joints | Pos Range: [{int(min_p)} .. {int(max_p)}] raw")
                if stats["hand_effort"]:
                    min_e, max_e = min(stats["hand_effort"]), max(stats["hand_effort"])
                    print(f"  Effort Range: [{int(min_e)} .. {int(max_e)}]")
                if stats["hand_names"]:
                    print(f"  Names ({len(stats['hand_names'])}): {', '.join(stats['hand_names'][:4])} ...")
            print("=" * 65)

    except KeyboardInterrupt:
        print("\n[Ros2Listen] Stopping listener cleanly...")
    finally:
        executor.shutdown()
        spin_thread.join(timeout=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[Ros2Listen] Shutdown complete.")


if __name__ == "__main__":
    main()

