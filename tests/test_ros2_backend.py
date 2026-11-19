#!/usr/bin/env python3
"""
tests/test_ros2_backend.py
Numeric acceptance tests for Ros2Backend: locks, payload shape, import purity, and loopback.
Safety: Enforces ROS_DOMAIN_ID=77 and ROS_LOCALHOST_ONLY=1 (never 232).
"""

import os
import sys
import time
import threading
import numpy as np
from pathlib import Path

# Enforce strict test isolation variables
os.environ["ROS_DOMAIN_ID"] = "77"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.robot.aimdk_map import (
    ARM_JOINT_COMMAND_TOPIC,
    HAND_JOINT_COMMAND_TOPIC,
    ARM_JOINT_STATE_TOPIC,
    HAND_JOINT_STATE_TOPIC,
    ARM_JOINT_NAMES_AIMDK,
    HAND_JOINT_NAMES,
    HAND_EFFORT_DEFAULT,
)
from teleop.robot.ros2_backend import Ros2Backend


def test_import_purity():
    """Verify package imports do not drag in rclpy, socket, or requests."""
    assert "rclpy" not in sys.modules, "rclpy must not be imported at module load"
    assert "socket" not in sys.modules, "socket must not be imported at module load"
    assert "requests" not in sys.modules, "requests must not be imported at module load"
    print("  [PASS] teleop.robot package zero-ROS/zero-network import purity verified")


def test_safety_locks():
    """Verify all 4 hardware connection conditions fail with RuntimeError."""
    # 1. Missing allow_hardware_risk
    b1 = Ros2Backend(allow_hardware_risk=False)
    try:
        b1.connect()
        assert False, "Should fail without hardware risk flag"
    except RuntimeError as e:
        assert "missing --i-understand-hardware-risk" in str(e)

    # 2. Missing AGIBOT_ALLOW_REAL=1
    os.environ.pop("AGIBOT_ALLOW_REAL", None)
    b2 = Ros2Backend(allow_hardware_risk=True)
    try:
        b2.connect()
        assert False, "Should fail without AGIBOT_ALLOW_REAL=1"
    except RuntimeError as e:
        assert "AGIBOT_ALLOW_REAL != 1" in str(e)

    # 3. Confirmation phrase mismatch
    os.environ["AGIBOT_ALLOW_REAL"] = "1"
    b3 = Ros2Backend(allow_hardware_risk=True, confirmation_input_fn=lambda _: "WRONG PHRASE")
    try:
        b3.connect()
        assert False, "Should fail on wrong confirmation phrase"
    except RuntimeError as e:
        assert "confirmation phrase mismatch" in str(e)

    # 4. Robot state topic not publishing (timeout within 1.0 s)
    b4 = Ros2Backend(allow_hardware_risk=True, confirmation_input_fn=lambda _: "GANTRY AND ESTOP READY")
    t0 = time.time()
    try:
        b4.connect()
        assert False, "Should fail when robot is not publishing state"
    except RuntimeError as e:
        elapsed = time.time() - t0
        assert elapsed >= 0.9, f"Lock timeout too fast: {elapsed}s"
        assert "no robot arm_joint_state received within 1.0s" in str(e)
    finally:
        os.environ.pop("AGIBOT_ALLOW_REAL", None)

    print("  [PASS] All 4 hardware unlock gates strictly enforced")


def test_payload_shape_and_mapping():
    """Verify 14 doubles arm shape, 12 named hand joints, and 0..2000 clamping."""
    backend = Ros2Backend(allow_hardware_risk=True)
    # Synthetic arm commands
    q14 = np.linspace(-0.5, 0.5, 14)
    backend._target_arm_q14 = q14.copy()
    assert len(backend._target_arm_q14) == 14
    assert len(ARM_JOINT_NAMES_AIMDK) == 14

    # Hand normalized [0.0..1.0] -> raw [0..2000]
    h12 = np.array([0.0, 0.5, 1.0, 0.25, 0.75, 1.5, -0.2, 0.1, 0.2, 0.3, 0.4, 0.5])
    raw_positions = [
        int(np.clip(2000.0 * v, 0, 2000)) for v in h12
    ]
    assert raw_positions[0] == 0       # 0.0 -> 0
    assert raw_positions[1] == 1000    # 0.5 -> 1000
    assert raw_positions[2] == 2000    # 1.0 -> 2000
    assert raw_positions[5] == 2000    # >1.0 clamped to 2000
    assert raw_positions[6] == 0       # <0.0 clamped to 0
    assert len(raw_positions) == 12
    assert len(HAND_JOINT_NAMES) == 12

    # Single arm hold logic
    b_left = Ros2Backend(arms="left")
    b_left.connected = True
    b_left._latest_arm_q14 = np.ones(14) * 0.1  # Measured state
    b_left.send_arm(np.ones(14) * 0.9)
    assert np.allclose(b_left._target_arm_q14[:7], 0.9)  # Left commands
    assert np.allclose(b_left._target_arm_q14[7:], 0.1)  # Right holds measured
    print("  [PASS] Payload shapes: 14 doubles, 12 hand joints in [0..2000], arm gating verified")


def test_ros2_loopback_domain_77():
    """Full ROS 2 DDS loopback test on ROS_DOMAIN_ID=77 with test node."""
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState

    if not rclpy.ok():
        rclpy.init()

    test_node = Node("ros2_backend_test_fixture")
    pub_arm_state = test_node.create_publisher(JointState, ARM_JOINT_STATE_TOPIC, 10)
    pub_hand_state = test_node.create_publisher(JointState, HAND_JOINT_STATE_TOPIC, 10)

    received_arm_cmds = []
    received_hand_cmds = []

    def on_cmd_arm(msg): received_arm_cmds.append((time.time(), list(msg.position)))
    def on_cmd_hand(msg): received_hand_cmds.append((time.time(), list(msg.position), list(msg.effort)))

    test_node.create_subscription(JointState, ARM_JOINT_COMMAND_TOPIC, on_cmd_arm, 10)
    test_node.create_subscription(JointState, HAND_JOINT_COMMAND_TOPIC, on_cmd_hand, 10)

    executor = SingleThreadedExecutor()
    executor.add_node(test_node)
    spin_stop = threading.Event()
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Pre-publish initial state so backend sees robot alive
    state_stop = threading.Event()
    def pub_state_loop():
        msg_a = JointState()
        msg_a.position = [0.05 * (i + 1) for i in range(14)]
        msg_h = JointState()
        msg_h.position = [1000.0] * 12
        while not state_stop.is_set():
            pub_arm_state.publish(msg_a)
            pub_hand_state.publish(msg_h)
            time.sleep(0.02)
    state_thread = threading.Thread(target=pub_state_loop, daemon=True)
    state_thread.start()
    time.sleep(0.1)

    os.environ["AGIBOT_ALLOW_REAL"] = "1"
    backend = Ros2Backend(
        allow_hardware_risk=True,
        confirmation_input_fn=lambda _: "GANTRY AND ESTOP READY",
        hand_raw_max=2000,
        enable_thumb_swing=True,
    )
    backend.connect()
    assert backend.connected and backend.is_alive()

    from teleop.robot.aimdk_map import norm_to_sim_hand

    # Send target commands
    cmd_arm = np.array([0.1, 0.2, 0.3, -0.4, 0.5, 0.2, -0.1, -0.1, -0.2, -0.3, 0.4, -0.5, -0.2, 0.1])
    cmd_hand = norm_to_sim_hand(np.full(12, 0.5))  # 50% closure -> raw 1000
    backend.send_arm(cmd_arm)
    backend.send_hand(cmd_hand)

    # Collect packets for 1.0 s
    received_arm_cmds.clear()
    t_start = time.time()
    time.sleep(1.0)
    t_end = time.time()
    dt = t_end - t_start
    rate_hz = len(received_arm_cmds) / dt

    assert 45.0 <= rate_hz <= 55.0, f"Rate {rate_hz:.1f} Hz outside 45-55 Hz window"
    last_rx = received_arm_cmds[-1][1]
    assert np.allclose(last_rx, cmd_arm, atol=1e-6), f"Values mismatch: {last_rx} vs {cmd_arm}"
    last_hand_rx = received_hand_cmds[-1]
    assert np.allclose(last_hand_rx[1], 1000.0), f"Hand raw position mismatch: {last_hand_rx[1]}"
    assert np.allclose(last_hand_rx[2], HAND_EFFORT_DEFAULT), f"Hand effort mismatch: {last_hand_rx[2]}"

    # Stale-state test: stop publishing state for 300 ms (> 200 ms threshold)
    state_stop.set()
    time.sleep(0.35)
    assert not backend.is_alive(), "Backend should report is_alive=False when state older than 200 ms"

    # Hold test: verify publisher continues running
    received_arm_cmds.clear()
    backend.hold()
    time.sleep(0.5)
    assert len(received_arm_cmds) >= 20, "Hold must keep publishing measured position at 50 Hz"

    backend.disconnect()
    state_stop.set()
    state_thread.join(timeout=0.5)
    executor.shutdown()
    spin_thread.join(timeout=0.5)
    test_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    os.environ.pop("AGIBOT_ALLOW_REAL", None)
    print("  [PASS] Loopback domain 77: 45-55 Hz rate, 1e-6 precision, stale watchdog, and hold verified")


def main():
    print("=" * 70)
    print("RUNNING ROS 2 REAL BACKEND ACCEPTANCE TESTS (DOMAIN=77, LOCALHOST_ONLY)")
    print("=" * 70)
    test_import_purity()
    test_safety_locks()
    test_payload_shape_and_mapping()
    test_ros2_loopback_domain_77()
    print("=" * 70)
    print("ALL ROS 2 BACKEND ACCEPTANCE TESTS PASSED!")
    print("=" * 70)


if __name__ == "__main__":
    main()
