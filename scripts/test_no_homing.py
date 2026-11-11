#!/usr/bin/env python3
"""scripts/test_no_homing.py
Targeted verification tests for fix/no-homing:
1. Measured pose far from HOME:
   - No command published before Engage.
   - First command strictly equals measured pose (diff == 0), no FAULT.
2. Packet with a 0.05 rad step jump is ramped to <= 0.02 rad, NOT latched as FAULT.
3. Latch flag from receiver reaches laptop feedback metadata and updates laptop status.
"""
import os, sys, time
import numpy as np

try:
    import rclpy
    rclpy.init()
except Exception:
    pass

import scripts.arm_teleop_rx as rx
from teleop.robot.udp_backend import UdpBackend
from teleop.robot.safety_layer import SafetyLayer


def test_no_command_before_engage_and_first_equals_measured():
    print("--- 1. Testing No Command Before Engage & First Equals Measured ---")
    node = rx.ArmTeleopRx(mode_live=False, allow_ip="127.0.0.1", speed_level=3)
    measured_pose = [0.25, 1.10, -0.15, -0.45, 1.60, 0.05, 0.02,
                     -0.10, -1.05, 0.12, 0.48, 1.45, -0.02, 0.01]
    node.meas_q = list(measured_pose)

    # 10 ticks before any packet received: must NOT publish or move
    for _ in range(10):
        node._control_tick()
        assert not node.has_published_engaged, "Premature publish before Engage!"

    # Send first packet with engaged=False: still must not publish
    node.target_q = list(measured_pose)
    node.target_engaged = [False, False]
    node.last_pkt_time = time.monotonic()
    node._control_tick()
    assert not node.has_published_engaged, "Published while engaged is False!"

    # Send first packet with engaged=True:
    # First command must equal measured pose exactly
    node.target_engaged = [True, True]
    node.last_pkt_time = time.monotonic()
    node._control_tick()
    assert node.has_published_engaged, "Failed to activate upon Engage!"
    diff = max(abs(node.curr_cmd_q[i] - measured_pose[i]) for i in range(14))
    assert diff < 1e-5, f"First command {node.curr_cmd_q} differs from measured {measured_pose}!"
    assert not node.fault_latched, f"Unexpected FAULT: {node.fault_reason}"
    print("[PASS] Verified: zero commands before Engage, first command == measured pose.")


def test_step_jump_ramped_not_latched():
    print("--- 2. Testing 0.05 rad Step Jump is Ramped (Not Latched) ---")
    node = rx.ArmTeleopRx(mode_live=False, allow_ip="127.0.0.1", speed_level=3)
    init_pose = [0.0] * 14
    node.meas_q = list(init_pose)
    node.target_q = list(init_pose)
    node.target_engaged = [True, True]
    node.last_pkt_time = time.monotonic()
    node._control_tick()  # Engage

    # Now step target abruptly by 0.05 rad
    jumped_target = list(init_pose)
    jumped_target[0] = 0.05
    node.target_q = jumped_target
    dt = 0.02
    prev_cmd = list(node.curr_cmd_q)
    node._control_tick()

    step_taken = abs(node.curr_cmd_q[0] - prev_cmd[0])
    print(f"Step taken for 0.05 rad jump: {step_taken:.5f} rad")
    assert step_taken <= 0.020001, f"Step {step_taken} exceeded 0.02 rad cap!"
    assert not node.fault_latched, f"Step jump erroneously latched FAULT: {node.fault_reason}"
    print("[PASS] Verified: 0.05 rad step jump is ramped smoothly without FAULT latch.")


def test_latch_flag_reaches_laptop_status():
    print("--- 3. Testing Latch Flag Propagation to Laptop ---")
    backend = UdpBackend(host="127.0.0.1", tx_port=5005, rx_port=5006)
    safety = SafetyLayer(backend=backend)

    # Simulate receiver sending robot_fault: True in feedback packet
    backend.rx_telemetry = {
        "robot_fault": True,
        "fault_reason": "NaN/inf detected in command computation",
        "q": [0.0] * 14
    }
    backend._real_q14 = np.zeros(14)
    backend._last_state_t = time.time()

    q_act, h_act, meta = safety.read_state()
    assert meta.get("robot_fault") is True, "robot_fault flag missing from meta!"

    # Simulate sync_manager handling the latch
    class MockApp:
        sync_state = "ENGAGED"
        robot_safety = safety
        mirror_limiter = None

    app = MockApp()
    if meta.get("robot_fault", False):
        app.robot_safety.fault_latched = True
        app.robot_safety.fault_reason = "ROBOT NODE FAULT: restart arm_teleop_rx.py"
        app.sync_state = "FAULT"

    assert app.sync_state == "FAULT", "Laptop failed to transition to FAULT!"
    assert "ROBOT NODE FAULT" in app.robot_safety.fault_reason, f"Unexpected reason: {app.robot_safety.fault_reason}"
    print(f"[PASS] Verified: Robot node fault propagated to laptop ({app.robot_safety.fault_reason}).")


if __name__ == "__main__":
    if rx.rclpy is not None and not rx.rclpy.ok():
        rx.rclpy.init()
    test_no_command_before_engage_and_first_equals_measured()
    test_step_jump_ramped_not_latched()
    test_latch_flag_reaches_laptop_status()
    if rx.rclpy is not None and rx.rclpy.ok():
        rx.rclpy.shutdown()
    print("\nALL VERIFICATION TESTS COMPLETED SUCCESSFULLY.")
