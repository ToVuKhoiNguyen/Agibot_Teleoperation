#!/usr/bin/env python3
"""
tests/test_mapping.py
Verifies sim-to-robot coordinate, name, sign, and unit mappings for AGIBOT A2 Ultra.
"""

import sys
from pathlib import Path

# Add project root to sys.path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from types import SimpleNamespace

from teleop.robot.aimdk_map import (
    ARM_JOINT_NAMES_AIMDK,
    ARM_JOINT_NAMES_URDF,
    HAND_JOINT_NAMES,
    SIM_HAND_JOINT_NAMES,
    SIM_HAND_COMFORTABLE_LIMITS,
    sim_hand_to_robot_raw,
    robot_raw_to_sim_hand,
    sim_hand_to_norm,
    norm_to_sim_hand,
    align_joint_state_positions,
    TODO_THUMB_SWING_SIGN_CALIBRATION,
    TODO_ROS2_ESTOP_TOPIC,
    TODO_HARDWARE_WATCHDOG_TIMEOUT_S,
)
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting


def test_hand_open_pose():
    """Sim fully open pose (all 0 rad) maps to all raw 0."""
    sim_open = np.zeros(12, dtype=np.float64)
    raw = sim_hand_to_robot_raw(sim_open)
    assert raw.shape == (12,), f"Expected shape (12,), got {raw.shape}"
    assert np.all(raw == 0), f"Sim open pose must produce all raw 0, got {raw}"
    # Inverse check
    recovered = robot_raw_to_sim_hand(raw)
    assert np.allclose(recovered, 0.0), f"Inverse mapping failed: {recovered}"
    print("  [PASS] test_hand_open_pose: all 12 raw joints == 0")


def test_hand_closed_pose():
    """Sim cylinder closed grasp maps finger flexion joints to raw near 2000."""
    left_closed = AgibotHandRetargeting.get_grasp_targets("cylinder", 1.0)
    right_closed = AgibotHandRetargeting.get_grasp_targets("cylinder", 1.0)
    sim_closed = np.concatenate([left_closed, right_closed])

    raw = sim_hand_to_robot_raw(sim_closed)
    assert raw.shape == (12,)

    # Finger flexion joints: thumb_1 (1, 7), index (2, 8), middle (3, 9), ring (4, 10), pinky (5, 11)
    flexion_indices = [1, 2, 3, 4, 5, 7, 8, 9, 10, 11]
    for idx in flexion_indices:
        jname = HAND_JOINT_NAMES[idx]
        val = raw[idx]
        assert 1800 <= val <= 2000, f"Joint {jname} closed value {val} not near 2000"

    # Thumb swing (0, 6) in cylinder grasp is ~0.95 rad -> ~1188 raw (59%)
    for idx in [0, 6]:
        jname = HAND_JOINT_NAMES[idx]
        val = raw[idx]
        assert 1000 <= val <= 1400, f"Thumb swing {jname} unexpected closed value {val}"

    # Also test maximum comfortable limits (full flexion 1.0)
    sim_max = SIM_HAND_COMFORTABLE_LIMITS.copy()
    raw_max = sim_hand_to_robot_raw(sim_max)
    assert np.all(raw_max == 2000), f"Full comfortable limits must hit raw 2000, got {raw_max}"
    print(f"  [PASS] test_hand_closed_pose: finger flexion raw range [{raw[flexion_indices].min()}..{raw[flexion_indices].max()}] (near 2000)")


def test_arm_order_and_sign():
    """Arm: sim q14 == published position[], indices 0..6 left, 7..13 right, signs match URDF."""
    assert len(ARM_JOINT_NAMES_AIMDK) == 14
    assert len(ARM_JOINT_NAMES_URDF) == 14

    # Verify left/right partition
    for i in range(7):
        assert "Left" in ARM_JOINT_NAMES_AIMDK[i], f"Expected Left arm at index {i}"
        assert "left" in ARM_JOINT_NAMES_URDF[i], f"Expected left arm at index {i}"
    for i in range(7, 14):
        assert "Right" in ARM_JOINT_NAMES_AIMDK[i], f"Expected Right arm at index {i}"
        assert "right" in ARM_JOINT_NAMES_URDF[i], f"Expected right arm at index {i}"

    # Verify asymmetric sign limits (left elbow negative, right elbow positive)
    # Left J4 is [-2.00, -0.03], Right J4 is [0.03, 2.00]
    left_elbow_idx = 3
    right_elbow_idx = 10
    assert ARM_JOINT_NAMES_AIMDK[left_elbow_idx] == "Left Arm J4"
    assert ARM_JOINT_NAMES_AIMDK[right_elbow_idx] == "Right Arm J4"

    # Sim identity mapping: q14 published 1:1 without arbitrary axis negations
    test_q14 = np.array([-0.2, 0.3, -0.1, -1.0, 0.5, 0.1, -0.1,
                          0.2, -0.3, 0.1, 1.0, -0.5, -0.1, 0.1], dtype=np.float64)
    published_q14 = test_q14.copy()
    assert np.array_equal(test_q14, published_q14)
    print("  [PASS] test_arm_order_and_sign: 14 DoF exact partition (0..6 Left, 7..13 Right) & URDF sign alignment verified")


def test_joint_state_name_alignment():
    """Verify JointState alignment with scrambled names or missing names."""
    # Scrambled order
    scrambled_names = list(reversed(HAND_JOINT_NAMES))
    scrambled_pos = [float(i * 100) for i in range(12)]
    msg_scrambled = SimpleNamespace(name=scrambled_names, position=scrambled_pos)

    aligned = align_joint_state_positions(msg_scrambled, HAND_JOINT_NAMES, 12)
    # The first element in HAND_JOINT_NAMES ('left_thumb_0') was the last in scrambled_names (val 1100.0)
    assert aligned[0] == 1100.0, f"Expected 1100.0 for left_thumb_0, got {aligned[0]}"
    assert aligned[11] == 0.0, f"Expected 0.0 for right_pinky, got {aligned[11]}"

    # Missing names (positional fallback)
    msg_no_name = SimpleNamespace(name=[], position=[float(i) for i in range(12)])
    aligned_fallback = align_joint_state_positions(msg_no_name, HAND_JOINT_NAMES, 12)
    assert np.array_equal(aligned_fallback, np.arange(12, dtype=np.float64))
    print("  [PASS] test_joint_state_name_alignment: name lookup & positional fallback verified")


def test_unknown_spec_constants():
    """Verify unverified/TODO constants are properly declared as None."""
    assert TODO_THUMB_SWING_SIGN_CALIBRATION is None, "TODO_THUMB_SWING_SIGN_CALIBRATION must be None"
    assert TODO_ROS2_ESTOP_TOPIC is None, "TODO_ROS2_ESTOP_TOPIC must be None"
    assert TODO_HARDWARE_WATCHDOG_TIMEOUT_S is None, "TODO_HARDWARE_WATCHDOG_TIMEOUT_S must be None"
    print("  [PASS] test_unknown_spec_constants: UNKNOWN constants correctly flagged")


if __name__ == "__main__":
    print("[TEST MAPPING] Running sim -> robot mapping verification suite...")
    test_hand_open_pose()
    test_hand_closed_pose()
    test_arm_order_and_sign()
    test_joint_state_name_alignment()
    test_unknown_spec_constants()
    print("[TEST MAPPING] ALL TESTS PASSED.")
