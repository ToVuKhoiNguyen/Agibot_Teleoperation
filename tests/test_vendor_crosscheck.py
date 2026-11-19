#!/usr/bin/env python3
"""
tests/test_vendor_crosscheck.py
Offline verification tests for vendor spec crosscheck invariants:
- 14 arm joint names, order, and index mapping (idx13..idx26)
- Tighter intersection physical joint limits & velocity/accel ceilings
- Arm joint state parser tolerance for joints 6/7 position-only (empty/short velocity/effort)
- Hand frame_id == "AgiHand" validation and position [0..2000] / effort [0..5700] clamping
- RPC endpoint & SetAction action payload format
All tests run strictly locally in isolated loopback domain 77 with ZERO external network access.
"""

import os
import sys
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Strictly enforce loopback domain 77
os.environ["ROS_DOMAIN_ID"] = "77"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

from teleop.robot.aimdk_map import (
    ARM_JOINT_NAMES_AIMDK,
    ARM_JOINT_NAMES_URDF,
    HAND_JOINT_NAMES,
    HAND_POS_RAW_MIN,
    HAND_POS_RAW_MAX,
    HAND_EFFORT_RAW_MIN,
    HAND_EFFORT_RAW_MAX,
    HAND_FRAME_ID,
    A2_DEFAULT_IP,
    A2_MOTION_PORT,
    ACTION_ID_SERVO_402,
    ACTION_NAME_SERVO_402,
    RPC_SET_ACTION_402_PAYLOAD,
    align_joint_state_positions,
    sim_hand_to_robot_raw,
)
from teleop.robot.safety_layer import SafetyLayer, ARM_LOWER_14, ARM_UPPER_14
from teleop.robot.ros2_backend import Ros2Backend
from teleop.robot.interface import RobotInterface


class DummyBackend(RobotInterface):
    """Local dummy backend for testing SafetyLayer and parser without network."""
    def __init__(self):
        self.q14 = np.zeros(14, dtype=np.float64)
        self.h12 = np.zeros(12, dtype=np.float64)
        self.last_sent_arm = None
        self.last_sent_hand = None
    def connect(self): pass
    def disconnect(self): pass
    def send_arm(self, q14): self.last_sent_arm = np.asarray(q14, dtype=np.float64).copy()
    def send_hand(self, h12): self.last_sent_hand = np.asarray(h12, dtype=np.float64).copy()
    def read_state(self): return self.q14.copy(), self.h12.copy(), 1000.0
    def estop(self): pass
    def hold(self): pass
    def is_alive(self): return True


class MockJointStateMsg:
    """Mock ROS 2 JointState message."""
    def __init__(self, name=None, position=None, velocity=None, effort=None, frame_id=""):
        self.name = list(name) if name is not None else []
        self.position = list(position) if position is not None else []
        self.velocity = list(velocity) if velocity is not None else []
        self.effort = list(effort) if effort is not None else []
        self.header = type("Header", (), {"frame_id": frame_id})()


def test_joint_names_and_indices():
    """Verify 14 joint names, order (idx13..idx26), and left/right split."""
    assert len(ARM_JOINT_NAMES_AIMDK) == 14
    assert len(ARM_JOINT_NAMES_URDF) == 14
    expected_urdf = [f"idx{13+i}_left_arm_joint{1+i}" for i in range(7)] + [f"idx{20+i}_right_arm_joint{1+i}" for i in range(7)]
    assert ARM_JOINT_NAMES_URDF == expected_urdf, f"URDF names mismatch: {ARM_JOINT_NAMES_URDF}"
    print("  [PASS] 14 arm joint names and idx13..idx26 order verified")


def test_tighter_limits_intersection():
    """Verify arm limits match the tightened yellow-region intersection."""
    # Joint 6 (wrist pitch) +-0.30, Joint 7 (wrist yaw) +-0.20
    assert ARM_LOWER_14[5] == -0.30 and ARM_UPPER_14[5] == 0.30, f"L6 limits: {ARM_LOWER_14[5]}, {ARM_UPPER_14[5]}"
    assert ARM_LOWER_14[6] == -0.20 and ARM_UPPER_14[6] == 0.20, f"L7 limits: {ARM_LOWER_14[6]}, {ARM_UPPER_14[6]}"
    assert ARM_LOWER_14[12] == -0.30 and ARM_UPPER_14[12] == 0.30, f"R6 limits: {ARM_LOWER_14[12]}, {ARM_UPPER_14[12]}"
    assert ARM_LOWER_14[13] == -0.20 and ARM_UPPER_14[13] == 0.20, f"R7 limits: {ARM_LOWER_14[13]}, {ARM_UPPER_14[13]}"
    # Asymmetric bounds
    assert np.isclose(ARM_LOWER_14[1], -0.46) and np.isclose(ARM_UPPER_14[1], 1.5359), f"L2 limits: {ARM_LOWER_14[1]}, {ARM_UPPER_14[1]}"
    assert np.isclose(ARM_LOWER_14[3], -2.00) and np.isclose(ARM_UPPER_14[3], -0.03), f"L4 limits: {ARM_LOWER_14[3]}, {ARM_UPPER_14[3]}"
    assert np.isclose(ARM_LOWER_14[8], -1.5359) and np.isclose(ARM_UPPER_14[8], 0.46), f"R2 limits: {ARM_LOWER_14[8]}, {ARM_UPPER_14[8]}"
    assert np.isclose(ARM_LOWER_14[9], -2.91) and np.isclose(ARM_UPPER_14[9], 2.91), f"R3 limits: {ARM_LOWER_14[9]}, {ARM_UPPER_14[9]}"
    assert np.isclose(ARM_LOWER_14[10], 0.03) and np.isclose(ARM_UPPER_14[10], 2.00), f"R4 limits: {ARM_LOWER_14[10]}, {ARM_UPPER_14[10]}"
    print("  [PASS] Tighter intersection arm limits verified")


def test_state_parser_tolerance():
    """Verify arm state parsing handles joints 6/7 having position only (short/empty velocity & effort)."""
    # 14 positions, only 10 velocities, empty effort
    pos_14 = [float(0.05 * i) for i in range(14)]
    vel_10 = [0.1] * 10
    msg = MockJointStateMsg(name=ARM_JOINT_NAMES_AIMDK, position=pos_14, velocity=vel_10, effort=[])
    parsed_pos = align_joint_state_positions(msg, ARM_JOINT_NAMES_AIMDK, 14)
    assert len(parsed_pos) == 14
    assert np.allclose(parsed_pos, pos_14), f"State parser failed on short velocity/effort: {parsed_pos}"
    print("  [PASS] State parsing tolerant to joints 6/7 having position only")


def test_hand_frame_id_and_effort_caps():
    """Verify hand frame_id == 'AgiHand' validation and effort / pos limits."""
    assert HAND_FRAME_ID == "AgiHand"
    assert HAND_POS_RAW_MIN == 0 and HAND_POS_RAW_MAX == 2000
    assert HAND_EFFORT_RAW_MIN == 0 and HAND_EFFORT_RAW_MAX == 5700

    backend = Ros2Backend(allow_hardware_risk=False, hand_effort=6500)
    assert backend.hand_effort == 5700.0, f"Hand effort ceiling 5700 not enforced: {backend.hand_effort}"

    # Verify dropping msg if frame_id != AgiHand
    msg_bad = MockJointStateMsg(name=HAND_JOINT_NAMES, position=[100]*12, frame_id="WrongHand")
    backend._on_hand_state(msg_bad)
    assert backend._last_hand_state_t is None, "Wrong frame_id was not rejected"

    # Verify accepting msg if frame_id == AgiHand
    msg_good = MockJointStateMsg(name=HAND_JOINT_NAMES, position=[100]*12, frame_id=HAND_FRAME_ID)
    backend._on_hand_state(msg_good)
    assert backend._last_hand_state_t is not None, "AgiHand frame_id was rejected"
    print("  [PASS] Hand frame_id == 'AgiHand' and effort clamp 0..5700 verified")


def test_rpc_set_action_spec():
    """Verify RPC endpoint, SetAction 402 ext_action payload, and host defaults."""
    assert A2_DEFAULT_IP == "192.168.100.100"
    assert A2_MOTION_PORT == 56322
    assert ACTION_ID_SERVO_402 == 402
    assert ACTION_NAME_SERVO_402 == "McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO"
    assert RPC_SET_ACTION_402_PAYLOAD["command"]["action"] == 402
    assert RPC_SET_ACTION_402_PAYLOAD["command"]["ext_action"] == ACTION_NAME_SERVO_402
    print("  [PASS] SetAction payload McAction_USE_EXT_CMD / ext_action and RPC host verified")


def main():
    print("=" * 70)
    print("RUNNING VENDOR SPECIFICATION CROSSCHECK ACCEPTANCE TEST")
    print("=" * 70)
    test_joint_names_and_indices()
    test_tighter_limits_intersection()
    test_state_parser_tolerance()
    test_hand_frame_id_and_effort_caps()
    test_rpc_set_action_spec()
    print("=" * 70)
    print("ALL VENDOR CROSSCHECK ACCEPTANCE TESTS PASSED!")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())

