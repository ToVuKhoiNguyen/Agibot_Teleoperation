#!/usr/bin/env python3
"""
tests/test_data_collection_preset.py
Unit tests for the Data Collection posture preset (HOME with arms raised ~30 deg).
"""

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.app.joint_preset import JointPresetPlanner
from teleop.app.cmd_targets import handle_target_cmd
from teleop.app.homing_controller import start_data_collection


class MockApp:
    def __init__(self):
        self.env = AgibotSimEnv("assets/agibot_a2_ultra/mjcf/scene.xml")
        self.armor_collision = None
        self.last_arm_cmd_left = np.zeros(7, dtype=np.float64)
        self.last_arm_cmd_right = np.zeros(7, dtype=np.float64)
        self.target_curls_l = np.ones(5, dtype=np.float64)
        self.target_curls_r = np.ones(5, dtype=np.float64)
        self.target_pinch_l = 1.0
        self.target_pinch_r = 1.0
        self.hand_closure_l = 1.0
        self.hand_closure_r = 1.0
        self.grasp_type_l = "pinch"
        self.grasp_type_r = "pinch"
        self.direct_hand_override_l = True
        self.direct_hand_override_r = True
        self.homing_active = False


def test_data_collection_joint_angles():
    app = MockApp()
    planner = JointPresetPlanner(app)

    q_home = planner.goal("home")
    q_dc = planner.goal("data_collection")

    diff_deg = np.rad2deg(q_dc - q_home)
    assert np.isclose(diff_deg[0], 50.0, atol=1e-2), f"Expected +50 deg on left J1, got {diff_deg[0]}"
    assert np.isclose(diff_deg[7], 50.0, atol=1e-2), f"Expected +50 deg on right J1, got {diff_deg[7]}"
    assert np.allclose(diff_deg[1:7], 0.0), "Expected 0 deg on left J2..J7"
    assert np.allclose(diff_deg[8:14], 0.0), "Expected 0 deg on right J2..J7"


def test_start_data_collection():
    app = MockApp()
    start_data_collection(app, "both")
    assert np.all(app.target_curls_l == 0.0)
    assert np.all(app.target_curls_r == 0.0)
    assert app.hand_closure_l == 0.0
    assert app.hand_closure_r == 0.0
    assert app.homing_active is True
    assert app.joint_motion["label"] == "data_collection"
    from teleop.robot.aimdk_map import THUMB_POSE_RAW
    assert app.thumb_pose_raw == THUMB_POSE_RAW


def test_handle_target_cmd_preset():
    app = MockApp()
    handled = handle_target_cmd(app, "preset", {"cmd": "preset", "pose": "data_collection"})
    assert handled is True
    assert app.homing_active is True
    assert app.joint_motion["label"] == "data_collection"


def test_handle_target_cmd_direct_name():
    app = MockApp()
    handled = handle_target_cmd(app, "data_collection", {"name": "data_collection"})
    assert handled is True
    assert app.homing_active is True
    assert app.joint_motion["label"] == "data_collection"


if __name__ == "__main__":
    test_data_collection_joint_angles()
    test_start_data_collection()
    test_handle_target_cmd_preset()
    test_handle_target_cmd_direct_name()
    print("ALL DATA COLLECTION PRESET TESTS PASSED")
