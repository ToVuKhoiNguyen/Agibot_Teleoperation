#!/usr/bin/env python3
"""
scripts/test_controller_axes.py
Synthetic controller motion test for AGIBOT A2 Ultra.
Feeds synthetic controller displacements (+up, -up, +forward, -forward, +left, -left, each 10 cm)
through the controller_axis_map and IK into MuJoCo, verifying that the robot wrist
displacement per axis matches its commanded direction within 15% and has the correct sign.
"""

import os
import sys
import yaml
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.kinematics.agibot_a2_ik import AgibotA2IK

SCENE_XML = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "mjcf", "scene.xml")
CONFIG_YAML = os.path.join(PROJECT_ROOT, "config", "quest3s.yaml")


def test_axes():
    print("=" * 80)
    print("RUNNING SYNTHETIC CONTROLLER AXES VERIFICATION TEST")
    print("=" * 80)

    # 1. Load config
    with open(CONFIG_YAML, "r") as f:
        cfg = yaml.safe_load(f)
    axis_map = np.array(cfg["controller_axis_map"], dtype=np.float64)
    print("Loaded Controller Axis Map (Quest XR -> Robot Frame):")
    print(axis_map)

    # 2. Init sim & IK
    env = AgibotSimEnv(SCENE_XML, render_gui=False)
    env.reset()
    ik_left = AgibotA2IK(SCENE_XML, arm="left")
    ik_right = AgibotA2IK(SCENE_XML, arm="right")

    # Starting robot wrist poses
    init_poses = env.get_ee_poses()
    home_pos_l = init_poses["left"]["pos"].copy()
    home_rot_l = init_poses["left"]["rot"].copy()

    # Define 6 test motions in Quest XR frame (10 cm each):
    # Quest XR axes: +x right, +y up, -z forward
    # Forward: delta_xr = [ 0.0,  0.0, -0.10]
    # Back:    delta_xr = [ 0.0,  0.0, +0.10]
    # Left:    delta_xr = [-0.10,  0.0,  0.0]
    # Right:   delta_xr = [+0.10,  0.0,  0.0]
    # Up:      delta_xr = [ 0.0, +0.10,  0.0]
    # Down:    delta_xr = [ 0.0, -0.10,  0.0]

    test_cases = [
        {"name": "FORWARD (+X robot)", "delta_xr": np.array([ 0.0,  0.0, -0.10]), "expected_robot_dir": np.array([+0.10,  0.00,  0.00]), "axis_idx": 0, "sign": +1},
        {"name": "BACK    (-X robot)", "delta_xr": np.array([ 0.0,  0.0, +0.10]), "expected_robot_dir": np.array([-0.10,  0.00,  0.00]), "axis_idx": 0, "sign": -1},
        {"name": "LEFT    (+Y robot)", "delta_xr": np.array([-0.10,  0.0,  0.00]), "expected_robot_dir": np.array([ 0.00, +0.10,  0.00]), "axis_idx": 1, "sign": +1},
        {"name": "RIGHT   (-Y robot)", "delta_xr": np.array([+0.10,  0.0,  0.00]), "expected_robot_dir": np.array([ 0.00, -0.10,  0.00]), "axis_idx": 1, "sign": -1},
        {"name": "UP      (+Z robot)", "delta_xr": np.array([ 0.00, +0.10,  0.00]), "expected_robot_dir": np.array([ 0.00,  0.00, +0.10]), "axis_idx": 2, "sign": +1},
        {"name": "DOWN    (-Z robot)", "delta_xr": np.array([ 0.00, -0.10,  0.00]), "expected_robot_dir": np.array([ 0.00,  0.00, -0.10]), "axis_idx": 2, "sign": -1},
    ]

    all_passed = True
    print("\nEvaluating 6 cardinal directions (Left Arm):")
    print("-" * 80)

    for tc in test_cases:
        # Reset to home
        env.reset()
        init_qpos = env.get_left_arm_qpos()

        # Map XR delta to robot frame
        delta_robot = axis_map @ tc["delta_xr"]
        target_pos = home_pos_l + delta_robot

        # Solve IK
        q_sol, succ, pos_err, rot_err, _ = ik_left.solve(
            target_pos=target_pos,
            target_rot=home_rot_l,
            init_qpos=init_qpos,
            max_iters=35,
            pos_tol=0.005,
            rot_tol=0.035
        )

        # Apply to sim
        env.set_left_arm_qpos(q_sol)
        env.step(num_substeps=2, hold_arms=True, hold_left_qpos=q_sol)

        # Measure displacement
        new_ee = env.get_ee_poses()["left"]["pos"]
        actual_disp = new_ee - home_pos_l

        target_mag = float(np.linalg.norm(tc["expected_robot_dir"]))
        actual_val = actual_disp[tc["axis_idx"]]
        expected_val = tc["expected_robot_dir"][tc["axis_idx"]]

        error_pct = abs(actual_val - expected_val) / target_mag * 100.0
        correct_sign = (actual_val * expected_val > 0)
        within_15_pct = error_pct <= 15.0

        passed = within_15_pct and correct_sign
        status_str = "[PASS]" if passed else "[FAIL]"
        if not passed:
            all_passed = False

        print(f" {status_str} {tc['name']:22s} | Commanded: [{delta_robot[0]:+.2f}, {delta_robot[1]:+.2f}, {delta_robot[2]:+.2f}] m | "
              f"Actual: [{actual_disp[0]:+.3f}, {actual_disp[1]:+.3f}, {actual_disp[2]:+.3f}] m | "
              f"Axis Err: {error_pct:4.1f}% | Sign OK: {correct_sign}")

    print("-" * 80)
    if all_passed:
        print("[SUCCESS] All 6 synthetic controller displacement tests PASSED!")
    else:
        print("[FAILURE] One or more axis displacement tests failed tolerance check.")

    return all_passed


if __name__ == "__main__":
    success = test_axes()
    sys.exit(0 if success else 1)

