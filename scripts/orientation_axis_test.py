#!/usr/bin/env python3
"""
scripts/orientation_axis_test.py
Headless simulation test verifying wrist flexion / axis rotation mapping
for WebXR hand tracking and Quest controllers.

Evaluates synthetic neutral + 30 deg rotations about XR axes (x, y, z)
for Left and Right hands, measuring end-effector orientation in robot base frame.
Expected semantics:
- XR x (flexion/extension) -> Robot +y (pitch)
- XR y (radial/ulnar)      -> Robot +z (yaw)
- XR z (pronation/supin)   -> Robot +x (roll)
"""

import sys
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from main import UnifiedTeleopApp


def run_test():
    print("=" * 80)
    print("ORIENTATION AXIS TEST (HEADLESS SIMULATION)")
    print("=" * 80)

    # Use non-default port to avoid conflicts
    app = UnifiedTeleopApp(port=8998, render_viewer=False, no_adb=True, no_quest_launch=True)
    ee_home = app.env.get_ee_poses()
    app.robot_home_rot_l = ee_home["left"]["rot"].copy()
    app.robot_home_rot_r = ee_home["right"]["rot"].copy()

    # Define test axes and expected robot base axis mapping
    xr_axes = [
        ("x", [1.0, 0.0, 0.0], "+y (pitch)", np.array([0.0, 1.0, 0.0])),
        ("y", [0.0, 1.0, 0.0], "+z (yaw)",   np.array([0.0, 0.0, 1.0])),
        ("z", [0.0, 0.0, 1.0], "+x (roll)",  np.array([1.0, 0.0, 0.0])),
    ]

    all_passed = True
    results = []

    # Test both paths: "controllers" and "hands"
    for path in ["controllers", "hands"]:
        print(f"\n--- Testing Path: {path.upper()} ---")
        app.input_mode = path

        for side in ["left", "right"]:
            R_0 = app.robot_home_rot_l if side == "left" else app.robot_home_rot_r

            for ax_name, ax_vec, exp_desc, exp_axis in xr_axes:
                # Reset to home before each test
                app.cur_target_rot_l = app.robot_home_rot_l.copy()
                app.cur_target_rot_r = app.robot_home_rot_r.copy()
                app.cur_target_pos_l = ee_home["left"]["pos"].copy()
                app.cur_target_pos_r = ee_home["right"]["pos"].copy()

                # Step 1: Feed synthetic neutral
                q_neutral = [1.0, 0.0, 0.0, 0.0]
                p_wrist_neutral = [-0.51262, 1.27804, -0.32257] if side == "left" else [0.51262, 1.27804, -0.32257]
                if path == "controllers":
                    pkt_zero = {"pos": ee_home[side]["pos"].tolist(), "quat": q_neutral, "grip": True, "trigger": True}
                    pkt_dict = {"cmd": "xr_controllers", side: pkt_zero, ("right" if side == "left" else "left"): None}
                else:
                    pkt_zero = {"tracking": True, "confidence": 1.0, "wrist": {"pos": p_wrist_neutral, "quat": q_neutral}, "joints": []}
                    pkt_dict = {"cmd": "xr_handtrack", side: pkt_zero, ("right" if side == "left" else "left"): None}

                for _ in range(40):
                    app._handle_panel_command(pkt_dict)
                    app._solve_arm_ik(side)
                    app.env.step()

                # Step 2: Feed +30 deg rotation about XR axis
                r_delta = R.from_rotvec(np.radians(30) * np.array(ax_vec))
                q_rot = r_delta.as_quat() # x, y, z, w
                q_pkt = [float(q_rot[3]), float(q_rot[0]), float(q_rot[1]), float(q_rot[2])]

                if path == "controllers":
                    pkt_step = {"pos": ee_home[side]["pos"].tolist(), "quat": q_pkt, "grip": True, "trigger": True}
                    pkt_cmd = {"cmd": "xr_controllers", side: pkt_step, ("right" if side == "left" else "left"): None}
                else:
                    pkt_step = {"tracking": True, "confidence": 1.0, "wrist": {"pos": p_wrist_neutral, "quat": q_pkt}, "joints": []}
                    pkt_cmd = {"cmd": "xr_handtrack", side: pkt_step, ("right" if side == "left" else "left"): None}

                # Wait for settle by continuously feeding packet and stepping sim
                for _ in range(80):
                    app._handle_panel_command(pkt_cmd)
                    app._solve_arm_ik(side)
                    app.env.step()

                # Read end-effector orientation in robot base frame
                ee_now = app.env.get_ee_poses()[side]["rot"]
                R_diff = ee_now @ R_0.T
                rv = R.from_matrix(R_diff).as_rotvec()
                angle_deg = float(np.degrees(np.linalg.norm(rv)))
                meas_axis = rv / np.linalg.norm(rv) if np.linalg.norm(rv) > 1e-6 else np.zeros(3)

                dot = float(np.clip(np.dot(meas_axis, exp_axis), -1.0, 1.0))
                axis_err_deg = float(np.degrees(np.arccos(dot)))

                # Check saturated joints
                qpos = app.env.get_left_arm_qpos() if side == "left" else app.env.get_right_arm_qpos()
                limits = app.ik_left.limits if side == "left" else app.ik_right.limits
                sat_joints = [i + 1 for i in range(7) if abs(qpos[i] - limits[i, 0]) < 0.02 or abs(qpos[i] - limits[i, 1]) < 0.02]

                # PASS criteria: axis error < 10 deg, angle within 25..35 deg
                passed = (axis_err_deg < 10.0 and 25.0 <= angle_deg <= 35.0)
                if not passed:
                    all_passed = False

                status = "PASS" if passed else "FAIL"
                results.append((path, side, ax_name, exp_desc, meas_axis, angle_deg, axis_err_deg, sat_joints, status))

    # Print Table
    print("\n" + "=" * 90)
    print(f"{'Path':<12} | {'Side':<5} | {'Input':<5} | {'Expected Robot Axis':<19} | {'Measured Axis':<22} | {'Angle':<7} | {'Err':<6} | {'Status'}")
    print("-" * 90)
    for path, side, inp, exp, meas, ang, err, sat, st in results:
        meas_str = f"[{meas[0]:+.2f}, {meas[1]:+.2f}, {meas[2]:+.2f}]"
        print(f"{path:<12} | {side:<5} | {inp:<5} | {exp:<19} | {meas_str:<22} | {ang:5.1f}° | {err:4.1f}° | {st}")
    print("=" * 90)

    # Position test (up / sideways / forward-back) within 5 mm
    print("\n--- Testing Position Invariance (Left Arm) ---")
    pos_0 = ee_home["left"]["pos"].copy()
    app.cur_target_pos_l = pos_0 + np.array([0.05, 0.05, 0.05])
    for _ in range(50):
        app._solve_arm_ik("left")
        app.env.step()
    pos_1 = app.env.get_ee_poses()["left"]["pos"]
    pos_err = np.linalg.norm(pos_1 - app.cur_target_pos_l)
    pos_pass = pos_err <= 0.005
    print(f"Position test: target delta norm={np.linalg.norm(app.cur_target_pos_l - pos_0):.3f} m, tracking err={pos_err*1000:.2f} mm => {'PASS' if pos_pass else 'FAIL'}")

    if not all_passed or not pos_pass:
        print("\n[RESULT] ORIENTATION AXIS TEST FAILED!")
        sys.exit(1)
    else:
        print("\n[RESULT] ALL ORIENTATION AXIS TESTS PASSED!")
        sys.exit(0)


if __name__ == "__main__":
    run_test()
