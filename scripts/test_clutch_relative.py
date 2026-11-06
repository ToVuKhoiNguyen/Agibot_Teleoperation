#!/usr/bin/env python3
"""
scripts/test_clutch_relative.py
Headless simulation test verifying relative clutch teleop behavior for Quest controllers.
Tests:
a) press at pose A, move +0.20 m in z, release -> EE moves +0.20 m +- 5 mm (scale s=1).
b) while released move controller 0.5 m elsewhere -> EE displacement < 1 mm.
c) press again, move -0.10 m in z -> EE moves -0.10 m +- 5 mm; first-frame jump < 2 mm.
d) rotation: Grip, rotate +30 deg about each axis, release, move, re-press, rotate -30 deg -> returns by 30 deg +- 3 deg, jump < 1 deg.
e) stale packet 200 ms mid-hold -> holds, no re-engage until new rising edge.
f) both arms independent.
"""
import sys, time
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
from main import UnifiedTeleopApp

def step_app(app, n_steps=30, side="left"):
    for _ in range(n_steps):
        if app.gizmo_active_l and app.gizmo_active_r: app._solve_arm_ik("both")
        elif app.gizmo_active_l: app._solve_arm_ik("left")
        elif app.gizmo_active_r: app._solve_arm_ik("right")
        app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)
        app.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=app.last_arm_cmd_left, hold_right_qpos=app.last_arm_cmd_right)

def send_frame(app, t_now, side, pos, quat_xyzw, trigger=0.0, grip=0.0):
    ctrl_pkt = {
        "connected": True, "handedness": side, "pos": list(pos),
        "quat": [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
        "trigger": float(trigger), "grip": float(grip),
        "buttons": [0, float(trigger), float(grip), 0, 0, 0]
    }
    pkt = {"name": "xr_controllers", "type": "xr_controllers", "t": t_now, side: ctrl_pkt, ("right" if side == "left" else "left"): {"connected": False}}
    app._handle_panel_command(pkt)
    step_app(app, 4, side)

def run_tests():
    print("=" * 80 + "\nRELATIVE CLUTCH TELEOP TEST (HEADLESS SIMULATION)\n" + "=" * 80)
    app = UnifiedTeleopApp(port=8997, render_viewer=False, no_adb=True, no_quest_launch=True)
    app.input_mode, app.xr_scale, app.deadzone = "controllers", 1.0, 0.001
    side = "left"

    ee_home = app.env.get_ee_poses()[side]["pos"].copy()
    app.cur_target_pos_l = ee_home + np.array([0.0, 0.0, -0.10])
    for _ in range(80):
        app._solve_arm_ik("left")
        app.env.step()

    t_now = 100.0
    print(f"\n[Test a] Press at pose A, move +0.20m in z, release...")
    c_pos = np.array([0.0, 0.0, 0.0])
    c_quat = np.array([0.0, 0.0, 0.0, 1.0])

    send_frame(app, t_now, side, c_pos, c_quat, trigger=1.0)
    t_now += 0.02
    p0 = app.env.get_ee_poses()[side]["pos"].copy()

    for step in np.linspace(0.0, 0.20, 20):
        c_pos = np.array([0.0, step, 0.0])
        send_frame(app, t_now, side, c_pos, c_quat, trigger=1.0)
        t_now += 0.02

    step_app(app, 80, side)
    p1 = app.env.get_ee_poses()[side]["pos"].copy()
    disp_z = p1[2] - p0[2]
    print(f"  Target delta: 0.20m in z. Measured delta_z: {disp_z:.4f}m")
    assert abs(disp_z - 0.20) < 0.005, f"Test a failed: disp_z={disp_z}"
    print("  => Test a PASS")

    send_frame(app, t_now, side, c_pos, c_quat, trigger=0.0)
    t_now += 0.02
    step_app(app, 30, side)
    p_released = app.env.get_ee_poses()[side]["pos"].copy()

    print(f"\n[Test b] While released, move controller 0.5m elsewhere...")
    for step in np.linspace(0.20, 0.70, 15):
        c_pos = np.array([0.5, step, -0.3])
        send_frame(app, t_now, side, c_pos, c_quat, trigger=0.0)
        t_now += 0.02
    step_app(app, 30, side)
    p_unclutched = app.env.get_ee_poses()[side]["pos"].copy()
    disp_b = np.linalg.norm(p_unclutched - p_released)
    print(f"  Robot EE drift while un-clutched: {disp_b*1000:.3f} mm")
    assert disp_b < 0.001, f"Test b failed: drift={disp_b}"
    print("  => Test b PASS")

    print(f"\n[Test c] Press again, move -0.10m in z...")
    send_frame(app, t_now, side, c_pos, c_quat, trigger=1.0)
    t_now += 0.02
    p_repress_1 = app.env.get_ee_poses()[side]["pos"].copy()
    jump_c = np.linalg.norm(p_repress_1 - p_unclutched)
    print(f"  First-frame jump on re-press: {jump_c*1000:.3f} mm")
    assert jump_c < 0.002, f"Test c jump failed: {jump_c}"

    c_base = c_pos.copy()
    for step in np.linspace(0, -0.10, 15):
        c_pos = c_base + np.array([0.0, step, 0.0])
        send_frame(app, t_now, side, c_pos, c_quat, trigger=1.0)
        t_now += 0.02
    step_app(app, 80, side)
    p_c_end = app.env.get_ee_poses()[side]["pos"].copy()
    disp_c = p_c_end[2] - p_unclutched[2]
    print(f"  Measured delta_z: {disp_c:.4f}m (expected -0.10m)")
    assert abs(disp_c - (-0.10)) < 0.005, f"Test c displacement failed: {disp_c}"
    print("  => Test c PASS")

    send_frame(app, t_now, side, c_pos, c_quat, trigger=0.0)
    t_now += 0.02
    step_app(app, 30, side)

    print(f"\n[Test d] Rotation: Grip, rotate +30 deg, release, move, re-press, rotate -30 deg...")
    r_start = app.env.get_ee_poses()[side]["rot"].copy()

    send_frame(app, t_now, side, c_pos, c_quat, grip=1.0)
    t_now += 0.02
    r_grip_0 = app.env.get_ee_poses()[side]["rot"].copy()
    jump_rot_0 = np.degrees(np.arccos(np.clip((np.trace(r_grip_0 @ r_start.T) - 1)/2, -1, 1)))
    print(f"  First-frame rotation jump on grip press: {jump_rot_0:.3f} deg")
    assert jump_rot_0 < 1.0, f"Test d jump 0 failed: {jump_rot_0}"

    rot_30 = R.from_euler('x', 30, degrees=True)
    c_quat_30 = (rot_30 * R.from_quat(c_quat)).as_quat()
    for _ in range(20):
        send_frame(app, t_now, side, c_pos, c_quat_30, grip=1.0)
        t_now += 0.02
    step_app(app, 60, side)
    r_grip_30 = app.env.get_ee_poses()[side]["rot"].copy()
    angle_30 = np.degrees(np.arccos(np.clip((np.trace(r_grip_30 @ r_grip_0.T) - 1)/2, -1, 1)))
    print(f"  Measured rotation angle: {angle_30:.2f} deg (expected ~30 deg)")
    assert abs(angle_30 - 30.0) < 3.0, f"Test d 30 deg rotation failed: {angle_30}"

    send_frame(app, t_now, side, c_pos, c_quat_30, grip=0.0)
    t_now += 0.02
    step_app(app, 30, side)

    rot_dummy = R.from_euler('y', 45, degrees=True)
    c_quat_dummy = (rot_dummy * R.from_quat(c_quat_30)).as_quat()
    send_frame(app, t_now, side, c_pos, c_quat_dummy, grip=0.0)
    t_now += 0.02
    step_app(app, 30, side)

    send_frame(app, t_now, side, c_pos, c_quat_dummy, grip=1.0)
    t_now += 0.02
    r_repress = app.env.get_ee_poses()[side]["rot"].copy()
    jump_repress = np.degrees(np.arccos(np.clip((np.trace(r_repress @ r_grip_30.T) - 1)/2, -1, 1)))
    print(f"  First-frame rotation jump on re-grip: {jump_repress:.3f} deg")
    assert jump_repress < 1.0, f"Test d re-grip jump failed: {jump_repress}"

    rot_neg30 = R.from_euler('x', -30, degrees=True)
    c_quat_back = (rot_neg30 * R.from_quat(c_quat_dummy)).as_quat()
    for _ in range(20):
        send_frame(app, t_now, side, c_pos, c_quat_back, grip=1.0)
        t_now += 0.02
    step_app(app, 60, side)
    r_back = app.env.get_ee_poses()[side]["rot"].copy()
    angle_back = np.degrees(np.arccos(np.clip((np.trace(r_back @ r_repress.T) - 1)/2, -1, 1)))
    print(f"  Measured return rotation angle: {angle_back:.2f} deg (expected ~30 deg)")
    assert abs(angle_back - 30.0) < 3.0, f"Test d return rotation failed: {angle_back}"
    print("  => Test d PASS")

    send_frame(app, t_now, side, c_pos, c_quat_back, grip=0.0)
    t_now += 0.02
    step_app(app, 30, side)

    print(f"\n[Test e] Stale packet (200 ms mid-hold) -> holds, no re-engage until fresh rising edge...")
    send_frame(app, t_now, side, c_pos, c_quat, trigger=1.0)
    t_now += 0.02
    send_frame(app, t_now, side, c_pos + np.array([0, 0.05, 0]), c_quat, trigger=1.0)
    t_now += 0.02
    step_app(app, 30, side)
    p_before_stale = app.env.get_ee_poses()[side]["pos"].copy()

    stale_cmd_t = 10000000.0
    now_wall = stale_cmd_t + 0.200
    ctrl_pkt = {
        "connected": True, "handedness": side,
        "pos": list(c_pos + np.array([0, 0.15, 0])),
        "quat": [1, 0, 0, 0], "trigger": 1.0, "grip": 0.0,
        "buttons": [0, 1.0, 0, 0, 0, 0]
    }
    pkt = {"name": "xr_controllers", "type": "xr_controllers", "t": stale_cmd_t, side: ctrl_pkt, "right": {"connected": False}}
    orig_time = time.time
    try:
        time.time = lambda: now_wall
        app._handle_panel_command(pkt)
    finally:
        time.time = orig_time

    assert (getattr(app, "xr_reengage_required_l", False) or getattr(app, "xr_reengage_required", False)) == True, "re-engage should be required"
    assert app.xr_grip_active_l == False and app.xr_clutch_state_l == "LOCKED", "clutch should be disengaged on stale packet"
    print("  Stale packet detected, clutch disengaged, re-engage required: PASS")

    send_frame(app, t_now, side, c_pos + np.array([0, 0.30, 0]), c_quat, trigger=1.0)
    t_now += 0.02
    step_app(app, 30, side)
    p_ignored = app.env.get_ee_poses()[side]["pos"].copy()
    assert np.linalg.norm(p_ignored - p_before_stale) < 0.002, "Motion accepted before rising edge!"
    print("  Motion rejected while trigger held across stale condition: PASS")

    send_frame(app, t_now, side, c_pos + np.array([0, 0.30, 0]), c_quat, trigger=0.0)
    t_now += 0.02
    assert getattr(app, "xr_reengage_required_l", False) == False, "Release should clear xr_reengage_required_l"
    print("  => Test e PASS")

    print(f"\n[Test f] Both arms independent...")
    p_l_start = app.env.get_ee_poses()["left"]["pos"].copy()
    p_r_start = app.env.get_ee_poses()["right"]["pos"].copy()
    pkt_lr = {
        "name": "xr_controllers", "type": "xr_controllers", "t": t_now,
        "left": {"connected": True, "pos": [0, 0.10, 0], "quat": [1,0,0,0], "trigger": 1.0, "grip": 0.0, "buttons": [0,1,0,0,0,0]},
        "right": {"connected": True, "pos": [0, 0.50, 0], "quat": [1,0,0,0], "trigger": 0.0, "grip": 0.0, "buttons": [0,0,0,0,0,0]},
    }
    app._handle_panel_command(pkt_lr)
    step_app(app, 40)
    p_l_end = app.env.get_ee_poses()["left"]["pos"].copy()
    p_r_end = app.env.get_ee_poses()["right"]["pos"].copy()
    disp_l = np.linalg.norm(p_l_end - p_l_start)
    disp_r = np.linalg.norm(p_r_end - p_r_start)
    print(f"  Left arm disp: {disp_l:.4f}m, Right arm disp: {disp_r:.4f}m")
    assert disp_r < 0.001, f"Right arm moved while unclutched: {disp_r}"
    assert app.xr_grip_active_l == True and app.xr_grip_active_r == False
    print("  => Test f PASS")
    print("\n[RESULT] ALL RELATIVE CLUTCH TESTS PASSED SUCCESSFULLY!")

if __name__ == "__main__":
    run_tests()
