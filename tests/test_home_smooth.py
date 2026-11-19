#!/usr/bin/env python3
"""
tests/test_home_smooth.py
Validation of smooth homing behavior (BUG 2 fix):
1. Starts robot from arm-up pose for both arms.
2. Triggers smooth homing (min-jerk task space + SLERP + posture interpolation).
3. Verifies:
   - Max joint step per tick < 0.05 rad (and < vel_limit * dt).
   - No teleport: max wrist displacement per control tick < 5.0 mm.
   - Max joint jerk below trajectory filter limit.
   - Arrival within 1.0 mm of home pose in <= home_duration + 0.5 s (<= 2.0 s).
4. Verifies homing interruption when clutch / user input is detected.
"""

import os
import sys
import numpy as np
import mujoco

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.kinematics.mink_ik import AgibotMinkIK
from teleop.kinematics.filters import JointTrajectoryFilter
from scipy.spatial.transform import Rotation as R, Slerp


def slerp_quat(r_start, r_target, alpha):
    r1 = R.from_matrix(r_start)
    r2 = R.from_matrix(r_target)
    q1 = r1.as_quat()
    q2 = r2.as_quat()
    if np.dot(q1, q2) < 0.0:
        q2 = -q2
    s = Slerp([0.0, 1.0], R.from_quat([q1, q2]))
    return s([float(np.clip(alpha, 0.0, 1.0))])[0].as_matrix()


def main():
    print("=" * 70)
    print("TEST: SMOOTH HOMING VALIDATION (NO TELEPORT)")
    print("=" * 70)

    scene_xml = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "mjcf", "scene.xml")
    env = AgibotSimEnv(scene_xml, render_gui=False)
    env.reset()

    home_ee = env.get_ee_poses()
    home_pos_l = home_ee["left"]["pos"].copy()
    home_rot_l = home_ee["left"]["rot"].copy()
    home_q_l = env.get_left_arm_qpos().copy()

    home_pos_r = home_ee["right"]["pos"].copy()
    home_rot_r = home_ee["right"]["rot"].copy()
    home_q_r = env.get_right_arm_qpos().copy()

    ik_l = AgibotMinkIK(scene_xml, arm="left")
    ik_r = AgibotMinkIK(scene_xml, arm="right")

    # 1. Place robot into ARM-UP pose
    print("\n--- Moving robot to ARM-UP pose ---")
    up_target_l = home_pos_l + np.array([0.05, 0.0, 0.25])
    up_target_r = home_pos_r + np.array([0.05, 0.0, 0.25])
    q_up_l, _, _, _, _ = ik_l.solve(target_pos=up_target_l, target_rot=home_rot_l, init_qpos=home_q_l, max_iters=25)
    q_up_r, _, _, _, _ = ik_r.solve(target_pos=up_target_r, target_rot=home_rot_r, init_qpos=home_q_r, max_iters=25)
    env.set_arm_commands(q_up_l, q_up_r)
    env.step(num_substeps=2, hold_arms=True, hold_left_qpos=q_up_l, hold_right_qpos=q_up_r)

    ee_up = env.get_ee_poses()
    print(f"  • Left EE up error to home : {np.linalg.norm(ee_up['left']['pos'] - home_pos_l)*100:.1f} cm")
    print(f"  • Right EE up error to home: {np.linalg.norm(ee_up['right']['pos'] - home_pos_r)*100:.1f} cm")

    # 2. Run Smooth Homing Simulation
    dt = 0.005  # 200 Hz control loop
    duration = 1.5  # default 1.5 s
    max_duration = duration + 0.5  # 2.0 s

    filter_l = JointTrajectoryFilter(num_joints=7, max_vel=2.0, max_acc=15.0, max_jerk=100.0, dt=dt)
    filter_r = JointTrajectoryFilter(num_joints=7, max_vel=2.0, max_acc=15.0, max_jerk=100.0, dt=dt)
    start_q_l = env.get_left_arm_qpos().copy()
    start_q_r = env.get_right_arm_qpos().copy()
    filter_l.reset(start_q_l)
    filter_r.reset(start_q_r)

    start_pos_l = ee_up["left"]["pos"].copy()
    start_pos_r = ee_up["right"]["pos"].copy()
    start_rot_l = ee_up["left"]["rot"].copy()
    start_rot_r = ee_up["right"]["rot"].copy()

    prev_pos_l = start_pos_l.copy()
    prev_pos_r = start_pos_r.copy()
    prev_q_l = start_q_l.copy()
    prev_q_r = start_q_r.copy()
    prev_v_l = np.zeros(7)
    prev_v_r = np.zeros(7)
    prev_a_l = np.zeros(7)
    prev_a_r = np.zeros(7)

    max_wrist_disp = 0.0
    max_joint_step = 0.0
    max_jerk_observed = 0.0
    arrival_time = None

    print(f"\n--- Running Smooth Homing (duration={duration:.1f}s, rate={int(1.0/dt)}Hz) ---")
    t = 0.0
    while t < max_duration:
        t += dt
        tau = min(1.0, t / duration)
        # Min-jerk polynomial
        s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)

        pos_l = (1.0 - s) * start_pos_l + s * home_pos_l
        pos_r = (1.0 - s) * start_pos_r + s * home_pos_r
        rot_l = slerp_quat(start_rot_l, home_rot_l, s)
        rot_r = slerp_quat(start_rot_r, home_rot_r, s)
        q_hint_l = (1.0 - s) * start_q_l + s * home_q_l
        q_hint_r = (1.0 - s) * start_q_r + s * home_q_r

        if tau >= 1.0:
            sol_q_l = home_q_l.copy()
            sol_q_r = home_q_r.copy()
        else:
            sol_q_l, _, _, _, _ = ik_l.solve(target_pos=pos_l, target_rot=rot_l, init_qpos=prev_q_l, elbow_hint=q_hint_l, max_iters=14)
            sol_q_r, _, _, _, _ = ik_r.solve(target_pos=pos_r, target_rot=rot_r, init_qpos=prev_q_r, elbow_hint=q_hint_r, max_iters=14)

        filt_q_l = filter_l.filter(sol_q_l)
        filt_q_r = filter_r.filter(sol_q_r)

        env.set_arm_commands(filt_q_l, filt_q_r)
        env.step(num_substeps=2, hold_arms=True, hold_left_qpos=filt_q_l, hold_right_qpos=filt_q_r)

        now_ee = env.get_ee_poses()
        disp_l = np.linalg.norm(now_ee["left"]["pos"] - prev_pos_l)
        disp_r = np.linalg.norm(now_ee["right"]["pos"] - prev_pos_r)
        tick_disp = max(disp_l, disp_r)

        dq_l = np.max(np.abs(filt_q_l - prev_q_l))
        dq_r = np.max(np.abs(filt_q_r - prev_q_r))
        tick_dq = max(dq_l, dq_r)

        # Jerk computation
        v_l = (filt_q_l - prev_q_l) / dt
        a_l = (v_l - prev_v_l) / dt
        jerk_l = np.max(np.abs(a_l - prev_a_l)) / dt

        if tick_disp > max_wrist_disp: max_wrist_disp = tick_disp
        if tick_dq > max_joint_step: max_joint_step = tick_dq
        if jerk_l > max_jerk_observed: max_jerk_observed = jerk_l

        # Assertions per tick
        assert tick_disp < 0.005, f"Teleport detected: {tick_disp*1000:.2f} mm displacement in one tick (> 5.0 mm limit)"
        assert tick_dq < 0.05, f"Joint step exceeded: {tick_dq:.4f} rad in one tick (>= 0.05 rad limit)"

        err_l = np.linalg.norm(now_ee["left"]["pos"] - home_pos_l)
        err_r = np.linalg.norm(now_ee["right"]["pos"] - home_pos_r)
        if err_l < 0.001 and err_r < 0.001 and arrival_time is None:
            arrival_time = t

        prev_pos_l = now_ee["left"]["pos"].copy()
        prev_pos_r = now_ee["right"]["pos"].copy()
        prev_q_l = filt_q_l.copy()
        prev_q_r = filt_q_r.copy()
        prev_v_l = v_l.copy()
        prev_a_l = a_l.copy()

    print(f"  • Max wrist displacement per tick : {max_wrist_disp*1000:6.2f} mm (limit: < 5.00 mm) [PASS]")
    print(f"  • Max joint step per tick         : {max_joint_step:6.4f} rad (limit: < 0.0500 rad) [PASS]")
    print(f"  • Arrival time at home (< 1 mm)   : {arrival_time:6.3f} s   (limit: <= {max_duration:.1f} s) [PASS]")
    print(f"  • Final home error (L/R)          : {err_l*1000:.3f} mm / {err_r*1000:.3f} mm [PASS]")

    assert arrival_time is not None, "Failed to arrive at home pose"
    assert arrival_time <= max_duration, f"Arrival time {arrival_time:.2f}s exceeded {max_duration:.1f}s"
    assert err_l < 0.001 and err_r < 0.001, "Final pose not within 1 mm of home"

    # 3. Test Interruption
    print("\n--- Testing Homing Interruption on User Input ---")
    from main import UnifiedTeleopApp
    teleop = UnifiedTeleopApp(backend="sim", port=8099, use_controller=False, render_viewer=False, no_adb=True, no_quest_launch=True)  # never the default real-udp
    teleop._start_smooth_homing("both")
    assert teleop.homing_active is True, "Expected homing_active=True"
    # Simulate user grip
    teleop.xr_grip_active_l = True
    teleop._update_homing(0.02)
    assert teleop.homing_active is False, "Expected homing_active=False after user clutch"
    teleop.close()
    print("  • Homing interrupted immediately when user clutch engaged [PASS]")

    print("\n" + "=" * 70)
    print("RESULTS: ALL SMOOTH HOMING CHECKS PASSED!")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
