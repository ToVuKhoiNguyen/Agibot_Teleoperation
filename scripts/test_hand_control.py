#!/usr/bin/env python3
"""
scripts/test_hand_control.py
Tests AGIBOT A2 Ultra dexterous hand control, mimic joint coupling,
scripted motion on all arm/hand joints in MuJoCo, and simulation reset.
"""

import os
import sys
import time
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting

SCENE_XML = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "mjcf", "scene.xml")


def test_hand_retargeting_logic():
    print("\n--- [A] Testing Hand Retargeting & Mimic Joint Coupling ---")
    retargeter = AgibotHandRetargeting(hand="left")

    # 1. Open hand
    open_cmd = retargeter.retarget_curls(np.zeros(5), pinch_strength=0.0)
    print(f"  - Open hand active joints (rad) : {np.round(open_cmd, 3)}")
    assert np.all(open_cmd < 0.1), "Open hand joints should be close to 0"

    # 2. Closed fist (transition over 15 steps)
    retargeter.reset()
    for _ in range(20):
        fist_cmd = retargeter.retarget_curls(np.ones(5), pinch_strength=0.5, dt=0.02)
    print(f"  - Closed fist active joints (rad): {np.round(fist_cmd, 3)}")
    assert fist_cmd[2] > 1.5, f"Index curl should be > 1.5, got {fist_cmd[2]}"
    assert fist_cmd[1] > 0.7, f"Thumb 1 curl should be > 0.7, got {fist_cmd[1]}"

    # 3. Mimic Coupling
    mimic = retargeter.compute_mimic_joints(fist_cmd)
    print(f"  - Computed mimic joints          : {[f'{k}={v:.2f}' for k, v in mimic.items()]}")
    assert np.isclose(mimic["thumb_2"], 0.40 * fist_cmd[1]), "Thumb 2 mimic ratio incorrect"
    assert np.isclose(mimic["thumb_3"], 0.60 * fist_cmd[1]), "Thumb 3 mimic ratio incorrect"
    assert np.isclose(mimic["index_2"], 1.00 * fist_cmd[2]), "Index 2 mimic ratio incorrect"

    # 4. Pinch
    retargeter.reset()
    pinch_curls = np.array([0.5, 0.5, 0.0, 0.0, 0.0])
    for _ in range(15):
        pinch_cmd = retargeter.retarget_curls(pinch_curls, pinch_strength=1.0, dt=0.02)
    print(f"  - Pinch grasp active joints (rad): {np.round(pinch_cmd, 3)}")
    assert pinch_cmd[0] > 0.5, "Thumb swing should be active during pinch"
    assert pinch_cmd[2] > 0.4 and pinch_cmd[3] < 0.2, "Index flexed while middle relaxed"
    print("[PASS] Hand retargeting and mimic coupling logic verified successfully!")
    return True


def test_scripted_mujoco_motion(duration_s: float = 2.0, gui: bool = False):
    print(f"\n--- [B] Testing Scripted Motion in MuJoCo ({duration_s} s) ---")
    env = AgibotSimEnv(SCENE_XML, render_gui=gui)

    ee_init = env.get_ee_poses()
    print(f"  - Initial Left EE pos : [{ee_init['left']['pos'][0]:+.3f}, {ee_init['left']['pos'][1]:+.3f}, {ee_init['left']['pos'][2]:+.3f}] m")
    print(f"  - Initial Right EE pos: [{ee_init['right']['pos'][0]:+.3f}, {ee_init['right']['pos'][1]:+.3f}, {ee_init['right']['pos'][2]:+.3f}] m")

    dt = 0.01
    total_steps = int(duration_s / dt)
    left_arm_init = env.get_left_arm_qpos()
    right_arm_init = env.get_right_arm_qpos()

    out_of_limits_count = 0
    start_time = time.time()

    for step in range(total_steps):
        t = step * dt
        phase = 2.0 * np.pi * (t / 1.5)

        # Arms: sinusoidal motion across joints
        cmd_l = left_arm_init.copy()
        cmd_l[0] += 0.25 * np.sin(phase)
        cmd_l[1] += 0.15 * np.sin(phase + 0.5)
        cmd_l[3] -= 0.25 * (0.5 + 0.5 * np.cos(phase))

        cmd_r = right_arm_init.copy()
        cmd_r[0] += 0.25 * np.sin(phase)
        cmd_r[1] -= 0.15 * np.sin(phase + 0.5)
        cmd_r[3] += 0.25 * (0.5 + 0.5 * np.cos(phase))

        # Hands: periodic open and close
        curl = float(0.6 * (1.0 - np.cos(phase)))
        t_swing = float(0.5 * (1.0 - np.cos(phase)))
        t_curl = float(0.3 * (1.0 - np.cos(phase)))
        hand_cmd = np.array([t_swing, t_curl, curl, curl, curl, curl], dtype=np.float64)

        env.set_arm_commands(cmd_l, cmd_r)
        env.set_hand_commands(hand_cmd, hand_cmd)
        env.step(num_substeps=5)

        # Check joint limits
        for jname in env.left_arm_joints + env.right_arm_joints + env.left_hand_active_joints + env.right_hand_active_joints:
            val = env.data.qpos[env.qpos_idx_map[jname]]
            lo, hi = env.joint_limits[jname]
            if val < lo - 0.05 or val > hi + 0.05:
                out_of_limits_count += 1

        if step % (total_steps // 3) == 0 or step == total_steps - 1:
            ee = env.get_ee_poses()
            lh = env.get_left_hand_qpos()
            print(f"  Step {step:3d}/{total_steps} (t={t:.2f}s) | "
                  f"Left EE: [{ee['left']['pos'][0]:+.3f}, {ee['left']['pos'][1]:+.3f}, {ee['left']['pos'][2]:+.3f}] | "
                  f"Index curl: {lh[2]:.2f} rad")

    elapsed = time.time() - start_time
    print(f"  Completed in {elapsed:.2f} s ({total_steps / elapsed:.1f} Hz rate).")
    print(f"  Joint limit violations: {out_of_limits_count}")

    # Test reset
    print("\n--- [C] Testing Environment Reset ---")
    env.reset()
    ee_after = env.get_ee_poses()
    diff_l = np.linalg.norm(ee_after["left"]["pos"] - ee_init["left"]["pos"])
    diff_r = np.linalg.norm(ee_after["right"]["pos"] - ee_init["right"]["pos"])
    print(f"  Left EE reset error : {diff_l * 1000:.2f} mm")
    print(f"  Right EE reset error: {diff_r * 1000:.2f} mm")

    passed = (out_of_limits_count == 0) and (diff_l < 0.005) and (diff_r < 0.005)
    env.close()
    return passed


def main():
    print("=" * 70)
    print("AGIBOT A2 ULTRA DEXTEROUS HAND AND ARM MOTION VALIDATION")
    print("=" * 70)

    pass_a = test_hand_retargeting_logic()
    pass_b = test_scripted_mujoco_motion(duration_s=1.5, gui=False)

    print("\n" + "=" * 70)
    if pass_a and pass_b:
        print("[PASS] PHASE 3 & 5 VALIDATION SUCCEEDED!")
        print("  - Hand retargeting (open/close/pinch) visibly and mathematically correct.")
        print("  - Scripted bilateral arm & hand motions ran smoothly in MuJoCo.")
        print("  - 0 joint limit violations; reset returns to nominal pose.")
    else:
        print("[FAIL] Hand control validation failed.")
    print("=" * 70 + "\n")
    sys.exit(0 if (pass_a and pass_b) else 1)


if __name__ == "__main__":
    main()
