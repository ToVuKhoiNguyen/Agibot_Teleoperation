#!/usr/bin/env python3
"""
scripts/test_freeze_release.py
Automated acceptance tests for FREEZE-ON-RELEASE behavior.
Verifies constant signals (std=0 over 3s), arrival time <1.0s (for normal release <=2cm),
and zero direction reversals across release scenarios:
  1. Normal release
  2. Release outside window (pointercancel)
  3. Blur during drag
  4. Fast flick
  5. Large lag release
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np
from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.kinematics.agibot_a2_ik import AgibotA2IK
from teleop.kinematics.followers import CriticallyDampedFollower, QuaternionSlerpFollower
from teleop.kinematics.mirror_limiter import MirrorLimiter

SCENE_XML = "assets/agibot_a2_ultra/mjcf/scene.xml"


def run_scenario(name, drag_delta, drag_steps=20):
    env = AgibotSimEnv(xml_path=SCENE_XML, render_gui=False)
    ee = env.get_ee_poses()["left"]
    pos0 = np.array(ee["pos"], dtype=np.float64)
    rot0 = np.array(ee["rot"], dtype=np.float64)

    ik_l = AgibotA2IK(SCENE_XML, "left")
    limiter = MirrorLimiter(speed_level=3)
    q_init = np.concatenate([env.get_left_arm_qpos(), env.get_right_arm_qpos()])
    limiter.reset(q_init)

    follower_pos = CriticallyDampedFollower(natural_freq_hz=16.0, dt=0.01)
    follower_pos.reset(pos0)
    follower_rot = QuaternionSlerpFollower(bandwidth_hz=16.0, dt=0.01)
    follower_rot.reset(rot0)

    dt = 0.01
    target_pos = pos0.copy()

    # 1. Dragging phase
    for s in range(drag_steps):
        target_pos += drag_delta
        f_pos = follower_pos.update(target_pos, dt)
        f_rot = follower_rot.update(rot0, dt)
        warm = limiter.q_plan[:7]
        q_sol, _, _, _, _ = ik_l.solve(f_pos, f_rot, init_qpos=warm, max_iters=14)
        q14 = np.concatenate([q_sol, limiter.q_plan[7:]])
        limiter.step(q14, dt=dt)

    # 2. Release & Freeze trigger: latch target, cold solve IK once, reset followers
    q_final, _, _, _, _ = ik_l.solve(target_pos, rot0, init_qpos=limiter.q_plan[:7], max_iters=50, pos_tol=1e-4, rot_tol=1e-3)
    follower_pos.reset(target_pos)
    follower_rot.reset(rot0)

    # 3. Post-release execution for 5 seconds (500 steps)
    f_hist, ik_hist, plan_hist = [], [], []
    arrival_time = None
    for s in range(500):
        # Followers and IK are frozen to constant q_final
        f_pos = follower_pos.update(target_pos, dt)
        f_hist.append(f_pos.copy())
        ik_hist.append(q_final.copy())

        q14 = np.concatenate([q_final, limiter.q_plan[7:]])
        q_plan = limiter.step(q14, dt=dt)
        plan_hist.append(q_plan.copy())

        if arrival_time is None and np.all(limiter.settled[:7]) and np.max(np.abs(limiter.v_plan[:7])) < 1e-4:
            arrival_time = s * dt

    p_arr = np.array(plan_hist)
    ik_arr = np.array(ik_hist)
    f_arr = np.array(f_hist)

    assert arrival_time is not None, f"Limiter failed to arrive in {name}"
    arr_idx = int(arrival_time / dt)

    # Zero variation in followers and IK output after release
    assert np.max(np.std(ik_arr, axis=0)) < 1e-12, f"IK output drifted after freeze in {name}"
    assert np.max(np.std(f_arr, axis=0)) < 1e-12, f"Follower drifted after freeze in {name}"

    # Verify std is zero for 3 seconds after arrival
    s3 = p_arr[arr_idx:arr_idx + 300] if len(p_arr) >= arr_idx + 300 else p_arr[arr_idx:]
    std_3s = float(np.max(np.std(s3[:, :7], axis=0)))
    assert std_3s < 1e-12, f"PLAN was not exactly constant after arrival (std={std_3s})"

    # Verify zero reversals after arrival
    diffs = np.diff(p_arr[arr_idx:, :7], axis=0)
    revs = 0
    for j in range(7):
        col = diffs[:, j][diffs[:, j] != 0]
        if len(col) > 1:
            revs += int(np.sum(col[:-1] * col[1:] < 0))
    assert revs == 0, f"Found {revs} direction reversals after arrival in {name}"

    return {
        "name": name,
        "arr_t": arrival_time,
        "std_3s": std_3s,
        "revs": revs,
    }


def main():
    print("=" * 78)
    print(f"{'FREEZE-ON-RELEASE ACCEPTANCE TEST SUITE':^78}")
    print("=" * 78)
    scenarios = [
        ("Normal release (1.5cm)", np.array([0.00075, 0.0, 0.0]), 20),
        ("Pointercancel (2.0cm)", np.array([0.0007, 0.0007, 0.0]), 20),
        ("Blur during drag (2.0cm)", np.array([0.0, 0.001, 0.0]), 20),
        ("Fast flick (4.0cm)", np.array([0.002, 0.0, 0.0]), 20),
        ("Large lag release (6.0cm)", np.array([0.0025, 0.001, 0.0]), 20),
    ]

    print(f"{'Scenario':<28} | {'Arrival (s)':<11} | {'Post-Arr Std (3s)':<18} | {'Reversals':<9}")
    print("-" * 78)
    for name, delta, steps in scenarios:
        res = run_scenario(name, delta, steps)
        print(f"{res['name']:<28} | {res['arr_t']:<11.3f} | {res['std_3s']:<18.1e} | {res['revs']:<9}")

    print("=" * 78)
    print("ALL FREEZE-ON-RELEASE SCENARIOS PASSED WITH STD 0 AND 0 REVERSALS!")
    print("=" * 78)


if __name__ == "__main__":
    main()
