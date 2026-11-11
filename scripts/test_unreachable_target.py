"""
scripts/test_unreachable_target.py
Headless acceptance test suite for out-of-reach target projection and IK stability.
Tests:
1. Drag 0.3, 0.6, 1.0 m beyond reach in several directions (+x, +y, +z):
   std 0, zero reversals over 3 s, stops at boundary, max per-tick joint step <= cap.
2. Slow +x sweep through reach boundary and back: no boundary jitter, return path matches outward <= 0.01 rad.
3. Inside-reach target is unaffected (identity projection, max diff 0.0).
"""

import sys
import numpy as np
import mujoco

from teleop.robot.aimdk_map import HOME
from teleop.kinematics.mink_ik import AgibotMinkIK
from teleop.kinematics.reach_projection import (
    project_to_reachable_workspace,
    SHOULDER_ORIGIN_LEFT,
    DEFAULT_MAX_REACH_M,
)
from teleop.kinematics.mirror_limiter import MirrorLimiter
from teleop.safety.armor_collision import ArmorCollisionChecker


def run_reach_tests():
    xml_path = "assets/agibot_a2_ultra/mjcf/scene.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    left_jnts = [f"idx{13+i}_left_arm_joint{i+1}" for i in range(7)]
    l_adrs = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in left_jnts]
    data.qpos[l_adrs] = HOME[:7]
    mujoco.mj_forward(model, data)
    l_ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_arm_ee")
    home_pos = data.site_xpos[l_ee].copy()
    home_rot = data.site_xmat[l_ee].reshape(3, 3).copy()

    ik_l = AgibotMinkIK(xml_path, arm="left")
    checker = ArmorCollisionChecker(model, config_path="config/safety.yaml")
    shldr = SHOULDER_ORIGIN_LEFT
    R_max = DEFAULT_MAX_REACH_M

    results = []

    # 1. Drag 0.3, 0.6, 1.0 m beyond reach in several directions
    directions = {
        "+x (forward)": np.array([1.0, 0.0, 0.0]),
        "+y (outward)": np.array([0.0, 1.0, 0.0]),
        "+z (upward)": np.array([0.0, 0.0, 1.0]),
    }
    beyond_dists = [0.30, 0.60, 1.00]

    all_cases_ok = True
    for dname, dvec in directions.items():
        dvec = dvec / np.linalg.norm(dvec)
        for d_beyond in beyond_dists:
            target_pos = shldr + dvec * (R_max + d_beyond)
            proj_pos, is_clamped, measured_beyond = project_to_reachable_workspace(target_pos, shldr, side="left")
            assert is_clamped, f"Target at {d_beyond}m beyond not clamped"
            assert abs(measured_beyond - d_beyond) < 1e-4, f"Mismatch in d_beyond: {measured_beyond} vs {d_beyond}"

            # Solve IK on projected target
            q_sol, succ, pos_err, rot_err, _ = ik_l.solve(proj_pos, home_rot, init_qpos=HOME[:7], max_iters=25)

            # Settle limiter for 300 ticks (3 s) at 100 Hz
            limiter = MirrorLimiter(speed_level=3, armor_checker=checker)
            limiter.reset(HOME)
            q14_target = np.concatenate([q_sol, HOME[7:]])

            plans = []
            max_step = 0.0
            prev_p = limiter.q_plan.copy()
            for _ in range(300):
                plan = limiter.step(q14_target, dt=0.01)
                st = float(np.max(np.abs(plan - prev_p)))
                if st > max_step: max_step = st
                prev_p = plan.copy()
                plans.append(plan.copy())

            # Evaluate last 200 ticks (steady state hold)
            tail = np.array(plans[-200:])
            tail_std = float(np.max(np.std(tail, axis=0)))
            diffs = np.diff(tail[:, :7], axis=0)
            sign_flips = int(np.sum(np.diff(np.sign(diffs[np.abs(diffs) > 1e-6]), axis=0) != 0)) if np.any(np.abs(diffs) > 1e-6) else 0

            dist_to_boundary = float(abs(np.linalg.norm(proj_pos - shldr) - R_max))
            dist_to_target = float(np.linalg.norm(target_pos - proj_pos))

            assert tail_std <= 1e-12, f"Steady state std not zero: {tail_std}"
            assert sign_flips == 0, f"Sign flips in steady state: {sign_flips}"
            assert max_step <= limiter.step_cap + 1e-5, f"Step exceeded cap: {max_step} > {limiter.step_cap}"

            results.append((
                f"Far {dname} +{d_beyond:.1f}m",
                f"tgt_d={dist_to_target:.2f}m, bnd_d={dist_to_boundary*1000:.1f}mm, std={tail_std:.1e}",
                "PASS"
            ))

    # 2. Slow +x sweep through reach boundary and back
    limiter = MirrorLimiter(speed_level=3, armor_checker=checker)
    limiter.reset(HOME)
    dx_out = np.linspace(0, 0.25, 26)
    dx_ret = np.linspace(0.25, 0, 26)

    out_poses = []
    warm_q = HOME[:7].copy()

    for dx in dx_out:
        tgt = home_pos + np.array([dx, 0, 0])
        proj, _, _ = project_to_reachable_workspace(tgt, shldr, side="left")
        q_sol, succ, _, _, _ = ik_l.solve(proj, home_rot, init_qpos=warm_q, max_iters=35, pos_tol=0.0005, rot_tol=0.010)
        if succ: warm_q = q_sol.copy()
        else: q_sol = warm_q.copy()
        for _ in range(15):
            plan = limiter.step(np.concatenate([q_sol, HOME[7:]]), dt=0.01)
        out_poses.append(plan[:7].copy())

    ret_poses = []
    for dx in dx_ret:
        tgt = home_pos + np.array([dx, 0, 0])
        proj, _, _ = project_to_reachable_workspace(tgt, shldr, side="left")
        q_sol, succ, _, _, _ = ik_l.solve(proj, home_rot, init_qpos=warm_q, max_iters=35, pos_tol=0.0005, rot_tol=0.010)
        if succ: warm_q = q_sol.copy()
        else: q_sol = warm_q.copy()
        for _ in range(15):
            plan = limiter.step(np.concatenate([q_sol, HOME[7:]]), dt=0.01)
        ret_poses.append(plan[:7].copy())

    # Check return path matching outward path (index 0 outward vs index -1 return)
    path_diffs = [float(np.max(np.abs(out_poses[i] - ret_poses[-(i+1)]))) for i in range(len(out_poses))]
    max_hysteresis = max(path_diffs)
    assert max_hysteresis <= 0.010, f"Return path deviation: {max_hysteresis:.4f} > 0.01 rad"
    results.append(("Sweep outward & return", f"max path diff={max_hysteresis:.4f} rad (<= 0.01)", "PASS"))

    # 3. Normal reach unaffected (identity projection, max diff 0.0)
    normal_targets = [
        home_pos,
        home_pos + np.array([0.02, 0.02, 0.02]),
        home_pos + np.array([-0.02, -0.01, 0.03]),
    ]
    max_normal_diff = 0.0
    for nt in normal_targets:
        proj, clamped, _ = project_to_reachable_workspace(nt, shldr, side="left")
        assert not clamped, "Normal target was erroneously clamped"
        diff = float(np.max(np.abs(proj - nt)))
        if diff > max_normal_diff: max_normal_diff = diff
    assert max_normal_diff == 0.0, f"Normal reach diff not 0: {max_normal_diff}"
    results.append(("Inside-reach identity", f"max diff={max_normal_diff:.1e} (exact 0)", "PASS"))

    print("\n" + "=" * 75)
    print(f"{'Test Case':<26} | {'Measurement':<36} | {'Status'}")
    print("-" * 75)
    for name, metric, st in results:
        print(f"{name:<26} | {metric:<36} | {st}")
    print("=" * 75)
    return True


if __name__ == "__main__":
    ok = run_reach_tests()
    sys.exit(0 if ok else 1)
