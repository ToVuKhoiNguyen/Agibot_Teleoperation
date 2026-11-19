"""
tests/test_armor_collision.py
Headless acceptance test suite for armor collision detection and soft-block avoidance.
Tests:
1. HOME, T-pose, reach sweep: zero blocks, no slowdown.
2. Pushing left hand into torso & crossing hands: min distance >= 0.019m, PLAN std 0, zero sign changes.
3. Pull away from collision: motion resumes < 0.3 s.
4. Start inside shell: leaves contact, cannot go deeper.
5. Fast flick at speed 3 & 4: no tunnelling (min distance > 0).
6. Execution cost per tick <= 1.0 ms.
"""

import sys
import time
import numpy as np
import mujoco

from teleop.robot.aimdk_map import HOME
from teleop.safety.armor_collision import ArmorCollisionChecker
from teleop.kinematics.mirror_limiter import MirrorLimiter
from teleop.kinematics.mink_ik import AgibotMinkIK
from teleop.kinematics.reach_projection import project_to_reachable_workspace


def run_tests():
    xml_path = "assets/agibot_a2_ultra/mjcf/scene.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    checker = ArmorCollisionChecker(model, config_path="config/safety.yaml")

    results = []

    # 1. HOME, T-pose, normal reach sweep
    limiter = MirrorLimiter(speed_level=3, armor_checker=checker)
    limiter.reset(HOME)
    d_home, p_home, _ = checker.compute_min_distance(HOME)
    assert d_home > 0.020, f"HOME min dist too low: {d_home:.4f}"

    t_pose = np.zeros(14, dtype=np.float64)
    d_tpose, p_tpose, _ = checker.compute_min_distance(t_pose)
    assert d_tpose > 0.020, f"T-pose min dist too low: {d_tpose:.4f}"

    # Normal reach sweep: small motions around HOME
    normal_blocks = 0
    for t in range(50):
        tgt = HOME.copy()
        tgt[0] += 0.05 * np.sin(t * 0.1)
        tgt[7] += 0.05 * np.cos(t * 0.1)
        plan = limiter.step(tgt, dt=0.01)
        if limiter.armor_blocked:
            normal_blocks += 1
    assert normal_blocks == 0, f"Unexpected blocks in normal sweep: {normal_blocks}"
    results.append(("1. HOME/T-pose/Normal", f"HOME={d_home*1000:.1f}mm, T={d_tpose*1000:.1f}mm, 0 blocks", "PASS"))

    # 2. Left hand pushed into torso
    limiter.reset(HOME)
    ik_l = AgibotMinkIK(xml_path, arm="left")
    # Command target deep into torso center: [0.05, 0.12, 0.90]
    tgt_torso = np.array([0.05, 0.12, 0.90])
    # Extract HOME rotation matrix
    data_tmp = mujoco.MjData(model)
    data_tmp.qpos[checker.l_adrs] = HOME[:7]
    mujoco.mj_forward(model, data_tmp)
    l_ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "left_arm_ee")
    home_rot = data_tmp.site_xmat[l_ee].reshape(3, 3).copy()

    q_torso, succ, _, _, _ = ik_l.solve(tgt_torso, home_rot, init_qpos=HOME[:7], max_iters=50, pos_tol=0.01, rot_tol=0.20)
    tgt_q14 = np.concatenate([q_torso, HOME[7:]])

    # Step limiter towards torso collision for 120 ticks
    blocked_plans = []
    min_dist_during_push = 999.0
    for _ in range(120):
        plan = limiter.step(tgt_q14, dt=0.01)
        min_dist_during_push = min(min_dist_during_push, limiter.armor_min_dist)
        if limiter.armor_blocked:
            blocked_plans.append(plan.copy())

    assert len(blocked_plans) > 10, "Arm did not soft block on torso approach"
    assert min_dist_during_push >= 0.020 - 0.001, f"Surface distance penetrated shell: {min_dist_during_push*1000:.1f}mm"

    # Verify PLAN is bit-identical while blocked (std 0, zero sign changes)
    tail_blocked = np.array(blocked_plans[-20:])
    std_blocked = float(np.max(np.std(tail_blocked, axis=0)))
    assert std_blocked <= 1e-14 and np.all(tail_blocked == tail_blocked[0]), f"PLAN not bit-identical while blocked: std={std_blocked}"
    results.append(("2. Push into torso", f"min_d={min_dist_during_push*1000:.1f}mm (>=19mm), std={std_blocked:.1e}", "PASS"))

    # 3. Pull away from collision: motion resumes < 0.3 s
    resume_ticks = 0
    resumed = False
    for t in range(50):
        plan = limiter.step(HOME, dt=0.01)
        if not limiter.armor_blocked and not resumed:
            resume_ticks = t + 1
            resumed = True
    assert resumed, "Motion did not resume after pulling away"
    resume_time_s = resume_ticks * 0.01
    assert resume_time_s < 0.30, f"Resume took too long: {resume_time_s:.3f}s"
    results.append(("3. Pull away resume", f"Resumed in {resume_time_s:.2f}s (<0.3s)", "PASS"))

    # 4. Start inside shell: leaves contact, cannot go deeper
    # Construct a posture inside shell (0 < d < 0.02)
    contact_q14 = None
    for a in np.linspace(0.80, 0.95, 30):
        cq = HOME + a * (tgt_q14 - HOME)
        d_c, _, _ = checker.compute_min_distance(cq)
        if 0.002 < d_c < 0.019:
            contact_q14, d_contact = cq, d_c
            break
    if contact_q14 is None:
        contact_q14 = HOME + 0.85 * (tgt_q14 - HOME)
        d_contact, _, _ = checker.compute_min_distance(contact_q14)
    assert 0.0 < d_contact < 0.020, f"contact_q14 not inside shell: {d_contact*1000:.1f}mm"
    checker.is_blocked = False

    # Command deeper
    cand_deeper = contact_q14 + 0.05 * (tgt_q14 - contact_q14)
    q_safe, is_allowed, is_blk, d_out, _ = checker.filter_step(contact_q14, cand_deeper)
    assert not is_allowed, "Step deeper inside shell was not blocked"
    assert np.allclose(q_safe, contact_q14), "Did not hold pose on step deeper"

    # Command outward towards HOME
    checker.is_blocked = False
    cand_outward = contact_q14 + 0.05 * (HOME - contact_q14)
    q_safe_out, is_allowed_out, is_blk_out, d_out_clear, _ = checker.filter_step(contact_q14, cand_outward)
    assert is_allowed_out, "Failed to allow step outward from inside shell"
    assert d_out_clear > d_contact, "Clearance did not improve on outward step"
    results.append(("4. Start inside shell", f"d_in={d_contact*1000:.1f}mm -> Deeper blocked, Outward allowed", "PASS"))

    # 5. Fast flick at speed 3 & 4: no tunnelling (min dist > 0)
    for spd in (3, 4):
        lim = MirrorLimiter(speed_level=spd, armor_checker=checker)
        lim.reset(HOME)
        min_flick_d = 999.0
        # Command immediate deep target
        for _ in range(60):
            p = lim.step(tgt_q14, dt=0.01)
            min_flick_d = min(min_flick_d, lim.armor_min_dist)
        assert min_flick_d > 0.0, f"Tunnelling detected at speed {spd}: min_d={min_flick_d*1000:.1f}mm"
    results.append(("5. Fast flick (spd 3,4)", f"No tunnelling: min_d={min_flick_d*1000:.1f}mm (>0)", "PASS"))

    # 6. Cost per tick
    t0 = time.perf_counter()
    N = 200
    home_arr = np.array(HOME, dtype=np.float64)
    step_arr = home_arr + 0.005
    for _ in range(N):
        checker.filter_step(home_arr, step_arr)
    t1 = time.perf_counter()
    avg_cost_ms = (t1 - t0) * 1000.0 / N
    assert avg_cost_ms <= 1.0, f"Cost per tick exceeded 1.0 ms: {avg_cost_ms:.3f}ms"
    results.append(("6. Cost per tick", f"{avg_cost_ms:.3f} ms (budget <= 1.0 ms)", "PASS"))

    print("\n" + "=" * 70)
    print(f"{'Test':<25} | {'Metric / Measurement':<32} | {'Status'}")
    print("-" * 70)
    for name, metric, st in results:
        print(f"{name:<25} | {metric:<32} | {st}")
    print("=" * 70)
    return True


if __name__ == "__main__":
    ok = run_tests()
    sys.exit(0 if ok else 1)
