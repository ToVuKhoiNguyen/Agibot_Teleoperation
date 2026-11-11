#!/usr/bin/env python3
"""
scripts/test_soft_collision.py
Tests soft proxy self-collision scaling and clearance on postures.
Verifies HOME, T-pose, forward reach, crossing hands, safe stop > 5 mm, and free retreat.
"""

from pathlib import Path
import time
import numpy as np
import mujoco

from teleop.safety.proxy_collision import ProxyCollisionChecker
from teleop.kinematics.mirror_limiter import MirrorLimiter
from teleop.robot.aimdk_map import HOME


def main():
    xml_path = str(Path("assets/agibot_a2_ultra/mjcf/scene.xml").resolve())
    model = mujoco.MjModel.from_xml_path(xml_path)
    checker = ProxyCollisionChecker(model, config_path=Path("config/safety.yaml"))

    # 1. Clearance checks on normal reach postures
    d_home = checker.compute_min_distance(HOME)
    d_tpose = checker.compute_min_distance([0.0] * 14)
    q_fwd = [0.0, 0.0, 0.0, -1.2, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0]
    d_fwd = checker.compute_min_distance(q_fwd)

    print(f"[POSES] HOME: {d_home*1000:.1f} mm, T-pose: {d_tpose*1000:.1f} mm, Forward: {d_fwd*1000:.1f} mm")
    assert d_home > 0.05, f"HOME distance {d_home} <= 0.05 m"
    assert d_tpose > 0.05, f"T-pose distance {d_tpose} <= 0.05 m"
    assert d_fwd > 0.05, f"Forward distance {d_fwd} <= 0.05 m"

    # Scale check on HOME
    scale, d_m, dec = checker.evaluate_scale(HOME)
    assert scale == 1.0, f"Expected scale 1.0 for HOME, got {scale}"

    # 2. Benchmark evaluation performance (< 0.5 ms per check)
    t0 = time.perf_counter()
    N = 1000
    for _ in range(N):
        checker.compute_min_distance(HOME)
    cost_ms = (time.perf_counter() - t0) * 1000.0 / N
    print(f"[PERF] Proxy collision computation cost: {cost_ms:.3f} ms / check (budget < 0.5 ms)")
    assert cost_ms < 0.50, f"Computation cost {cost_ms} ms exceeds 0.5 ms budget!"

    # 3. Crossing test: hands moving toward each other
    q_home = np.array(HOME)
    q_close = np.array([0.744, 0.756, -0.005, -0.23, 0.861, 0.033, -0.319, -0.416, -1.461, 0.925, 1.435, -0.393, 0.49, -0.421])
    q_target_crossing = (1.0 - 0.68) * q_home + 0.68 * q_close

    limiter = MirrorLimiter(speed_level=3, initial_q14=HOME)
    d_prev = checker.compute_min_distance(limiter.q_plan)
    dt = 0.01
    min_scale = 1.0

    for _ in range(300):
        scale, d_curr, dec = checker.evaluate_scale(limiter.q_plan)
        if scale < min_scale:
            min_scale = scale
        limiter.step(q_target_crossing, dt=dt, speed_scale=scale)

    final_d = checker.compute_min_distance(limiter.q_plan)
    print(f"[CROSSING] Final distance: {final_d*1000:.1f} mm | Min scale: {min_scale:.2f}")
    assert final_d > 0.005, f"Stopped distance {final_d} <= 0.005 m!"
    assert min_scale <= 0.40, f"Scale did not slow down below 0.40 (got {min_scale})"

    # 4. Moving away test: retreat back to HOME
    scales_away = []
    for _ in range(300):
        scale, d_curr, dec = checker.evaluate_scale(limiter.q_plan)
        scales_away.append(scale)
        limiter.step(HOME, dt=dt, speed_scale=scale)

    returned_d = checker.compute_min_distance(limiter.q_plan)
    print(f"[RETREAT] Returned distance: {returned_d*1000:.1f} mm | Min retreat scale: {min(scales_away):.2f}")
    assert returned_d > 0.05, f"Retreat did not restore clearance: {returned_d}"
    assert min(scales_away) == 1.0, f"Moving away must never be slowed! Min scale was {min(scales_away)}"

    print("[SUCCESS] All soft self-collision tests passed!")


if __name__ == "__main__":
    main()
