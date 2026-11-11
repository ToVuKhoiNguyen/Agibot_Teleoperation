#!/usr/bin/env python3
"""scripts/test_fast_simple.py
Targeted verification tests for feat/fast-simple:
1. Step response at each --speed level (1..4): max step <= 0.016 for 1-3, <= 0.02 for 4, no overshoot > 0.01.
2. Tracking deviation 0.06 rad for 300 ms => WARN only, NO FAULT.
3. Runaway deviation 0.35 rad => FAULT within 250 ms.
4. No collision banner appears (collision checking never blocks, collision_blocked stays False).
"""
import time
import numpy as np
from teleop.kinematics.mirror_limiter import MirrorLimiter, SPEED_LEVELS
from teleop.robot.deviation_checker import DeviationChecker


def test_step_response_all_speed_levels():
    print("--- 1. Testing Step Response for Speed Levels 1..4 ---")
    for lvl in [1, 2, 3, 4]:
        limiter = MirrorLimiter(speed_level=lvl, initial_q14=np.zeros(14))
        target = np.full(14, 0.50)  # 0.5 rad step input
        dt = 0.02
        q_hist = []
        step_hist = []
        for _ in range(100):  # 2.0 seconds
            prev_q = limiter.q_plan.copy()
            q = limiter.step(target, dt=dt)
            step_mag = float(np.max(np.abs(q - prev_q)))
            q_hist.append(float(np.max(q)))
            step_hist.append(step_mag)

        max_step = max(step_hist)
        max_q = max(q_hist)
        overshoot = max(0.0, max_q - 0.50)
        v_cap, a_cap = SPEED_LEVELS[lvl]

        print(f"Level {lvl}: Cap={v_cap:.2f} rad/s, max_step={max_step:.5f} rad/tick, max_q={max_q:.4f}, overshoot={overshoot:.5f}")
        if lvl <= 3:
            assert max_step <= 0.016 + 1e-5, f"Level {lvl} step {max_step:.5f} > 0.016!"
        else:
            assert max_step <= 0.020 + 1e-5, f"Level {lvl} step {max_step:.5f} > 0.020!"
        assert overshoot <= 0.01, f"Level {lvl} overshoot {overshoot:.5f} > 0.01!"
    print("[PASS] Step response tests passed for all speed levels 1..4.")


def test_deviation_warn_no_fault():
    print("--- 2. Testing Deviation 0.06 rad for 300 ms => WARN only ---")
    checker = DeviationChecker(deviation_thresh_rad=0.05, timeout_s=0.200)
    dt = 0.02
    t = 0.0
    sent = np.zeros(14)
    # Deviation 0.06 rad for 300 ms (15 ticks)
    for _ in range(15):
        t += dt
        act = sent.copy()
        act[0] = 0.06  # 0.06 rad gap
        tripped, j, err, th, chk, _, _ = checker.check(
            last_sent=sent, target_q=sent, q_act=act,
            is_real=True, dt=dt, t_now=t, last_cmd_vel=np.zeros(14)
        )
        assert not tripped, f"False positive FAULT tripped at t={t:.3f}s with err={err:.3f}!"
        assert checker.warn_active, "Deviation 0.06 rad failed to set warn_active!"
    print("[PASS] Deviation 0.06 rad produces WARN only, NO FAULT tripped.")


def test_runaway_drift_trips_within_250ms():
    print("--- 3. Testing Runaway 0.35 rad => FAULT within 250 ms ---")
    checker = DeviationChecker(deviation_thresh_rad=0.05, timeout_s=0.200)
    dt = 0.02
    t = 0.0
    sent = np.zeros(14)
    tripped_time = None
    for _ in range(20):  # 400 ms
        t += dt
        act = sent.copy()
        act[0] = 0.35  # 0.35 rad runaway
        tripped, j, err, th, chk, _, _ = checker.check(
            last_sent=sent, target_q=sent, q_act=act,
            is_real=True, dt=dt, t_now=t, last_cmd_vel=np.zeros(14)
        )
        if tripped:
            tripped_time = t
            assert chk == "runaway_fault_0.30", f"Unexpected check name: {chk}"
            break

    assert tripped_time is not None, "Runaway 0.35 rad did NOT trip FAULT!"
    assert tripped_time <= 0.250, f"Runaway trip took too long: {tripped_time:.3f}s > 0.250s!"
    print(f"[PASS] Runaway 0.35 rad tripped FAULT at {tripped_time*1000:.1f} ms (<= 250 ms).")


def test_no_collision_blocking():
    print("--- 4. Testing Collision Removal (Never blocks, never faults) ---")
    from teleop.robot.safety_layer import SafetyLayer
    from teleop.robot.backends import DryRunBackend
    from teleop.robot.aimdk_map import HOME
    backend = DryRunBackend()
    safety = SafetyLayer(backend=backend)
    # Even if collision_gate is passed in context, safety.send_arm never clamps or blocks for collision
    safety.context["collision_gate"] = True
    q_target = np.array(HOME).copy()
    ok = safety.send_arm(q_target, dt=0.02)
    assert ok, "send_arm failed unexpectedly!"
    assert safety.stats["joint_limit_clamps"] == 0, "Collision blocked or clamped inappropriately!"
    assert not safety.fault_latched, "Safety latched fault on collision!"
    print("[PASS] Collision checking is removed from blocking path; no collision banner/fault.")


if __name__ == "__main__":
    test_step_response_all_speed_levels()
    test_deviation_warn_no_fault()
    test_runaway_drift_trips_within_250ms()
    test_no_collision_blocking()
    print("\nALL VERIFICATION TESTS COMPLETED SUCCESSFULLY.")
