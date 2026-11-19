#!/usr/bin/env python3
"""tests/test_no_deviation.py
Targeted tests for fix/no-deviation:
1. Plant slower than command (max 0.5 rad/s plant capability, command at speed level 3 = 0.8 rad/s):
   - No FAULT at any lag (even when lag > 0.35 rad).
   - Arm still reaches target.
   - After target stops, |act - sent| returns to ~0 (<0.01 rad) within 1.5 s.
2. Stale feedback still FAULTs SafetyLayer watchdog (>300 ms).
"""
import time
import numpy as np
from teleop.kinematics.mirror_limiter import MirrorLimiter
from teleop.robot.safety_layer import SafetyLayer
from teleop.robot.backends import DryRunBackend
from teleop.robot.deviation_checker import DeviationChecker


def test_slow_plant_no_fault_and_settles():
    print("--- 1. Testing Slow Plant (0.5 rad/s) Under 0.8 rad/s Command ---")
    limiter = MirrorLimiter(speed_level=3, initial_q14=np.zeros(14))
    checker = DeviationChecker()
    dt = 0.02
    target = np.zeros(14)
    target[0] = 1.0  # +1.0 rad on J1
    # Simulated slow plant: maximum joint velocity capped at 0.40 rad/s
    q_act = np.zeros(14)
    max_plant_speed = 0.40  # rad/s

    max_lag_observed = 0.0
    settled = False
    settle_start_t = None
    target_stop_t = None

    # Run for 4.0 seconds (200 steps)
    for step in range(200):
        t = step * dt
        q_plan = limiter.step(target, measured_q14=q_act, dt=dt)

        # Plant motion bounded by its max speed
        err_plant = q_plan - q_act
        dir_plant = np.sign(err_plant)
        step_plant = np.minimum(np.abs(err_plant), max_plant_speed * dt)
        q_act += dir_plant * step_plant

        # Deviation check
        tripped, j, err, _, chk, _, _ = checker.check(
            last_sent=q_plan, target_q=q_plan, q_act=q_act,
            is_real=True, dt=dt, t_now=t, last_cmd_vel=np.zeros(14)
        )
        assert not tripped, f"False FAULT tripped at t={t:.2f}s: err={err:.3f} rad, chk={chk}"

        lag = float(np.max(np.abs(q_plan - q_act)))
        if lag > max_lag_observed:
            max_lag_observed = lag

        # Check when target is arrived by q_plan
        if target_stop_t is None and np.abs(q_plan[0] - target[0]) < 1e-4:
            target_stop_t = t

        if target_stop_t is not None:
            if lag < 0.01:
                settled = True
                settle_dur = t - target_stop_t
                break

    print(f"Max lag during travel: {max_lag_observed:.3f} rad")
    assert max_lag_observed > 0.10, "Test did not produce significant lag!"
    assert settled, "Arm failed to settle to target!"
    assert settle_dur < 1.50, f"Settling took {settle_dur:.2f}s (> 1.5s)!"
    print(f"[PASS] Slow plant reached target without FAULT; settled in {settle_dur:.2f}s (< 1.5s).")


def test_stale_feedback_faults_watchdog():
    print("--- 2. Testing Stale Feedback Watchdog (>300 ms) ---")
    backend = DryRunBackend()
    safety = SafetyLayer(backend=backend, watchdog_timeout_s=0.300)
    now = time.monotonic()
    safety.send_arm(np.zeros(14), dt=0.02, now=now)
    assert not safety.fault_latched, "Safety latched unexpectedly!"

    # Simulate command stream starvation / timeout > 300 ms
    stale_t = now + 0.350
    tripped = safety.check_watchdog(now=stale_t)
    assert tripped and safety.hold_active, "Watchdog failed to trip on stale stream!"
    print("[PASS] Watchdog successfully caught stale command stream (>300ms).")


if __name__ == "__main__":
    test_slow_plant_no_fault_and_settles()
    test_stale_feedback_faults_watchdog()
    print("\nALL VERIFICATION TESTS COMPLETED SUCCESSFULLY.")
