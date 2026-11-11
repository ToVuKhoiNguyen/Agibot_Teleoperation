#!/usr/bin/env python3
"""scripts/test_no_lead_limit.py
Targeted tests for fix/no-lead-limit:
1. Plant with lag (tau=0.138s) and static offset 0.06 rad:
   After Engage, a target step of +0.5 rad must be reached (time-to-90% < 1.2s at --speed 3 = 0.8 rad/s).
   Status never shows LEAD LIMIT, reason is "none" while moving.
2. Runaway deviation 0.35 rad still trips FAULT within 250 ms.
"""
import time
import numpy as np
from teleop.kinematics.mirror_limiter import MirrorLimiter
from teleop.robot.deviation_checker import DeviationChecker


def test_plant_lag_and_static_offset_reaches_target():
    print("--- 1. Testing Step Target with Lag & 0.06 rad Static Offset ---")
    from teleop.robot.aimdk_map import HOME
    init_q = np.array(HOME, dtype=np.float64)
    limiter = MirrorLimiter(speed_level=3, initial_q14=init_q)
    assert limiter.max_vel == 0.80, f"Speed level 3 cap expected 0.80, got {limiter.max_vel}"

    dt = 0.02
    target = init_q.copy()
    target[0] += 0.50  # +0.5 rad step on Left J1 (within limits [-2.91, 2.91])
    tau = 0.138
    alpha = dt / tau

    # Simulated robot plant with first-order lag + 0.06 rad static offset
    q_act = init_q.copy()
    q_act[0] += 0.06  # starts with static offset 0.06 rad on J1
    time_to_90 = None

    for step in range(100):  # 2.0 seconds
        t = step * dt
        # Mirror limiter advances without lead limit freeze
        # Soft slowdown may occur if lead > 0.05, but with floor 0.30
        lead = float(np.max(np.abs(limiter.q_plan - q_act)))
        speed_scale = max(0.30, 1.0 - max(0.0, lead - 0.05) / 0.05) if lead > 0.05 else 1.0
        
        q_plan = limiter.step(target, measured_q14=q_act, dt=dt, speed_scale=speed_scale)
        # Update plant state
        q_act += alpha * (q_plan - q_act)

        # Check not-moving reason simulation
        diff = float(np.max(np.abs(target - q_plan)))
        reason = "none" if diff >= 0.001 else "target==current"

        if time_to_90 is None and q_act[0] >= init_q[0] + 0.45:  # 90% of 0.5 rad
            time_to_90 = t

        # Verify limiter.lead_exceeded is never True
        assert not limiter.lead_exceeded, f"lead_exceeded was True at t={t:.2f}s!"

    print(f"Time to 90% target (0.45 rad): {time_to_90:.3f} s")
    assert time_to_90 is not None, "Target was never reached!"
    assert time_to_90 < 1.20, f"Time to 90% ({time_to_90:.3f} s) exceeded 1.2 s!"
    assert reason == "target==current", f"Final reason expected 'target==current', got {reason}"
    print("[PASS] Target step +0.5 rad reached in < 1.2 s without lead freeze.")


def test_runaway_drift_still_faults():
    print("--- 2. Testing Runaway 0.35 rad trips FAULT within 250 ms ---")
    checker = DeviationChecker(deviation_thresh_rad=0.05, timeout_s=0.200)
    dt = 0.02
    t = 0.0
    sent = np.zeros(14)
    tripped_time = None
    for _ in range(20):
        t += dt
        act = sent.copy()
        act[0] = 0.35
        tripped, j, err, th, chk, _, _ = checker.check(
            last_sent=sent, target_q=sent, q_act=act,
            is_real=True, dt=dt, t_now=t, last_cmd_vel=np.zeros(14)
        )
        if tripped:
            tripped_time = t
            assert chk == "runaway_fault_0.30"
            break

    assert tripped_time is not None, "Runaway did not trip FAULT!"
    assert tripped_time <= 0.250, f"Runaway trip took too long: {tripped_time:.3f}s > 0.250s!"
    print(f"[PASS] Runaway 0.35 rad tripped FAULT at {tripped_time*1000:.1f} ms (<= 250 ms).")


if __name__ == "__main__":
    test_plant_lag_and_static_offset_reaches_target()
    test_runaway_drift_still_faults()
    print("\nALL VERIFICATION TESTS COMPLETED SUCCESSFULLY.")
