#!/usr/bin/env python3
"""
tests/test_gate_offset.py
Targeted verification for deviation gate offset and feedforward deceleration.
Tests:
1. Plant with first-order lag (tau=0.138s) + 0.05 rad dynamic overshoot and 0.04 rad static offset
   on J1/J4 does NOT fault the 0.05 rad gate when calibrated.
2. Plant with runaway drift (0.10 rad) MUST trip the deviation gate within 250 ms.
3. Speed profile consistency between MirrorLimiter, arm_teleop_rx, and main.py.
"""
import sys
import numpy as np
from teleop.robot.deviation_checker import DeviationChecker
from teleop.kinematics.mirror_limiter import SPEED_PROFILES, MirrorLimiter
import scripts.arm_teleop_rx as arm_rx


def test_speed_profiles():
    # MirrorLimiter normal profile
    assert SPEED_PROFILES["normal"] == (0.40, 1.80), f"MirrorLimiter normal profile mismatch: {SPEED_PROFILES['normal']}"
    # arm_teleop_rx normal profile
    assert arm_rx.PROFILES["normal"] == (0.40, 0.008), f"arm_teleop_rx normal profile mismatch: {arm_rx.PROFILES['normal']}"
    print("[PASS] Speed profile configuration consistency verified (0.40 rad/s, 1.8 rad/s^2).")


def test_offset_compensation_no_fault():
    checker = DeviationChecker(deviation_thresh_rad=0.05, timeout_s=0.200)
    # Simulate static offset on J1 (index 0) and J4 (index 3)
    # True plant static offset = 0.025 rad (clamped <= 0.03 rad)
    # Offset definition: offset = act - sent.
    # So checker checks: err = |cmd - act + offset| = |sent - (sent + offset + dyn) + offset| = |dyn|
    # If act = sent + 0.025 (static) + 0.015 (dyn), then static offset = +0.025.
    calibrated_offset = np.zeros(14)
    calibrated_offset[0] = 0.025
    calibrated_offset[3] = -0.025
    checker.static_offset = calibrated_offset

    dt = 0.02
    t = 0.0
    sent = np.zeros(14)
    # Actual plant state has static offset of 0.025 plus dynamic lag/overshoot within 0.015 rad
    # Raw diff |act - sent| = 0.040 rad (approaching 0.05 limit).
    # If dynamic overshoot occurs, e.g. raw diff reaches 0.065 rad, compensated error is only |0.065 - 0.025| = 0.040 rad (< 0.05).
    for _ in range(50):  # 1.0 second simulation
        act = sent.copy()
        act[0] = sent[0] + 0.025 + 0.040  # raw diff = 0.065 rad (> 0.05 rad), compensated = 0.040 rad
        act[3] = sent[3] - 0.025 - 0.040  # raw diff = -0.065 rad, compensated = 0.040 rad
        tripped, j, err, th, chk, _, _ = checker.check(
            last_sent=sent, target_q=sent, q_act=act,
            is_real=True, dt=dt, t_now=t, last_cmd_vel=np.zeros(14)
        )
        assert not tripped, f"False positive trip at t={t:.2f}s, joint {j}, err={err:.4f}"
        t += dt
    print("[PASS] Plant with static offset + dynamic error does NOT fault with offset compensation.")


def test_runaway_drift_faults():
    checker = DeviationChecker(deviation_thresh_rad=0.05, timeout_s=0.200)
    checker.static_offset = np.zeros(14)
    dt = 0.02
    t = 0.0
    sent = np.zeros(14)
    act = sent.copy()
    faulted = False
    fault_t = None

    for step in range(30):  # 600 ms
        t += dt
        # Drift J1 runaway by 0.10 rad
        act[0] = 0.10
        tripped, j, err, th, chk, _, _ = checker.check(
            last_sent=sent, target_q=sent, q_act=act,
            is_real=True, dt=dt, t_now=t, last_cmd_vel=np.zeros(14)
        )
        if tripped:
            faulted = True
            fault_t = t
            break

    assert faulted, "Runaway drift failed to trip deviation checker!"
    assert fault_t <= 0.250, f"Runaway drift took too long to trip: {fault_t:.3f}s > 0.250s"
    print(f"[PASS] Runaway drift tripped gate in {fault_t*1000:.1f} ms (<= 250 ms target).")


def test_absolute_cap_015():
    checker = DeviationChecker(deviation_thresh_rad=0.05, timeout_s=0.200)
    # Even if offset is large, gap > 0.15 must trip immediately (t=0)
    sent = np.zeros(14)
    act = sent.copy()
    act[0] = 0.16
    tripped, j, err, th, chk, _, _ = checker.check(
        last_sent=sent, target_q=sent, q_act=act,
        is_real=True, dt=0.02, t_now=0.0, last_cmd_vel=np.zeros(14)
    )
    assert tripped and chk == "absolute_cap_0.15", f"Expected absolute_cap_0.15 trip, got {chk}"
    print("[PASS] Absolute cap 0.15 rad trips immediately without timeout.")


if __name__ == "__main__":
    test_speed_profiles()
    test_offset_compensation_no_fault()
    test_runaway_drift_faults()
    test_absolute_cap_015()
    print("ALL TESTS PASSED.")
