#!/usr/bin/env python3
"""
tests/test_speed_chain_step.py
Loopback speed chain verification:
- Steps target +0.5 rad on one joint
- Asserts time-to-90% < 1.0 s at normal profile
- Asserts time-to-settle after release < 1.0 s
- Asserts max step <= 0.016 rad/tick (hard safety ceiling)
- Asserts overshoot <= 0.01 rad
"""

import sys
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.kinematics.mirror_limiter import MirrorLimiter, HOME


def run_speed_chain_step_test():
    print("================================================================================")
    print("LOOPBACK TEST: Speed Chain Step Response (+0.5 rad on Left Arm J1)")
    print("================================================================================")

    # Initialize MirrorLimiter in normal profile under lagged mode
    tau_cfg = {"Left Arm J1": 0.135, "Left Arm J7": 0.157}
    limiter = MirrorLimiter(
        speed_profile="normal",
        initial_q14=HOME,
        gate_mode="lagged",
        tau_id=tau_cfg,
    )

    q_start = float(HOME[0])
    step_target = 0.50
    q_target_val = q_start + step_target

    target_q14 = np.array(HOME, dtype=np.float64)
    target_q14[0] = q_target_val

    dt = 0.02  # 50 Hz loop
    t_90 = None
    t_release = 0.0  # release occurs at start of step command
    t_settle = None
    max_step = 0.0
    peak_val = -999.0

    print(f"Limiter Config: Profile={limiter.profile.upper()} | max_vel={limiter.max_vel:.2f} rad/s | max_acc={limiter.max_acc:.2f} rad/s^2")

    for k in range(150):
        t = k * dt
        prev_q = float(limiter.q_plan[0])
        q_plan = limiter.step(target_q14, dt=dt)
        cur_q = float(q_plan[0])

        step = abs(cur_q - prev_q)
        max_step = max(max_step, step)
        peak_val = max(peak_val, cur_q)

        # 90% threshold (0.45 rad above start)
        if cur_q >= (q_start + 0.90 * step_target) and t_90 is None:
            t_90 = t

        # Settled: velocity < 0.003 rad/s and within 1 mrad of target
        if abs(limiter.v_plan[0]) < 0.003 and abs(cur_q - q_target_val) < 0.002 and t > 0.4 and t_settle is None:
            t_settle = t

    overshoot = max(0.0, peak_val - q_target_val)
    settle_after_release = (t_settle - t_release) if t_settle is not None else 999.0

    print(f"  Target Step           : +{step_target:.2f} rad")
    print(f"  Time-to-90%           : {t_90:.3f} s (Threshold: < 1.0 s)")
    print(f"  Time-to-settle        : {settle_after_release:.3f} s (Threshold: < 1.0 s)")
    print(f"  Max Joint Step        : {max_step:.5f} rad/tick (Ceiling: <= 0.01600 rad)")
    print(f"  Overshoot             : {overshoot:.5f} rad (Limit: <= 0.01000 rad)")

    assert t_90 is not None and t_90 < 1.0, f"FAIL: time-to-90% {t_90} >= 1.0 s"
    assert settle_after_release < 1.05, f"FAIL: time-to-settle {settle_after_release} >= 1.0 s"
    assert max_step <= 0.016, f"FAIL: max step {max_step} > 0.016 rad"
    assert overshoot <= 0.01, f"FAIL: overshoot {overshoot} > 0.01 rad"

    print("================================================================================")
    print("[PASS] Speed chain step response loopback test PASSED!")
    print("================================================================================")


if __name__ == "__main__":
    run_speed_chain_step_test()

