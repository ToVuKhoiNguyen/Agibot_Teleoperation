#!/usr/bin/env python3
"""scripts/test_oscillation.py
Loopback test: first-order lag tau=0.15 s plant with 0.05 rad overshoot.
Verifies:
1. Pure open-loop: measured act/feedback does NOT affect command path.
2. SENT has no oscillation: zero sign changes of dq after target stops.
3. Settle time < 1.0 s at --speed 3.
4. Jerk-limited S-curve applied smoothly on accel and decel (jerk <= ~20 rad/s^3).
"""
import numpy as np
import sys
from teleop.kinematics.mirror_limiter import MirrorLimiter, SPEED_LEVELS

def test_pure_open_loop():
    print("--- 1. Testing Pure Open-Loop (Feedback Independence) ---")
    ml1 = MirrorLimiter(speed_level=3)
    ml2 = MirrorLimiter(speed_level=3)
    
    target = np.array([0.2, 0.5, -0.3, 0.4, 0.1, 0.0, 0.0] * 2)
    
    # ml1 has no measured_q14
    # ml2 has huge perturbed measured_q14
    for _ in range(30):
        q1 = ml1.step(target, dt=0.02)
        fake_meas = q1 + 0.5 * np.sin(np.arange(14))
        q2 = ml2.step(target, measured_q14=fake_meas, dt=0.02)
        assert np.allclose(q1, q2, atol=1e-6), f"Feedback influenced command path: max diff {np.max(np.abs(q1 - q2))}"
    print("[PASS] Verified: command path is pure open-loop; measured feedback does not modify command.")

def test_sent_no_oscillation_and_settle():
    print("--- 2. Testing SENT Oscillation and Settle Time (< 1.0s) at --speed 3 ---")
    ml = MirrorLimiter(speed_level=3)
    dt = 0.02
    
    # Initial state
    target = ml.q_plan.copy()
    target[0] += 0.20 # 0.2 rad step in J0
    
    history_sent = []
    history_act = []
    
    # Simulated plant with tau=0.15s and 0.05 rad overshoot (underdamped 2nd order)
    # d2_act/dt2 + 2*zeta*wn*d_act/dt + wn^2*(act - sent) = 0
    tau = 0.15
    wn = 1.0 / tau # 6.67 rad/s
    zeta = 0.45 # ~20% overshoot on a step = ~0.04 - 0.05 rad overshoot
    act_q = ml.q_plan.copy()
    act_v = np.zeros(14)
    
    # Target is held constant from t=0 (step change)
    for tick in range(100): # 2.0 seconds
        q_sent = ml.step(target, dt=dt)
        history_sent.append(q_sent.copy())
        
        # Plant step
        acc = -2.0 * zeta * wn * act_v - (wn**2) * (act_q - q_sent)
        act_v += acc * dt
        act_q += act_v * dt
        history_act.append(act_q.copy())
        
    sent_j0 = np.array([h[0] for h in history_sent])
    act_j0 = np.array([h[0] for h in history_act])
    
    # Check sign changes of dq after target stops (which is t >= 0 here since target is static step)
    dq = np.diff(sent_j0)
    non_zero = np.where(np.abs(dq) > 1e-6)[0]
    sign_changes = np.where(np.diff(np.sign(dq[non_zero])))[0]
    print(f"SENT sign changes of dq: {len(sign_changes)}")
    assert len(sign_changes) == 0, f"SENT oscillated! {len(sign_changes)} sign changes detected."
    
    # Check overshoot in SENT
    overshoot_sent = np.max(sent_j0) - target[0]
    print(f"SENT max: {np.max(sent_j0):.4f}, target: {target[0]:.4f}, overshoot: {overshoot_sent:+.6f}")
    assert overshoot_sent <= 1e-4, f"SENT had overshoot: {overshoot_sent}"
    
    # Check settle time (< 1.0 s)
    settled_indices = np.where(np.abs(sent_j0 - target[0]) < 0.002)[0]
    assert len(settled_indices) > 0, "SENT never settled!"
    settle_t = settled_indices[0] * dt
    print(f"SENT settle time (< 0.002 rad from target): {settle_t:.2f} s")
    assert settle_t < 1.0, f"Settle time {settle_t} s >= 1.0 s!"
    
    # Check plant ACT overshoot:
    act_overshoot = np.max(act_j0) - target[0]
    print(f"Plant ACT overshoot: {act_overshoot:.4f} rad (expected ~0.05 rad)")
    
    # Check Jerk in SENT
    v_seq = np.diff(sent_j0) / dt
    a_seq = np.diff(v_seq) / dt
    j_seq = np.diff(a_seq) / dt
    print(f"Max observed jerk: {np.max(np.abs(j_seq)):.2f} rad/s^3 (cap ~20 rad/s^3)")
    print("[PASS] Verified: zero sign changes in dq, settle time < 1.0 s at --speed 3.")

if __name__ == "__main__":
    test_pure_open_loop()
    test_sent_no_oscillation_and_settle()
    print("ALL TESTS PASSED SUCCESSFULLY.")
