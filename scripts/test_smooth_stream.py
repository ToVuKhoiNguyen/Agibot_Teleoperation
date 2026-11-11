#!/usr/bin/env python3
"""scripts/test_smooth_stream.py
Verification tests for smooth 100 Hz streaming and jittered/dropped-packet link:
1. Pure open-loop: target smoother + S-curve limiter running at 100 Hz.
2. Jittered 10% drop link:
   - Zero-dq ticks < 2% during a ramp move.
   - No tick with step > cap (0.008 rad at speed 3, 100 Hz).
   - Sent after settle constant (std 0.0).
   - Time-to-90% at --speed 3 stays < 1.2 s.
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from teleop.kinematics.mirror_limiter import MirrorLimiter
import scripts.arm_teleop_rx as rx

def test_100hz_stream_and_jitter_drop():
    print("--- Testing 100 Hz Smooth Stream with Jittered 10% Drop Link ---")
    dt = 0.010  # 100 Hz
    ml = MirrorLimiter(speed_level=3, dt=dt)
    
    rx_node = rx.ArmTeleopRx(mode_live=False, allow_ip="127.0.0.1", port=55305, tx_state_port=55306, speed_level=3)
    rx_node.has_published_engaged = True
    rx_node.curr_cmd_q = list(rx.HOME)
    
    np.random.seed(42)
    step_target = rx.HOME.copy()
    step_target[0] += 0.30  # 0.3 rad step
    
    laptop_history = []
    rx_published = []
    
    t_start = 0.0
    # Simulate 200 ticks = 2.0 seconds at 100 Hz
    for tick in range(200):
        t_now = tick * dt
        # Staircase update: target changes at tick 10 (t=0.1s)
        curr_raw_target = step_target if tick >= 10 else rx.HOME
        
        # Laptop limiter step at 100 Hz
        plan_q = ml.step(curr_raw_target, dt=dt)
        laptop_history.append((t_now, plan_q[0]))
        
        # UDP link simulation with 10% packet drop and jitter (10ms to 20ms network latency)
        # We record arrival time aligned with t_now
        if np.random.rand() > 0.10:
            jitter = np.random.uniform(-0.003, 0.003)
            arrival_t = t_now + 0.015 + jitter
            rx_node.pkt_buffer.append((arrival_t, np.array(plan_q, dtype=np.float64)))
            if len(rx_node.pkt_buffer) > 10:
                rx_node.pkt_buffer = rx_node.pkt_buffer[-10:]
            rx_node.target_q = list(plan_q)
            rx_node.target_engaged = [True, True]
            rx_node.last_pkt_time = t_now
            
        # Receiver 100 Hz control tick at t_now
        rx_node.last_tick_t = t_now - dt
        rx_node._control_tick()
        # Override now in _get_playout_target by calling with t_now
        playout_target = rx_node._get_playout_target(t_now)
        rx_node._final_line_guard_and_publish(playout_target, dt)
        rx_published.append(rx_node.curr_cmd_q[0])
        
    pub = np.array(rx_published)
    d_pub = np.abs(np.diff(pub))
    
    # Check max step cap (at 100 Hz, speed 3 cap is 0.008 rad/tick)
    max_step = np.max(d_pub)
    print(f"Max step in receiver published output: {max_step:.5f} rad (speed 3 cap: 0.0080)")
    assert max_step <= 0.008001, f"Max step violated! {max_step} > 0.008"
    
    # Ramp region: from when motion actually begins in receiver output (pub > 0.001) to 90% threshold (0.3 * 0.9 = 0.27)
    idx_motion_start = np.where(pub > 0.001)[0]
    assert len(idx_motion_start) > 0, "Motion never started!"
    ramp_start = idx_motion_start[0]
    
    idx_90 = np.where(pub >= 0.27)[0]
    assert len(idx_90) > 0, "Never reached 90% threshold!"
    ramp_end = idx_90[0]
    time_to_90 = (ramp_end - 10) * dt
    print(f"Time-to-90% at speed 3: {time_to_90:.3f} s (requirement < 1.2 s)")
    assert time_to_90 < 1.20, f"Time-to-90% {time_to_90} s >= 1.2 s!"
    
    # Zero dq ticks during ramp:
    ramp_dq = d_pub[ramp_start:ramp_end]
    zero_dq_ticks = np.sum(ramp_dq < 1e-6)
    zero_dq_frac = (zero_dq_ticks / len(ramp_dq)) * 100.0
    print(f"Zero-dq ticks during ramp: {zero_dq_ticks}/{len(ramp_dq)} ({zero_dq_frac:.1f}%, requirement < 2.0%)")
    assert zero_dq_frac < 2.0, f"Zero-dq ticks {zero_dq_frac:.1f}% >= 2.0%!"
    
    # Settle region (last 50 ticks):
    settle_slice = pub[-50:]
    settle_std = np.std(settle_slice)
    print(f"Settle region std: {settle_std:.6f} (target: 0.000000)")
    assert settle_std < 1e-5, f"Settle region not constant! std={settle_std}"
    
    # Check no overshoot / sign changes after ramp
    after_ramp_dq = np.diff(pub[ramp_end:])
    non_zero = np.where(np.abs(after_ramp_dq) > 1e-6)[0]
    sign_changes = np.where(np.diff(np.sign(after_ramp_dq[non_zero])))[0]
    print(f"Sign changes of dq after ramp: {len(sign_changes)}")
    assert len(sign_changes) == 0, f"Oscillation detected after ramp! {len(sign_changes)} sign changes"
    
    print("[PASS] Verified: zero-dq < 2%, no step > cap, settle constant, time-to-90% < 1.2s.")
    rx_node.destroy_node()

if __name__ == "__main__":
    if rx.rclpy is not None and not rx.rclpy.ok():
        rx.rclpy.init()
    test_100hz_stream_and_jitter_drop()
    print("ALL SMOOTH STREAM VERIFICATION TESTS PASSED.")
