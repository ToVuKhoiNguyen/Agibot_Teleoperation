#!/usr/bin/env python3
"""scripts/arm_homing.py - Standalone Safe Homing Phase for AGIBOT A2 Ultra"""
import time, math, sys

HOME = [0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0]
HOME_MAX_DELTA = 1.0  # Max permitted delta from measured to HOME in rad

def check_home_limits(lim_min, lim_max):
    for i in range(14):
        assert lim_min[i] <= HOME[i] <= lim_max[i], f"Joint {i} HOME {HOME[i]} exceeds limits [{lim_min[i]}, {lim_max[i]}]"

def execute_homing(node, prompt_fn=input):
    """Execute homing phase before teleoperation starts. Returns True if homed, False on abort."""
    check_home_limits(node.lim_min, node.lim_max)
    t0 = time.time()
    while time.time() - t0 < 1.0:
        with node.meas_lock:
            if node.meas_q is not None:
                break
        time.sleep(0.02)
    with node.meas_lock:
        meas = list(node.meas_q) if node.meas_q else None
    if not meas or len(meas) != 14 or any(math.isnan(x) or math.isinf(x) for x in meas):
        print("FATAL: State not alive or invalid within 1 s. Refusing homing.")
        return False

    deltas = [HOME[i] - meas[i] for i in range(14)]
    max_d = max(abs(d) for d in deltas)
    est_dur = max(2.5, max_d / 0.15) if max_d > 1e-4 else 0.5
    print(f"[HOMING] Per-joint deltas: {[round(d, 4) for d in deltas]}")
    print(f"[HOMING] max|delta| = {max_d:.4f} rad | Est duration = {est_dur:.2f} s")

    if max_d > HOME_MAX_DELTA:
        print(f"FATAL: max|delta| ({max_d:.3f} rad) > {HOME_MAX_DELTA} rad. Move arms manually closer to home.")
        return False

    if not node.is_live:
        print("[HOMING] --dry mode: Plan computed successfully. 0 commands published.")
        return True

    print("\n" + "!" * 70)
    print("SECOND PERSON WITH WIRELESS E-STOP MUST BE READY!")
    print("Type exactly 'GO HOME' to begin automatic homing move:")
    ans = prompt_fn("> ").strip()
    if ans != "GO HOME":
        print(f"Aborted: typed '{ans}' != 'GO HOME'. No commands published.")
        return False

    return _run_homing_trajectory(node, meas, deltas, max_d, est_dur)

def _run_homing_trajectory(node, meas, deltas, max_d, duration):
    from sensor_msgs.msg import JointState
    node.homing_active = True
    node.curr_cmd_q = list(meas)
    # First command == measured state
    node._publish_raw_q(node.curr_cmd_q)
    t_start = time.time()
    fault_counter = 0

    while True:
        now = time.time()
        tau = min(1.0, (now - t_start) / duration) if duration > 0 else 1.0
        s = tau * tau * (3.0 - 2.0 * tau)  # Smoothstep (zero initial/final velocity)
        target_step = [meas[i] + deltas[i] * s for i in range(14)]
        clamped = [max(node.lim_min[i], min(node.lim_max[i], target_step[i])) for i in range(14)]
        
        node.curr_cmd_q = clamped
        node._publish_raw_q(node.curr_cmd_q)

        with node.meas_lock:
            curr_meas = list(node.meas_q) if node.meas_q else None
        if curr_meas:
            dev = max(abs(curr_meas[i] - node.curr_cmd_q[i]) for i in range(14))
            if dev > 0.05:
                fault_counter += 1
                if fault_counter >= 10:  # >200 ms at 50 Hz
                    print(f"FATAL FAULT: Joint deviation {dev:.4f} rad > 0.05 rad for >200 ms!")
                    node.shutdown_hold()
                    return False
            else:
                fault_counter = 0

        if tau >= 1.0:
            break
        time.sleep(0.02)

    # Stability check: max|measured - HOME| < 0.02 rad for 0.5 s
    t_settle = time.time()
    while time.time() - t_settle < 0.5:
        node._publish_raw_q(HOME)
        with node.meas_lock:
            curr_meas = list(node.meas_q) if node.meas_q else None
        if curr_meas and max(abs(curr_meas[i] - HOME[i]) for i in range(14)) >= 0.02:
            t_settle = time.time()
        time.sleep(0.02)

    node.curr_cmd_q = list(HOME)
    node.anchored = True
    node.homing_active = False
    print(f"[HOMING] Dropped {node.homing_dropped_udp} UDP packets during homing.")
    print("HOMED")
    return True
