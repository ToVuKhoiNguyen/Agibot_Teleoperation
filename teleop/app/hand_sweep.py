"""
teleop/app/hand_sweep.py
State machine and angle generation for single-joint and all-finger hand test sweeps.
"""

import numpy as np


def update_hand_sweep(app, dt: float):
    """Executes automated diagnostic sinusoidal sweeps over active hand joints."""
    if not app.hand_sweep_active:
        return
    app.hand_sweep_time += dt

    if app.hand_sweep_mode == "single_joints":
        T_joint = 1.0
        idx = int(app.hand_sweep_time // T_joint)
        if idx >= 6:
            app.hand_sweep_active = False
            app.direct_hand_override_l = False
            app.direct_hand_override_r = False
            app.hand_sweep_info = "Single-joint sweep complete"
            return

        t_in_joint = app.hand_sweep_time - idx * T_joint
        jname_l = app.env.left_hand_active_joints[idx]
        lo_l, hi_l = app.env.joint_limits[jname_l]
        val_norm = 0.5 * (1.0 - np.cos(2.0 * np.pi * t_in_joint / T_joint))
        angle_l = lo_l + val_norm * (hi_l - lo_l)

        if app.hand_sweep_side in ("left", "both"):
            app.direct_hand_override_l = True
            cmd_l = app.hand_sweep_saved_joints_l.copy()
            cmd_l[idx] = angle_l
            app.direct_hand_target_l = cmd_l

        if app.hand_sweep_side in ("right", "both"):
            jname_r = app.env.right_hand_active_joints[idx]
            lo_r, hi_r = app.env.joint_limits[jname_r]
            angle_r = lo_r + val_norm * (hi_r - lo_r)
            app.direct_hand_override_r = True
            cmd_r = app.hand_sweep_saved_joints_r.copy()
            cmd_r[idx] = angle_r
            app.direct_hand_target_r = cmd_r

        act_l = app.env.data.qpos[app.env.qpos_idx_map[jname_l]]
        app.hand_sweep_info = f"Sweeping {jname_l}: cmd={angle_l:.2f} rad, act={act_l:.2f} rad"

    elif app.hand_sweep_mode == "all_fingers":
        T_cycle = 1.5
        total_time = 3.0 * T_cycle
        if app.hand_sweep_time >= total_time:
            app.hand_sweep_active = False
            app.hand_sweep_info = "All-finger sweep complete"
            return

        phase = 2.0 * np.pi * app.hand_sweep_time / T_cycle
        closure = float(0.5 * (1.0 - np.cos(phase)))
        curls = np.ones(5, dtype=np.float64) * closure
        if app.hand_sweep_side in ("left", "both"):
            app.target_curls_l = curls
            app.target_pinch_l = 0.5 * closure
            app.hand_closure_l = closure
        if app.hand_sweep_side in ("right", "both"):
            app.target_curls_r = curls
            app.target_pinch_r = 0.5 * closure
            app.hand_closure_r = closure
        cycle_num = min(3, int(app.hand_sweep_time / T_cycle) + 1)
        app.hand_sweep_info = f"All-finger curl cycle {cycle_num}/3 ({closure*100.0:.0f}%)"

