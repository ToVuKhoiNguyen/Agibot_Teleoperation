"""
teleop/app/sync_manager.py
Manages the SYNC -> READY -> ENGAGED -> FAULT state machine for real teleoperation,
source sanity gating, soft back-off at 0.035 rad, and mirror limiter dispatch.
"""

import time
import numpy as np


def process_arm_sync_and_dispatch(app, is_real: bool, dt: float):
    """Processes state machine transitions, mirror limiter plan, and backend send."""
    if not hasattr(app, "mirror_limiter"):
        return

    if is_real and hasattr(app, "robot_safety") and app.robot_safety is not None:
        q_act, _, meta = app.robot_safety.read_state()
        age = meta.get("age_s", 999.0) if isinstance(meta, dict) else 0.0
        app.last_state_feedback_age_s = age

        if q_act is not None and len(q_act) == 14 and not np.any(np.isnan(q_act)):
            app.real_q14_measured = q_act.copy()
            app.max_sim_real_gap_rad = float(np.max(np.abs(app.mirror_limiter.q_plan - q_act)))
        else:
            q_act = None

        # 1. State: SYNC
        if app.sync_state == "SYNC":
            if q_act is not None and age < 0.200:
                app.last_arm_cmd_left = q_act[:7].copy()
                app.last_arm_cmd_right = q_act[7:].copy()
                app.mirror_limiter.reset(q_act)
                app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)
                ee_poses = app.env.get_ee_poses()
                app.cur_target_pos_l = ee_poses["left"]["pos"].copy()
                app.cur_target_rot_l = ee_poses["left"]["rot"].copy()
                app.cur_target_pos_r = ee_poses["right"]["pos"].copy()
                app.cur_target_rot_r = ee_poses["right"]["rot"].copy()
                app.gizmo_target_pos_l = app.cur_target_pos_l.copy()
                app.gizmo_target_pos_r = app.cur_target_pos_r.copy()
                if hasattr(app, "output_filter_left"): app.output_filter_left.reset(app.last_arm_cmd_left)
                if hasattr(app, "output_filter_right"): app.output_filter_right.reset(app.last_arm_cmd_right)
                if hasattr(app, "output_filter_left_gizmo"): app.output_filter_left_gizmo.reset(app.last_arm_cmd_left)
                if hasattr(app, "output_filter_right_gizmo"): app.output_filter_right_gizmo.reset(app.last_arm_cmd_right)
                app.last_ik_sol_left = app.last_arm_cmd_left.copy()
                app.last_ik_sol_right = app.last_arm_cmd_right.copy()
                app.sync_state = "READY"
                print("[SYNC] Robot state synchronized! Transitioned SYNC -> READY.")
            return

        # 2. State: READY
        if app.sync_state == "READY":
            if getattr(app, "arm_engaged_l", False) or getattr(app, "arm_engaged_r", False):
                app.sync_state = "ENGAGED"
            else:
                return

        # 3. State: FAULT
        if app.sync_state == "FAULT" or getattr(app.robot_safety, "fault_latched", False):
            app.sync_state = "FAULT"
            return

        # 4. State: ENGAGED
        raw_target_q14 = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
        last_plan = app.mirror_limiter.q_plan

        # Source sanity check
        step_jump = float(np.max(np.abs(raw_target_q14 - last_plan)))
        if np.any(np.isnan(raw_target_q14)) or np.any(np.isinf(raw_target_q14)) or step_jump > 0.20:
            raw_target_q14 = last_plan.copy()

        # Soft back-off at 0.035 rad
        lead = float(np.max(np.abs(last_plan - q_act))) if q_act is not None else 0.0
        if lead > 0.035:
            speed_scale = max(0.1, 1.0 - (lead - 0.035) / 0.015)
        else:
            speed_scale = 1.0

        q_plan = app.mirror_limiter.step(raw_target_q14, measured_q14=q_act, dt=dt, speed_scale=speed_scale)
        app.last_arm_cmd_left = q_plan[:7].copy()
        app.last_arm_cmd_right = q_plan[7:].copy()
        app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)

        h12 = np.concatenate([app.last_hand_cmd_left, app.last_hand_cmd_right])
        app.robot_safety.context["input_mode"] = getattr(app, "input_mode", "mouse")
        app.robot_safety.context["collision_gate"] = bool(getattr(app, "collision_blocked", False))
        ok = app.robot_safety.send_arm(q_plan, dt=dt)
        app.robot_safety.send_hand(h12)
    elif getattr(app, "real_follower_mode", False):
        # Sim mode with real follower enabled (--sim-follower real): self-tracking via sim qpos
        raw_target_q14 = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
        meas_q14 = np.concatenate([app.env.get_left_arm_qpos(), app.env.get_right_arm_qpos()])
        q_plan = app.mirror_limiter.step(raw_target_q14, measured_q14=meas_q14, dt=dt)
        app.last_arm_cmd_left = q_plan[:7].copy()
        app.last_arm_cmd_right = q_plan[7:].copy()
        app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)
        app.env.set_hand_commands(app.last_hand_cmd_left, app.last_hand_cmd_right)
    else:
        # Default ideal sim mode
        app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)
        app.env.set_hand_commands(app.last_hand_cmd_left, app.last_hand_cmd_right)
