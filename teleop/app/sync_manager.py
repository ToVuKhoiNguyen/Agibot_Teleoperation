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
        q_act, h_act, meta = app.robot_safety.read_state()
        age = meta.get("age_s", 999.0) if isinstance(meta, dict) else 0.0
        app.last_state_feedback_age_s = age

        if q_act is not None and len(q_act) == 14 and not np.any(np.isnan(q_act)):
            app.real_q14_measured = q_act.copy()
            app.max_sim_real_gap_rad = float(np.max(np.abs(app.mirror_limiter.q_plan - q_act)))
        else:
            q_act = None

        rx_st = meta.get("rx_state", "WAITING") if isinstance(meta, dict) else "WAITING"
        app.orin_rx_state = rx_st

        # Check Orin robot node fault latch
        if meta.get("robot_fault", False) or rx_st.startswith("FAULT"):
            app.robot_safety.fault_latched = True
            reason = meta.get("fault_reason", "") or rx_st
            if "DUAL PUBLISHER" in reason:
                app.robot_safety.fault_reason = "DUAL PUBLISHER: another node is publishing on robot topics!"
            else:
                app.robot_safety.fault_reason = f"ROBOT NODE FAULT ({reason}): restart arm_teleop_rx.py"
            app.sync_state = "FAULT"
            return

        # 1. State: SYNC
        if app.sync_state == "SYNC":
            if q_act is not None and age < 0.200:
                app.last_arm_cmd_left = q_act[:7].copy()
                app.last_arm_cmd_right = q_act[7:].copy()
                app.mirror_limiter.reset(q_act)
                app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)
                if h_act is not None and len(h_act) == 12:
                    app.last_hand_cmd_left = h_act[:6].copy()
                    app.last_hand_cmd_right = h_act[6:].copy()
                    app.current_safe_hand_l = h_act[:6].copy()
                    app.current_safe_hand_r = h_act[6:].copy()
                    app.env.set_hand_commands(app.last_hand_cmd_left, app.last_hand_cmd_right)
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
                if hasattr(app, "robot_safety") and app.robot_safety is not None:
                    app.robot_safety._last_cmd_q14 = q_act.copy()
                    app.robot_safety._last_cmd_vel = np.zeros(14, dtype=np.float64)
                    app.robot_safety._deviation_start_t = None
                app.sync_state = "READY"
                print("[SYNC] Robot state synchronized! Transitioned SYNC -> READY.")
            return

        # 2. State: READY
        if app.sync_state == "READY":
            if getattr(app, "arm_engaged_l", False) or getattr(app, "arm_engaged_r", False):
                if rx_st == "RX_OK" or rx_st == "NOT_ENGAGED":
                    app.sync_state = "ENGAGED"
            return

        # 3. State: FAULT
        if app.sync_state == "FAULT" or getattr(app.robot_safety, "fault_latched", False):
            app.sync_state = "FAULT"
            return

        # 4. State: ENGAGED - must never show ENGAGED while rx_state != RX_OK
        if rx_st not in ("RX_OK", "WAITING"):
            if rx_st.startswith("REJECT") or rx_st == "HOLDING":
                app.sync_state = "READY"
                app.status_warning = f"Orin state: {rx_st}"
                return
        raw_target_q14 = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
        last_plan = app.mirror_limiter.q_plan

        # Source sanity check (NaN / inf protection; step bounding handled by mirror_limiter)
        if np.any(np.isnan(raw_target_q14)) or np.any(np.isinf(raw_target_q14)):
            raw_target_q14 = last_plan.copy()

        # Deviation is log-only (NEVER slowdowns or scales command rate)
        lead = float(np.max(np.abs(last_plan - q_act))) if q_act is not None else 0.0
        if getattr(app, "status_warning", "").startswith("Tracking deviation"):
            app.status_warning = ""

        # Update digital twin hand commands gated by ENGAGED
        eng_l = bool(getattr(app, "arm_engaged_l", False))
        eng_r = bool(getattr(app, "arm_engaged_r", False))
        h_left = app.last_hand_cmd_left if eng_l else (h_act[:6] if h_act is not None else np.zeros(6))
        h_right = app.last_hand_cmd_right if eng_r else (h_act[6:] if h_act is not None else np.zeros(6))
        app.env.set_hand_commands(h_left, h_right)

        # Static offset calibration during hold phase (>= 2 s stationary after ENGAGED)
        if not getattr(app, "_static_offset_calibrated", False):
            if not hasattr(app, "_hold_samples"):
                app._hold_samples = []
            dq_sent = np.max(np.abs(getattr(app.mirror_limiter, "v_plan", np.zeros(14))))
            if dq_sent < 0.002 and q_act is not None:
                app._hold_samples.append(q_act - last_plan)
                if len(app._hold_samples) >= int(2.0 / dt):
                    raw_offset = np.median(app._hold_samples, axis=0)
                    clamped_offset = np.clip(raw_offset, -0.03, 0.03)
                    if hasattr(app.robot_safety, "dev_checker"):
                        app.robot_safety.dev_checker.static_offset = clamped_offset.copy()
                    app._static_offset_calibrated = True
                    off_str = " ".join(f"{x:+.4f}" for x in clamped_offset)
                    print(f"[OFFSET] Static offset calibrated ({len(app._hold_samples)} samples, clamped <= 0.03): {off_str}")
            else:
                app._hold_samples = []

        # Deliver raw target to backend; limiter runs at 100 Hz inside UdpBackend
        cur_q = app.mirror_limiter.q_plan if hasattr(app, "mirror_limiter") else app.last_arm_cmd_left
        app.env.set_arm_commands(cur_q[:7], cur_q[7:])
        h12 = np.concatenate([h_left, h_right])
        app.robot_safety.context["input_mode"] = getattr(app, "input_mode", "mouse")
        app.robot_safety.context["collision_gate"] = bool(getattr(app, "collision_blocked", False))
        ok = app.robot_safety.send_arm(raw_target_q14, dt=dt)
        if hasattr(app.robot_safety, "send_hand"):
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
        # Default sim mode: step through the SAME mirror_limiter as real-udp
        if hasattr(app, "mirror_limiter") and app.mirror_limiter is not None:
            raw_target_q14 = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
            q_plan = app.mirror_limiter.step(raw_target_q14, dt=dt)
            app.env.set_arm_commands(q_plan[:7], q_plan[7:])
        else:
            app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)
        app.env.set_hand_commands(app.last_hand_cmd_left, app.last_hand_cmd_right)
