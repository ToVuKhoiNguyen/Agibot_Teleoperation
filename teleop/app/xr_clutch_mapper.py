"""
teleop/app/xr_clutch_mapper.py
4-state smart clutch state machine (LOCKED, TRANSLATE, ROTATE, 6D) and SE(3) target integration.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def get_effective_axis_map(app) -> np.ndarray:
    rig_yaw = float(getattr(app, "xr_rig_yaw", 0.0))
    if abs(rig_yaw) < 1e-4:
        return app.xr_axis_map
    c, s = np.cos(-rig_yaw), np.sin(-rig_yaw)
    Ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)
    return app.xr_axis_map @ Ry


def update_xr_clutch(app, left_pkt: dict, right_pkt: dict, rot_enabled: bool):
    """Updates 4-state clutch machine and SE(3) targets for both arms."""
    axis_map = get_effective_axis_map(app)

    # --- Left Arm Clutch & Tracking Loss Handling ---
    left_present = left_pkt is not None
    left_loss = left_present and (
        left_pkt.get("pos") is None
        or left_pkt.get("tracking") is False
        or left_pkt.get("tracking_loss") is True
    )
    if left_loss:
        if app.xr_clutch_state_l != "LOCKED" or getattr(app, "gizmo_active_l", False):
            app.xr_clutch_state_l = "LOCKED"
            app.xr_anchor_ctrl_pos_l = None
            app.xr_anchor_ctrl_rot_l = None
            app.gizmo_active_l = False
            app.xr_grip_active_l = False
            app.xr_reengage_required_l = True
            app._log_xr_event("L tracking lost -> clutch released & target held")
        if not getattr(app, "xr_tracking_lost_l", False):
            app.tracking_loss_events = getattr(app, "tracking_loss_events", 0) + 1
        app.xr_tracking_lost_l = True
    elif left_present and left_pkt.get("pos") is not None:
        grip_down_l = app.xr_grip_val_l >= app.xr_grip_thresh or bool(left_pkt.get("grip", False))
        trig_down_l = app.xr_trig_val_l >= app.xr_trigger_thresh
        if getattr(app, "xr_reengage_required_l", False):
            if not grip_down_l and not trig_down_l:
                app.xr_reengage_required_l = False
                app.xr_tracking_lost_l = False
                app._log_xr_event("L tracking restored & clutch re-armed")
            else:
                app.xr_clutch_state_l = "LOCKED"
                app.xr_anchor_ctrl_pos_l = None
                app.xr_anchor_ctrl_rot_l = None
                app.gizmo_active_l = False
                app.xr_grip_active_l = False

        if not getattr(app, "xr_reengage_required_l", False):
            p_ctrl_l = np.array(left_pkt["pos"], dtype=np.float64)
            q_ctrl_l = left_pkt.get("quat")
            if grip_down_l and trig_down_l:
                next_state_l = "6D"
            elif trig_down_l and not grip_down_l:
                next_state_l = "TRANSLATE"
            elif grip_down_l and not trig_down_l:
                next_state_l = "ROTATE"
            else:
                next_state_l = "LOCKED"

            if next_state_l != app.xr_clutch_state_l:
                app.xr_clutch_state_l = next_state_l
                if next_state_l != "LOCKED":
                    app.xr_anchor_ctrl_pos_l = p_ctrl_l.copy()
                    app.xr_anchor_marker_pos_l = app.cur_target_pos_l.copy()
                    if q_ctrl_l is not None and len(q_ctrl_l) == 4:
                        app.xr_anchor_ctrl_rot_l = R.from_quat([q_ctrl_l[1], q_ctrl_l[2], q_ctrl_l[3], q_ctrl_l[0]]).as_matrix()
                        app.xr_anchor_marker_rot_l = app.cur_target_rot_l.copy()
                    app.gizmo_active_l = True
                    app._log_xr_event(f"L clutch -> {next_state_l}")
                else:
                    app.xr_anchor_ctrl_pos_l = None
                    app.xr_anchor_ctrl_rot_l = None
                    app.gizmo_active_l = False
                    app._log_xr_event("L clutch -> LOCKED")

            if app.xr_clutch_state_l in ("TRANSLATE", "6D") and app.xr_anchor_ctrl_pos_l is not None:
                delta_ctrl_xr = p_ctrl_l - app.xr_anchor_ctrl_pos_l
                if np.linalg.norm(delta_ctrl_xr) >= app.xr_deadband:
                    delta_rob = axis_map @ delta_ctrl_xr
                    eff_scale = app.xr_scale * (app.xr_precision_scale_factor if app.precision_l else 1.0)
                    new_pos = app.xr_anchor_marker_pos_l + eff_scale * delta_rob
                    new_pos[0] = np.clip(new_pos[0], -0.2, 0.9)
                    new_pos[1] = np.clip(new_pos[1], -0.9, 0.9)
                    new_pos[2] = np.clip(new_pos[2], 0.5, 1.8)
                    app.cur_target_pos_l = new_pos
                    app.gizmo_active_l = True
                    if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("left", "controller clutch")

            if app.xr_clutch_state_l in ("ROTATE", "6D") and rot_enabled and q_ctrl_l is not None and len(q_ctrl_l) == 4 and app.xr_anchor_ctrl_rot_l is not None:
                r_curr_ctrl = R.from_quat([q_ctrl_l[1], q_ctrl_l[2], q_ctrl_l[3], q_ctrl_l[0]]).as_matrix()
                r_rel_xr = r_curr_ctrl @ app.xr_anchor_ctrl_rot_l.T
                r_rel_rob = axis_map @ r_rel_xr @ axis_map.T
                app.cur_target_rot_l = r_rel_rob @ app.xr_anchor_marker_rot_l
                app.gizmo_active_l = True
                if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("left", "controller clutch")

    app.xr_grip_active_l = (app.xr_clutch_state_l != "LOCKED")

    # --- Right Arm Clutch & Tracking Loss Handling ---
    right_present = right_pkt is not None
    right_loss = right_present and (
        right_pkt.get("pos") is None
        or right_pkt.get("tracking") is False
        or right_pkt.get("tracking_loss") is True
    )
    if right_loss:
        if app.xr_clutch_state_r != "LOCKED" or getattr(app, "gizmo_active_r", False):
            app.xr_clutch_state_r = "LOCKED"
            app.xr_anchor_ctrl_pos_r = None
            app.xr_anchor_ctrl_rot_r = None
            app.gizmo_active_r = False
            app.xr_grip_active_r = False
            app.xr_reengage_required_r = True
            app._log_xr_event("R tracking lost -> clutch released & target held")
        if not getattr(app, "xr_tracking_lost_r", False):
            app.tracking_loss_events = getattr(app, "tracking_loss_events", 0) + 1
        app.xr_tracking_lost_r = True
    elif right_present and right_pkt.get("pos") is not None:
        grip_down_r = app.xr_grip_val_r >= app.xr_grip_thresh or bool(right_pkt.get("grip", False))
        trig_down_r = app.xr_trig_val_r >= app.xr_trigger_thresh
        if getattr(app, "xr_reengage_required_r", False):
            if not grip_down_r and not trig_down_r:
                app.xr_reengage_required_r = False
                app.xr_tracking_lost_r = False
                app._log_xr_event("R tracking restored & clutch re-armed")
            else:
                app.xr_clutch_state_r = "LOCKED"
                app.xr_anchor_ctrl_pos_r = None
                app.xr_anchor_ctrl_rot_r = None
                app.gizmo_active_r = False
                app.xr_grip_active_r = False

        if not getattr(app, "xr_reengage_required_r", False):
            p_ctrl_r = np.array(right_pkt["pos"], dtype=np.float64)
            q_ctrl_r = right_pkt.get("quat")
            if grip_down_r and trig_down_r:
                next_state_r = "6D"
            elif trig_down_r and not grip_down_r:
                next_state_r = "TRANSLATE"
            elif grip_down_r and not trig_down_r:
                next_state_r = "ROTATE"
            else:
                next_state_r = "LOCKED"

            if next_state_r != app.xr_clutch_state_r:
                app.xr_clutch_state_r = next_state_r
                if next_state_r != "LOCKED":
                    app.xr_anchor_ctrl_pos_r = p_ctrl_r.copy()
                    app.xr_anchor_marker_pos_r = app.cur_target_pos_r.copy()
                    if q_ctrl_r is not None and len(q_ctrl_r) == 4:
                        app.xr_anchor_ctrl_rot_r = R.from_quat([q_ctrl_r[1], q_ctrl_r[2], q_ctrl_r[3], q_ctrl_r[0]]).as_matrix()
                        app.xr_anchor_marker_rot_r = app.cur_target_rot_r.copy()
                    app.gizmo_active_r = True
                    app._log_xr_event(f"R clutch -> {next_state_r}")
                else:
                    app.xr_anchor_ctrl_pos_r = None
                    app.xr_anchor_ctrl_rot_r = None
                    app.gizmo_active_r = False
                    app._log_xr_event("R clutch -> LOCKED")

            if app.xr_clutch_state_r in ("TRANSLATE", "6D") and app.xr_anchor_ctrl_pos_r is not None:
                delta_ctrl_xr = p_ctrl_r - app.xr_anchor_ctrl_pos_r
                if np.linalg.norm(delta_ctrl_xr) >= app.xr_deadband:
                    delta_rob = axis_map @ delta_ctrl_xr
                    eff_scale = app.xr_scale * (app.xr_precision_scale_factor if app.precision_r else 1.0)
                    new_pos = app.xr_anchor_marker_pos_r + eff_scale * delta_rob
                    new_pos[0] = np.clip(new_pos[0], -0.2, 0.9)
                    new_pos[1] = np.clip(new_pos[1], -0.9, 0.9)
                    new_pos[2] = np.clip(new_pos[2], 0.5, 1.8)
                    app.cur_target_pos_r = new_pos
                    app.gizmo_active_r = True
                    if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("right", "controller clutch")

            if app.xr_clutch_state_r in ("ROTATE", "6D") and rot_enabled and q_ctrl_r is not None and len(q_ctrl_r) == 4 and app.xr_anchor_ctrl_rot_r is not None:
                r_curr_ctrl = R.from_quat([q_ctrl_r[1], q_ctrl_r[2], q_ctrl_r[3], q_ctrl_r[0]]).as_matrix()
                r_rel_xr = r_curr_ctrl @ app.xr_anchor_ctrl_rot_r.T
                r_rel_rob = axis_map @ r_rel_xr @ axis_map.T
                app.cur_target_rot_r = r_rel_rob @ app.xr_anchor_marker_rot_r
                app.gizmo_active_r = True
                if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("right", "controller clutch")

    app.xr_grip_active_r = (app.xr_clutch_state_r != "LOCKED")

    if app.xr_grip_active_l and app.xr_grip_active_r:
        app._solve_arm_ik("both")
    elif app.xr_grip_active_l:
        app._solve_arm_ik("left")
    elif app.xr_grip_active_r:
        app._solve_arm_ik("right")
