"""
teleop/app/xr_clutch_mapper.py
4-state smart clutch state machine (LOCKED, TRANSLATE, ROTATE, 6D) and SE(3) target integration.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
from teleop.kinematics.reach_projection import project_to_reachable_workspace


def get_effective_axis_map(app) -> np.ndarray:
    rig_yaw = float(getattr(app, "xr_rig_yaw", 0.0))
    if abs(rig_yaw) < 1e-4:
        return app.xr_axis_map
    c, s = np.cos(-rig_yaw), np.sin(-rig_yaw)
    Ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)
    return app.xr_axis_map @ Ry


def get_effective_rot_axis_map(app) -> np.ndarray:
    rot_map = getattr(app, "xr_rot_axis_map", np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64))
    rig_yaw = float(getattr(app, "xr_rig_yaw", 0.0))
    if abs(rig_yaw) < 1e-4:
        return rot_map
    c, s = np.cos(-rig_yaw), np.sin(-rig_yaw)
    Ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)
    return rot_map @ Ry


def _process_arm_clutch(app, side: str, pkt: dict, rot_enabled: bool, axis_map: np.ndarray, rot_axis_map: np.ndarray):
    """Processes relative clutch state machine and SE(3) targets for a single arm."""
    s = side[0]  # 'l' or 'r'
    present = pkt is not None
    loss = present and (pkt.get("pos") is None or pkt.get("tracking") is False or pkt.get("tracking_loss") is True)

    if loss:
        clutch_state = getattr(app, f"xr_clutch_state_{s}")
        if clutch_state != "LOCKED" or getattr(app, f"gizmo_active_{s}", False):
            setattr(app, f"xr_clutch_state_{s}", "LOCKED")
            setattr(app, f"xr_anchor_ctrl_pos_{s}", None)
            setattr(app, f"xr_anchor_ctrl_rot_{s}", None)
            setattr(app, f"gizmo_active_{s}", False)
            setattr(app, f"xr_grip_active_{s}", False)
            setattr(app, f"xr_reengage_required_{s}", True)
            setattr(app, f"xr_trig_down_{s}", False)
            setattr(app, f"xr_grip_down_{s}", False)
            app._log_xr_event(f"{side.upper()} tracking lost -> clutch released & target held")
        if not getattr(app, f"xr_tracking_lost_{s}", False):
            app.tracking_loss_events = getattr(app, "tracking_loss_events", 0) + 1
        setattr(app, f"xr_tracking_lost_{s}", True)
        return

    if not present or pkt.get("pos") is None:
        return

    # Trigger & Grip detection with hysteresis (Trigger: rise 0.5, fall 0.4; Grip: rise 0.5, fall 0.4 or bool)
    pkt_grip_raw = pkt.get("grip")
    pkt_trig_raw = pkt.get("trigger")
    t_val = float(pkt_trig_raw) if pkt_trig_raw is not None else getattr(app, f"xr_trig_val_{s}", 0.0)
    g_val = float(pkt_grip_raw) if pkt_grip_raw is not None else getattr(app, f"xr_grip_val_{s}", 0.0)
    cur_trig_down = getattr(app, f"xr_trig_down_{s}", False)
    cur_grip_down = getattr(app, f"xr_grip_down_{s}", False)

    # If pkt explicitly gives boolean grip/trigger, honor them as fallbacks
    pkt_grip = bool(pkt.get("grip", False))
    pkt_trig = bool(pkt.get("trigger_pressed", False) or (t_val >= 0.5))

    new_trig_down = (t_val >= 0.5 or pkt_trig) if not cur_trig_down else (t_val >= 0.4 or pkt_trig)
    new_grip_down = (g_val >= 0.5 or pkt_grip) if not cur_grip_down else (g_val >= 0.4 or pkt_grip)
    setattr(app, f"xr_trig_down_{s}", new_trig_down)
    setattr(app, f"xr_grip_down_{s}", new_grip_down)

    if getattr(app, f"xr_reengage_required_{s}", False):
        if not new_trig_down and not new_grip_down:
            setattr(app, f"xr_reengage_required_{s}", False)
            setattr(app, f"xr_tracking_lost_{s}", False)
            app._log_xr_event(f"{side.upper()} tracking restored & clutch re-armed")
        else:
            setattr(app, f"xr_clutch_state_{s}", "LOCKED")
            setattr(app, f"xr_anchor_ctrl_pos_{s}", None)
            setattr(app, f"xr_anchor_ctrl_rot_{s}", None)
            setattr(app, f"gizmo_active_{s}", False)
            setattr(app, f"xr_grip_active_{s}", False)
            return

    p_ctrl = np.array(pkt["pos"], dtype=np.float64)
    q_ctrl = pkt.get("quat")

    # Evaluate 4-state clutch mode
    if new_grip_down and new_trig_down:
        next_state = "6D"
    elif new_trig_down and not new_grip_down:
        next_state = "TRANSLATE"
    elif new_grip_down and not new_trig_down:
        next_state = "ROTATE"
    else:
        next_state = "LOCKED"

    prev_state = getattr(app, f"xr_clutch_state_{s}")
    setattr(app, f"xr_clutch_state_{s}", next_state)

    # Independent channel re-anchoring when newly engaged or transitioning
    newly_translating = next_state in ("TRANSLATE", "6D") and prev_state not in ("TRANSLATE", "6D")
    newly_rotating = next_state in ("ROTATE", "6D") and prev_state not in ("ROTATE", "6D")

    # Read current EE pose from sim or current target as base anchor T0
    ee_curr = app.env.get_ee_poses()[side] if (hasattr(app, "env") and hasattr(app.env, "get_ee_poses")) else None
    curr_target_pos = getattr(app, f"cur_target_pos_{s}")
    curr_target_rot = getattr(app, f"cur_target_rot_{s}")

    if newly_translating:
        setattr(app, f"xr_anchor_ctrl_pos_{s}", p_ctrl.copy())
        # T0 = current commanded target pose (holds last commanded target without jump)
        p_t0 = curr_target_pos.copy()
        setattr(app, f"xr_anchor_marker_pos_{s}", p_t0)
        setattr(app, f"gizmo_active_{s}", True)
        app._log_xr_event(f"{side.upper()} clutch translation re-anchored")

    if newly_rotating:
        if q_ctrl is not None and len(q_ctrl) == 4:
            r_c0 = R.from_quat([q_ctrl[1], q_ctrl[2], q_ctrl[3], q_ctrl[0]]).as_matrix()
            setattr(app, f"xr_anchor_ctrl_rot_{s}", r_c0)
            r_t0 = curr_target_rot.copy()
            setattr(app, f"xr_anchor_marker_rot_{s}", r_t0)
            setattr(app, f"gizmo_active_{s}", True)
            app._log_xr_event(f"{side.upper()} clutch rotation re-anchored")

    if next_state == "LOCKED":
        was_active = getattr(app, f"xr_anchor_ctrl_pos_{s}") is not None or getattr(app, f"xr_anchor_ctrl_rot_{s}") is not None
        setattr(app, f"xr_anchor_ctrl_pos_{s}", None)
        setattr(app, f"xr_anchor_ctrl_rot_{s}", None)
        setattr(app, f"gizmo_active_{s}", False)
        if was_active:
            from teleop.app.ik_dispatcher import freeze_arm_target
            freeze_arm_target(app, side)

    # While held: apply relative displacement
    anchor_ctrl_pos = getattr(app, f"xr_anchor_ctrl_pos_{s}")
    anchor_marker_pos = getattr(app, f"xr_anchor_marker_pos_{s}")
    if next_state in ("TRANSLATE", "6D") and anchor_ctrl_pos is not None and anchor_marker_pos is not None:
        delta_ctrl_xr = p_ctrl - anchor_ctrl_pos
        delta_rob = axis_map @ delta_ctrl_xr
        eff_scale = app.xr_scale * (app.xr_precision_scale_factor if getattr(app, f"precision_{s}") else 1.0)
        new_pos = anchor_marker_pos + eff_scale * delta_rob
        new_pos[0] = np.clip(new_pos[0], -0.2, 0.9)
        new_pos[1] = np.clip(new_pos[1], -0.9, 0.9)
        new_pos[2] = np.clip(new_pos[2], 0.5, 1.8)

        # Reach projection & tether anchor shifting
        clamped_pos, is_clamped, d_beyond = project_to_reachable_workspace(new_pos, side=side)
        if is_clamped:
            excess_m = new_pos - clamped_pos
            max_shift = float(getattr(app, "max_anchor_shift_m", 0.30))
            if np.linalg.norm(excess_m) > max_shift:
                excess_m = excess_m * (max_shift / np.linalg.norm(excess_m))
            anchor_marker_pos = anchor_marker_pos - excess_m
            setattr(app, f"xr_anchor_marker_pos_{s}", anchor_marker_pos)
            new_pos = clamped_pos
            setattr(app, f"reach_warning_{s}", f"Target out of reach ({d_beyond*100:.0f} cm beyond)")
        else:
            setattr(app, f"reach_warning_{s}", "")

        setattr(app, f"cur_target_pos_{s}", new_pos)
        setattr(app, f"gizmo_active_{s}", True)
        if hasattr(app, "target_tracer"):
            app.target_tracer.record_target_source(side, "controller clutch")

    anchor_ctrl_rot = getattr(app, f"xr_anchor_ctrl_rot_{s}")
    anchor_marker_rot = getattr(app, f"xr_anchor_marker_rot_{s}")
    if next_state in ("ROTATE", "6D") and rot_enabled and q_ctrl is not None and len(q_ctrl) == 4 and anchor_ctrl_rot is not None and anchor_marker_rot is not None:
        r_curr_ctrl = R.from_quat([q_ctrl[1], q_ctrl[2], q_ctrl[3], q_ctrl[0]]).as_matrix()
        r_rel_xr = r_curr_ctrl @ anchor_ctrl_rot.T
        r_rel_rob = rot_axis_map @ r_rel_xr @ rot_axis_map.T
        setattr(app, f"cur_target_rot_{s}", r_rel_rob @ anchor_marker_rot)
        setattr(app, f"gizmo_active_{s}", True)
        if hasattr(app, "target_tracer"):
            app.target_tracer.record_target_source(side, "controller clutch")

    setattr(app, f"xr_grip_active_{s}", bool(next_state != "LOCKED"))


def update_xr_clutch(app, left_pkt: dict, right_pkt: dict, rot_enabled: bool):
    """Updates 4-state clutch machine and SE(3) targets for both arms."""
    axis_map = get_effective_axis_map(app)
    rot_axis_map = get_effective_rot_axis_map(app)

    _process_arm_clutch(app, "left", left_pkt, rot_enabled, axis_map, rot_axis_map)
    _process_arm_clutch(app, "right", right_pkt, rot_enabled, axis_map, rot_axis_map)

    if app.xr_grip_active_l and app.xr_grip_active_r:
        app._solve_arm_ik("both")
    elif app.xr_grip_active_l:
        app._solve_arm_ik("left")
    elif app.xr_grip_active_r:
        app._solve_arm_ik("right")
