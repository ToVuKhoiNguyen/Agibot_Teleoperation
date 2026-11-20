"""
teleop/app/xr_input_handler.py
Processes Meta Quest WebXR controller packets, button inputs, gestures, and triggers clutch updates.
"""

import time
import math
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting
from teleop.app.xr_clutch_mapper import update_xr_clutch


def handle_xr_controllers_input(app, cmd: dict):
    """Processes incoming xr_controllers WebSocket packet."""
    now_t = time.time()
    cmd_t = float(cmd.get("t", 0.0))
    if cmd_t >= 1e7 and (now_t - cmd_t) > 0.150:
        if app.xr_clutch_state_l != "LOCKED" or app.xr_clutch_state_r != "LOCKED" or app.xr_grip_active_l or app.xr_grip_active_r:
            app.xr_clutch_state_l = "LOCKED"; app.xr_clutch_state_r = "LOCKED"
            app.xr_anchor_ctrl_pos_l = app.xr_anchor_ctrl_pos_r = None
            app.xr_anchor_ctrl_rot_l = app.xr_anchor_ctrl_rot_r = None
            app.gizmo_active_l = app.gizmo_active_r = False
            app.xr_grip_active_l = app.xr_grip_active_r = False
            app.xr_reengage_required_l = app.xr_reengage_required_r = True
        return

    dt_pkt = min(0.1, max(0.005, now_t - getattr(app, "xr_last_pkt_t", now_t - 0.02)))
    app.xr_last_pkt_t = now_t
    app.xr_last_packet_time = now_t
    app.xr_input_active = True
    app.xr_pkt_count += 1
    app.xr_total_pkts += 1

    if now_t - app.xr_rate_timer >= 1.0:
        app.xr_rate_hz = app.xr_pkt_count / (now_t - app.xr_rate_timer)
        app.xr_rate_timer = now_t
        app.xr_pkt_count = 0

    left_pkt = cmd.get("left")
    right_pkt = cmd.get("right")
    rot_enabled = cmd.get("rotation_enabled", app.xr_rotation_enabled)
    app.xr_rig_yaw = float(cmd.get("rig_yaw", math.radians(cmd.get("rig_yaw_deg", 0.0))))

    left_track = bool(left_pkt and left_pkt.get("pos") is not None and left_pkt.get("tracking", True) and not left_pkt.get("tracking_loss", False))
    right_track = bool(right_pkt and right_pkt.get("pos") is not None and right_pkt.get("tracking", True) and not right_pkt.get("tracking_loss", False))

    app.xr_left_connected = bool(left_pkt is not None)
    app.xr_right_connected = bool(right_pkt is not None)

    # Button states extraction
    btn_x = False
    btn_y = False
    btn_menu_l = False
    if left_pkt:
        btns_l = left_pkt.get("buttons") if isinstance(left_pkt.get("buttons"), dict) else {}
        btn_x = bool(left_pkt.get("btn_x", btns_l.get("primary", False)))
        btn_y = bool(left_pkt.get("btn_y", btns_l.get("secondary", False)))
        btn_menu_l = bool(left_pkt.get("btn_menu", btns_l.get("menu", False)))
        app.xr_grip_val_l = float(left_pkt.get("grip_val", 1.0 if left_pkt.get("grip") else 0.0))
        app.xr_trig_val_l = float(left_pkt.get("trigger", 0.0))
        app.xr_thumb_click_l = bool(left_pkt.get("thumbstick_click", False))
        if "thumbstick" in left_pkt:
            app.xr_stick_x_l = float(left_pkt["thumbstick"][0])
            app.xr_stick_y_l = float(left_pkt["thumbstick"][1])

    btn_a = False
    btn_b = False
    btn_menu_r = False
    if right_pkt:
        btns_r = right_pkt.get("buttons") if isinstance(right_pkt.get("buttons"), dict) else {}
        btn_a = bool(right_pkt.get("btn_a", btns_r.get("primary", False)))
        btn_b = bool(right_pkt.get("btn_b", btns_r.get("secondary", False)))
        btn_menu_r = bool(right_pkt.get("btn_menu", btns_r.get("menu", False)))
        app.xr_grip_val_r = float(right_pkt.get("grip_val", 1.0 if right_pkt.get("grip") else 0.0))
        app.xr_trig_val_r = float(right_pkt.get("trigger", 0.0))
        app.xr_thumb_click_r = bool(right_pkt.get("thumbstick_click", False))
        if "thumbstick" in right_pkt:
            app.xr_stick_x_r = float(right_pkt["thumbstick"][0])
            app.xr_stick_y_r = float(right_pkt["thumbstick"][1])

    app.xr_btn_x = btn_x
    app.xr_btn_y = btn_y
    app.xr_btn_a = btn_a
    app.xr_btn_b = btn_b
    app.xr_btn_menu_l = btn_menu_l
    app.xr_btn_menu_r = btn_menu_r

    # No controller button triggers E-STOP (Menu and the B+Y combo were removed). E-STOP stays on the web panel and the
    # physical E-Stop held by the second person. The Menu button is only displayed in the panel.

    # B (right) / Y (left) short tap = start / stop recording an episode (was: precision toggle).
    # Toggles on release if held < 0.5 s. If both are down at the same time the press is ignored (neither toggles).
    if btn_b and btn_y:
        app.by_combo_seen = True
    for btn, attr in ((btn_b, "b_tap_start"), (btn_y, "y_tap_start")):
        t_start = getattr(app, attr, None)
        if btn and t_start is None: setattr(app, attr, now_t)
        elif not btn and t_start is not None:
            setattr(app, attr, None)
            if (now_t - t_start) < 0.5 and not getattr(app, "by_combo_seen", False):
                from teleop.app import episode_recorder as er
                if app.rec_active: er.stop_episode(app)
                else: er.start_episode(app)
    if not btn_b and not btn_y:
        app.by_combo_seen = False

    app.xr_prev_btn_b = btn_b
    app.xr_prev_btn_y = btn_y

    # Buttons X (left) and A (right):
    # Short tap (press & release < 3.0s): switch both arms to Arms Up posture (preset 'up').
    # Hold >= 3.0s: return BOTH arms to canonical HOME posture.
    for btn, attr_start, attr_prev, attr_trig, prog_attr, label in (
        (btn_x, "x_hold_start", "xr_prev_btn_x", "x_home_triggered", "xr_home_progress_l", "L X"),
        (btn_a, "a_hold_start", "xr_prev_btn_a", "a_home_triggered", "xr_home_progress_r", "R A"),
    ):
        prev = getattr(app, attr_prev, False)
        t_start = getattr(app, attr_start, None)
        trig = getattr(app, attr_trig, False)

        if btn:
            if t_start is None:
                t_start = now_t
                setattr(app, attr_start, t_start)
                setattr(app, attr_trig, False)
                trig = False
            dur = now_t - t_start
            setattr(app, prog_attr, min(1.0, dur / 3.0))
            if dur >= 3.0 and not trig:
                from teleop.app.homing_controller import reset_arm_to_home
                reset_arm_to_home(app, "both")
                setattr(app, attr_trig, True)
                app._log_xr_event(f"{label} hold 3.0s -> HOME (both arms)")
        else:
            if t_start is not None:
                dur = now_t - t_start
                if dur < 3.0 and not trig:
                    from teleop.app.joint_preset import start_preset
                    start_preset(app, "up", side="both")
                    app._log_xr_event(f"{label} tap -> ARMS UP (both arms)")
            setattr(app, attr_start, None)
            setattr(app, attr_trig, False)
            setattr(app, prog_attr, 0.0)

        setattr(app, attr_prev, btn)

    # Thumbstick Y: finger closure velocity control (hold value if tracking lost)
    if left_track and left_pkt and "thumbstick" in left_pkt:
        sy = app.xr_stick_y_l
        if sy < -app.xr_stick_deadband:
            app.hand_closure_l = min(1.0, app.hand_closure_l + abs(sy) * app.xr_closure_rate * dt_pkt)
        elif sy > app.xr_stick_deadband:
            app.hand_closure_l = max(0.0, app.hand_closure_l - abs(sy) * app.xr_closure_rate * dt_pkt)

    # Left Thumbstick click: Panic Open
    if left_track and app.xr_thumb_click_l:
        if not app.xr_thumb_click_prev_l:
            app.hand_closure_l = 0.0
            app._log_xr_event("L Thumb click -> PANIC OPEN")
        app.xr_thumb_click_prev_l = True
    else:
        app.xr_thumb_click_prev_l = False

    # Left Thumbstick X: cycle grasp type
    if left_track and left_pkt and "thumbstick" in left_pkt:
        sx = app.xr_stick_x_l
        sy = app.xr_stick_y_l
        g_list = AgibotHandRetargeting.GRASP_TYPES
        y_neutral = abs(sy) < app.xr_stick_deadband
        if y_neutral and abs(sx) > abs(sy):
            if sx > app.xr_stick_x_deadband and app.stick_x_state_l != 1:
                idx = g_list.index(app.grasp_type_l) if app.grasp_type_l in g_list else 0
                app.grasp_type_l = g_list[(idx + 1) % len(g_list)]
                app.stick_x_state_l = 1
                app._log_xr_event(f"L Grasp: {app.grasp_type_l}")
            elif sx < -app.xr_stick_x_deadband and app.stick_x_state_l != -1:
                idx = g_list.index(app.grasp_type_l) if app.grasp_type_l in g_list else 0
                app.grasp_type_l = g_list[(idx - 1) % len(g_list)]
                app.stick_x_state_l = -1
                app._log_xr_event(f"L Grasp: {app.grasp_type_l}")
        if abs(sx) < app.xr_stick_x_deadband * 0.7:
            app.stick_x_state_l = 0

    if not getattr(app, "direct_hand_override_l", False) and not app.hand_sweep_active:
        if str(app.grasp_type_l).lower() == "open":
            app.grasp_type_l = "cylinder"  # "open" ignores closure: the thumbstick could never close the hand
        curls_l, pinch_l = AgibotHandRetargeting.get_grasp_curls(app.grasp_type_l, app.hand_closure_l)
        app.target_curls_l = curls_l
        app.target_pinch_l = pinch_l

    # Right Thumbstick Y
    if right_track and right_pkt and "thumbstick" in right_pkt:
        sy = app.xr_stick_y_r
        if sy < -app.xr_stick_deadband:
            app.hand_closure_r = min(1.0, app.hand_closure_r + abs(sy) * app.xr_closure_rate * dt_pkt)
        elif sy > app.xr_stick_deadband:
            app.hand_closure_r = max(0.0, app.hand_closure_r - abs(sy) * app.xr_closure_rate * dt_pkt)

    # Right Thumbstick click: Panic Open
    if right_track and app.xr_thumb_click_r:
        if not app.xr_thumb_click_prev_r:
            app.hand_closure_r = 0.0
            app._log_xr_event("R Thumb click -> PANIC OPEN")
        app.xr_thumb_click_prev_r = True
    else:
        app.xr_thumb_click_prev_r = False

    # Right Thumbstick X: cycle grasp type
    if right_track and right_pkt and "thumbstick" in right_pkt:
        sx = app.xr_stick_x_r
        sy = app.xr_stick_y_r
        g_list = AgibotHandRetargeting.GRASP_TYPES
        y_neutral = abs(sy) < app.xr_stick_deadband
        if y_neutral and abs(sx) > abs(sy):
            if sx > app.xr_stick_x_deadband and app.stick_x_state_r != 1:
                idx = g_list.index(app.grasp_type_r) if app.grasp_type_r in g_list else 0
                app.grasp_type_r = g_list[(idx + 1) % len(g_list)]
                app.stick_x_state_r = 1
                app._log_xr_event(f"R Grasp: {app.grasp_type_r}")
            elif sx < -app.xr_stick_x_deadband and app.stick_x_state_r != -1:
                idx = g_list.index(app.grasp_type_r) if app.grasp_type_r in g_list else 0
                app.grasp_type_r = g_list[(idx - 1) % len(g_list)]
                app.stick_x_state_r = -1
                app._log_xr_event(f"R Grasp: {app.grasp_type_r}")
        if abs(sx) < app.xr_stick_x_deadband * 0.7:
            app.stick_x_state_r = 0

    if not getattr(app, "direct_hand_override_r", False) and not app.hand_sweep_active:
        if str(app.grasp_type_r).lower() == "open":
            app.grasp_type_r = "cylinder"  # "open" ignores closure: the thumbstick could never close the hand
        curls_r, pinch_r = AgibotHandRetargeting.get_grasp_curls(app.grasp_type_r, app.hand_closure_r)
        app.target_curls_r = curls_r
        app.target_pinch_r = pinch_r

    # Run 4-State Smart Clutch integration
    update_xr_clutch(app, left_pkt, right_pkt, rot_enabled)

