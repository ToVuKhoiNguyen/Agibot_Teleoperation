"""
teleop/app/xr_input_handler.py
Processes Meta Quest WebXR controller packets, button inputs, gestures, and triggers clutch updates.
"""

import time
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting
from teleop.app.xr_clutch_mapper import update_xr_clutch


def handle_xr_controllers_input(app, cmd: dict):
    """Processes incoming xr_controllers WebSocket packet."""
    now_t = time.time()
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

    app.xr_left_connected = bool(left_pkt is not None)
    app.xr_right_connected = bool(right_pkt is not None)

    # Button states extraction
    btn_x = False
    btn_y = False
    btn_menu_l = False
    if left_pkt:
        btn_x = bool(left_pkt.get("btn_x", left_pkt.get("buttons", {}).get("primary", False)))
        btn_y = bool(left_pkt.get("btn_y", left_pkt.get("buttons", {}).get("secondary", False)))
        btn_menu_l = bool(left_pkt.get("btn_menu", left_pkt.get("buttons", {}).get("menu", False)))
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
        btn_a = bool(right_pkt.get("btn_a", right_pkt.get("buttons", {}).get("primary", False)))
        btn_b = bool(right_pkt.get("btn_b", right_pkt.get("buttons", {}).get("secondary", False)))
        btn_menu_r = bool(right_pkt.get("btn_menu", right_pkt.get("buttons", {}).get("menu", False)))
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

    # Menu button: immediate E-STOP
    if btn_menu_l or btn_menu_r:
        if not app.state_estop:
            app.state_estop = True
            app._log_xr_event("Menu button -> E-STOP")
            print("[ALERT] E-STOP ACTIVE (Quest Menu button pressed)")

    # Fallback E-STOP Combo: Hold B + Y simultaneously > 0.5s
    if btn_b and btn_y:
        if app.xr_estop_combo_start_time is None:
            app.xr_estop_combo_start_time = now_t
        elif (now_t - app.xr_estop_combo_start_time) >= app.xr_estop_hold_thresh:
            if not app.state_estop:
                app.state_estop = True
                app._log_xr_event("B+Y hold 0.5s -> E-STOP")
                print("[ALERT] E-STOP ACTIVE (Quest Combo B+Y held > 0.5s)")
    else:
        app.xr_estop_combo_start_time = None
        # Precision mode toggles on rising edge
        if btn_b and not app.xr_prev_btn_b:
            app.precision_r = not app.precision_r
            app._update_precision("right", app.precision_r)
            app._log_xr_event(f"R B pressed -> Precision {'ON' if app.precision_r else 'OFF'}")
        if btn_y and not app.xr_prev_btn_y:
            app.precision_l = not app.precision_l
            app._update_precision("left", app.precision_l)
            app._log_xr_event(f"L Y pressed -> Precision {'ON' if app.precision_l else 'OFF'}")

    app.xr_prev_btn_b = btn_b
    app.xr_prev_btn_y = btn_y

    # Home hold: X (left) hold 1.5s -> left arm goes Home
    if btn_x:
        if app.x_hold_start is None:
            app.x_hold_start = now_t
            app.x_home_triggered = False
        dur = now_t - app.x_hold_start
        app.xr_home_progress_l = min(1.0, dur / app.xr_home_hold_s)
        if app.xr_home_progress_l >= 1.0 and not app.x_home_triggered:
            app._reset_arm_to_home("left")
            app.x_home_triggered = True
            app._log_xr_event("L X hold 1.5s -> HOME")
    else:
        app.x_hold_start = None
        app.x_home_triggered = False
        app.xr_home_progress_l = 0.0

    # Home hold: A (right) hold 1.5s -> right arm goes Home
    if btn_a:
        if app.a_hold_start is None:
            app.a_hold_start = now_t
            app.a_home_triggered = False
        dur = now_t - app.a_hold_start
        app.xr_home_progress_r = min(1.0, dur / app.xr_home_hold_s)
        if app.xr_home_progress_r >= 1.0 and not app.a_home_triggered:
            app._reset_arm_to_home("right")
            app.a_home_triggered = True
            app._log_xr_event("R A hold 1.5s -> HOME")
    else:
        app.a_hold_start = None
        app.a_home_triggered = False
        app.xr_home_progress_r = 0.0

    # Thumbstick Y: finger closure velocity control
    if left_pkt and "thumbstick" in left_pkt:
        sy = app.xr_stick_y_l
        if sy < -app.xr_stick_deadband:
            app.hand_closure_l = min(1.0, app.hand_closure_l + abs(sy) * app.xr_closure_rate * dt_pkt)
        elif sy > app.xr_stick_deadband:
            app.hand_closure_l = max(0.0, app.hand_closure_l - abs(sy) * app.xr_closure_rate * dt_pkt)

    # Left Thumbstick click: Panic Open
    if app.xr_thumb_click_l:
        if not app.xr_thumb_click_prev_l:
            app.hand_closure_l = 0.0
            app._log_xr_event("L Thumb click -> PANIC OPEN")
        app.xr_thumb_click_prev_l = True
    else:
        app.xr_thumb_click_prev_l = False

    # Left Thumbstick X: cycle grasp type
    if left_pkt and "thumbstick" in left_pkt:
        sx = app.xr_stick_x_l
        g_list = AgibotHandRetargeting.GRASP_TYPES
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
        elif abs(sx) < app.xr_stick_x_deadband * 0.7:
            app.stick_x_state_l = 0

    if not getattr(app, "direct_hand_override_l", False) and not app.hand_sweep_active:
        curls_l, pinch_l = AgibotHandRetargeting.get_grasp_curls(app.grasp_type_l, app.hand_closure_l)
        app.target_curls_l = curls_l
        app.target_pinch_l = pinch_l

    # Right Thumbstick Y
    if right_pkt and "thumbstick" in right_pkt:
        sy = app.xr_stick_y_r
        if sy < -app.xr_stick_deadband:
            app.hand_closure_r = min(1.0, app.hand_closure_r + abs(sy) * app.xr_closure_rate * dt_pkt)
        elif sy > app.xr_stick_deadband:
            app.hand_closure_r = max(0.0, app.hand_closure_r - abs(sy) * app.xr_closure_rate * dt_pkt)

    # Right Thumbstick click: Panic Open
    if app.xr_thumb_click_r:
        if not app.xr_thumb_click_prev_r:
            app.hand_closure_r = 0.0
            app._log_xr_event("R Thumb click -> PANIC OPEN")
        app.xr_thumb_click_prev_r = True
    else:
        app.xr_thumb_click_prev_r = False

    # Right Thumbstick X: cycle grasp type
    if right_pkt and "thumbstick" in right_pkt:
        sx = app.xr_stick_x_r
        g_list = AgibotHandRetargeting.GRASP_TYPES
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
        elif abs(sx) < app.xr_stick_x_deadband * 0.7:
            app.stick_x_state_r = 0

    if not getattr(app, "direct_hand_override_r", False) and not app.hand_sweep_active:
        curls_r, pinch_r = AgibotHandRetargeting.get_grasp_curls(app.grasp_type_r, app.hand_closure_r)
        app.target_curls_r = curls_r
        app.target_pinch_r = pinch_r

    # Run 4-State Smart Clutch integration
    update_xr_clutch(app, left_pkt, right_pkt, rot_enabled)

