"""
teleop/app/handtrack_cmd.py
WebXR bare-hand tracking: absolute 1:1, One-Euro pos/rot filter, lead prediction,
dropout bridging, low-latency finger One-Euro, and 200 Hz follower integration.
"""
import time, math
import numpy as np
from scipy.spatial.transform import Rotation as R
from teleop.hand.retarget import HandRetargeter, COMFORTABLE_LIMITS, resolve_joints_array
from teleop.kinematics.signal_filters import OneEuroFilter, OneEuroQuaternionFilter
from teleop.app.xr_clutch_mapper import get_effective_axis_map, get_effective_rot_axis_map

P_ROBOT_SHOULDER_L = np.array([-0.038, 0.191, 1.272], dtype=np.float64)
P_ROBOT_SHOULDER_R = np.array([-0.038, -0.191, 1.272], dtype=np.float64)

def _ensure_handtrack_state(app):
    if hasattr(app, "handtrack_retargeter_l"): return
    mc, bv, dc = float(getattr(app, "hand_min_cutoff", 1.5)), float(getattr(app, "hand_beta", 15.0)), float(getattr(app, "hand_dcutoff", 1.0))
    app.handtrack_retargeter_l, app.handtrack_retargeter_r = HandRetargeter("left"), HandRetargeter("right")
    app.handtrack_filter_pos_l, app.handtrack_filter_pos_r = OneEuroFilter(mc, bv, dc), OneEuroFilter(mc, bv, dc)
    app.handtrack_filter_rot_l, app.handtrack_filter_rot_r = OneEuroQuaternionFilter(mc, bv, dc), OneEuroQuaternionFilter(mc, bv, dc)
    app.hand_follow_mode, app.hand_workspace_scale = getattr(app, "hand_follow_mode", "absolute"), float(getattr(app, "hand_workspace_scale", 1.0))
    app.hand_origin_offset, app.hand_lead_ms = np.array(getattr(app, "hand_origin_offset", [0.0, 0.0, 0.0]), dtype=np.float64), float(getattr(app, "hand_lead_ms", 20.0))
    app.handtrack_clutch_l = app.handtrack_clutch_r = False
    app.handtrack_anchor_w_l = app.handtrack_anchor_w_r = app.handtrack_anchor_r_l = app.handtrack_anchor_r_r = None
    app.handtrack_rcal_l, app.handtrack_rcal_r = getattr(app, "robot_home_rot_l", np.eye(3)).copy(), getattr(app, "robot_home_rot_r", np.eye(3)).copy()
    app.handtrack_prev_wrist_l = app.handtrack_prev_wrist_r = None
    app.handtrack_loss_start_l = app.handtrack_loss_start_r = app.handtrack_regain_t_l = app.handtrack_regain_t_r = 0.0
    app.handtrack_was_lost_l = app.handtrack_was_lost_r = False
    app.handtrack_recenter_hold = app.handtrack_pkt_count = 0
    app.handtrack_rate_hz, app.handtrack_rate_timer = 0.0, time.time()
    app.fingers_follow_always = True
    app.handtrack_live_bars, app.handtrack_diag_status = {"left": None, "right": None}, {"left": {}, "right": {}}
    app.limit_indicator, app.clamp_counters = None, {"workspace": 0, "collision": 0, "total": 0}
    app.cal_head_pos = getattr(app, "cal_head_pos", np.array([0.0, 1.5, 0.0], dtype=np.float64))
    app.cal_head_yaw, app.head_follow, app.handtrack_e2e_ms = float(getattr(app, "cal_head_yaw", 0.0)), getattr(app, "head_follow", True), 0.0
    app.handtrack_pinch_time_l = app.handtrack_pinch_time_r = 0.0
    app.handtrack_pinch_clutch_l = app.handtrack_pinch_clutch_r = False

def _compute_abs_target(app, side: str, p_w: np.ndarray, axis_map: np.ndarray) -> np.ndarray:
    h_pos, h_yaw = getattr(app, "cal_head_pos", np.array([0.0, 1.5, 0.0])), float(getattr(app, "cal_head_yaw", 0.0))
    lat = -0.20 if side == "left" else 0.20
    d_sh = np.array([lat * math.cos(h_yaw) - 0.05 * math.sin(h_yaw), -0.20, -lat * math.sin(h_yaw) - 0.05 * math.cos(h_yaw)])
    diff = p_w - h_pos - d_sh
    v_yaw = np.array([diff[0] * math.cos(h_yaw) + diff[2] * math.sin(h_yaw), diff[1], -diff[0] * math.sin(h_yaw) + diff[2] * math.cos(h_yaw)])
    return (P_ROBOT_SHOULDER_L if side == "left" else P_ROBOT_SHOULDER_R) + app.hand_origin_offset + app.hand_workspace_scale * (axis_map @ v_yaw)

def _apply_head_tracking(app, hd: dict, now_t: float):
    if not hd or not getattr(app, "head_follow", True) or not getattr(app, "env", None): return
    yaw, pitch = float(np.clip(hd.get("yaw", 0.0), -2.2, 2.2)), float(np.clip(hd.get("pitch", 0.0), -0.5, 0.5))
    if hasattr(app, "filter_head_yaw") and app.filter_head_yaw: yaw, pitch = float(app.filter_head_yaw.filter(yaw, now_t)), float(app.filter_head_pitch.filter(pitch, now_t))
    try:
        d = app.env.data
        if len(d.ctrl) > 27: d.ctrl[26], d.ctrl[27] = yaw, pitch
        if len(d.qpos) > 51: d.qpos[50], d.qpos[51] = yaw, pitch
    except Exception: pass

def _process_single_hand(app, side: str, pkt: dict, now_t: float, dt: float, axis_map: np.ndarray, rot_axis_map: np.ndarray = None) -> dict:
    mode = getattr(app, "hand_follow_mode", "absolute").lower()
    if mode == "always": mode = "relative"
    retargeter = app.handtrack_retargeter_l if side == "left" else app.handtrack_retargeter_r
    filt_pos = app.handtrack_filter_pos_l if side == "left" else app.handtrack_filter_pos_r
    filt_rot = app.handtrack_filter_rot_l if side == "left" else app.handtrack_filter_rot_r
    is_clutched = getattr(app, f"handtrack_clutch_{side[0]}")
    real_tracked = bool(pkt and pkt.get("tracking", False) and pkt.get("confidence", 0.0) >= 0.35)
    is_tracked = real_tracked

    if not is_tracked:
        ls = getattr(app, f"handtrack_loss_start_{side[0]}") or now_t
        setattr(app, f"handtrack_loss_start_{side[0]}", ls)
        dt_loss = now_t - ls
        if dt_loss < 0.100 and getattr(app, f"handtrack_prev_wrist_{side[0]}") is not None and filt_pos.dx_filt.y is not None:
            v_ext = filt_pos.dx_filt.y
            if np.linalg.norm(v_ext) > 0.02: track_p, is_tracked = getattr(app, f"handtrack_prev_wrist_{side[0]}") + v_ext * dt, True
        if not is_tracked:
            setattr(app, f"handtrack_valid_since_{side[0]}", 0.0); setattr(app, f"handtrack_valid_count_{side[0]}", 0)
            if side == "left": app.gizmo_active_l = False
            else: app.gizmo_active_r = False
            if dt_loss >= 0.150:
                setattr(app, f"handtrack_was_lost_{side[0]}", True); setattr(app, f"handtrack_anchor_w_{side[0]}", None); setattr(app, f"handtrack_clutch_{side[0]}", False)
                app.status_warning = f"{side.upper()} HAND TRACKING LOST"
            return {"tracking": False, "clutch": False, "flexion": [0]*5, "is_open": False, "vel": 0.0, "upd": False}

    if real_tracked: setattr(app, f"handtrack_loss_start_{side[0]}", 0.0)
    pkt_dict = pkt or {}
    w_data, joints = pkt_dict.get("wrist") or {}, resolve_joints_array(pkt_dict.get("joints", []))
    raw_p, raw_q = np.array(w_data.get("pos", [0, 0, 0]), dtype=np.float64), w_data.get("quat", [1, 0, 0, 0])
    if not ('track_p' in locals() and not ('joints' in pkt and len(joints) >= 25)):
        track_p = 0.5 * (joints[0] + joints[10]) if len(joints) >= 25 else raw_p

    prev_p = getattr(app, f"handtrack_prev_wrist_{side[0]}")
    if prev_p is not None and np.linalg.norm(track_p - prev_p) > 0.35:
        return {"tracking": True, "clutch": is_clutched, "flexion": retargeter.prev_flexion.tolist(), "is_open": False, "vel": 0.0, "upd": False}
    setattr(app, f"handtrack_prev_wrist_{side[0]}", track_p.copy())

    p_w = filt_pos.filter(track_p, now_t)
    p_w_lead, v_filt = p_w, filt_pos.dx_filt.y
    lead_ms = getattr(app, "hand_lead_ms", 20.0)
    if lead_ms > 0 and v_filt is not None and np.linalg.norm(v_filt) >= 0.05:
        lv = v_filt * (lead_ms / 1000.0)
        ld = float(np.linalg.norm(lv))
        if ld > 0.03: lv *= (0.03 / ld)
        p_w_lead = p_w + lv
    v_eff = axis_map @ (v_filt if v_filt is not None else np.zeros(3))
    if side == "left": app.handtrack_vel_l = v_eff
    else: app.handtrack_vel_r = v_eff

    angles, opp, normal = retargeter.compute_finger_angles(joints)
    flexion, norm_opp = retargeter.normalize_flexion(angles, opp, now_t=now_t)
    flex_filt, opp_filt = retargeter.apply_filter_and_hysteresis(flexion, norm_opp, now_t=now_t, dt=dt)
    gestures = retargeter.detect_gestures(joints, p_w, flex_filt, normal, dt, current_clutch=is_clutched)
    was_lost, regain_t = getattr(app, f"handtrack_was_lost_{side[0]}"), getattr(app, f"handtrack_regain_t_{side[0]}")
    v_since = getattr(app, f"handtrack_valid_since_{side[0]}", 0.0) or now_t
    setattr(app, f"handtrack_valid_since_{side[0]}", v_since)
    v_cnt = getattr(app, f"handtrack_valid_count_{side[0]}", 0) + 1
    setattr(app, f"handtrack_valid_count_{side[0]}", v_cnt)
    is_stable = (now_t - v_since >= 0.28) or (v_cnt >= 10)

    h_pos, h_yaw = getattr(app, "cal_head_pos", np.array([0.0, 1.5, 0.0])), float(getattr(app, "cal_head_yaw", 0.0))
    d_h = p_w - h_pos
    cy, sy = math.cos(-h_yaw), math.sin(-h_yaw)
    dx_h, dy_h, dz_h = d_h[0]*cy - d_h[2]*sy, d_h[1], d_h[0]*sy + d_h[2]*cy
    can_follow = is_stable and (abs(dx_h) <= 0.75 and -0.85 <= dy_h <= 0.45 and -1.05 <= dz_h <= 0.05)
    target_updating = False

    # Gesture or pinch clutch detection
    pinch_dist = np.linalg.norm(joints[4] - joints[8]) if len(joints) >= 9 else 0.10
    p_time = getattr(app, f"handtrack_pinch_time_{side[0]}", 0.0) + (dt if pinch_dist < 0.035 else -dt)
    p_time = max(0.0, p_time)
    p_clutch = True if p_time >= 0.30 else (False if pinch_dist > 0.050 else getattr(app, f"handtrack_pinch_clutch_{side[0]}", False))
    setattr(app, f"handtrack_pinch_time_{side[0]}", p_time)
    setattr(app, f"handtrack_pinch_clutch_{side[0]}", p_clutch)

    if mode == "absolute":
        p_target_abs = _compute_abs_target(app, side, p_w_lead, axis_map)
        if was_lost or getattr(app, f"handtrack_anchor_r_{side[0]}") is None:
            regain_t = now_t if was_lost else 0.0
            setattr(app, f"handtrack_regain_t_{side[0]}", regain_t)
            setattr(app, f"handtrack_anchor_r_{side[0]}", (app.cur_target_pos_l if side == "left" else app.cur_target_pos_r).copy())
            if was_lost: setattr(app, f"handtrack_was_lost_{side[0]}", False)
        anchor_r = getattr(app, f"handtrack_anchor_r_{side[0]}")
        alpha = float(np.clip((now_t - regain_t) / 0.5, 0.0, 1.0)) if (regain_t > 0 and now_t - regain_t < 0.5) else 1.0
        if alpha >= 1.0: setattr(app, f"handtrack_regain_t_{side[0]}", 0.0)
        raw_target, target_updating, is_clutched = (1.0 - alpha) * anchor_r + alpha * p_target_abs, can_follow, can_follow
        setattr(app, f"handtrack_clutch_{side[0]}", is_clutched)
    elif mode == "relative":
        anchor_w = getattr(app, f"handtrack_anchor_w_{side[0]}")
        if was_lost or anchor_w is None:
            setattr(app, f"handtrack_regain_t_{side[0]}", now_t if was_lost else 0.0)
            setattr(app, f"handtrack_was_lost_{side[0]}", False); setattr(app, f"handtrack_anchor_w_{side[0]}", p_w_lead.copy())
            setattr(app, f"handtrack_anchor_r_{side[0]}", (app.cur_target_pos_l if side == "left" else app.cur_target_pos_r).copy())
            is_clutched = True; setattr(app, f"handtrack_clutch_{side[0]}", True)
        anchor_w, anchor_r = getattr(app, f"handtrack_anchor_w_{side[0]}"), getattr(app, f"handtrack_anchor_r_{side[0]}")
        alpha = float(np.clip((now_t - regain_t) / 0.5, 0.0, 1.0)) if (now_t - regain_t < 0.5) else 1.0
        raw_target, target_updating = anchor_r + app.hand_origin_offset + app.hand_workspace_scale * alpha * (axis_map @ (p_w_lead - anchor_w)), can_follow
    else:  # gesture
        if gestures["engage"] or p_clutch:
            setattr(app, f"handtrack_clutch_{side[0]}", True); setattr(app, f"handtrack_anchor_w_{side[0]}", p_w_lead.copy())
            setattr(app, f"handtrack_anchor_r_{side[0]}", (app.cur_target_pos_l if side == "left" else app.cur_target_pos_r).copy())
            is_clutched = True
        elif gestures["release"] or (not p_clutch and getattr(app, f"handtrack_pinch_clutch_{side[0]}", False)):
            setattr(app, f"handtrack_clutch_{side[0]}", False); setattr(app, f"handtrack_anchor_w_{side[0]}", None)
            is_clutched = False
        anchor_w, anchor_r = getattr(app, f"handtrack_anchor_w_{side[0]}"), getattr(app, f"handtrack_anchor_r_{side[0]}")
        if is_clutched and anchor_w is not None and anchor_r is not None:
            raw_target, target_updating = anchor_r + app.hand_origin_offset + app.hand_workspace_scale * (axis_map @ (p_w_lead - anchor_w)), can_follow

    if len(raw_q) == 4 and target_updating:
        r_map = rot_axis_map if rot_axis_map is not None else axis_map
        r_cal = getattr(app, f"handtrack_rcal_{side[0]}", np.eye(3))
        r_filt = filt_rot.filter(R.from_quat([raw_q[1], raw_q[2], raw_q[3], raw_q[0]]).as_matrix(), now_t)
        rot_tgt = (r_map @ r_filt @ r_map.T) @ r_cal
        if side == "left": app.cur_target_rot_l = rot_tgt
        else: app.cur_target_rot_r = rot_tgt

    clamped_ws = False
    if target_updating:
        new_pos = np.clip(raw_target, [-0.2, -0.9, 0.5], [0.9, 0.9, 1.8])
        clamped_ws = not np.allclose(raw_target, new_pos, atol=1e-4)
        cur_pos = app.cur_target_pos_l if side == "left" else app.cur_target_pos_r
        if np.linalg.norm(new_pos - cur_pos) <= 0.00201: target_updating = False
    if target_updating:
        if hasattr(app, "target_tracer"): app.target_tracer.record_target_source(side, "hand tracking")
        if side == "left": app.cur_target_pos_l, app.gizmo_active_l = new_pos, True; app._solve_arm_ik("left")
        else: app.cur_target_pos_r, app.gizmo_active_r = new_pos, True; app._solve_arm_ik("right")
    elif side == "left": app.gizmo_active_l = False
    else: app.gizmo_active_r = False

    safe_q6 = retargeter.retarget_to_active_joints(flex_filt, opp_filt)
    curr_safe = app.current_safe_hand_l if side == "left" else app.current_safe_hand_r
    safe_q6, is_col, _ = app.collision_checker.filter_hand_step(side, curr_safe, safe_q6, margin=0.001)
    if side == "left": app.current_safe_hand_l = app.last_hand_cmd_left = safe_q6; app.hand_closure_l = float(np.mean(flex_filt))
    else: app.current_safe_hand_r = app.last_hand_cmd_right = safe_q6; app.hand_closure_r = float(np.mean(flex_filt))

    if hasattr(app, "hand_recorder") and app.hand_recorder and getattr(app.hand_recorder, "enabled", False):
        act_p = app.env.get_ee_poses()[side]["pos"] if hasattr(app.env, "get_ee_poses") else [0, 0, 0]
        app.hand_recorder.record_frame(now_t, side, {"pos": raw_p.tolist(), "quat": raw_q}, {"pos": p_w.tolist(), "quat": raw_q}, is_clutched, {"pos": (app.cur_target_pos_l if side == "left" else app.cur_target_pos_r).tolist()}, {"pos": list(act_p)}, joints.tolist() if len(joints) >= 25 else [], [float(a) for a in angles], [float(f) for f in flexion], [float(f) for f in flex_filt], [float(s) for s in safe_q6[:5]], retargeter.cal, bool(pkt_dict.get("tracking", True)), float(pkt_dict.get("confidence", 1.0)), float(getattr(app, f"last_ik_residual_{side[0]}", 0.0)), {"workspace": bool(clamped_ws), "collision": bool(is_col), "joint_limit": False})

    c_str = "ABSOLUTE" if mode == "absolute" else ("ENGAGED" if is_clutched else "LOCKED")
    nz = int(np.sum([np.linalg.norm(j) > 1e-4 for j in joints])) if len(joints) >= 25 else 0
    app.handtrack_diag_status[side] = {"mode": mode, "valid": bool(pkt_dict.get("tracking", True)), "conf": round(float(pkt_dict.get("confidence", 1.0)), 2), "src": "hand", "clutch": c_str, "palm": np.round(track_p, 3).tolist(), "nz": nz, "flex": round(float(np.mean(flex_filt)), 2), "upd": bool(target_updating and not clamped_ws), "e2e_ms": getattr(app, "handtrack_e2e_ms", 0.0)}
    c_norm = [safe_q6[i] / COMFORTABLE_LIMITS[i] for i in (1, 2, 3, 4, 5)]
    app.handtrack_live_bars[side] = {"raw": [round(float(f), 3) for f in flex_filt], "cmd": [round(float(c), 3) for c in c_norm], "act": [round(float(c), 3) for c in c_norm]}
    return {"tracking": True, "clutch": bool(is_clutched), "flexion": [round(float(f), 3) for f in flex_filt], "is_open": bool(gestures["is_open"]), "vel": float(gestures["vel"]), "estop_candidate": bool(gestures["estop_candidate"]), "upd": bool(target_updating)}

def handle_xr_handtrack_input(app, cmd: dict):
    _ensure_handtrack_state(app)
    if (getattr(app, "backend", "sim") or "sim").lower() in ("real", "ros2") and not getattr(app, "allow_handtracking_real", False):
        app.status_warning, app.handtrack_clutch_l = "HAND-TRACKING REFUSED ON REAL ROBOT (Sim-only feature)", False
        if hasattr(app, "robot_safety") and app.robot_safety: app.robot_safety.hold()
        return

    cmd_t = float(cmd.get("t", 0.0))
    now_t = cmd_t if (0 < cmd_t < 1e7) else time.time()
    dt = min(0.1, max(0.005, now_t - getattr(app, "handtrack_last_t", now_t - 0.014)))
    app.handtrack_last_t, app.handtrack_pkt_count = now_t, app.handtrack_pkt_count + 1
    if now_t - app.handtrack_rate_timer >= 1.0:
        app.handtrack_rate_hz = app.handtrack_pkt_count / (now_t - app.handtrack_rate_timer)
        app.handtrack_rate_timer, app.handtrack_pkt_count = now_t, 0

    if cmd_t >= 1e7:
        age_ms = max(0.0, (now_t - cmd_t) * 1000.0)
        app.handtrack_e2e_ms = round(age_ms + 16.0, 1)
        if age_ms > 150.0:
            app.gizmo_active_l = app.gizmo_active_r = False
            return
    else: app.handtrack_e2e_ms = 18.0

    if "head" in cmd:
        hd = cmd["head"]
        app.last_head_pos = np.array(hd.get("pos", [0.0, 1.5, 0.0]), dtype=np.float64)
        app.last_head_yaw = float(hd.get("yaw", 0.0))
        _apply_head_tracking(app, hd, now_t)

    axis_map, rot_axis_map = get_effective_axis_map(app), get_effective_rot_axis_map(app)
    res_l = _process_single_hand(app, "left", cmd.get("left"), now_t, dt, axis_map, rot_axis_map)
    res_r = _process_single_hand(app, "right", cmd.get("right"), now_t, dt, axis_map, rot_axis_map)

    if res_l.get("is_open") and res_r.get("is_open") and res_l.get("vel", 1.0) < 0.08 and res_r.get("vel", 1.0) < 0.08:
        app.handtrack_recenter_hold += dt
        if app.handtrack_recenter_hold >= 1.0:
            app.handtrack_anchor_w_l = app.handtrack_anchor_w_r = None
            app.cal_head_pos = getattr(app, "last_head_pos", np.array([0.0, 1.5, 0.0])).copy()
            app.cal_head_yaw, app.handtrack_recenter_hold = float(getattr(app, "last_head_yaw", 0.0)), 0.0
            app._log_xr_event("Both palms hold 1s -> RECENTERED")
    else: app.handtrack_recenter_hold = 0.0

    if res_l.get("estop_candidate", False) and res_r.get("estop_candidate", False) and not app.state_estop:
        app.state_estop, app.status_warning = True, "E-STOP: Both hands raised above head"
    app.xr_handtrack_status = {"active": True, "mode": app.hand_follow_mode, "rate_hz": round(float(app.handtrack_rate_hz), 1), "limit": app.limit_indicator, "e2e_ms": app.handtrack_e2e_ms, "left": res_l, "right": res_r}
