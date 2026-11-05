"""
teleop/app/telemetry_builder.py
Constructs comprehensive 20 Hz JSON status telemetry packets for the 3D Web Panel and Quest.
"""

import time
import numpy as np


def build_status_packet(app) -> dict:
    """Assembles robot simulation telemetry, joint positions, IK errors, and Quest controller stats."""
    ee = app.env.get_ee_poses()
    now_t = time.time()
    elapsed_rate = now_t - app.cmd_rate_timer
    if elapsed_rate >= 1.0:
        app.cmd_rate_hz = app.cmd_count / elapsed_rate
        app.cmd_count = 0
        app.cmd_rate_timer = now_t

    elapsed_xr = now_t - app.xr_rate_timer
    if elapsed_xr >= 1.0:
        app.xr_rate_hz = app.xr_pkt_count / elapsed_xr
        app.xr_pkt_count = 0
        app.xr_rate_timer = now_t

    closure_l = app.hand_closure_l if not getattr(app, "direct_hand_override_l", False) else float(np.mean(app.last_hand_cmd_left[2:6])) / 1.21
    closure_r = app.hand_closure_r if not getattr(app, "direct_hand_override_r", False) else float(np.mean(app.last_hand_cmd_right[2:6])) / 1.21

    comfort_targets = np.array([0.95, 0.32, 1.18, 1.22, 1.22, 1.22], dtype=np.float64)
    safe_l = getattr(app, "current_safe_hand_l", np.zeros(6))
    safe_r = getattr(app, "current_safe_hand_r", np.zeros(6))
    finger_pcts_l = [round(float(x), 1) for x in np.clip(safe_l[1:6] / comfort_targets[1:6], 0.0, 1.0) * 100.0]
    finger_pcts_r = [round(float(x), 1) for x in np.clip(safe_r[1:6] / comfort_targets[1:6], 0.0, 1.0) * 100.0]

    # Hand diagnostics ring buffer logging (20 Hz)
    if hasattr(app, "hand_log_ring"):
        reason_l = "none"
        if getattr(app, "hand_collision_blocked", False) and hasattr(app, "collision_checker"):
            _, _, p_l = app.collision_checker.check_hand_collision("left", safe_l, margin=0.001)
            if p_l and len(p_l) > 2 and p_l[2]:
                reason_l = str(p_l[2])
        reason_r = "none"
        if getattr(app, "hand_collision_blocked", False) and hasattr(app, "collision_checker"):
            _, _, p_r = app.collision_checker.check_hand_collision("right", safe_r, margin=0.001)
            if p_r and len(p_r) > 2 and p_r[2]:
                reason_r = str(p_r[2])

        actual_l = app.env.get_left_hand_qpos()
        actual_r = app.env.get_right_hand_qpos()
        f_names = ["thumb", "index", "middle", "ring", "pinky"]
        f_indices = [1, 2, 3, 4, 5]
        f_dict_l = {
            fn: {
                "target": round(float(safe_l[fi]), 4),
                "actual": round(float(actual_l[fi]), 4),
                "clamped": bool(reason_l != "none"),
                "reason": reason_l if reason_l != "none" else "none"
            }
            for fn, fi in zip(f_names, f_indices)
        }
        f_dict_r = {
            fn: {
                "target": round(float(safe_r[fi]), 4),
                "actual": round(float(actual_r[fi]), 4),
                "clamped": bool(reason_r != "none"),
                "reason": reason_r if reason_r != "none" else "none"
            }
            for fn, fi in zip(f_names, f_indices)
        }
        app.hand_log_ring.append({
            "timestamp": float(now_t),
            "left": {
                "grasp_type": str(app.grasp_type_l),
                "stick_y_raw": float(getattr(app, "xr_stick_y_l", 0.0)),
                "close_cmd": float(np.clip(app.hand_closure_l, 0.0, 1.0)),
                "fingers": f_dict_l
            },
            "right": {
                "grasp_type": str(app.grasp_type_r),
                "stick_y_raw": float(getattr(app, "xr_stick_y_r", 0.0)),
                "close_cmd": float(np.clip(app.hand_closure_r, 0.0, 1.0)),
                "fingers": f_dict_r
            }
        })

    xr_active = False
    if app.tracker is not None and getattr(app.tracker, "tv_wrapper", None) is not None:
        tvuer = getattr(app.tracker.tv_wrapper, "tvuer", None)
        if tvuer is not None and hasattr(tvuer, "motion_data_ready_shared"):
            with tvuer.motion_data_ready_shared.get_lock():
                xr_active = bool(tvuer.motion_data_ready_shared.value)

    err_l = float(np.linalg.norm(np.array(ee["left"]["pos"]) - app.cur_target_pos_l)) * 1000.0
    err_r = float(np.linalg.norm(np.array(ee["right"]["pos"]) - app.cur_target_pos_r)) * 1000.0

    return {
        "sim_running": True,
        "nan_detected": bool(app.nan_detected),
        "viewer_enabled": bool(app.render_viewer),
        "viewer_running": bool(app.env.viewer is not None and getattr(app.env.viewer, "is_running", lambda: False)()),
        "xr_active": xr_active or app.xr_input_active,
        "input_source": "QUEST VR" if app.xr_input_active else "MOUSE",
        "xr_controllers_active": bool(app.xr_input_active),
        "xr_rate_hz": float(app.xr_rate_hz),
        "xr_grip_l": bool(app.xr_clutch_state_l != "LOCKED"),
        "xr_grip_r": bool(app.xr_clutch_state_r != "LOCKED"),
        "ee_l": [float(x) for x in ee["left"]["pos"]],
        "ee_r": [float(x) for x in ee["right"]["pos"]],
        "rot_l": [[float(c) for c in row] for row in ee["left"]["rot"]],
        "rot_r": [[float(c) for c in row] for row in ee["right"]["rot"]],
        "tgt_l": [float(x) for x in app.cur_target_pos_l],
        "tgt_r": [float(x) for x in app.cur_target_pos_r],
        "tgt_rot_l": [[float(c) for c in row] for row in app.cur_target_rot_l],
        "tgt_rot_r": [[float(c) for c in row] for row in app.cur_target_rot_r],
        "ik_err_l": err_l,
        "ik_err_r": err_r,
        "hand_closure_l": float(np.clip(app.hand_closure_l, 0.0, 1.0)),
        "hand_closure_r": float(np.clip(app.hand_closure_r, 0.0, 1.0)),
        "hand_fingers_l": finger_pcts_l,
        "hand_fingers_r": finger_pcts_r,
        "held": bool(app.manual_held),
        "estop": bool(app.state_estop),
        "collision_blocked": bool(app.collision_blocked),
        "hand_collision_blocked": bool(app.hand_collision_blocked),
        "min_distance_mm": float(app.min_collision_distance_m * 1000.0),
        "min_hand_distance_mm": float(app.min_hand_distance_m * 1000.0),
        "homing": bool(app.homing_active),
        "qpos_l": [float(x) for x in app.last_arm_cmd_left],
        "qpos_r": [float(x) for x in app.last_arm_cmd_right],
        "walking": bool(app.walking_demo_active),
        "loop_hz": float(getattr(app, "loop_hz", 200.0)),
        "cmd_rate_hz": float(app.cmd_rate_hz),
        "last_cmd_name": str(app.last_cmd_name),
        "last_cmd_time": float(app.last_cmd_time),
        "fault_latched": bool(getattr(getattr(app, "robot_safety", None), "fault_latched", False)),
        "fault_reason": str(getattr(getattr(app, "robot_safety", None), "fault_reason", "")),
        "warning": str(app.status_warning),
        "xr_status": {
            "pkt_count": int(app.xr_total_pkts),
            "rate_hz": float(app.xr_rate_hz),
            "age_ms": float(max(0.0, (now_t - app.xr_last_packet_time) * 1000.0)) if app.xr_last_packet_time > 0 else 999.0,
            "connected": bool(app.xr_input_active),
            "left": {
                "connected": bool(app.xr_left_connected),
                "clutch_state": app.xr_clutch_state_l,
                "grip_val": float(app.xr_grip_val_l),
                "trigger_val": float(app.xr_trig_val_l),
                "thumbstick": [float(app.xr_stick_x_l), float(app.xr_stick_y_l)],
                "thumb_click": bool(app.xr_thumb_click_l),
                "btn_primary": bool(app.xr_btn_x),
                "btn_secondary": bool(app.xr_btn_y),
                "btn_menu": bool(app.xr_btn_menu_l),
                "precision": bool(app.precision_l),
                "grasp_type": app.grasp_type_l,
                "closure": float(app.hand_closure_l),
                "finger_pcts": finger_pcts_l,
                "home_progress": float(app.xr_home_progress_l)
            },
            "right": {
                "connected": bool(app.xr_right_connected),
                "clutch_state": app.xr_clutch_state_r,
                "grip_val": float(app.xr_grip_val_r),
                "trigger_val": float(app.xr_trig_val_r),
                "thumbstick": [float(app.xr_stick_x_r), float(app.xr_stick_y_r)],
                "thumb_click": bool(app.xr_thumb_click_r),
                "btn_primary": bool(app.xr_btn_a),
                "btn_secondary": bool(app.xr_btn_b),
                "btn_menu": bool(app.xr_btn_menu_r),
                "precision": bool(app.precision_r),
                "grasp_type": app.grasp_type_r,
                "closure": float(app.hand_closure_r),
                "finger_pcts": finger_pcts_r,
                "home_progress": float(app.xr_home_progress_r)
            },
            "events": app.xr_event_log[-8:]
        },
        "hand_status": {
            "left": {
                "active_joints": {
                    jname: {
                        "val": float(app.env.data.qpos[app.env.qpos_idx_map[jname]]),
                        "cmd": float(app.last_hand_cmd_left[i]),
                        "min": float(app.env.joint_limits[jname][0]),
                        "max": float(app.env.joint_limits[jname][1])
                    }
                    for i, jname in enumerate(app.env.left_hand_active_joints)
                },
                "mimic_joints": {
                    jname: float(app.env.data.qpos[app.env.qpos_idx_map[jname]])
                    for jname in app.env.left_hand_mimic_joints
                },
                "curls": [float(c) for c in app.target_curls_l],
                "closure": float(app.hand_closure_l),
                "grasp_type": app.grasp_type_l
            },
            "right": {
                "active_joints": {
                    jname: {
                        "val": float(app.env.data.qpos[app.env.qpos_idx_map[jname]]),
                        "cmd": float(app.last_hand_cmd_right[i]),
                        "min": float(app.env.joint_limits[jname][0]),
                        "max": float(app.env.joint_limits[jname][1])
                    }
                    for i, jname in enumerate(app.env.right_hand_active_joints)
                },
                "mimic_joints": {
                    jname: float(app.env.data.qpos[app.env.qpos_idx_map[jname]])
                    for jname in app.env.right_hand_mimic_joints
                },
                "curls": [float(c) for c in app.target_curls_r],
                "closure": float(app.hand_closure_r),
                "grasp_type": app.grasp_type_r
            },
            "sweep_active": bool(app.hand_sweep_active),
            "sweep_info": str(app.hand_sweep_info)
        }
    }

