"""
teleop/app/cmd_system.py
Handles core system commands including E-stop, hold toggle, and arm jogging.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

from teleop.app.sync_manager import engage_arms


def handle_system_cmd(app, name: str, cmd: dict) -> bool:
    """Processes system lifecycle commands. Returns True if command was handled."""
    if name == "estop":
        app.state_estop = True
        print("[ALERT] E-STOP ACTIVE")
        return True
    elif name == "clear_estop":
        app.state_estop = False
        print("[INFO] E-STOP CLEARED")
        return True
    elif name == "clear_fault":
        confirmed = bool(cmd.get("confirmed", False))
        if hasattr(app, "robot_safety") and app.robot_safety is not None:
            q_act, _, meta = app.robot_safety.read_state()
            age = meta.get("age_s", 999.0) if isinstance(meta, dict) else 0.0
            if age > 0.200 or q_act is None:
                app.status_warning = "Clear Fault refused: robot telemetry is stale (>200ms) or missing."
                print(f"[FAULT] Clear refused: feedback age={age*1000:.1f}ms > 200ms")
                return False
            if app.robot_safety.clear_fault(confirm=confirmed):
                app.status_warning = ""
                if hasattr(app, "mirror_limiter"): app.mirror_limiter.reset(q_act)
                app.last_arm_cmd_left = q_act[:7].copy()
                app.last_arm_cmd_right = q_act[7:].copy()
                if hasattr(app, "env"):
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
                app.arm_engaged_l = False
                app.arm_engaged_r = False
                print("[INFO] FAULT CLEARED: Plan dropped, resynced to measured state, transitioned to READY.")
            else:
                app.status_warning = "Clear fault requires explicit confirmation."
        else:
            app.status_warning = ""
        return True
    elif name == "engage_arm":
        app.auto_engage_suppressed = False
        return engage_arms(app, cmd.get("arm", "both"))
    elif name == "disengage_arm":
        arm = cmd.get("arm", "both")
        app.auto_engage_suppressed = True
        if arm in ("left", "both"): app.arm_engaged_l = False
        if arm in ("right", "both"): app.arm_engaged_r = False
        if not app.arm_engaged_l and not app.arm_engaged_r and app.sync_state == "ENGAGED":
            app.sync_state = "READY"
        print(f"[SYNC] Arms disengaged: L={app.arm_engaged_l}, R={app.arm_engaged_r} (State: {app.sync_state})")
        return True
    elif name == "hold_toggle":
        app.manual_held = not app.manual_held
        if app.manual_held:
            from teleop.app.joint_preset import stop_auto_motion
            stop_auto_motion(app)
        app._log_xr_event("Hold ON: arms frozen" if app.manual_held else "Hold OFF")
        return True
    elif name in ("neck", "set_neck"):
        pitch_val = cmd.get("pitch_deg") if "pitch_deg" in cmd else cmd.get("pitch", getattr(app, "neck_pitch_deg", 5.0))
        yaw_val = cmd.get("yaw_deg") if "yaw_deg" in cmd else cmd.get("yaw", getattr(app, "neck_yaw_deg", 0.0))
        pitch_deg = float(np.clip(float(pitch_val), -23.0, 23.0))
        yaw_deg = float(np.clip(float(yaw_val), -45.0, 45.0))
        app.neck_pitch_deg = pitch_deg
        app.neck_yaw_deg = yaw_deg
        app.target_neck = np.array([np.radians(yaw_deg), np.radians(pitch_deg)], dtype=np.float64)
        if hasattr(app, "env") and hasattr(app.env, "set_head_qpos"):
            app.env.set_head_qpos(app.target_neck[0], app.target_neck[1])
        print(f"[NECK] Posture target: pitch={pitch_deg:.1f} deg, yaw={yaw_deg:.1f} deg")
        if hasattr(app, "server") and hasattr(app.server, "broadcast_event"):
            app.server.broadcast_event({
                "type": "neck_state",
                "pitch_deg": pitch_deg,
                "yaw_deg": yaw_deg,
            })
        return True
    elif name in ("rec_start", "rec_stop", "rec_toggle", "rec_discard", "set_rec_task", "set_discard_reason"):
        from teleop.app import episode_recorder as er
        if name == "set_rec_task":
            if not app.rec_active: app.rec_task = er.safe_name(cmd.get("task"))
        elif name == "set_discard_reason": er.set_discard_reason(app, cmd.get("reason"))
        elif name == "rec_discard":
            if cmd.get("reason"): er.set_discard_reason(app, cmd.get("reason"))
            er.discard_last(app, note=cmd.get("note"))
        elif name == "rec_stop" or (name == "rec_toggle" and app.rec_active): er.stop_episode(app)
        else: er.start_episode(app, cmd.get("task"), cmd.get("instruction"))
        return True
    elif name == "set_xr_scale":
        # Controller -> robot motion scale from the panel (1.0 = 1:1). Re-anchor any held clutch so the target
        # does not jump when the ratio changes mid-drag.
        try: val = float(cmd.get("scale", 1.0))
        except (TypeError, ValueError): return True
        if not np.isfinite(val): return True
        app.xr_scale = max(0.1, min(2.0, val))
        for s in ("l", "r"):
            if getattr(app, f"xr_clutch_state_{s}", "LOCKED") != "LOCKED":
                setattr(app, f"xr_clutch_state_{s}", "LOCKED")
        app._log_xr_event(f"Controller->robot scale set to {app.xr_scale:.2f}")
        return True
    elif name == "set_input_mode":
        # Server stores launch-configured input mode as single source of truth; never auto-switches
        app._log_xr_event(f"Ignored client mode request (server strict mode: {getattr(app, 'input_mode', 'controllers')})")
        return True
    elif name == "calibrate_hand":
        if hasattr(app, "handtrack_retargeter_l") and hasattr(app, "handtrack_retargeter_r"):
            cal_data = cmd.get("calibration", {})
            if "left" in cal_data:
                app.handtrack_retargeter_l.cal = cal_data["left"].copy()
            if "right" in cal_data:
                app.handtrack_retargeter_r.cal = cal_data["right"].copy()
        app._log_xr_event("Hand calibration updated")
        return True
    elif name in ("recenter_hand", "recenter"):
        app.handtrack_anchor_w_l, app.handtrack_anchor_w_r = None, None
        app.cal_head_pos = getattr(app, "last_head_pos", np.array([0.0, 1.5, 0.0])).copy()
        app.cal_head_yaw = float(getattr(app, "last_head_yaw", 0.0))
        app._log_xr_event("Hand tracking recentered (head & anchors refreshed)")
        return True
    elif name == "clear_target_trace":
        if hasattr(app, "target_tracer"):
            app.target_tracer.clear()
        return True
    elif name in ("calibrate_pose", "calibrate_hand_pose"):
        app.cal_head_pos = getattr(app, "last_head_pos", np.array([0.0, 1.5, 0.0])).copy()
        app.cal_head_yaw = float(getattr(app, "last_head_yaw", 0.0))
        app.handtrack_rcal_l = getattr(app, "robot_home_rot_l", getattr(app, "cur_target_rot_l", np.eye(3))).copy()
        app.handtrack_rcal_r = getattr(app, "robot_home_rot_r", getattr(app, "cur_target_rot_r", np.eye(3))).copy()
        if hasattr(app, "save_pose_calibration"):
            app.save_pose_calibration()
        app._log_xr_event("Hand pose calibrated (forward/palms down)")
        return True
    elif name == "set_hand_follow_mode":
        mode = str(cmd.get("mode", "absolute")).lower()
        if mode == "always": mode = "relative"
        app.hand_follow_mode = mode
        app._log_xr_event(f"Hand follow mode set to {app.hand_follow_mode}")
        return True
    elif name == "set_hand_filter_config":
        if "min_cutoff" in cmd: app.hand_min_cutoff = float(cmd["min_cutoff"])
        if "beta" in cmd: app.hand_beta = float(cmd["beta"])
        if "lead_ms" in cmd: app.hand_lead_ms = float(cmd["lead_ms"])
        if hasattr(app, "handtrack_filter_pos_l"):
            app.handtrack_filter_pos_l.min_cutoff = app.handtrack_filter_pos_r.min_cutoff = getattr(app, "hand_min_cutoff", 1.5)
            app.handtrack_filter_pos_l.beta = app.handtrack_filter_pos_r.beta = getattr(app, "hand_beta", 0.03)
        return True

    if name in ("arm_jog", "jog_l", "jog_r"):
        if app.state_estop:
            app.status_warning = "Command ignored: E-STOP is active."
            return True
        if getattr(app, "status_warning", "") == "TARGET_FROZEN":
            app.status_warning = ""

        arm = cmd.get("arm", "left" if name == "jog_l" else "right")
        arm_c = "l" if arm == "left" else "r"
        axis_val = cmd.get("axis", 0)
        if isinstance(axis_val, str):
            axis_val = axis_val.lower()
            if axis_val in ("x", "0"):
                axis = 0
            elif axis_val in ("y", "1"):
                axis = 1
            elif axis_val in ("z", "2"):
                axis = 2
            elif axis_val in ("roll", "pitch", "yaw"):
                r_axis = 0 if axis_val == "roll" else (1 if axis_val == "pitch" else 2)
                direction = float(cmd.get("dir", 1.0))
                step_deg = float(cmd.get("step_deg", 5.0))
                rot_vec = np.zeros(3)
                rot_vec[r_axis] = np.radians(step_deg) * direction
                dR = R.from_rotvec(rot_vec).as_matrix()
                cur_rot = getattr(app, f"cur_target_rot_{arm_c}")
                new_rot = dR @ cur_rot
                setattr(app, f"cur_target_rot_{arm_c}", new_rot)
                setattr(app, f"soft_ori_{arm_c}", False)   # an explicit rotation command: hold it exactly
                app._solve_arm_ik(arm)
                from teleop.app.ik_dispatcher import NUDGE_GLIDE, ik_glide_start
                if NUDGE_GLIDE: ik_glide_start(app, arm_c)      # the rotation follower turns the goal gradually
                if hasattr(app, "server") and hasattr(app.server, "broadcast_event"):
                    cur_p = getattr(app, f"cur_target_pos_{arm_c}")
                    app.server.broadcast_event({
                        "type": "gizmo_set", "arm": arm, "pos": cur_p.tolist(),
                        "rot": R.from_matrix(new_rot).as_quat().tolist(), "source": "nudge"
                    })
                return True
            else:
                axis = 0
        else:
            axis = int(axis_val)

        direction = float(cmd.get("dir", 1.0))
        if "step_cm" in cmd:
            step = float(cmd.get("step_cm", 2.0)) / 100.0 * direction
        else:
            step = float(cmd.get("step", 0.02)) * (direction if "dir" in cmd else 1.0)

        base_p = np.array(cmd["cur_pos"], dtype=np.float64) if "cur_pos" in cmd else getattr(app, f"cur_target_pos_{arm_c}").copy()
        new_t = base_p.copy()
        new_t[axis] += step
        new_t[0] = np.clip(new_t[0], -0.2, 0.9)
        new_t[1] = np.clip(new_t[1], -0.9, 0.9)
        new_t[2] = np.clip(new_t[2], 0.5, 1.8)

        setattr(app, f"cur_target_pos_{arm_c}", new_t)
        follower = getattr(app, f"gizmo_follower_{arm_c}", None)
        from teleop.app.ik_dispatcher import NUDGE_GLIDE, ik_glide_start
        # A step used to be applied in one tick (follower reset onto the new target, one IK solve): the joint target
        # jumped 0.1-0.17 rad per 2-3 cm step, and held / repeated steps (one every ~120 ms) made the arm speed up and
        # slow down at every step. Now the Cartesian goal glides to the new target through the same critically
        # damped follower as a mouse drag and IK is stepped every control tick until the posture is still.
        if follower and not NUDGE_GLIDE: follower.reset(new_t)
        setattr(app, f"gizmo_active_{arm_c}", False)
        setattr(app, f"soft_ori_{arm_c}", True)        # position-only step: wrist stays straight, hand follows the forearm
        if hasattr(app, "target_tracer"): app.target_tracer.record_target_source(arm, "mouse panel step")
        app._solve_arm_ik(arm)
        if NUDGE_GLIDE: ik_glide_start(app, arm_c)

        if hasattr(app, "server") and hasattr(app.server, "broadcast_event"):
            cur_rot = getattr(app, f"cur_target_rot_{arm_c}")
            app.server.broadcast_event({
                "type": "gizmo_set", "arm": arm, "pos": new_t.tolist(),
                "rot": R.from_matrix(cur_rot).as_quat().tolist(), "source": "nudge"
            })
        return True

    return False

