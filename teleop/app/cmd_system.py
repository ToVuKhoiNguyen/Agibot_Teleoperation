"""
teleop/app/cmd_system.py
Handles core system commands including E-stop, hold toggle, walk demo, and arm jogging.
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R


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
        arm = cmd.get("arm", "both")
        if getattr(app, "sync_state", "READY") in ("READY", "ENGAGED"):
            if arm in ("left", "both"): app.arm_engaged_l = True
            if arm in ("right", "both"): app.arm_engaged_r = True
            app.sync_state = "ENGAGED"
            print(f"[SYNC] Arms engaged: L={app.arm_engaged_l}, R={app.arm_engaged_r} (State: ENGAGED)")
            return True
        else:
            print(f"[SYNC] Cannot engage in state {app.sync_state}")
            return False
    elif name == "disengage_arm":
        arm = cmd.get("arm", "both")
        if arm in ("left", "both"): app.arm_engaged_l = False
        if arm in ("right", "both"): app.arm_engaged_r = False
        if not app.arm_engaged_l and not app.arm_engaged_r and app.sync_state == "ENGAGED":
            app.sync_state = "READY"
        print(f"[SYNC] Arms disengaged: L={app.arm_engaged_l}, R={app.arm_engaged_r} (State: {app.sync_state})")
        return True
    elif name == "hold_toggle":
        app.manual_held = not app.manual_held
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
    elif name in ("walk_toggle", "walk_demo"):
        app.walking_demo_active = not app.walking_demo_active
        if not app.walking_demo_active:
            if hasattr(app.env, "non_manip_qpos_indices"):
                app.env.data.qpos[app.env.non_manip_qpos_indices] = 0.0
        else:
            app.walk_start_time = time.time()
        return True

    if name in ("arm_jog", "jog_l", "jog_r"):
        if app.state_estop:
            app.status_warning = "Command ignored: E-STOP is active."
            return True

        arm = cmd.get("arm", "left" if name == "jog_l" else "right")
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
                if arm == "left":
                    app.cur_target_rot_l = dR @ app.cur_target_rot_l
                    app._solve_arm_ik("left")
                else:
                    app.cur_target_rot_r = dR @ app.cur_target_rot_r
                    app._solve_arm_ik("right")
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

        if arm == "left":
            new_t = app.cur_target_pos_l.copy()
            new_t[axis] += step
            new_t[0] = np.clip(new_t[0], -0.2, 0.9)
            new_t[1] = np.clip(new_t[1], -0.9, 0.9)
            new_t[2] = np.clip(new_t[2], 0.5, 1.8)
            app.cur_target_pos_l = new_t
            app.gizmo_active_l = True
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("left", "mouse panel step")
            app._solve_arm_ik("left")
        else:
            new_t = app.cur_target_pos_r.copy()
            new_t[axis] += step
            new_t[0] = np.clip(new_t[0], -0.2, 0.9)
            new_t[1] = np.clip(new_t[1], -0.9, 0.9)
            new_t[2] = np.clip(new_t[2], 0.5, 1.8)
            app.cur_target_pos_r = new_t
            app.gizmo_active_r = True
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("right", "mouse panel step")
            app._solve_arm_ik("right")
        return True

    return False

