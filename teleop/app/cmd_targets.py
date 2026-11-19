"""
teleop/app/cmd_targets.py
Command dispatchers for 3D wrist targets, grasping sliders, keyframe presets, and motion sequences.
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting


def handle_target_cmd(app, name: str, cmd: dict) -> bool:
    """Processes target pose and sequence commands. Returns True if handled."""
    if name in ("set_wrist_target", "wrist_target"):
        if getattr(app, "manual_held", False):
            app.status_warning = "Hold is ON: release Toggle Hold to move the arms"
            return True
        if app.homing_active:
            app.homing_active = False
        app.preset_track_until = 0.0
        arm = cmd.get("arm", cmd.get("side", "left"))
        pos = cmd.get("pos")
        rot_quat = cmd.get("rot_quat")
        now_t = time.time()
        was_idle = not getattr(app, "gizmo_active_l", False) and not getattr(app, "gizmo_active_r", False)
        app.last_gizmo_cmd_time = now_t
        if was_idle:
            if hasattr(app, "robot_backend") and hasattr(app.robot_backend, "_trace_logger"):
                tl = app.robot_backend._trace_logger
                if tl:
                    seq = getattr(app.robot_backend, "_seq", None)
                    tl.log_event("GIZMO_START", seq=seq, arm=arm)
        if getattr(app, "status_warning", "") == "TARGET_FROZEN":
            app.status_warning = ""
        if pos is not None and len(pos) == 3:
            p = np.array(pos, dtype=np.float64)
            p_orig = p.copy()
            p[0] = np.clip(p[0], -0.2, 0.9)
            p[1] = np.clip(p[1], -0.9, 0.9)
            p[2] = np.clip(p[2], 0.5, 1.8)
            if arm == "left":
                if np.linalg.norm(p - p_orig) > 1e-4:
                    app.status_warning = "Left wrist target clamped to workspace"
                app.cur_target_pos_l = p
                app.gizmo_active_l = True
                if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("left", "gizmo")
                if hasattr(app, "input_filter_left"):
                    app.input_filter_left.reset()
            else:
                if np.linalg.norm(p - p_orig) > 1e-4:
                    app.status_warning = "Right wrist target clamped to workspace"
                app.cur_target_pos_r = p
                app.gizmo_active_r = True
                if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("right", "gizmo")
                if hasattr(app, "input_filter_right"):
                    app.input_filter_right.reset()

        c_arm = "l" if arm == "left" else "r"
        if pos is not None and len(pos) == 3:
            setattr(app, f"soft_ori_{c_arm}", True)    # translating; cleared just below if this message also rotates
        if rot_quat is not None and len(rot_quat) == 4:
            scipy_quat = [rot_quat[1], rot_quat[2], rot_quat[3], rot_quat[0]]
            rot_mat = R.from_quat(scipy_quat).as_matrix()
            prev_rot = getattr(app, f"cur_target_rot_{c_arm}")
            cos_d = float(np.clip((np.trace(prev_rot.T @ rot_mat) - 1.0) * 0.5, -1.0, 1.0))
            if np.arccos(cos_d) > np.radians(0.5):
                setattr(app, f"soft_ori_{c_arm}", False)   # the gizmo was rotated: hold that orientation exactly
            if arm == "left":
                app.cur_target_rot_l = rot_mat
                app.gizmo_active_l = True
                if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("left", "gizmo")
            else:
                app.cur_target_rot_r = rot_mat
                app.gizmo_active_r = True
                if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("right", "gizmo")

        app._solve_arm_ik(arm)
        return True

    elif name in ("gizmo_end", "gizmo_release", "stop_drag"):
        arm = cmd.get("arm", "both")
        if arm not in ("left", "right", "both", "all"):
            return True  # malformed release (e.g. a numeric arm): ignore instead of freezing the wrong arm
        if hasattr(app, "robot_backend") and hasattr(app.robot_backend, "_trace_logger"):
            tl = app.robot_backend._trace_logger
            if tl:
                seq = getattr(app.robot_backend, "_seq", None)
                tl.log_event("GIZMO_END", seq=seq, arm=arm)

        from teleop.kinematics.reach_projection import project_to_reachable_workspace
        arms_to_check = ["left", "right"] if arm in ("both", "all") else [arm]
        for a in arms_to_check:
            cur_p = getattr(app, f"cur_target_pos_{'l' if a == 'left' else 'r'}")
            clamped_p, is_c, d_beyond = project_to_reachable_workspace(cur_p, side=a)
            if is_c and abs(d_beyond) > 0.01:
                setattr(app, f"cur_target_pos_{'l' if a == 'left' else 'r'}", clamped_p)
                if hasattr(app, "server") and hasattr(app.server, "broadcast_event"):
                    app.server.broadcast_event({
                        "type": "gizmo_recall",
                        "arm": a,
                        "pos": clamped_p.tolist(),
                        "duration_ms": 250
                    })

        from teleop.app.ik_dispatcher import freeze_arm_target
        freeze_arm_target(app, arm)
        return True

    elif name in ("hand", "hand_l", "hand_r"):
        arm = cmd.get("arm", "left" if name == "hand_l" else "right")
        action = cmd.get("action", "")
        if action == "open":
            curls, pinch = AgibotHandRetargeting.get_grasp_curls("open", 0.0)
            if arm == "left":
                app.hand_closure_l = 0.0
                app.grasp_type_l = "cylinder"
                app.direct_hand_override_l = False
            else:
                app.hand_closure_r = 0.0
                app.grasp_type_r = "cylinder"
                app.direct_hand_override_r = False
        elif action == "close":
            curls, pinch = AgibotHandRetargeting.get_grasp_curls("cylinder", 1.0)
            if arm == "left":
                app.hand_closure_l = 1.0
                app.grasp_type_l = "cylinder"
                app.direct_hand_override_l = False
            else:
                app.hand_closure_r = 1.0
                app.grasp_type_r = "cylinder"
                app.direct_hand_override_r = False
        elif action == "pinch":
            curls, pinch = AgibotHandRetargeting.get_grasp_curls("pinch", 1.0)
            if arm == "left":
                app.hand_closure_l = 1.0
                app.grasp_type_l = "pinch"
                app.direct_hand_override_l = False
            else:
                app.hand_closure_r = 1.0
                app.grasp_type_r = "pinch"
                app.direct_hand_override_r = False
        else:
            val = float(cmd.get("value", cmd.get("closure", 0.0)))
            gt = app.grasp_type_l if arm == "left" else app.grasp_type_r
            if str(gt).lower() == "open":
                gt = "cylinder"  # homing leaves grasp "open" (all-zero curls): the slider would do nothing
            curls, pinch = AgibotHandRetargeting.get_grasp_curls(gt, val)
            if arm == "left":
                app.grasp_type_l = gt
                app.hand_closure_l = val
                app.direct_hand_override_l = False
            else:
                app.grasp_type_r = gt
                app.hand_closure_r = val
                app.direct_hand_override_r = False

        if arm == "left":
            app.target_curls_l, app.target_pinch_l = curls, pinch
        else:
            app.target_curls_r, app.target_pinch_r = curls, pinch
        return True

    elif name in ("preset", "reset", "go_home", "home"):
        pose = cmd.get("pose", "home") if name == "preset" else ("reset" if name == "reset" else "home")
        app.preset_track_until = 0.0
        from teleop.app.joint_preset import stop_auto_motion, start_preset
        stop_auto_motion(app)
        app.manual_held = False
        is_real = str(getattr(app, "backend", "sim")).lower() in ("real", "ros2", "real-udp", "udp")
        if (name == "reset" or pose == "reset") and is_real:
            # A real arm cannot teleport: an instant reset would jump the limiter plan to HOME.
            app._start_smooth_homing("both")
        elif name == "reset" or pose == "reset":
            app.active_sequence = None
            app.homing_active = False
            app.direct_hand_override_l = False
            app.direct_hand_override_r = False
            app.hand_closure_l = 0.0
            app.hand_closure_r = 0.0
            app.target_curls_l = np.zeros(5, dtype=np.float64)
            app.target_curls_r = np.zeros(5, dtype=np.float64)
            app.target_pinch_l = 0.0
            app.target_pinch_r = 0.0
            app.env.reset()
            app.last_arm_cmd_left = app.env.get_left_arm_qpos()
            app.last_arm_cmd_right = app.env.get_right_arm_qpos()
            if hasattr(app, "robot_safety") and app.robot_safety is not None:
                q_home = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
                app.robot_safety._last_cmd_q14 = q_home.copy()
                app.robot_safety._last_cmd_vel = np.zeros(14)
                app.robot_safety._deviation_start_t = None
            if hasattr(app, "mirror_limiter") and app.mirror_limiter is not None:
                app.mirror_limiter.reset(np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right]))
            if hasattr(app, "sim_plant") and app.sim_plant is not None:
                q_reset = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
                app.sim_plant.reset(q_reset)
                app.sim_plant_q = q_reset.copy()
            app.last_hand_cmd_left = np.zeros(6, dtype=np.float64)
            app.last_hand_cmd_right = np.zeros(6, dtype=np.float64)
            app.current_safe_hand_l = np.zeros(6, dtype=np.float64)
            app.current_safe_hand_r = np.zeros(6, dtype=np.float64)
            app.hand_collision_blocked = False
            home_p = app.env.get_ee_poses()
            app.cur_target_pos_l = home_p["left"]["pos"].copy()
            app.cur_target_pos_r = home_p["right"]["pos"].copy()
            app.cur_target_rot_l = home_p["left"]["rot"].copy()
            app.cur_target_rot_r = home_p["right"]["rot"].copy()
            if hasattr(app, "input_filter_left"): app.input_filter_left.reset()
            if hasattr(app, "input_filter_right"): app.input_filter_right.reset()
            if hasattr(app, "output_filter_left"): app.output_filter_left.reset(app.last_arm_cmd_left)
            if hasattr(app, "output_filter_right"): app.output_filter_right.reset(app.last_arm_cmd_right)
            if hasattr(app, "gizmo_follower_l"): app.gizmo_follower_l.reset(app.cur_target_pos_l)
            if hasattr(app, "gizmo_follower_r"): app.gizmo_follower_r.reset(app.cur_target_pos_r)
            if hasattr(app, "gizmo_rot_follower_l"): app.gizmo_rot_follower_l.reset(app.cur_target_rot_l)
            if hasattr(app, "gizmo_rot_follower_r"): app.gizmo_rot_follower_r.reset(app.cur_target_rot_r)
            if hasattr(app, "output_filter_left_gizmo"): app.output_filter_left_gizmo.reset(app.last_arm_cmd_left)
            if hasattr(app, "output_filter_right_gizmo"): app.output_filter_right_gizmo.reset(app.last_arm_cmd_right)
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("both", "home/smooth-home")
            app.last_ik_sol_left = app.last_arm_cmd_left.copy()
            app.last_ik_sol_right = app.last_arm_cmd_right.copy()
            app.gizmo_active_l = app.gizmo_active_r = False
            app.xr_grip_active_l = app.xr_grip_active_r = False
            app.xr_clutch_state_l = app.xr_clutch_state_r = "LOCKED"
            app.xr_anchor_ctrl_pos_l = app.xr_anchor_ctrl_pos_r = None
            app.xr_anchor_ctrl_rot_l = app.xr_anchor_ctrl_rot_r = None
            app.xr_anchor_marker_pos_l = app.xr_anchor_marker_pos_r = None
            app.xr_anchor_marker_rot_l = app.xr_anchor_marker_rot_r = None
        elif pose == "home" or name in ("go_home", "home"):
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("both", "home/smooth-home")
            app._start_smooth_homing("both")
        elif pose == "forward":
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("both", "preset/demo/sequence")
            start_preset(app, "forward")
        elif pose == "up":
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("both", "preset/demo/sequence")
            start_preset(app, "up")
        elif pose == "tpose":
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("both", "preset/demo/sequence")
            start_preset(app, "tpose")
        elif pose == "wave":
            app.active_sequence = "wave"
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("both", "preset/demo/sequence")
            app.seq_start_time = time.time()
            app.seq_origin_r = app.cur_target_pos_r.copy()
        elif pose == "circle":
            app.active_sequence = "circle"
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("both", "preset/demo/sequence")
            app.seq_start_time = time.time()
            app.seq_origin_l = app.cur_target_pos_l.copy()
            app.seq_origin_r = app.cur_target_pos_r.copy()
        elif pose == "stop":
            # Stop where the hand is now: target and gizmo jump onto the hand, nothing keeps pulling the arm.
            ee = app.env.get_ee_poses()
            for a, c in (("left", "l"), ("right", "r")):
                setattr(app, f"cur_target_pos_{c}", ee[a]["pos"].copy())
                setattr(app, f"cur_target_rot_{c}", ee[a]["rot"].copy())
                for fname, val in ((f"gizmo_follower_{c}", ee[a]["pos"]), (f"gizmo_rot_follower_{c}", ee[a]["rot"])):
                    f = getattr(app, fname, None)
                    if f is not None: f.reset(val.copy())
            app.preset_track_until = time.time() + 0.5
        return True

    return False

