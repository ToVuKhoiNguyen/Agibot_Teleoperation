"""
teleop/app/ik_dispatcher.py
Orchestrates dual-arm inverse kinematics solving, follower updates, and collision safety gating.
"""

import numpy as np


def solve_arm_ik(app, arm="both", posture_hint_l: np.ndarray = None, posture_hint_r: np.ndarray = None):
    """Solves IK for requested arms, applies critically damped filtering and self-collision gating."""
    ee_poses = app.env.get_ee_poses()

    slid_blocked_l = False
    slid_blocked_r = False

    is_hands = getattr(app, "input_mode", "") == "hands"

    if arm in ("left", "both"):
        vel_l = getattr(app, "handtrack_vel_l", None) if is_hands else None
        follow_pos_l = app.gizmo_follower_l.update(app.cur_target_pos_l, app.control_dt, target_vel=vel_l)
        follow_rot_l = app.gizmo_rot_follower_l.update(app.cur_target_rot_l, app.control_dt)
        slid_pos_l, slid_blocked_l = app.collision_checker.project_target_tangent(
            ee_poses["left"]["pos"], follow_pos_l, app.env.data
        )
        warm_q_l = getattr(app, "last_ik_sol_left", app.env.get_left_arm_qpos())
        hint_l = posture_hint_l if posture_hint_l is not None else (warm_q_l if is_hands else None)
        kwargs_l = {"elbow_hint": hint_l} if (hint_l is not None and hasattr(app.ik_left, "set_elbow_hint")) else {}
        q_sol_l, _, pos_err_l, _, _ = app.ik_left.solve(
            target_pos=slid_pos_l,
            target_rot=follow_rot_l,
            init_qpos=warm_q_l,
            max_iters=14,
            pos_tol=0.001, rot_tol=0.035,
            **kwargs_l
        )
        app.last_ik_residual_l = float(pos_err_l)
        app.last_ik_sol_left = q_sol_l.copy()
        cand_left = q_sol_l if is_hands else app.output_filter_left_gizmo.filter(q_sol_l)
    else:
        cand_left = app.last_arm_cmd_left.copy()

    if arm in ("right", "both"):
        vel_r = getattr(app, "handtrack_vel_r", None) if is_hands else None
        follow_pos_r = app.gizmo_follower_r.update(app.cur_target_pos_r, app.control_dt, target_vel=vel_r)
        follow_rot_r = app.gizmo_rot_follower_r.update(app.cur_target_rot_r, app.control_dt)
        slid_pos_r, slid_blocked_r = app.collision_checker.project_target_tangent(
            ee_poses["right"]["pos"], follow_pos_r, app.env.data
        )
        warm_q_r = getattr(app, "last_ik_sol_right", app.env.get_right_arm_qpos())
        hint_r = posture_hint_r if posture_hint_r is not None else (warm_q_r if is_hands else None)
        kwargs_r = {"elbow_hint": hint_r} if (hint_r is not None and hasattr(app.ik_right, "set_elbow_hint")) else {}
        q_sol_r, _, pos_err_r, _, _ = app.ik_right.solve(
            target_pos=slid_pos_r,
            target_rot=follow_rot_r,
            init_qpos=warm_q_r,
            max_iters=14,
            pos_tol=0.001, rot_tol=0.035,
            **kwargs_r
        )
        app.last_ik_residual_r = float(pos_err_r)
        app.last_ik_sol_right = q_sol_r.copy()
        cand_right = q_sol_r if is_hands else app.output_filter_right_gizmo.filter(q_sol_r)
    else:
        cand_right = app.last_arm_cmd_right.copy()

    # Gate 3: SafetyLayer self-collision verification & line-search sub-step interpolation
    if app.self_collision and app.collision_checker.enabled:
        curr_qpos = app.env.data.qpos.copy()
        target_qpos = curr_qpos.copy()
        target_qpos[app.left_arm_qadr] = cand_left
        target_qpos[app.right_arm_qadr] = cand_right

        # Run collision check with hysteresis (<0.5 cm block, >1.0 cm release)
        safe_qpos, blocked, min_dist = app.collision_checker.filter_step(curr_qpos, target_qpos)

        margin_block = 0.005
        margin_release = 0.010
        is_currently_blocked = getattr(app, "collision_blocked", False)
        if is_currently_blocked:
            hard_stop = blocked or (min_dist < margin_release)
        else:
            hard_stop = blocked or (min_dist < margin_block)

        collision_stopped = hard_stop or (app.gizmo_active_l and slid_blocked_l) or (app.gizmo_active_r and slid_blocked_r)

        app.collision_blocked = bool(collision_stopped)
        app.min_collision_distance_m = float(min_dist)

        if collision_stopped:
            app.status_warning = "Self-collision blocked"

        if hard_stop:
            app.last_arm_cmd_left = safe_qpos[app.left_arm_qadr].copy()
            app.last_arm_cmd_right = safe_qpos[app.right_arm_qadr].copy()
        else:
            app.last_arm_cmd_left = cand_left
            app.last_arm_cmd_right = cand_right
    else:
        app.collision_blocked = False
        app.last_arm_cmd_left = cand_left
        app.last_arm_cmd_right = cand_right

    app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)

