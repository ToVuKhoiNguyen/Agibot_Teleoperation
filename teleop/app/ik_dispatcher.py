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
        last_ee_l = getattr(app, "_last_ee_target_l", None)
        ee_move_l = float(np.linalg.norm(slid_pos_l - last_ee_l)) if last_ee_l is not None else 1.0
        if float(np.max(np.abs(q_sol_l - warm_q_l))) > 0.10 and ee_move_l < 0.005:
            q_sol_l = warm_q_l.copy()
        else:
            app._last_ee_target_l = np.array(slid_pos_l).copy()
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
        last_ee_r = getattr(app, "_last_ee_target_r", None)
        ee_move_r = float(np.linalg.norm(slid_pos_r - last_ee_r)) if last_ee_r is not None else 1.0
        if float(np.max(np.abs(q_sol_r - warm_q_r))) > 0.10 and ee_move_r < 0.005:
            q_sol_r = warm_q_r.copy()
        else:
            app._last_ee_target_r = np.array(slid_pos_r).copy()
        app.last_ik_sol_right = q_sol_r.copy()
        cand_right = q_sol_r if is_hands else app.output_filter_right_gizmo.filter(q_sol_r)
    else:
        cand_right = app.last_arm_cmd_right.copy()

    # Gate 3: SafetyLayer self-collision verification & shadow follower evaluation
    if app.self_collision and app.collision_checker.enabled:
        curr_qpos = app.env.data.qpos.copy()

        # Shadow follower: post-limiter command tracking what will actually be sent
        if not hasattr(app, "shadow_cmd_q14") or app.shadow_cmd_q14 is None:
            app.shadow_cmd_q14 = np.concatenate([app.env.get_left_arm_qpos(), app.env.get_right_arm_qpos()])

        raw_q14 = np.concatenate([cand_left, cand_right])
        diff_q14 = raw_q14 - app.shadow_cmd_q14
        max_d = float(np.max(np.abs(diff_q14)))
        step_cap = getattr(app, "limiter_max_step", 0.01)
        scale = min(1.0, step_cap / max_d) if max_d > 1e-6 else 1.0
        post_limiter_q14 = app.shadow_cmd_q14 + diff_q14 * scale

        use_shadow = getattr(app, "real_follower_mode", False) or getattr(app, "backend", "") in ("real-udp", "real", "ros2")
        target_qpos = curr_qpos.copy()
        if use_shadow:
            target_qpos[app.left_arm_qadr] = post_limiter_q14[:7]
            target_qpos[app.right_arm_qadr] = post_limiter_q14[7:]
        else:
            target_qpos[app.left_arm_qadr] = cand_left
            target_qpos[app.right_arm_qadr] = cand_right

        # Predictive shadow collision check on future follower steps
        pred_blocked = False
        pred_clearance_m = 999.0
        if getattr(app.collision_checker, "shadow_check_enabled", False):
            if not hasattr(app, "real_follower_instance") or app.real_follower_instance is None:
                from teleop.kinematics.real_follower import RealLikeFollower
                app.real_follower_instance = RealLikeFollower(
                    max_step_rad=step_cap,
                    scaling_mode="uniform",
                    initial_q14=app.shadow_cmd_q14.copy(),
                )
            if use_shadow:
                n_horizon = getattr(app.collision_checker, "shadow_horizon", 10)
                future_poses = app.real_follower_instance.predict_ahead(raw_q14, n_steps=n_horizon)
                pred_clearance_m, _ = app.collision_checker.predict_clearance(future_poses, app.left_arm_qadr, app.right_arm_qadr)
                shadow_margin = getattr(app.collision_checker, "safety_margin", 0.005) + getattr(app.collision_checker, "shadow_inflation_margin", 0.03)
                if pred_clearance_m < shadow_margin:
                    pred_blocked = True
            else:
                app.real_follower_instance.reset(raw_q14)

        app.predicted_clearance_m = float(pred_clearance_m)

        # Run collision check with hysteresis (<0.5 cm block, >1.0 cm release)
        safe_qpos, blocked, min_dist = app.collision_checker.filter_step(curr_qpos, target_qpos)

        margin_block = 0.005
        margin_release = 0.010
        is_currently_blocked = getattr(app, "collision_blocked", False)
        if is_currently_blocked:
            hard_stop = blocked or (min_dist < margin_release)
        else:
            hard_stop = blocked or (min_dist < margin_block)

        if pred_blocked:
            hard_stop = True

        collision_stopped = hard_stop or (app.gizmo_active_l and slid_blocked_l) or (app.gizmo_active_r and slid_blocked_r)

        app.collision_blocked = bool(collision_stopped)
        app.min_collision_distance_m = float(min_dist)

        if collision_stopped:
            app.status_warning = f"Shadow collision predicted ({pred_clearance_m*1000:.1f} mm)" if pred_blocked else "Self-collision blocked"

        if hard_stop:
            app.last_arm_cmd_left = safe_qpos[app.left_arm_qadr].copy()
            app.last_arm_cmd_right = safe_qpos[app.right_arm_qadr].copy()
            app.shadow_cmd_q14 = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
            if hasattr(app, "real_follower_instance") and app.real_follower_instance:
                app.real_follower_instance.reset(app.shadow_cmd_q14)
        else:
            if use_shadow:
                if hasattr(app, "real_follower_instance") and app.real_follower_instance:
                    app.real_follower_instance.step(raw_q14)
                app.last_arm_cmd_left = post_limiter_q14[:7].copy()
                app.last_arm_cmd_right = post_limiter_q14[7:].copy()
                app.shadow_cmd_q14 = post_limiter_q14.copy()
            else:
                app.last_arm_cmd_left = cand_left
                app.last_arm_cmd_right = cand_right
                app.shadow_cmd_q14 = raw_q14.copy()
                if hasattr(app, "real_follower_instance") and app.real_follower_instance:
                    app.real_follower_instance.reset(raw_q14)
    else:
        app.collision_blocked = False
        app.last_arm_cmd_left = cand_left
        app.last_arm_cmd_right = cand_right

    backend_key = (getattr(app, "backend", "sim") or "sim").lower()
    if backend_key not in ("real", "ros2", "real-udp", "udp"):
        app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)

