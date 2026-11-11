"""
teleop/app/ik_dispatcher.py
Orchestrates dual-arm inverse kinematics solving, follower updates, and collision safety gating.
"""

import os
import time
import numpy as np
from teleop.kinematics.reach_projection import project_to_reachable_workspace


def solve_arm_ik(app, arm="both", posture_hint_l: np.ndarray = None, posture_hint_r: np.ndarray = None):
    """Solves IK for requested arms, applies critically damped filtering and self-collision gating."""
    ee_poses = app.env.get_ee_poses()

    slid_blocked_l = False
    slid_blocked_r = False

    is_hands = getattr(app, "input_mode", "") == "hands"
    use_cart = os.environ.get("CART_FOLLOWER", "1") != "0"

    if arm in ("left", "both"):
        vel_l = getattr(app, "handtrack_vel_l", None) if is_hands else None
        if use_cart:
            follow_pos_l = app.gizmo_follower_l.update(app.cur_target_pos_l, app.control_dt, target_vel=vel_l)
            follow_rot_l = app.gizmo_rot_follower_l.update(app.cur_target_rot_l, app.control_dt)
        else:
            follow_pos_l = np.array(app.cur_target_pos_l, dtype=np.float64).copy()
            follow_rot_l = np.array(app.cur_target_rot_l, dtype=np.float64).copy()

        reach_pos_l, clamped_l, d_beyond_l = project_to_reachable_workspace(follow_pos_l, side="left")
        app.reach_warning_l = f"Target out of reach ({d_beyond_l*100:.0f} cm beyond)" if clamped_l else ""

        slid_pos_l, slid_blocked_l = app.collision_checker.project_target_tangent(
            ee_poses["left"]["pos"], reach_pos_l, app.env.data
        )
        warm_q_l = getattr(app, "last_ik_sol_left", app.env.get_left_arm_qpos())
        hint_l = posture_hint_l if posture_hint_l is not None else (warm_q_l if is_hands else None)
        kwargs_l = {"elbow_hint": hint_l} if (hint_l is not None and hasattr(app.ik_left, "set_elbow_hint")) else {}
        q_sol_l, succ_l, pos_err_l, _, _ = app.ik_left.solve(
            target_pos=slid_pos_l,
            target_rot=follow_rot_l,
            init_qpos=warm_q_l,
            max_iters=14,
            pos_tol=0.005, rot_tol=0.040,
            **kwargs_l
        )
        app.last_ik_residual_l = float(pos_err_l)
        if not succ_l:
            q_sol_l = warm_q_l.copy()
        else:
            last_ee_l = getattr(app, "_last_ee_target_l", None)
            last_rot_l = getattr(app, "_last_ee_rot_l", None)
            ee_move_l = float(np.linalg.norm(slid_pos_l - last_ee_l)) if last_ee_l is not None else 1.0
            rot_move_l = float(np.linalg.norm(follow_rot_l - last_rot_l)) if last_rot_l is not None else 1.0
            if float(np.max(np.abs(q_sol_l - warm_q_l))) > 0.10 and ee_move_l < 0.005 and rot_move_l < 0.05:
                q_sol_l = warm_q_l.copy()
            else:
                app._last_ee_target_l = np.array(slid_pos_l).copy()
                app._last_ee_rot_l = np.array(follow_rot_l).copy()
        app.last_ik_sol_left = q_sol_l.copy()
        cand_left = q_sol_l.copy()
    else:
        cand_left = app.last_arm_cmd_left.copy()

    if arm in ("right", "both"):
        vel_r = getattr(app, "handtrack_vel_r", None) if is_hands else None
        if use_cart:
            follow_pos_r = app.gizmo_follower_r.update(app.cur_target_pos_r, app.control_dt, target_vel=vel_r)
            follow_rot_r = app.gizmo_rot_follower_r.update(app.cur_target_rot_r, app.control_dt)
        else:
            follow_pos_r = np.array(app.cur_target_pos_r, dtype=np.float64).copy()
            follow_rot_r = np.array(app.cur_target_rot_r, dtype=np.float64).copy()

        reach_pos_r, clamped_r, d_beyond_r = project_to_reachable_workspace(follow_pos_r, side="right")
        app.reach_warning_r = f"Target out of reach ({d_beyond_r*100:.0f} cm beyond)" if clamped_r else ""

        slid_pos_r, slid_blocked_r = app.collision_checker.project_target_tangent(
            ee_poses["right"]["pos"], reach_pos_r, app.env.data
        )
        warm_q_r = getattr(app, "last_ik_sol_right", app.env.get_right_arm_qpos())
        hint_r = posture_hint_r if posture_hint_r is not None else (warm_q_r if is_hands else None)
        kwargs_r = {"elbow_hint": hint_r} if (hint_r is not None and hasattr(app.ik_right, "set_elbow_hint")) else {}
        q_sol_r, succ_r, pos_err_r, _, _ = app.ik_right.solve(
            target_pos=slid_pos_r,
            target_rot=follow_rot_r,
            init_qpos=warm_q_r,
            max_iters=14,
            pos_tol=0.005, rot_tol=0.040,
            **kwargs_r
        )
        app.last_ik_residual_r = float(pos_err_r)
        if not succ_r:
            q_sol_r = warm_q_r.copy()
        else:
            last_ee_r = getattr(app, "_last_ee_target_r", None)
            last_rot_r = getattr(app, "_last_ee_rot_r", None)
            ee_move_r = float(np.linalg.norm(slid_pos_r - last_ee_r)) if last_ee_r is not None else 1.0
            rot_move_r = float(np.linalg.norm(follow_rot_r - last_rot_r)) if last_rot_r is not None else 1.0
            if float(np.max(np.abs(q_sol_r - warm_q_r))) > 0.10 and ee_move_r < 0.005 and rot_move_r < 0.05:
                q_sol_r = warm_q_r.copy()
            else:
                app._last_ee_target_r = np.array(slid_pos_r).copy()
                app._last_ee_rot_r = np.array(follow_rot_r).copy()
        app.last_ik_sol_right = q_sol_r.copy()
        cand_right = q_sol_r.copy()
    else:
        cand_right = app.last_arm_cmd_right.copy()

    app.reach_warning = getattr(app, "reach_warning_l", "") or getattr(app, "reach_warning_r", "")
    if app.reach_warning:
        now_t = time.time()
        if now_t - getattr(app, "_last_reach_log_t", 0.0) >= 1.0:
            app._last_reach_log_t = now_t
            print(f"[REACH] {app.reach_warning}")

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

        # Log-only collision calculation (never blocks, never faults, never sets collision_blocked)
        _, _, min_dist = app.collision_checker.filter_step(curr_qpos, target_qpos)
        app.collision_blocked = False
        app.min_collision_distance_m = float(min_dist)

        if hasattr(app, "real_follower_instance") and app.real_follower_instance:
            app.real_follower_instance.step(raw_q14)
        app.last_arm_cmd_left = cand_left.copy()
        app.last_arm_cmd_right = cand_right.copy()
        app.shadow_cmd_q14 = raw_q14.copy()
    else:
        app.collision_blocked = False
        app.last_arm_cmd_left = cand_left
        app.last_arm_cmd_right = cand_right

    backend_key = (getattr(app, "backend", "sim") or "sim").lower()
    if backend_key not in ("real", "ros2", "real-udp", "udp"):
        app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)


def freeze_arm_target(app, arm: str = "both"):
    """Latches final Cartesian target pose and solves IK once to high-precision convergence."""
    ee_poses = app.env.get_ee_poses() if hasattr(app, "env") else None
    arms_to_freeze = ["left", "right"] if arm in ("both", "all") else [arm]

    for a in arms_to_freeze:
        setattr(app, f"gizmo_active_{'l' if a == 'left' else 'r'}", False)
        ik_solver = getattr(app, f"ik_{a}", None)
        follower_pos = getattr(app, f"gizmo_follower_{'l' if a == 'left' else 'r'}", None)
        follower_rot = getattr(app, f"gizmo_rot_follower_{'l' if a == 'left' else 'r'}", None)
        cur_pos = getattr(app, f"cur_target_pos_{'l' if a == 'left' else 'r'}")
        cur_rot = getattr(app, f"cur_target_rot_{'l' if a == 'left' else 'r'}")

        if follower_pos:
            follower_pos.reset(cur_pos)
        if follower_rot:
            follower_rot.reset(cur_rot)

        if ik_solver:
            warm_q = getattr(app, f"last_ik_sol_{a}", getattr(app, f"last_arm_cmd_{a}"))
            proj_pos, _, _ = project_to_reachable_workspace(cur_pos, side=a)
            q_conv, _, pos_err, _, _ = ik_solver.solve(
                target_pos=proj_pos,
                target_rot=cur_rot,
                init_qpos=warm_q,
                max_iters=50,
                pos_tol=1e-4,
                rot_tol=1e-3,
            )
            setattr(app, f"last_ik_sol_{a}", q_conv.copy())
            setattr(app, f"last_arm_cmd_{a}", q_conv.copy())

    app.status_warning = "TARGET_FROZEN"
    seq = None
    if hasattr(app, "robot_backend") and hasattr(app.robot_backend, "_trace_logger"):
        tl = app.robot_backend._trace_logger
        if tl:
            seq = getattr(app.robot_backend, "_seq", None)
            tl.log_event("TARGET_FROZEN", seq=seq)
    print(f"[FREEZE] Target frozen for {arm}: status=TARGET_FROZEN")


