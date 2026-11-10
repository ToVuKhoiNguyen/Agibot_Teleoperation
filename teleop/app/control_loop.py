"""
teleop/app/control_loop.py
Fixed-rate 200 Hz main teleoperation simulation and control loop.
"""

import os
import time
import numpy as np
from teleop.utils.startup_banner import get_local_ip, print_startup_banner
from teleop.app.telemetry_builder import build_status_packet
from teleop.app.sync_manager import process_arm_sync_and_dispatch


def run_control_loop(app):
    """Executes the fixed-rate simulation loop, safety checks, and telemetry broadcasting."""
    dt = getattr(app, "control_dt", 0.01)
    step = 0
    local_ip = get_local_ip()
    backend_key = (getattr(app, "backend", "sim") or "sim").lower()
    is_real = backend_key in ("real", "ros2", "real-udp", "udp")

    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    viewer_label = "Active (Passive GUI Window)" if (app.env.viewer is not None) else ("Disabled (No display)" if not has_display else "OFF")
    sim_label = "Bypassed (Real Robot Hardware)" if is_real else "MuJoCo (home keyframe, legs locked, arms active)"
    initial_status = {
        "sim": sim_label,
        "viewer": viewer_label,
        "server": f"HTTPS/WSS Listening on 0.0.0.0:{app.port}",
        "panel": "Ready (3D Viewport + Gizmos + Buttons)",
        "xr_receiver": "Active & Listening (Meta Quest 3S WebXR)" if app.tracker else "Idle",
        "safety": "Active (Joint limits, velocity clamps, E-Stop cleared)",
    }
    quest_status = app.quest_manager.get_status() if hasattr(app, "quest_manager") else {}
    print_startup_banner(app.port, local_ip, initial_status, quest_status)

    try:
        target_step_dt = dt / app.speed_scale
        next_step_time = time.perf_counter()

        last_exc_logged = None
        while app.running:
            try:
                # Process Web Panel commands
                cmd = app.server.get_command()
                while cmd is not None:
                    app._handle_panel_command(cmd)
                    cmd = app.server.get_command()

                # Quest 3S Watchdog: if no packet for timeout, release clutches
            now_t = time.time()
            if app.xr_input_active and (now_t - app.xr_last_packet_time) > app.xr_timeout:
                if app.xr_clutch_state_l != "LOCKED" or app.xr_clutch_state_r != "LOCKED" or app.xr_grip_active_l or app.xr_grip_active_r:
                    app.xr_clutch_state_l = "LOCKED"
                    app.xr_clutch_state_r = "LOCKED"
                    app.xr_grip_active_l = False
                    app.xr_grip_active_r = False
                    app.xr_anchor_ctrl_pos_l = None
                    app.xr_anchor_ctrl_pos_r = None
                    app.xr_anchor_ctrl_rot_l = None
                    app.xr_anchor_ctrl_rot_r = None
                    app.gizmo_active_l = False
                    app.gizmo_active_r = False
                    if hasattr(app, "robot_safety") and app.robot_safety is not None:
                        app.robot_safety.hold()
                    app.status_warning = "Quest XR tracking lost (watchdog 350ms): clutches released"
                    app._log_xr_event("Watchdog 350ms -> clutches released")
                app.xr_input_active = False

            if getattr(app, "input_mode", "") == "hands":
                if (now_t - getattr(app, "handtrack_last_t", now_t)) > 0.150:
                    app.gizmo_active_l = False
                    app.gizmo_active_r = False
                    app.handtrack_clutch_l = False
                    app.handtrack_clutch_r = False

            if hasattr(app, "robot_safety") and app.robot_safety is not None:
                app.robot_safety.check_watchdog(now=now_t)

            if app.homing_active:
                app._update_homing(dt)
            elif app.active_sequence:
                app._update_sequence(dt)
            elif not app.state_estop:
                if app.gizmo_active_l and app.gizmo_active_r:
                    app._solve_arm_ik("both")
                elif app.gizmo_active_l:
                    app._solve_arm_ik("left")
                elif app.gizmo_active_r:
                    app._solve_arm_ik("right")
            if app.walking_demo_active:
                app._update_walk_demo()

            # Step simulation
            if not app.state_estop:
                if app.hand_sweep_active:
                    app._update_hand_sweep(dt)

                if getattr(app, "direct_hand_override_l", False):
                    cand_hand_l = app.hand_retarget_left.retarget_direct_joints(app.direct_hand_target_l, dt=dt)
                else:
                    cand_hand_l = app.hand_retarget_left.retarget_curls(app.target_curls_l, pinch_strength=app.target_pinch_l, dt=dt)

                if getattr(app, "direct_hand_override_r", False):
                    cand_hand_r = app.hand_retarget_right.retarget_direct_joints(app.direct_hand_target_r, dt=dt)
                else:
                    cand_hand_r = app.hand_retarget_right.retarget_curls(app.target_curls_r, pinch_strength=app.target_pinch_r, dt=dt)

                # Hand Command Gate: Forward kinematics check & line search backoff
                if np.max(np.abs(cand_hand_l - app.current_safe_hand_l)) < 1e-4:
                    safe_hand_l, blocked_l, dist_l = app.current_safe_hand_l, False, 0.005
                else:
                    safe_hand_l, blocked_l, dist_l = app.collision_checker.filter_hand_step(
                        "left", app.current_safe_hand_l, cand_hand_l, margin=0.001
                    )

                if np.max(np.abs(cand_hand_r - app.current_safe_hand_r)) < 1e-4:
                    safe_hand_r, blocked_r, dist_r = app.current_safe_hand_r, False, 0.005
                else:
                    safe_hand_r, blocked_r, dist_r = app.collision_checker.filter_hand_step(
                        "right", app.current_safe_hand_r, cand_hand_r, margin=0.001
                    )
                app.current_safe_hand_l = safe_hand_l
                app.current_safe_hand_r = safe_hand_r
                app.last_hand_cmd_left = safe_hand_l
                app.last_hand_cmd_right = safe_hand_r
                app.hand_collision_blocked = bool(blocked_l or blocked_r)
                app.min_hand_distance_m = min(dist_l, dist_r)
                if app.collision_blocked:
                    app.clamp_counter_arm = getattr(app, "clamp_counter_arm", 0) + 1
                if app.hand_collision_blocked:
                    app.clamp_counter_hand = getattr(app, "clamp_counter_hand", 0) + 1

                process_arm_sync_and_dispatch(app, is_real, app.control_dt)
                if (app.hand_closure_l >= 0.95 or app.hand_closure_r >= 0.95) and not getattr(app, "_logged_hand_close_trace", False):
                    app._logged_hand_close_trace = True
                    print(f"[HAND_CLOSE_TRACE] Commanded 6 joints L: {app.last_hand_cmd_left.round(3)}, R: {app.last_hand_cmd_right.round(3)}")

                if not is_real:
                    if getattr(app, "real_follower_mode", False):
                        if not hasattr(app, "_sim_lag_queue"):
                            app._sim_lag_queue = []
                        app._sim_lag_queue.append((app.last_arm_cmd_left.copy(), app.last_arm_cmd_right.copy()))
                        exec_l, exec_r = app._sim_lag_queue.pop(0) if len(app._sim_lag_queue) > 2 else app._sim_lag_queue[0]
                    else:
                        exec_l, exec_r = app.last_arm_cmd_left, app.last_arm_cmd_right
                    app.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=exec_l, hold_right_qpos=exec_r)
                elif app.env.viewer is not None and app.env.viewer.is_running():
                    app.env.viewer.sync()
            else:
                if not is_real:
                    app.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=app.last_arm_cmd_left, hold_right_qpos=app.last_arm_cmd_right)
                elif app.env.viewer is not None and app.env.viewer.is_running():
                    app.env.viewer.sync()

            # Performance tracking telemetry (lag and steady error)
            if step % 20 == 0 and (app.gizmo_active_l or app.gizmo_active_r):
                ee_now = app.env.get_ee_poses()
                if app.gizmo_active_l and app.gizmo_follower_l.v is not None:
                    v_mag = float(np.linalg.norm(app.gizmo_follower_l.v))
                    err_m = float(np.linalg.norm(np.array(ee_now["left"]["pos"]) - app.cur_target_pos_l))
                    if v_mag > 0.03:
                        lag_ms = (err_m / v_mag) * 1000.0
                        print(f"[GIZMO TRACK] L lag: {lag_ms:.1f} ms | steady err: {err_m*100.0:.2f} cm (vel: {v_mag:.2f} m/s)")
                if app.gizmo_active_r and app.gizmo_follower_r.v is not None:
                    v_mag = float(np.linalg.norm(app.gizmo_follower_r.v))
                    err_m = float(np.linalg.norm(np.array(ee_now["right"]["pos"]) - app.cur_target_pos_r))
                    if v_mag > 0.03:
                        lag_ms = (err_m / v_mag) * 1000.0
                        print(f"[GIZMO TRACK] R lag: {lag_ms:.1f} ms | steady err: {err_m*100.0:.2f} cm (vel: {v_mag:.2f} m/s)")

            # Update visual body poses for steady 60 Hz streaming (every 2nd step at 200 Hz = 100 Hz sampling)
            if step % 2 == 0:
                poses = app.env.get_visual_body_poses()
                has_nan = False
                for k, v in poses.items():
                    if any(np.isnan(x) or np.isinf(x) for x in v):
                        has_nan = True
                        app.nan_detected = True
                        if not app.nan_logged_warning:
                            print(f"[WARN] NaN/Inf detected in streamed pose for '{k}'. Skipping frame.")
                            app.nan_logged_warning = True
                        break
                if not has_nan:
                    app.server.broadcast_poses(poses)

            # Telemetry updates (20 Hz at 200 Hz loop = every 10 steps)
            if step % 10 == 0:
                now_perf_hz = time.perf_counter()
                if not hasattr(app, "_loop_hz_last_t"):
                    app._loop_hz_count = 0
                    app._loop_hz_last_t = now_perf_hz
                    app.loop_hz = 200.0
                app._loop_hz_count += 10
                if now_perf_hz - app._loop_hz_last_t >= 0.5:
                    app.loop_hz = app._loop_hz_count / (now_perf_hz - app._loop_hz_last_t)
                    app._loop_hz_count = 0
                    app._loop_hz_last_t = now_perf_hz
                status_packet = build_status_packet(app)
                app.server.set_status(status_packet)
                if hasattr(app, "robot_safety") and getattr(app.robot_safety, "fault_latched", False):
                    app.status_warning = f"HARDWARE FAULT: {app.robot_safety.fault_reason}"
                else:
                    app.status_warning = ""
                if hasattr(app, "target_tracer") and app.target_tracer:
                    app.target_tracer.tick(app)

                # Periodic real-udp state logger (every ~2 s = 400 steps)
                if is_real and step % 400 == 0:
                    st_str = getattr(app, "sync_state", "READY")
                    age_s = getattr(app, "last_state_feedback_age_s", 999.0)
                    if st_str == "SYNC": reason = "WAITING FEEDBACK (no packet on UDP 5006)"
                    elif st_str == "READY": reason = "NOT ENGAGED"
                    elif getattr(getattr(app, "robot_safety", None), "fault_latched", False):
                        reason = f"FAULT ({app.robot_safety.fault_reason})"
                    elif getattr(app, "max_sim_real_gap_rad", 0.0) >= 0.095: reason = "LEAD LIMIT"
                    elif age_s > 0.200: reason = f"STALE FEEDBACK ({age_s*1000:.0f}ms)"
                    else: reason = "ACTIVE"
                    print(f"[STATUS 2s] State: {st_str} | Arm not moving reason: {reason}")

            except Exception as e:
                msg = str(e)
                if msg != last_exc_logged:
                    last_exc_logged = msg
                    print(f"[CONTROL_LOOP ERROR] {type(e).__name__}: {msg}")
                if hasattr(app, "mirror_limiter") and hasattr(app, "real_q14_measured") and app.real_q14_measured is not None:
                    app.mirror_limiter.reset(app.real_q14_measured)

            step += 1
            next_step_time += target_step_dt
            now_perf = time.perf_counter()
            sleep_t = next_step_time - now_perf
            if sleep_t > 0.0005:
                time.sleep(sleep_t)
            elif sleep_t < -0.05:
                app.frame_gaps = getattr(app, "frame_gaps", 0) + 1
                next_step_time = now_perf

    except KeyboardInterrupt:
        pass
    finally:
        app.close()

