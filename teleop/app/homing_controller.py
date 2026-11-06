"""
teleop/app/homing_controller.py
Smooth homing interpolation machine, precision scale updates, and event logging.
"""

import time
import numpy as np


def handle_signal(app, signum, frame):
    """Handle shutdown signals cleanly."""
    print("\n[INFO] Shutdown signal received. Cleaning up resources...")
    app.running = False


def log_xr_event(app, msg: str):
    """Logs an XR event to the ring buffer (max 16 entries)."""
    t_str = time.strftime("%H:%M:%S")
    app.xr_event_log.append({"t": t_str, "msg": str(msg)})
    if len(app.xr_event_log) > 16:
        app.xr_event_log.pop(0)


def update_precision(app, side: str, is_precision: bool):
    """Adjusts the critically damped follower bandwidth for fine precision tracking."""
    base_hz = 30.0
    factor = app.xr_precision_follower_factor if is_precision else 1.0
    new_omega = 2.0 * np.pi * base_hz * factor
    if side == "left":
        app.gizmo_follower_l.omega = new_omega
    elif side == "right":
        app.gizmo_follower_r.omega = new_omega


def slerp_quaternion(rot_start: np.ndarray, rot_target: np.ndarray, alpha: float) -> np.ndarray:
    """Spherical linear interpolation between two 3x3 rotation matrices."""
    try:
        from scipy.spatial.transform import Rotation as R, Slerp
        r1 = R.from_matrix(rot_start)
        r2 = R.from_matrix(rot_target)
        q1 = r1.as_quat()
        q2 = r2.as_quat()
        if np.dot(q1, q2) < 0.0:
            q2 = -q2
        slerp = Slerp([0.0, 1.0], R.from_quat([q1, q2]))
        return slerp([float(np.clip(alpha, 0.0, 1.0))])[0].as_matrix()
    except Exception:
        return (1.0 - alpha) * rot_start + alpha * rot_target


def start_smooth_homing(app, side: str = "both", duration: float = None):
    """Initiates smooth min-jerk trajectory transition to the home keyframe."""
    dur = duration if duration is not None else app.home_duration
    app.homing_duration_current = max(0.5, float(dur))
    app.homing_elapsed = 0.0
    app.homing_side = side
    app.active_sequence = None
    app.walking_demo_active = False

    ee = app.env.get_ee_poses()
    if side in ("left", "both"):
        app.homing_start_pos_l = ee["left"]["pos"].copy()
        app.homing_start_rot_l = ee["left"]["rot"].copy()
        app.homing_start_q_l = app.last_arm_cmd_left.copy()
        app.cur_target_pos_l = app.homing_start_pos_l.copy()
        app.cur_target_rot_l = app.homing_start_rot_l.copy()
        app.gizmo_follower_l.reset(app.homing_start_pos_l)
        app.gizmo_rot_follower_l.reset(app.homing_start_rot_l)
        app.xr_clutch_state_l = "LOCKED"
        app.xr_grip_active_l = False
        app.target_curls_l = np.zeros(5, dtype=np.float64)
        app.target_pinch_l = 0.0
        app.hand_closure_l = 0.0
        app.grasp_type_l = "open"
        app.direct_hand_override_l = False
        app.gizmo_active_l = True

    if side in ("right", "both"):
        app.homing_start_pos_r = ee["right"]["pos"].copy()
        app.homing_start_rot_r = ee["right"]["rot"].copy()
        app.homing_start_q_r = app.last_arm_cmd_right.copy()
        app.cur_target_pos_r = app.homing_start_pos_r.copy()
        app.cur_target_rot_r = app.homing_start_rot_r.copy()
        app.gizmo_follower_r.reset(app.homing_start_pos_r)
        app.gizmo_rot_follower_r.reset(app.homing_start_rot_r)
        app.xr_clutch_state_r = "LOCKED"
        app.xr_grip_active_r = False
        app.target_curls_r = np.zeros(5, dtype=np.float64)
        app.target_pinch_r = 0.0
        app.hand_closure_r = 0.0
        app.grasp_type_r = "open"
        app.direct_hand_override_r = False
        app.gizmo_active_r = True

    app.hand_sweep_active = False
    app.homing_active = True
    log_xr_event(app, f"Smooth homing started ({side}, {app.homing_duration_current:.1f}s)")


def reset_arm_to_home(app, side: str):
    start_smooth_homing(app, side)


def update_homing(app, dt: float):
    """Steps the smooth homing interpolation polynomial and drives IK."""
    if not app.homing_active:
        return

    # Interrupt homing if user engages clutch
    if (app.xr_grip_active_l and app.homing_side in ("left", "both")) or \
       (app.xr_grip_active_r and app.homing_side in ("right", "both")):
        app.homing_active = False
        log_xr_event(app, "Homing interrupted by user clutch")
        return

    app.homing_elapsed += dt
    tau = np.clip(app.homing_elapsed / app.homing_duration_current, 0.0, 1.0)
    # Min-jerk polynomial s(tau) = 10*tau^3 - 15*tau^4 + 6*tau^5
    s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)

    hint_l = None
    hint_r = None

    if app.homing_side in ("left", "both"):
        app.cur_target_pos_l = (1.0 - s) * app.homing_start_pos_l + s * app.robot_home_pos_l
        app.cur_target_rot_l = slerp_quaternion(app.homing_start_rot_l, app.robot_home_rot_l, s)
        hint_l = app.robot_home_q_l if tau >= 1.0 else ((1.0 - s) * app.homing_start_q_l + s * app.robot_home_q_l)

    if app.homing_side in ("right", "both"):
        app.cur_target_pos_r = (1.0 - s) * app.homing_start_pos_r + s * app.robot_home_pos_r
        app.cur_target_rot_r = slerp_quaternion(app.homing_start_rot_r, app.robot_home_rot_r, s)
        hint_r = app.robot_home_q_r if tau >= 1.0 else ((1.0 - s) * app.homing_start_q_r + s * app.robot_home_q_r)

    if hasattr(app, "target_tracer"): app.target_tracer.record_target_source(app.homing_side, "home/smooth-home")
    app._solve_arm_ik(app.homing_side, posture_hint_l=hint_l, posture_hint_r=hint_r)

    if tau >= 1.0:
        ee = app.env.get_ee_poses()
        err_l = np.linalg.norm(ee["left"]["pos"] - app.robot_home_pos_l) if app.homing_side in ("left", "both") else 0.0
        err_r = np.linalg.norm(ee["right"]["pos"] - app.robot_home_pos_r) if app.homing_side in ("right", "both") else 0.0
        if (err_l < 0.0015 and err_r < 0.0015) or (app.homing_elapsed >= app.homing_duration_current + 0.4):
            app.homing_active = False
            if app.homing_side in ("left", "both"):
                app.cur_target_pos_l = ee["left"]["pos"].copy()
                app.cur_target_rot_l = ee["left"]["rot"].copy()
                app.gizmo_follower_l.reset(app.cur_target_pos_l)
                app.gizmo_rot_follower_l.reset(app.cur_target_rot_l)
                app.gizmo_active_l = False
            if app.homing_side in ("right", "both"):
                app.cur_target_pos_r = ee["right"]["pos"].copy()
                app.cur_target_rot_r = ee["right"]["rot"].copy()
                app.gizmo_follower_r.reset(app.cur_target_pos_r)
                app.gizmo_rot_follower_r.reset(app.cur_target_rot_r)
                app.gizmo_active_r = False

