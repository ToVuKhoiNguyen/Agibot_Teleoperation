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


def start_smooth_homing(app, side: str = "both", duration: float = None):
    """Planned min-jerk joint trajectory to HOME (armor-checked path), hands opened. `duration` is ignored:
    the time follows the joint distance and the limiter speed."""
    from teleop.app.joint_preset import start_preset
    if side in ("left", "both"):
        app.target_curls_l = np.zeros(5, dtype=np.float64)
        app.target_pinch_l = 0.0
        app.hand_closure_l = 0.0
        app.grasp_type_l = "cylinder"   # open = closure 0; a grasp type of "open" would block stick/slider closing
        app.direct_hand_override_l = False
    if side in ("right", "both"):
        app.target_curls_r = np.zeros(5, dtype=np.float64)
        app.target_pinch_r = 0.0
        app.hand_closure_r = 0.0
        app.grasp_type_r = "cylinder"   # open = closure 0; a grasp type of "open" would block stick/slider closing
        app.direct_hand_override_r = False
    start_preset(app, "home", side=side)


def reset_arm_to_home(app, side: str):
    start_smooth_homing(app, side)


def update_homing(app, dt: float):
    """Steps the planned joint trajectory (Home / presets)."""
    if not app.homing_active:
        return

    # Interrupt homing if user engages clutch
    if (app.xr_grip_active_l and app.homing_side in ("left", "both")) or \
       (app.xr_grip_active_r and app.homing_side in ("right", "both")):
        app.homing_active = False
        app.joint_motion = None
        log_xr_event(app, "Homing interrupted by user clutch")
        return

    from teleop.app.joint_preset import update_joint_motion
    update_joint_motion(app, dt)
