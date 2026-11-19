"""
teleop/app/motion_demos.py
Demonstration sequences: hand wave and circular trajectories.
"""

import time
import numpy as np


def update_sequence(app, dt: float):
    """Updates dynamic preset motion sequences (wave, circle)."""
    t_seq = time.time() - app.seq_start_time
    if app.active_sequence == "wave":
        if t_seq > 3.0:
            app.active_sequence = None
            app.cur_target_pos_r = app.seq_origin_r.copy()  # end where it started (gizmo follows telemetry)
            app.preset_track_until = time.time() + 1.5     # keep solving IK until the hand has settled there
            return
        omega = 2.0 * np.pi * 1.5
        dy = 0.08 * np.sin(omega * t_seq)
        dz = 0.04 * np.cos(omega * t_seq)
        app.cur_target_pos_r = app.seq_origin_r + np.array([0.0, dy, dz])
        if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("right", "preset/demo/sequence")
        app._solve_arm_ik("right")
    elif app.active_sequence == "circle":
        omega = 2.0 * np.pi * 0.5
        dy = 0.06 * np.cos(omega * t_seq)
        dz = 0.06 * np.sin(omega * t_seq)
        app.cur_target_pos_l = app.seq_origin_l + np.array([0.0, dy, dz])
        app.cur_target_pos_r = app.seq_origin_r + np.array([0.0, -dy, dz])
        if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("both", "preset/demo/sequence")
        app._solve_arm_ik("both")
