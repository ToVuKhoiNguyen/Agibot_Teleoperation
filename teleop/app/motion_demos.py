"""
teleop/app/motion_demos.py
Demonstration routines including hand-waving, circular trajectories, and simulated walking.
"""

import time
import numpy as np


def update_sequence(app, dt: float):
    """Updates dynamic preset motion sequences (wave, circle, reach-grasp)."""
    t_seq = time.time() - app.seq_start_time
    if app.active_sequence == "wave":
        if t_seq > 3.0:
            app.active_sequence = None
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
    elif app.active_sequence == "grasp":
        if t_seq < 1.0:
            frac = t_seq / 1.0
            app.cur_target_pos_l = app.robot_home_pos_l + frac * np.array([0.25, 0.0, 0.0])
            app.cur_target_pos_r = app.robot_home_pos_r + frac * np.array([0.25, 0.0, 0.0])
            if hasattr(app, "target_tracer"): app.target_tracer.record_target_source("both", "preset/demo/sequence")
            app._solve_arm_ik("both")
        elif t_seq < 2.0:
            app.target_curls_l = np.ones(5)
            app.target_curls_r = np.ones(5)
        elif t_seq < 3.0:
            app.target_curls_l = np.zeros(5)
            app.target_curls_r = np.zeros(5)
        else:
            app.active_sequence = None


def update_walk_demo(app):
    """Animates hip pitch joints with an out-of-phase sinusoidal leg swing."""
    t = (time.time() - app.walk_start_time)
    omega = 2.0 * np.pi * 0.8
    swing = 0.25 * np.sin(omega * t)
    if hasattr(app.env, "qpos_idx_map"):
        if "idx03_left_hip_pitch" in app.env.qpos_idx_map:
            app.env.data.qpos[app.env.qpos_idx_map["idx03_left_hip_pitch"]] = swing
        if "idx09_right_hip_pitch" in app.env.qpos_idx_map:
            app.env.data.qpos[app.env.qpos_idx_map["idx09_right_hip_pitch"]] = -swing

