"""
teleop/kinematics/filters.py
Signal processing and smoothing filters:
- One Euro filter (position and finger inputs)
- Quaternion SLERP low-pass filter
- Tracking spike rejection
- Dead zone filter
- Smooth clutch ramp blender
- Second-order critically damped & Ruckig trajectory output smoothing
"""

import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp


class LowPassFilter:
    def __init__(self, alpha: float = 1.0):
        self.alpha = float(alpha)
        self.y = None

    def reset(self):
        self.y = None

    def filter(self, x: np.ndarray, alpha: float = None) -> np.ndarray:
        if alpha is not None:
            self.alpha = float(alpha)
        if self.y is None:
            self.y = np.copy(x)
        else:
            self.y = self.alpha * x + (1.0 - self.alpha) * self.y
        return self.y


class OneEuroFilter:
    """
    One Euro filter for adaptive noise filtering and low lag.
    Casiez et al., CHI 2012.
    """

    def __init__(self, min_cutoff: float = 1.2, beta: float = 0.007, d_cutoff: float = 1.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.x_filt = LowPassFilter()
        self.dx_filt = LowPassFilter()
        self.last_time = None

    def reset(self):
        self.x_filt.reset()
        self.dx_filt.reset()
        self.last_time = None

    def _alpha(self, cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def filter(self, x: np.ndarray, timestamp: float = None) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        if timestamp is None:
            timestamp = time.perf_counter()

        if self.last_time is None:
            self.last_time = timestamp
            self.x_filt.filter(x, 1.0)
            return x

        dt = timestamp - self.last_time
        self.last_time = timestamp
        if dt <= 1e-5:
            dt = 1e-4

        # Compute derivative
        prev_x = self.x_filt.y
        dx = (x - prev_x) / dt
        edx = self.dx_filt.filter(dx, self._alpha(self.d_cutoff, dt))

        # Adaptive cutoff
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(edx))
        return self.x_filt.filter(x, self._alpha(cutoff, dt))


class QuaternionFilter:
    """Smooths orientations using spherical linear interpolation (SLERP)."""

    def __init__(self, alpha: float = 0.35):
        self.alpha = float(alpha)
        self.current_rot = None

    def reset(self):
        self.current_rot = None

    def filter(self, rot_mat: np.ndarray) -> np.ndarray:
        target_r = R.from_matrix(rot_mat)
        if self.current_rot is None:
            self.current_rot = target_r
            return rot_mat

        key_times = [0.0, 1.0]
        # Align quaternion signs to avoid 360-deg flips
        q_cur = self.current_rot.as_quat()
        q_tgt = target_r.as_quat()
        if np.dot(q_cur, q_tgt) < 0.0:
            target_r = R.from_quat(-q_tgt)

        slerp = Slerp(key_times, R.from_quat([self.current_rot.as_quat(), target_r.as_quat()]))
        interp_r = slerp([self.alpha])[0]
        self.current_rot = interp_r
        return interp_r.as_matrix()


class InputFilterPipeline:
    """Combined pipeline for position OneEuro, orientation SLERP, spike rejection and deadzone."""

    def __init__(
        self,
        min_cutoff: float = 1.2,
        beta: float = 0.007,
        d_cutoff: float = 1.0,
        slerp_alpha: float = 0.35,
        pos_jump_threshold: float = 0.30,
        pos_deadzone: float = 0.0015,
        rot_deadzone: float = 0.015
    ):
        self.pos_filter = OneEuroFilter(min_cutoff=min_cutoff, beta=beta, d_cutoff=d_cutoff)
        self.rot_filter = QuaternionFilter(alpha=slerp_alpha)
        self.pos_jump_threshold = float(pos_jump_threshold)
        self.pos_deadzone = float(pos_deadzone)
        self.rot_deadzone = float(rot_deadzone)

        self.last_valid_pos = None
        self.last_valid_rot = None

    def reset(self):
        self.pos_filter.reset()
        self.rot_filter.reset()
        self.last_valid_pos = None
        self.last_valid_rot = None

    def filter(self, pos: np.ndarray, rot: np.ndarray, timestamp: float = None) -> tuple[np.ndarray, np.ndarray]:
        pos = np.asarray(pos, dtype=np.float64)
        rot = np.asarray(rot, dtype=np.float64)

        if self.last_valid_pos is not None:
            # 1. Spike rejection check
            jump = float(np.linalg.norm(pos - self.last_valid_pos))
            if jump > self.pos_jump_threshold:
                # Reject spike, hold previous valid position
                pos = np.copy(self.last_valid_pos)

            # 2. Dead zone check
            if jump < self.pos_deadzone:
                pos = np.copy(self.last_valid_pos)

        # One Euro position filtering
        smoothed_pos = self.pos_filter.filter(pos, timestamp)
        self.last_valid_pos = np.copy(smoothed_pos)

        # Orientation filtering
        smoothed_rot = self.rot_filter.filter(rot)
        self.last_valid_rot = np.copy(smoothed_rot)

        return smoothed_pos, smoothed_rot


class ClutchRamp:
    """Smooth ramp blend (0.3s) when tracking starts, resumes or clutch re-anchors."""

    def __init__(self, duration_s: float = 0.30):
        self.duration_s = float(duration_s)
        self.start_time = None
        self.start_pos = None
        self.active = False

    def trigger(self, current_pos: np.ndarray):
        self.start_time = time.perf_counter()
        self.start_pos = np.copy(current_pos)
        self.active = True

    def blend(self, target_pos: np.ndarray) -> np.ndarray:
        if not self.active or self.start_pos is None:
            return target_pos

        elapsed = time.perf_counter() - self.start_time
        if elapsed >= self.duration_s:
            self.active = False
            return target_pos

        # Smooth cubic S-curve (smoothstep)
        t = np.clip(elapsed / self.duration_s, 0.0, 1.0)
        s = t * t * (3.0 - 2.0 * t)
        return (1.0 - s) * self.start_pos + s * target_pos


class JointTrajectoryFilter:
    """Acceleration/jerk-limited trajectory filter for 7-DoF arm joints using Ruckig or 2nd-order critically damped filter."""

    def __init__(self, num_joints: int = 7, max_vel: float = 3.0, max_acc: float = 6.28, max_jerk: float = 40.0, dt: float = 0.02):
        self.num_joints = num_joints
        self.dt = dt
        self.max_vel = float(max_vel)
        self.max_acc = float(max_acc)
        self.max_jerk = float(max_jerk)

        self.cur_q = None
        self.cur_v = np.zeros(num_joints)
        self.cur_a = np.zeros(num_joints)

        try:
            import ruckig
            self.otg = ruckig.Ruckig(num_joints, dt)
            self.inp = ruckig.InputParameter(num_joints)
            self.out = ruckig.OutputParameter(num_joints)
            self.inp.max_velocity = [self.max_vel] * num_joints
            self.inp.max_acceleration = [self.max_acc] * num_joints
            self.inp.max_jerk = [self.max_jerk] * num_joints
            self.use_ruckig = True
        except Exception:
            self.use_ruckig = False

    def reset(self, q_init: np.ndarray):
        self.cur_q = np.copy(q_init)
        self.cur_v = np.zeros(self.num_joints)
        self.cur_a = np.zeros(self.num_joints)
        if self.use_ruckig:
            self.inp.current_position = list(q_init)
            self.inp.current_velocity = [0.0] * self.num_joints
            self.inp.current_acceleration = [0.0] * self.num_joints

    def filter(self, target_q: np.ndarray) -> np.ndarray:
        if self.cur_q is None:
            self.reset(target_q)
            return np.copy(target_q)

        if self.use_ruckig:
            self.inp.target_position = list(target_q)
            res = self.otg.update(self.inp, self.out)
            self.out.pass_to_input(self.inp)
            self.cur_q = np.array(self.out.new_position)
            self.cur_v = np.array(self.out.new_velocity)
            self.cur_a = np.array(self.out.new_acceleration)
            return self.cur_q
        else:
            # 2nd-order critically damped filter: d2q/dt2 = omega^2*(q_tgt - q) - 2*omega*dq
            omega = 25.0
            error = target_q - self.cur_q
            acc = (omega ** 2) * error - 2.0 * omega * self.cur_v
            # Clamp acceleration and velocity
            acc = np.clip(acc, -self.max_acc, self.max_acc)
            self.cur_v += acc * self.dt
            self.cur_v = np.clip(self.cur_v, -self.max_vel, self.max_vel)
            self.cur_q += self.cur_v * self.dt
            return self.cur_q

