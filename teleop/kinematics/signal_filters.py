"""
teleop/kinematics/signal_filters.py
Signal processing and smoothing filters:
- LowPassFilter
- OneEuroFilter (Casiez et al., CHI 2012)
- QuaternionFilter (SLERP)
- InputFilterPipeline (combined pos/rot/spike/deadzone)
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
        self.spike_count = 0

    def reset(self):
        self.pos_filter.reset()
        self.rot_filter.reset()
        self.last_valid_pos = None
        self.last_valid_rot = None
        self.spike_count = 0

    def filter(self, pos: np.ndarray, rot: np.ndarray, timestamp: float = None) -> tuple[np.ndarray, np.ndarray]:
        pos = np.asarray(pos, dtype=np.float64)
        rot = np.asarray(rot, dtype=np.float64)

        if self.last_valid_pos is not None:
            # 1. Spike rejection check
            jump = float(np.linalg.norm(pos - self.last_valid_pos))
            if jump > self.pos_jump_threshold:
                self.spike_count += 1
                if self.spike_count < 3:
                    # Reject spike, hold previous valid position
                    pos = np.copy(self.last_valid_pos)
                else:
                    self.spike_count = 0
            else:
                self.spike_count = 0

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
