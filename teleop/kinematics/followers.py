"""
teleop/kinematics/followers.py
Trajectory following and tracking filters:
- ClutchRamp (smooth S-curve re-anchor blender)
- JointTrajectoryFilter (Ruckig or 2nd-order critically damped filter for joints)
- CriticallyDampedFollower (2nd-order critically damped gizmo follower)
- QuaternionSlerpFollower (continuous SLERP rotation follower)
"""

import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp


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
            try:
                self.inp.target_position = list(target_q)
                res = self.otg.update(self.inp, self.out)
                self.out.pass_to_input(self.inp)
                self.cur_q = np.array(self.out.new_position)
                self.cur_v = np.array(self.out.new_velocity)
                self.cur_a = np.array(self.out.new_acceleration)
                return self.cur_q
            except Exception:
                # Fallback to critically damped filter if ruckig has numerical issue
                pass

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


class CriticallyDampedFollower:
    """
    Light critically damped second-order follower for gizmo input.
    Bandwidth ~15-20 Hz, zero overshoot, lag < 35 ms.
    """
    def __init__(self, natural_freq_hz: float = 15.0, dt: float = 0.01):
        self.omega = 2.0 * math.pi * float(natural_freq_hz)
        self.dt = float(dt)
        self.x = None
        self.v = None

    def reset(self, x_init: np.ndarray = None):
        self.x = np.copy(x_init) if x_init is not None else None
        self.v = np.zeros_like(self.x) if self.x is not None else None

    def update(self, target: np.ndarray, dt: float = None, target_vel: np.ndarray = None) -> np.ndarray:
        target = np.asarray(target, dtype=np.float64)
        if self.x is None:
            self.reset(target)
            return np.copy(target)

        h = dt if dt is not None else self.dt
        if h <= 1e-6:
            h = 0.01

        tgt_eff = target + (2.0 / self.omega) * target_vel if target_vel is not None else target
        exp_wt = math.exp(-self.omega * h)
        diff = self.x - tgt_eff
        c1 = diff
        c2 = self.v + self.omega * diff
        new_x = tgt_eff + (c1 + c2 * h) * exp_wt
        new_v = (c2 - self.omega * (c1 + c2 * h)) * exp_wt
        self.x = new_x
        self.v = new_v
        return self.x


class QuaternionSlerpFollower:
    """
    Continuous SLERP follower for gizmo quaternion orientation.
    Filters rotation every control step with ~15 Hz bandwidth.
    """
    def __init__(self, bandwidth_hz: float = 15.0, dt: float = 0.01):
        self.bandwidth_hz = float(bandwidth_hz)
        self.dt = float(dt)
        # alpha = 1 - exp(-2*pi*f*dt)
        self.alpha = float(np.clip(1.0 - math.exp(-2.0 * math.pi * self.bandwidth_hz * self.dt), 0.01, 1.0))
        self.current_rot = None

    def reset(self, rot_mat_or_r = None):
        if rot_mat_or_r is None:
            self.current_rot = None
        elif isinstance(rot_mat_or_r, R):
            self.current_rot = rot_mat_or_r
        else:
            self.current_rot = R.from_matrix(rot_mat_or_r)

    def update(self, target_rot_mat: np.ndarray, dt: float = None) -> np.ndarray:
        target_r = R.from_matrix(target_rot_mat)
        if self.current_rot is None:
            self.current_rot = target_r
            return target_rot_mat.copy()

        h = dt if dt is not None else self.dt
        alpha = float(np.clip(1.0 - math.exp(-2.0 * math.pi * self.bandwidth_hz * h), 0.01, 1.0))

        q_cur = self.current_rot.as_quat()
        q_tgt = target_r.as_quat()
        if np.dot(q_cur, q_tgt) < 0.0:
            target_r = R.from_quat(-q_tgt)

        slerp = Slerp([0.0, 1.0], R.from_quat([self.current_rot.as_quat(), target_r.as_quat()]))
        interp_r = slerp([alpha])[0]
        self.current_rot = interp_r
        return interp_r.as_matrix()
