"""
teleop/robot/safety_layer.py
SafetyLayer wrapping any RobotInterface: kinematic limits, rate clamping, watchdog, and deviation gates.
"""

import time
import numpy as np
from teleop.robot.interface import RobotInterface

# 14-DoF physical bounds: [L0..L6, R0..R6]
ARM_LOWER_14 = np.array([
    -2.91, -0.46, -2.91, -2.00, -2.88, -0.45, -0.35,  # Left
    -2.91, -1.60, -2.91,  0.03, -2.88, -0.45, -0.35   # Right
], dtype=np.float64)

ARM_UPPER_14 = np.array([
    2.91,  1.60,  2.91, -0.03,  2.88,  0.45,  0.35,   # Left
    2.91,  0.46,  2.91,  2.00,  2.88,  0.45,  0.35    # Right
], dtype=np.float64)

HAND_LOWER_12 = np.zeros(12, dtype=np.float64)
HAND_UPPER_12 = np.ones(12, dtype=np.float64)


class SafetyLayer:
    """Rigorous real-robot safety enforcement layer."""

    def __init__(
        self,
        backend: RobotInterface,
        max_velocity_rad_s: float = 1.0,
        max_accel_rad_s2: float = 3.0,
        watchdog_timeout_s: float = 0.300,
        deviation_thresh_rad: float = 0.05,
        deviation_timeout_s: float = 0.200,
    ):
        self.backend = backend
        self.max_velocity = min(float(max_velocity_rad_s), 3.0)  # Hard ceiling: 3.0 rad/s
        self.max_accel = float(max_accel_rad_s2)
        self.watchdog_timeout_s = float(watchdog_timeout_s)
        self.deviation_thresh_rad = float(deviation_thresh_rad)
        self.deviation_timeout_s = float(deviation_timeout_s)

        self.is_connected = False
        self.estop_active = False
        self.hold_active = False

        self._last_cmd_t = None
        self._last_cmd_q14 = None
        self._last_cmd_h12 = None
        self._last_cmd_vel = np.zeros(14, dtype=np.float64)
        self._deviation_start_t = None

        self.stats = {
            "nan_rejected": 0,
            "joint_limit_clamps": 0,
            "velocity_clamps": 0,
            "accel_clamps": 0,
            "watchdog_trips": 0,
            "deviation_trips": 0,
            "first_command_ramps": 0,
        }

    def connect(self) -> None:
        self.backend.connect()
        self.is_connected = True
        self.estop_active = False
        self.hold_active = False
        self._last_cmd_t = time.time()
        q_act, h_act, _ = self.backend.read_state()
        self._last_cmd_q14 = q_act.copy()
        self._last_cmd_h12 = h_act.copy()
        self._last_cmd_vel = np.zeros(14, dtype=np.float64)

    def disconnect(self) -> None:
        self.backend.disconnect()
        self.is_connected = False

    def estop(self) -> None:
        self.estop_active = True
        self.backend.estop()

    def clear_estop(self) -> None:
        self.estop_active = False
        q_act, h_act, _ = self.backend.read_state()
        self._last_cmd_q14 = q_act.copy()
        self._last_cmd_h12 = h_act.copy()
        self._last_cmd_vel = np.zeros(14, dtype=np.float64)
        self._last_cmd_t = time.time()

    def hold(self) -> None:
        self.hold_active = True
        self.backend.hold()

    def clear_hold(self) -> None:
        self.hold_active = False
        q_act, h_act, _ = self.backend.read_state()
        self._last_cmd_q14 = q_act.copy()
        self._last_cmd_h12 = h_act.copy()
        self._last_cmd_t = time.time()

    def read_state(self) -> tuple[np.ndarray, np.ndarray, float]:
        return self.backend.read_state()

    def check_watchdog(self, now: float = None) -> bool:
        """Returns True if watchdog tripped due to stale command stream."""
        t = now if now is not None else time.time()
        if self._last_cmd_t is not None and (t - self._last_cmd_t) > self.watchdog_timeout_s:
            if not self.hold_active and not self.estop_active:
                self.stats["watchdog_trips"] += 1
                self.hold()
            return True
        return False

    def send_arm(self, q14: np.ndarray, dt: float = None, now: float = None) -> bool:
        """Filters and delivers 14-DoF arm commands. Returns False if rejected."""
        t_now = now if now is not None else time.time()

        if self.estop_active:
            self.backend.hold()
            return False

        q = np.asarray(q14, dtype=np.float64)
        if len(q) != 14 or not np.all(np.isfinite(q)):
            self.stats["nan_rejected"] += 1
            self.hold()
            return False

        dt_eff = dt if dt is not None and dt > 0 else (t_now - self._last_cmd_t if self._last_cmd_t else 0.02)
        dt_eff = min(0.1, max(0.001, dt_eff))

        # First command / resume initialization: ramp to avoid jump
        if self._last_cmd_q14 is None:
            q_act, _, _ = self.backend.read_state()
            self._last_cmd_q14 = q_act.copy()
            self._last_cmd_vel = np.zeros(14, dtype=np.float64)

        target_q = q.copy()

        # 1. Joint limit clamp
        clamped_q = np.clip(target_q, ARM_LOWER_14, ARM_UPPER_14)
        if not np.allclose(clamped_q, target_q):
            self.stats["joint_limit_clamps"] += 1
            target_q = clamped_q

        # 2. First-command jump ramp
        step_disp = target_q - self._last_cmd_q14
        max_first_step = 0.01  # <= 0.01 rad (approx 1 mm wrist)
        if np.max(np.abs(step_disp)) > max_first_step and (self.stats["first_command_ramps"] == 0 or self.hold_active):
            target_q = self._last_cmd_q14 + np.clip(step_disp, -max_first_step, max_first_step)
            self.stats["first_command_ramps"] += 1

        # 3. Velocity cap
        max_vel_step = self.max_velocity * dt_eff
        raw_delta = target_q - self._last_cmd_q14
        vel_clamped_delta = np.clip(raw_delta, -max_vel_step, max_vel_step)
        if not np.allclose(raw_delta, vel_clamped_delta):
            self.stats["velocity_clamps"] += 1
            target_q = self._last_cmd_q14 + vel_clamped_delta

        # 4. Acceleration cap
        inst_vel = (target_q - self._last_cmd_q14) / dt_eff
        max_acc_step = self.max_accel * dt_eff
        acc_clamped_vel = self._last_cmd_vel + np.clip(inst_vel - self._last_cmd_vel, -max_acc_step, max_acc_step)
        if not np.allclose(inst_vel, acc_clamped_vel):
            self.stats["accel_clamps"] += 1
            target_q = self._last_cmd_q14 + acc_clamped_vel * dt_eff

        # 5. Deviation gate check (sim vs real / read-back)
        q_act, _, _ = self.backend.read_state()
        err = np.abs(target_q - q_act)
        if np.any(err > self.deviation_thresh_rad):
            if self._deviation_start_t is None:
                self._deviation_start_t = t_now
            elif (t_now - self._deviation_start_t) > self.deviation_timeout_s:
                self.stats["deviation_trips"] += 1
                self.hold()
                return False
        else:
            self._deviation_start_t = None

        self._last_cmd_vel = (target_q - self._last_cmd_q14) / dt_eff
        self._last_cmd_q14 = target_q.copy()
        self._last_cmd_t = t_now
        self.hold_active = False

        self.backend.send_arm(target_q)
        return True

    def send_hand(self, h12: np.ndarray) -> bool:
        """Filters and delivers 12-DoF hand normalized commands."""
        if self.estop_active:
            return False

        h = np.asarray(h12, dtype=np.float64)
        if len(h) != 12 or not np.all(np.isfinite(h)):
            self.stats["nan_rejected"] += 1
            return False

        clamped_h = np.clip(h, HAND_LOWER_12, HAND_UPPER_12)
        self._last_cmd_h12 = clamped_h.copy()
        self.backend.send_hand(clamped_h)
        return True

