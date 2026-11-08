"""
teleop/robot/safety_layer.py
SafetyLayer wrapping any RobotInterface: kinematic limits, rate clamping, watchdog, and deviation gates.
"""

import time
import numpy as np
from teleop.robot.interface import RobotInterface
from teleop.robot.aimdk_map import ARM_JOINT_NAMES_AIMDK
from teleop.robot.fault_tracer import FaultTracer

# 14-DoF physical bounds: [L0..L6, R0..R6]
ARM_LOWER_14 = np.array([-2.91, -0.46, -2.91, -2.00, -2.88, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.10, -2.88, -0.30, -0.20], dtype=np.float64)
ARM_UPPER_14 = np.array([2.91, 1.5359, 2.91, -0.10, 2.88, 0.30, 0.20, 2.91, 0.46, 2.91, 2.00, 2.88, 0.30, 0.20], dtype=np.float64)
HAND_LOWER_12 = np.zeros(12, dtype=np.float64)
HAND_UPPER_12 = np.array([2.286, 0.78, 1.7, 1.7, 1.7, 1.7, 2.286, 0.78, 1.7, 1.7, 1.7, 1.7], dtype=np.float64)


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
        self.fault_latched = False
        self.fault_reason = ""
        self.fault_tracer = FaultTracer()
        self.context = {"input_mode": "mouse", "collision_gate": False}

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

    def _resync_state(self) -> None:
        q_act, h_act, _ = self.backend.read_state()
        self._last_cmd_q14, self._last_cmd_h12 = q_act.copy(), h_act.copy()
        self._last_cmd_vel = np.zeros(14, dtype=np.float64)
        self._last_cmd_t = time.time()

    def connect(self) -> None:
        self.backend.connect()
        self.is_connected, self.estop_active, self.hold_active = True, False, False
        self._resync_state()

    def disconnect(self) -> None:
        self.backend.disconnect()
        self.is_connected = False

    def estop(self) -> None:
        self.estop_active = True
        self.backend.estop()

    def clear_estop(self) -> None:
        self.estop_active = False
        self._resync_state()

    def hold(self) -> None:
        self.hold_active = True
        self.backend.hold()

    def clear_hold(self) -> None:
        self.hold_active = False
        self._resync_state()

    def clear_fault(self, confirm: bool = False) -> bool:
        """Requires explicit operator confirmation to clear a latched hardware fault."""
        if not confirm:
            return False
        self.fault_latched = False
        self.fault_reason = ""
        self._deviation_start_t = None
        self.clear_hold()
        if hasattr(self.backend, "clear_fault"):
            self.backend.clear_fault(confirm=True)
        return True

    def read_state(self) -> tuple[np.ndarray, np.ndarray, float]:
        return self.backend.read_state()

    def check_watchdog(self, now: float = None) -> bool:
        """Returns True if watchdog tripped due to stale command stream or stale state."""
        t = now if now is not None else time.time()
        if self._last_cmd_t is not None and (t - self._last_cmd_t) > self.watchdog_timeout_s:
            if not self.hold_active and not self.estop_active:
                self.stats["watchdog_trips"] += 1
                self.hold()
            return True
        if hasattr(self.backend, "is_alive") and not self.backend.is_alive():
            self.fault_latched = True
            self.fault_reason = getattr(self.backend, "fault_reason", "Robot state stale (>200ms)")
            self.hold()
            return True
        return False

    def send_arm(self, q14: np.ndarray, dt: float = None, now: float = None) -> bool:
        """Filters and delivers 14-DoF arm commands. Returns False if rejected."""
        t_now = now if now is not None else time.time()

        if self.estop_active or self.fault_latched:
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
        active_clamps = ["none"] * 14

        # 1. Joint limit clamp & collision hold
        if self.context.get("collision_gate", False):
            target_q = self._last_cmd_q14.copy()
            active_clamps = ["collision"] * 14
        else:
            clamped_q = np.clip(target_q, ARM_LOWER_14, ARM_UPPER_14)
            for i in range(14):
                if not np.isclose(clamped_q[i], target_q[i]): active_clamps[i] = "limit"
            if not np.allclose(clamped_q, target_q):
                self.stats["joint_limit_clamps"] += 1
                target_q = clamped_q

        # 2. First-command jump ramp
        step_disp = target_q - self._last_cmd_q14
        max_first_step = 0.01  # <= 0.01 rad (approx 1 mm wrist)
        if np.max(np.abs(step_disp)) > max_first_step and (self.stats["first_command_ramps"] == 0 or self.hold_active):
            target_q = self._last_cmd_q14 + np.clip(step_disp, -max_first_step, max_first_step)
            self.stats["first_command_ramps"] += 1
            for i in range(14):
                if active_clamps[i] == "none": active_clamps[i] = "ramp"

        # 3. Velocity cap
        max_vel_step = self.max_velocity * dt_eff
        raw_delta = target_q - self._last_cmd_q14
        vel_clamped_delta = np.clip(raw_delta, -max_vel_step, max_vel_step)
        for i in range(14):
            if not np.isclose(raw_delta[i], vel_clamped_delta[i]): active_clamps[i] = "velocity"
        if not np.allclose(raw_delta, vel_clamped_delta):
            self.stats["velocity_clamps"] += 1
            target_q = self._last_cmd_q14 + vel_clamped_delta

        # 4. Acceleration cap
        inst_vel = (target_q - self._last_cmd_q14) / dt_eff
        max_acc_step = self.max_accel * dt_eff
        acc_clamped_vel = self._last_cmd_vel + np.clip(inst_vel - self._last_cmd_vel, -max_acc_step, max_acc_step)
        for i in range(14):
            if not np.isclose(inst_vel[i], acc_clamped_vel[i]): active_clamps[i] = "accel"
        if not np.allclose(inst_vel, acc_clamped_vel):
            self.stats["accel_clamps"] += 1
            target_q = self._last_cmd_q14 + acc_clamped_vel * dt_eff

        # Enforce physical bounds so accel filter never pushes past joint limits
        target_q = np.clip(target_q, ARM_LOWER_14, ARM_UPPER_14)

        # 5. Deviation gate check (sim vs real / read-back)
        is_real = getattr(self.backend, "__class__", type(self.backend)).__name__ in ("Ros2Backend", "RealBackend", "UdpBackend")
        eff_thresh = np.full(14, self.deviation_thresh_rad) if is_real else np.clip(
            self.deviation_thresh_rad + 0.05 * np.abs(self._last_cmd_vel), 0.05, 0.10
        )
        q_act, _, _ = self.backend.read_state()
        cmd_cmp = target_q if is_real else q
        err = np.abs(cmd_cmp - q_act)
        tripped_idx = np.where(err > eff_thresh)[0]
        if len(tripped_idx) > 0:
            if self._deviation_start_t is None:
                self._deviation_start_t = t_now
            elif (t_now - self._deviation_start_t) > self.deviation_timeout_s:
                self.stats["deviation_trips"] += 1
                self.fault_latched = True
                j = int(np.argmax(err))
                self.fault_reason, ev = self.fault_tracer.format_trip(
                    j, float(err[j]), float(eff_thresh[j]), target_q, q_act,
                    ARM_LOWER_14, ARM_UPPER_14, active_clamps[j],
                    self.backend.__class__.__name__, self.context, t_now
                )
                self.fault_tracer.record_trip(ev)
                self.hold()
                return False
        else:
            self._deviation_start_t = None

        lim_pairs = [(float(ARM_LOWER_14[i]), float(ARM_UPPER_14[i])) for i in range(14)]
        self.fault_tracer.sample(
            self.backend.__class__.__name__, q, target_q, target_q, q_act, self._last_cmd_vel,
            active_clamps, lim_pairs, abs(dt_eff - 0.005), self.context.get("input_mode", "mouse"),
            self.context.get("collision_gate", False), int(np.argmax(err)), t_now=t_now
        )

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

