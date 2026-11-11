"""
teleop/robot/safety_layer.py
SafetyLayer wrapping any RobotInterface: kinematic limits, rate clamping, watchdog, and deviation gates.
"""

import time
from pathlib import Path
import numpy as np
from teleop.robot.interface import RobotInterface
from teleop.robot.aimdk_map import ARM_JOINT_NAMES_AIMDK, ARM_LOWER_14, ARM_UPPER_14, HOME
from teleop.robot.fault_tracer import FaultTracer
from teleop.robot.deviation_checker import DeviationChecker

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
        self.dev_checker = DeviationChecker(deviation_thresh_rad, deviation_timeout_s)
        self._load_tau_config()

        self.is_connected, self.estop_active, self.hold_active, self.fault_latched = False, False, False, False
        self.fault_reason = ""
        self.fault_tracer = FaultTracer()
        self.context = {"input_mode": "mouse", "collision_gate": False}
        self._last_cmd_t, self._last_cmd_q14, self._last_cmd_h12 = None, None, None
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

    @property
    def gate_mode(self): return self.dev_checker.gate_mode
    @gate_mode.setter
    def gate_mode(self, val): self.dev_checker.gate_mode = val
    @property
    def q_model(self): return self.dev_checker.q_model
    @q_model.setter
    def q_model(self, val): self.dev_checker.q_model = val
    @property
    def tau_id(self): return self.dev_checker.tau_id
    @tau_id.setter
    def tau_id(self, val): self.dev_checker.tau_id = val

    def _load_tau_config(self) -> None:
        cfg_p = Path(__file__).resolve().parent.parent.parent / "config" / "tau_id.json"
        if cfg_p.exists():
            try:
                import json
                with open(cfg_p, "r", encoding="utf-8") as f:
                    d = json.load(f)
                    self.tau_id = d.get("tau_arm") or d.get("tau_max")
            except Exception: pass

    def _resync_state(self) -> None:
        q_act, h_act, _ = self.backend.read_state()
        self._last_cmd_q14, self._last_cmd_h12 = q_act.copy(), h_act.copy()
        self._last_cmd_vel = np.zeros(14, dtype=np.float64)
        self.dev_checker.reset_model(q_act)
        self._last_cmd_t = time.time()

    def connect(self) -> None:
        self.backend.connect(); self.is_connected, self.estop_active, self.hold_active = True, False, False; self._resync_state()

    def disconnect(self) -> None:
        self.backend.disconnect(); self.is_connected = False

    def estop(self) -> None:
        self.estop_active = True; self.backend.estop()

    def clear_estop(self) -> None:
        self.estop_active = False; self._resync_state()

    def hold(self) -> None:
        self.hold_active = True; self.backend.hold()

    def clear_hold(self) -> None:
        self.hold_active = False; self._resync_state()

    def clear_fault(self, confirm: bool = False) -> bool:
        if not confirm: return False
        self.fault_latched, self.fault_reason, self._deviation_start_t = False, "", None
        self.clear_hold()
        if hasattr(self.backend, "clear_fault"): self.backend.clear_fault(confirm=True)
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

        # 1. Joint limit clamp (collision never blocks)
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
        q_act, _, meta = self.backend.read_state()
        age_s = meta.get("age_s", 0.0) if isinstance(meta, dict) else (float(meta) if isinstance(meta, (int, float)) else 0.0)
        
        trip, j, err_val, thresh_val, chk_name, mod_q, res_v = self.dev_checker.check(
            self._last_cmd_q14, q if not is_real else target_q, q_act, is_real, dt_eff, t_now, self._last_cmd_vel
        )
        if trip:
            self.stats["deviation_trips"] += 1
            self.fault_latched = True
            self.fault_reason, ev = self.fault_tracer.format_trip(
                j, err_val, thresh_val, target_q, q_act, ARM_LOWER_14, ARM_UPPER_14,
                active_clamps[j], self.backend.__class__.__name__, self.context, t_now,
                sent_q=self._last_cmd_q14, raw_q=q, age_s=age_s, check_name=chk_name,
                model_q=mod_q, res_v=res_v
            )
            self.fault_tracer.record_trip(ev); self.hold(); return False

        lim_pairs = [(float(ARM_LOWER_14[i]), float(ARM_UPPER_14[i])) for i in range(14)]
        worst_j = int(np.argmax(np.abs(q_act - target_q))) if q_act is not None else 0
        self.fault_tracer.sample(
            self.backend.__class__.__name__, q, target_q, target_q, q_act, self._last_cmd_vel,
            active_clamps, lim_pairs, abs(dt_eff - 0.005), self.context.get("input_mode", "mouse"),
            self.context.get("collision_gate", False), worst_j, t_now=t_now
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

