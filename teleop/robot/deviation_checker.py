"""
teleop/robot/deviation_checker.py
Model-based and legacy deviation gates for SafetyLayer.
"""
import numpy as np
from teleop.robot.aimdk_map import ARM_LOWER_14, ARM_UPPER_14


class DeviationChecker:
    def __init__(self, deviation_thresh_rad: float = 0.05, timeout_s: float = 0.200):
        self.thresh = float(deviation_thresh_rad)
        self.timeout_s = float(timeout_s)
        self.runaway_thresh = 0.30
        self.gate_mode = "legacy"
        self.q_model = None
        self.tau_id = None
        self.start_t = None
        self.warn_active = False
        self.static_offset = np.zeros(14, dtype=np.float64)

    def reset_model(self, q: np.ndarray) -> None:
        self.q_model = q.copy() if q is not None else None
        self.start_t = None
        self.warn_active = False

    def check(self, last_sent: np.ndarray, target_q: np.ndarray, q_act: np.ndarray,
              is_real: bool, dt: float, t_now: float, last_cmd_vel: np.ndarray):
        if q_act is None:
            return False, 0, 0.0, 0.0, "ok", None, None

        cmd_cmp = last_sent if is_real else target_q
        gap = np.abs(cmd_cmp - q_act)
        worst_j = int(np.argmax(gap))
        max_gap = float(gap[worst_j])

        # Deviation and runaway are log-only (NEVER fault, never block)
        self.warn_active = False
        return False, worst_j, max_gap, 0.0, "log_only", None, None
