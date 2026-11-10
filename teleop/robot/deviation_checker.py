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
        self.gate_mode = "legacy"
        self.q_model = None
        self.tau_id = None
        self.start_t = None

    def reset_model(self, q: np.ndarray) -> None:
        self.q_model = q.copy() if q is not None else None
        self.start_t = None

    def check(self, last_sent: np.ndarray, target_q: np.ndarray, q_act: np.ndarray,
              is_real: bool, dt: float, t_now: float, last_cmd_vel: np.ndarray):
        sent_act_gap = np.abs(last_sent - q_act)
        if is_real and np.any(sent_act_gap > 0.15):
            j = int(np.argmax(sent_act_gap))
            return True, j, float(sent_act_gap[j]), 0.15, "absolute_cap_0.15", None, None

        use_lagged = (self.gate_mode == "lagged" and self.tau_id is not None)
        if use_lagged:
            if self.q_model is None: self.q_model = q_act.copy()
            tau_vec = np.full(14, 0.138)
            if isinstance(self.tau_id, dict):
                from teleop.robot.aimdk_map import ARM_JOINT_NAMES_AIMDK
                for i, name in enumerate(ARM_JOINT_NAMES_AIMDK):
                    if name in self.tau_id: tau_vec[i] = float(self.tau_id[name])
            elif isinstance(self.tau_id, (int, float)):
                tau_vec.fill(float(self.tau_id))
            alpha = np.clip(dt / np.maximum(1e-3, tau_vec), 0.0, 1.0)
            self.q_model += alpha * (last_sent - self.q_model)
            err = np.abs(q_act - self.q_model)
            eff_thresh = np.full(14, self.thresh)
            chk_name = "lagged_model_gate"
        else:
            eff_thresh = np.full(14, self.thresh) if is_real else np.clip(
                self.thresh + 0.05 * np.abs(last_cmd_vel), 0.05, 0.10
            )
            cmd_cmp = last_sent if is_real else target_q
            err = np.abs(cmd_cmp - q_act)
            chk_name = "hard_deviation_gate"

        tripped_idx = np.where(err > eff_thresh)[0]
        if len(tripped_idx) > 0:
            if self.start_t is None: self.start_t = t_now
            elif (t_now - self.start_t) > self.timeout_s:
                j = int(np.argmax(err))
                return True, j, float(err[j]), float(eff_thresh[j]), chk_name, (self.q_model if use_lagged else None), (err if use_lagged else None)
        else:
            self.start_t = None
        return False, int(np.argmax(err)), float(np.max(err)), float(eff_thresh[int(np.argmax(err))]), chk_name, None, None
