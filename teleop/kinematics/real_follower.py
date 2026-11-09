"""
teleop/kinematics/real_follower.py
Predictive real-robot follower model matching Orin arm_teleop_rx.py dynamics:
- Imports physical joint limits directly from scripts.arm_teleop_rx (no duplication)
- Supports uniform vector scaling vs independent per-joint clamping
- 1st-order actuator lag (tau identified from newest real logs, default 0.15s)
- N-step lookahead forward prediction for predictive shadow collision gating
"""

import numpy as np
from scripts.arm_teleop_rx import LIMS_MIN, LIMS_MAX

IDENTIFIED_TAU_S = 0.150  # Identified from logs/real_20261106_165245.jsonl (0.15s)


class RealLikeFollower:
    """Simulates real-robot actuator pursuit dynamics (rate limiter + first-order response lag)."""

    def __init__(
        self,
        max_step_rad: float = 0.01,
        scaling_mode: str = "uniform",
        tau_s: float = IDENTIFIED_TAU_S,
        dt: float = 0.02,
        initial_q14: np.ndarray = None,
    ):
        self.max_step_rad = float(max_step_rad)
        self.scaling_mode = scaling_mode.lower()
        self.tau_s = float(tau_s)
        self.dt = float(dt)
        self.lim_min = np.array(LIMS_MIN, dtype=np.float64)
        self.lim_max = np.array(LIMS_MAX, dtype=np.float64)

        if initial_q14 is not None:
            self.curr_lim_q = np.array(initial_q14, dtype=np.float64).copy()
            self.curr_exec_q = np.array(initial_q14, dtype=np.float64).copy()
        else:
            self.curr_lim_q = None
            self.curr_exec_q = None

    def reset(self, initial_q14: np.ndarray):
        self.curr_lim_q = np.array(initial_q14, dtype=np.float64).copy()
        self.curr_exec_q = np.array(initial_q14, dtype=np.float64).copy()

    def step(self, target_q14: np.ndarray, dt: float = None) -> np.ndarray:
        target = np.asarray(target_q14, dtype=np.float64)
        if self.curr_lim_q is None:
            self.reset(target)
            return self.curr_exec_q.copy()

        dt_eff = dt if (dt is not None and dt > 0) else self.dt
        # Scale step ceiling if dt differs from 50Hz (0.02s)
        step_ceiling = self.max_step_rad * (dt_eff / 0.02)

        diff = target - self.curr_lim_q
        if self.scaling_mode == "uniform":
            max_d = float(np.max(np.abs(diff)))
            scale = min(1.0, step_ceiling / max_d) if max_d > 1e-6 else 1.0
            step = diff * scale
        else:
            step = np.clip(diff, -step_ceiling, step_ceiling)

        self.curr_lim_q = np.clip(self.curr_lim_q + step, self.lim_min, self.lim_max)

        # First-order actuator response lag
        if self.tau_s > 1e-4:
            alpha = dt_eff / (self.tau_s + dt_eff)
            self.curr_exec_q += alpha * (self.curr_lim_q - self.curr_exec_q)
        else:
            self.curr_exec_q = self.curr_lim_q.copy()

        return self.curr_exec_q.copy()

    def predict_ahead(self, target_q14: np.ndarray, n_steps: int = 10, dt: float = None) -> list[np.ndarray]:
        """Simulates N future cycles forward from current state towards target."""
        target = np.asarray(target_q14, dtype=np.float64)
        if self.curr_lim_q is None:
            return [target.copy() for _ in range(n_steps)]

        dt_eff = dt if (dt is not None and dt > 0) else self.dt
        step_ceiling = self.max_step_rad * (dt_eff / 0.02)

        pred_lim = self.curr_lim_q.copy()
        pred_exec = self.curr_exec_q.copy()
        future_poses = []

        for _ in range(n_steps):
            diff = target - pred_lim
            if self.scaling_mode == "uniform":
                max_d = float(np.max(np.abs(diff)))
                scale = min(1.0, step_ceiling / max_d) if max_d > 1e-6 else 1.0
                step = diff * scale
            else:
                step = np.clip(diff, -step_ceiling, step_ceiling)

            pred_lim = np.clip(pred_lim + step, self.lim_min, self.lim_max)
            if self.tau_s > 1e-4:
                alpha = dt_eff / (self.tau_s + dt_eff)
                pred_exec += alpha * (pred_lim - pred_exec)
            else:
                pred_exec = pred_lim.copy()

            future_poses.append(pred_exec.copy())

        return future_poses
