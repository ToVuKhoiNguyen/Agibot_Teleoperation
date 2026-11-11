"""
teleop/kinematics/mirror_limiter.py
Monotone, open-loop rate & acceleration limiter for dual-arm teleoperation.
- Per-joint first-order pursuit: v_des = clamp((target - x)/tau, -vmax, +vmax)
- Slew rate bounded by |dv/dt| <= amax: stopping distance < error by construction (zero overshoot)
- Pure open-loop: measured feedback is strictly log-only metric
- Log-only overshoot monitor tracking deviations outside target-history range
"""

import os
import numpy as np
from teleop.robot.aimdk_map import ARM_LOWER_14, ARM_UPPER_14, HOME

SPEED_LEVELS = {
    1: (0.40, 1.00),
    2: (0.60, 1.50),
    3: (0.80, 2.00),
    4: (1.00, 2.50),
}
SPEED_PROFILES = {
    "1": SPEED_LEVELS[1], "2": SPEED_LEVELS[2], "3": SPEED_LEVELS[3], "4": SPEED_LEVELS[4],
    "slow": SPEED_LEVELS[1], "normal": SPEED_LEVELS[3], "fast": SPEED_LEVELS[3], "fast2": SPEED_LEVELS[4],
}
HARD_VEL_CEILING = 1.00
HARD_ACC_CEILING = 2.50
DEFAULT_LEAD_MAX = 0.10


class MirrorLimiter:
    """Monotone, open-loop velocity and acceleration limiter ensuring zero overshoot."""

    def __init__(
        self,
        speed_level: int = 3,
        speed_profile: str = None,
        max_vel: float = None,
        max_acc: float = None,
        lead_max: float = DEFAULT_LEAD_MAX,
        dt: float = 0.01,
        initial_q14: np.ndarray = None,
        accept_higher_speed: bool = False,
        gate_mode: str = "legacy",
        tau_id = None,
    ):
        if speed_profile is not None and str(speed_profile) in SPEED_PROFILES:
            base_vel, base_acc = SPEED_PROFILES[str(speed_profile)]
            self.speed_level = int(speed_profile) if str(speed_profile).isdigit() else 3
        else:
            lvl = int(speed_level) if speed_level in SPEED_LEVELS else 3
            self.speed_level = lvl
            base_vel, base_acc = SPEED_LEVELS[lvl]

        self.max_vel = base_vel if max_vel is None else min(HARD_VEL_CEILING, float(max_vel))
        base_acc_val = min(HARD_ACC_CEILING, base_acc if max_acc is None else float(max_acc))
        self.max_acc = np.ones(14, dtype=np.float64) * base_acc_val
        self.wrist_scale = float(os.environ.get("WRIST_SCALE", "0.5"))
        wrist_indices = [5, 6, 12, 13]
        self.max_acc[wrist_indices] *= self.wrist_scale

        # Monotone law: tau = max(0.15, 1.2*vmax/(2*amax)) per joint
        self.tau = np.maximum(0.15, 1.2 * self.max_vel / (2.0 * self.max_acc))
        self.tau_val = float(np.max(self.tau))
        self.v_cap = self.max_vel
        self.step_cap = 0.020 if self.speed_level == 4 else 0.016

        self.dt = float(dt)
        self.lim_min = np.array(ARM_LOWER_14, dtype=np.float64)
        self.lim_max = np.array(ARM_UPPER_14, dtype=np.float64)

        init_q = initial_q14 if initial_q14 is not None else HOME
        self.q_plan = np.clip(np.array(init_q, dtype=np.float64), self.lim_min, self.lim_max)
        self.v_plan = np.zeros(14, dtype=np.float64)
        self.a_plan = np.zeros(14, dtype=np.float64)
        self.target_history = [self.q_plan.copy()]
        self.overshoot_count = 0
        self.settled = np.ones(14, dtype=bool)
        self.settled_target = self.q_plan.copy()

        self.lead_max = float(lead_max)
        self.lead_val = 0.0
        self.lead_exceeded = False
        self.backoff_active = False

        print(
            f"[MirrorLimiter] Monotone: lvl={self.speed_level}, vmax={self.max_vel:.2f}, "
            f"amax={base_acc_val:.2f}, tau_base={self.tau[0]:.3f}s, tau_wrist={self.tau[5]:.3f}s, "
            f"step_cap={self.step_cap:.4f}"
        )

    def reset(self, q14: np.ndarray):
        """Drops active motion and locks plan to the provided posture."""
        self.q_plan = np.clip(np.array(q14, dtype=np.float64), self.lim_min, self.lim_max)
        self.v_plan = np.zeros(14, dtype=np.float64)
        self.a_plan = np.zeros(14, dtype=np.float64)
        self.target_history = [self.q_plan.copy()]
        self.overshoot_count = 0
        self.settled = np.ones(14, dtype=bool)
        self.settled_target = self.q_plan.copy()
        self.lead_val = 0.0
        self.lead_exceeded = False
        self.backoff_active = False

    def step(
        self,
        target_q14: np.ndarray,
        measured_q14: np.ndarray = None,
        dt: float = None,
        speed_scale: float = 1.0,
    ) -> np.ndarray:
        """Computes next bounded position plan q_plan strictly open-loop."""
        dt_eff = dt if (dt is not None and 0.005 <= dt <= 0.050) else self.dt
        target = np.clip(np.asarray(target_q14, dtype=np.float64), self.lim_min, self.lim_max)

        # Strictly log-only feedback metric
        if measured_q14 is not None:
            self.lead_val = float(np.max(np.abs(self.q_plan - np.asarray(measured_q14, dtype=np.float64))))
        else:
            self.lead_val = 0.0
        self.lead_exceeded = False

        eff_max_vel = self.max_vel * max(0.0, min(1.0, speed_scale))

        # Rest deadband with hysteresis: enter at |e|<=0.002, leave when |e|>0.004 or tgt moved >0.004
        for i in range(14):
            e = target[i] - self.q_plan[i]
            if self.settled[i]:
                if abs(e) > 0.004 or abs(target[i] - self.settled_target[i]) > 0.004:
                    self.settled[i] = False
            elif abs(e) <= 0.002:
                self.settled[i] = True
                self.settled_target[i] = target[i]

            if self.settled[i]:
                if abs(self.v_plan[i]) > 1e-5:
                    dv = -self.v_plan[i]
                    max_dv = self.max_acc[i] * dt_eff
                    self.v_plan[i] += np.clip(dv, -max_dv, max_dv)
                    dx = np.clip(self.v_plan[i] * dt_eff, -self.step_cap, self.step_cap)
                    self.v_plan[i] = dx / dt_eff
                    self.q_plan[i] += dx
                else:
                    self.v_plan[i] = 0.0
            else:
                v_des = np.clip(e / self.tau[i], -eff_max_vel, eff_max_vel)
                dv = v_des - self.v_plan[i]
                max_dv = self.max_acc[i] * dt_eff
                self.v_plan[i] += np.clip(dv, -max_dv, max_dv)
                dx = np.clip(self.v_plan[i] * dt_eff, -self.step_cap, self.step_cap)
                self.v_plan[i] = dx / dt_eff
                self.q_plan[i] += dx

        self.q_plan = np.clip(self.q_plan, self.lim_min, self.lim_max)

        # Log-only overshoot monitor
        self.target_history.append(target.copy())
        if len(self.target_history) > 500:
            self.target_history.pop(0)
        t_min = np.min(self.target_history, axis=0)
        t_max = np.max(self.target_history, axis=0)
        overshot = (self.q_plan < t_min - 0.003) | (self.q_plan > t_max + 0.003)
        if np.any(overshot):
            self.overshoot_count += int(np.sum(overshot))

        return self.q_plan.copy()
