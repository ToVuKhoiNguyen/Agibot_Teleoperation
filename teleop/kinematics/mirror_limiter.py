"""
teleop/kinematics/mirror_limiter.py
Mirror Limiter for synchronous 1:1 teleoperation between sim and real robot:
- q_plan moves toward IK target with active speed profile and accel cap.
- Preserves 14-vector direction via strict uniform scaling.
- Discrete parabolic braking ensures exact arrival without overshoot.
- LEAD_MAX check: stops advancing q_plan if lead over measured robot state exceeds cap.
"""

import math
import numpy as np
from teleop.robot.aimdk_map import ARM_LOWER_14, ARM_UPPER_14, HOME

SPEED_PROFILES = {
    "slow": 0.20,
    "normal": 0.30,
    "fast": 0.48,
}
HARD_VEL_CEILING = 0.50
DEFAULT_MAX_ACC = 0.95
DEFAULT_LEAD_MAX = 0.10
LEAD_MAX_DEFAULT = DEFAULT_LEAD_MAX
ASSUMED_TAU_S = 0.10


class MirrorLimiter:
    """Computes bounded, direction-preserving motion plans q_plan locked to measured state."""

    def __init__(
        self,
        speed_profile: str = "normal",
        max_vel: float = None,
        max_acc: float = DEFAULT_MAX_ACC,
        lead_max: float = DEFAULT_LEAD_MAX,
        dt: float = 0.02,
        initial_q14: np.ndarray = None,
    ):
        self.profile = speed_profile if speed_profile in SPEED_PROFILES else "normal"
        base_vel = SPEED_PROFILES[self.profile]
        self.max_vel = min(HARD_VEL_CEILING, max_vel if max_vel is not None else base_vel)
        self.max_acc = float(max_acc)
        self.lead_max = float(lead_max)
        self.dt = float(dt)
        self.lim_min = np.array(ARM_LOWER_14, dtype=np.float64)
        self.lim_max = np.array(ARM_UPPER_14, dtype=np.float64)

        init_q = initial_q14 if initial_q14 is not None else HOME
        self.q_plan = np.clip(np.array(init_q, dtype=np.float64), self.lim_min, self.lim_max)
        self.v_plan = np.zeros(14, dtype=np.float64)
        self.lead_exceeded = False
        self.lead_val = 0.0
        self.backoff_active = False

    def reset(self, q14: np.ndarray):
        """Drops any active trajectory and locks plan to the provided posture."""
        self.q_plan = np.clip(np.array(q14, dtype=np.float64), self.lim_min, self.lim_max)
        self.v_plan = np.zeros(14, dtype=np.float64)
        self.lead_exceeded = False
        self.lead_val = 0.0
        self.backoff_active = False

    def step(
        self,
        target_q14: np.ndarray,
        measured_q14: np.ndarray = None,
        dt: float = None,
        speed_scale: float = 1.0,
    ) -> np.ndarray:
        """Advances q_plan toward target_q14 bounded by lead_max, speed profile and accel cap."""
        dt_eff = dt if (dt is not None and 0.005 <= dt <= 0.050) else self.dt
        target = np.clip(np.asarray(target_q14, dtype=np.float64), self.lim_min, self.lim_max)

        # 1. Check LEAD_MAX against real robot state if available
        if measured_q14 is not None:
            meas = np.asarray(measured_q14, dtype=np.float64)
            self.lead_val = float(np.max(np.abs(self.q_plan - meas)))
            if self.lead_val >= self.lead_max:
                self.lead_exceeded = True
                # Robot lagging: stop advancing q_plan and smoothly decelerate to zero
                max_dv = self.max_acc * dt_eff
                curr_v_mag = float(np.max(np.abs(self.v_plan)))
                if curr_v_mag > 1e-6:
                    self.v_plan *= max(0.0, 1.0 - max_dv / curr_v_mag)
                else:
                    self.v_plan = np.zeros(14)
                self.q_plan += self.v_plan * dt_eff
                self.q_plan = np.clip(self.q_plan, self.lim_min, self.lim_max)
                return self.q_plan.copy()
            else:
                self.lead_exceeded = False
        else:
            self.lead_val = 0.0
            self.lead_exceeded = False

        # 2. Compute error to target
        err = target - self.q_plan
        D = float(np.max(np.abs(err)))
        if D < 1e-4:
            max_dv = self.max_acc * dt_eff
            v_mag = float(np.max(np.abs(self.v_plan)))
            if v_mag > 1e-6:
                self.v_plan *= max(0.0, 1.0 - max_dv / v_mag)
            else:
                self.v_plan = np.zeros(14)
            return self.q_plan.copy()

        # 3. Speed limit with soft back-off scale
        eff_max_vel = self.max_vel * max(0.0, min(1.0, speed_scale))
        self.backoff_active = bool(speed_scale < 0.999)

        # 4. Parabolic braking profile
        v_stop = -0.5 * self.max_acc * dt_eff + math.sqrt(
            max(0.0, (0.5 * self.max_acc * dt_eff)**2 + 2.0 * self.max_acc * D)
        )
        V = min(eff_max_vel, v_stop, D / dt_eff)
        v_req = err * (V / D)

        # 5. Acceleration limit with uniform vector scaling (preserves direction)
        dv = v_req - self.v_plan
        max_dv = float(np.max(np.abs(dv)))
        acc_cap = self.max_acc * dt_eff
        if max_dv > acc_cap:
            dv = dv * (acc_cap / max_dv)
        self.v_plan = self.v_plan + dv

        # 6. Velocity cap with uniform vector scaling
        curr_v = float(np.max(np.abs(self.v_plan)))
        if curr_v > eff_max_vel and curr_v > 1e-6:
            self.v_plan = self.v_plan * (eff_max_vel / curr_v)

        next_q = self.q_plan + self.v_plan * dt_eff
        self.q_plan = np.clip(next_q, self.lim_min, self.lim_max)
        return self.q_plan.copy()
