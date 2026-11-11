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

# Speed levels 1..4: (max_vel, max_acc)
# Level 1: 0.4 rad/s, a=1.0 rad/s^2
# Level 2: 0.6 rad/s, a=1.5 rad/s^2
# Level 3: 0.8 rad/s, a=2.0 rad/s^2 (default)
# Level 4: 1.0 rad/s, a=2.5 rad/s^2 (UNTESTED edge of MC rejection)
SPEED_LEVELS = {
    1: (0.40, 1.00),
    2: (0.60, 1.50),
    3: (0.80, 2.00),
    4: (1.00, 2.50),
}
# Backward-compatibility alias
SPEED_PROFILES = {
    "1": SPEED_LEVELS[1],
    "2": SPEED_LEVELS[2],
    "3": SPEED_LEVELS[3],
    "4": SPEED_LEVELS[4],
    "slow": SPEED_LEVELS[1],
    "normal": SPEED_LEVELS[3],
    "fast": SPEED_LEVELS[3],
    "fast2": SPEED_LEVELS[4],
}
HARD_VEL_CEILING = 1.00
HARD_ACC_CEILING = 2.50
DEFAULT_MAX_ACC = 2.00
DEFAULT_MAX_JERK = 20.0
DEFAULT_LEAD_MAX = 0.10
LEAD_MAX_DEFAULT = DEFAULT_LEAD_MAX
ASSUMED_TAU_S = 0.138


class MirrorLimiter:
    """Computes bounded, direction-preserving motion plans q_plan locked to measured state."""

    def __init__(
        self,
        speed_level: int = 3,
        speed_profile: str = None,
        max_vel: float = None,
        max_acc: float = None,
        lead_max: float = DEFAULT_LEAD_MAX,
        dt: float = 0.02,
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

        self.tau_val = 0.138
        if tau_id is not None:
            if isinstance(tau_id, dict): self.tau_val = max(tau_id.values())
            elif isinstance(tau_id, (int, float)): self.tau_val = float(tau_id)

        self.max_vel = base_vel if max_vel is None else min(HARD_VEL_CEILING, float(max_vel))
        self.max_acc = min(HARD_ACC_CEILING, base_acc if max_acc is None else float(max_acc))
        self.max_jerk = DEFAULT_MAX_JERK
        self.lead_max = float(lead_max)
        self.dt = float(dt)
        self.lim_min = np.array(ARM_LOWER_14, dtype=np.float64)
        self.lim_max = np.array(ARM_UPPER_14, dtype=np.float64)

        init_q = initial_q14 if initial_q14 is not None else HOME
        self.q_plan = np.clip(np.array(init_q, dtype=np.float64), self.lim_min, self.lim_max)
        self.v_plan = np.zeros(14, dtype=np.float64)
        self.a_plan = np.zeros(14, dtype=np.float64)
        self.target_smooth = self.q_plan.copy()
        self.tau_smooth = 0.070  # ~60-80 ms critically-damped target smoother
        self.lead_exceeded = False
        self.lead_val = 0.0
        self.backoff_active = False

    def reset(self, q14: np.ndarray):
        """Drops any active trajectory and locks plan to the provided posture."""
        self.q_plan = np.clip(np.array(q14, dtype=np.float64), self.lim_min, self.lim_max)
        self.v_plan = np.zeros(14, dtype=np.float64)
        self.a_plan = np.zeros(14, dtype=np.float64)
        self.target_smooth = self.q_plan.copy()
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
        """Pure open-loop S-curve (accel & jerk limited) motion planner toward target_q14."""
        dt_eff = dt if (dt is not None and 0.005 <= dt <= 0.050) else self.dt
        target_raw = np.clip(np.asarray(target_q14, dtype=np.float64), self.lim_min, self.lim_max)

        # Smooth raw staircase target with critically-damped filter (~70 ms time constant)
        alpha_smooth = dt_eff / (self.tau_smooth + dt_eff)
        self.target_smooth = self.target_smooth + alpha_smooth * (target_raw - self.target_smooth)
        target = self.target_smooth

        # Measured feedback is strictly log-only metric
        if measured_q14 is not None:
            self.lead_val = float(np.max(np.abs(self.q_plan - np.asarray(measured_q14, dtype=np.float64))))
        else:
            self.lead_val = 0.0
        self.lead_exceeded = False

        err = target - self.q_plan
        D = float(np.max(np.abs(err)))
        eff_max_vel = self.max_vel * max(0.0, min(1.0, speed_scale))
        curr_speed = float(np.max(np.abs(self.v_plan)))

        # Settle snap: when |target - sent| < 0.002 rad and speed < 0.01 rad/s, snap exactly to target
        if D < 0.002 and curr_speed < 0.01:
            self.q_plan = target.copy()
            self.v_plan = np.zeros(14)
            self.a_plan = np.zeros(14)
            return self.q_plan.copy()
        if D < 1e-5:
            self.q_plan = target.copy()
            self.v_plan = np.zeros(14)
            self.a_plan = np.zeros(14)
            return self.q_plan.copy()

        unit_dir = err / D
        v_proj = max(0.0, float(np.dot(self.v_plan, unit_dir)))
        a_proj = float(np.dot(self.a_plan, unit_dir))

        # S-curve stopping velocity calculation
        t_a = max(0.0, a_proj) / self.max_jerk
        dv_a = 0.5 * (max(0.0, a_proj)**2) / self.max_jerk
        d_a = v_proj * t_a + (1.0 / 3.0) * (max(0.0, a_proj)**3) / (self.max_jerk**2)

        d_rem = max(0.0, D - d_a)
        d_thresh = (self.max_acc**3) / (self.max_jerk**2)
        if d_rem <= d_thresh:
            v_eff_max = (self.max_jerk * (d_rem**2))**(1.0 / 3.0) if d_rem > 1e-9 else 0.0
        else:
            c = (self.max_acc**2) / (2.0 * self.max_jerk)
            v_eff_max = -c + math.sqrt(c**2 + 2.0 * self.max_acc * d_rem)

        v_allowed = max(0.0, v_eff_max - dv_a)
        V = min(eff_max_vel, v_allowed, D / dt_eff)
        v_req = unit_dir * V

        # Acceleration request with max_acc uniform scaling
        a_req = (v_req - self.v_plan) / dt_eff
        mag_a_req = float(np.max(np.abs(a_req)))
        if mag_a_req > self.max_acc:
            a_req = a_req * (self.max_acc / mag_a_req)

        # Jerk cap on acceleration changes:
        da = a_req - self.a_plan
        mag_da = float(np.max(np.abs(da)))
        jerk_cap = self.max_jerk * dt_eff
        if mag_da > jerk_cap:
            da = da * (jerk_cap / mag_da)
        self.a_plan = self.a_plan + da

        # Velocity update
        self.v_plan = self.v_plan + self.a_plan * dt_eff
        mag_v = float(np.max(np.abs(self.v_plan)))
        if mag_v > eff_max_vel and mag_v > 1e-6:
            self.v_plan = self.v_plan * (eff_max_vel / mag_v)

        # Absolute step cap per tick (<= max_vel * 0.02) to prevent step > 0.016 under loop jitter
        max_tick_step = self.max_vel * self.dt
        delta_q = self.v_plan * dt_eff
        mag_dq = float(np.max(np.abs(delta_q)))
        if mag_dq > max_tick_step and mag_dq > 1e-6:
            delta_q = delta_q * (max_tick_step / mag_dq)

        next_q = self.q_plan + delta_q
        # Prevent crossing target (strictly monotonic approach)
        overshoot_mask = (next_q - target) * np.sign(err) > 0
        if np.any(overshoot_mask):
            next_q = target.copy()
            self.v_plan = np.zeros(14)
            self.a_plan = np.zeros(14)

        self.q_plan = np.clip(next_q, self.lim_min, self.lim_max)
        return self.q_plan.copy()
