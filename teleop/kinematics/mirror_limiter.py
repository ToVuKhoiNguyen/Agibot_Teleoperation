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
from teleop.robot.aimdk_map import ARM_LOWER_14, ARM_UPPER_14, HOME, start_limits

SPEED_LEVELS = {
    1: (0.40, 1.00),
    2: (0.60, 1.50),
    3: (0.80, 2.00),
    4: (1.00, 2.50),
    # Opt-in fast levels (--speed 5 / 6 on BOTH laptop and Orin receiver). Firmware limits are 3.0 rad/s and
    # 6.28 rad/s^2 (docs/A2_REAL_ROBOT_SPEC.md). Validated in SIL only: SIL does not model MC step rejection.
    5: (1.50, 4.00),
    6: (2.00, 6.28),
}
SPEED_PROFILES = {
    "1": SPEED_LEVELS[1], "2": SPEED_LEVELS[2], "3": SPEED_LEVELS[3], "4": SPEED_LEVELS[4],
    "5": SPEED_LEVELS[5], "6": SPEED_LEVELS[6],
    "slow": SPEED_LEVELS[1], "normal": SPEED_LEVELS[3], "fast": SPEED_LEVELS[3], "fast2": SPEED_LEVELS[4],
}
HARD_VEL_CEILING = 2.00
HARD_ACC_CEILING = 6.28
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
        armor_checker = None,
    ):
        self.armor_checker = armor_checker
        self.armor_blocked = False
        self.armor_pair = ""
        self.armor_min_dist = 999.0
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

        # Pursuit law (per joint): v_des = min(vmax, |e|/tau, v_brake(|e|)).
        #   v_brake = largest speed from which a constant deceleration of BRAKE_FRAC*amax still stops within |e|
        #             (discrete-time form, same as SafetyLayer) -> the plan can always stop in time: no overshoot.
        #   |e|/tau = soft landing for the last few mrad (tau = LIMITER_TAU, default 0.10 s).
        # Peak velocity and acceleration are unchanged (vmax, amax); only the approach is tighter than the
        # former pure first-order law (tau = max(0.15, 1.2*vmax/(2*amax)) = 0.24 s arm / 0.48 s wrist),
        # whose lag was tau at every speed. LIMITER_LAW=first_order restores the former law.
        self.law = os.environ.get("LIMITER_LAW", "brake").strip().lower()
        self.brake_frac = 0.8
        # LIMITER_LAW=direct (main.py --limiter direct): pass-through pursuit for live teleop, in the style of
        # low-latency teleop stacks: v = min(vmax, |e|/DIRECT_TAU). No braking curve and no deceleration limit, so the
        # plan does not trail a moving hand by the braking distance (lag ~DIRECT_TAU instead of v/(1.6*amax)).
        # Still monotone (each step is a fraction dt/tau < 1 of the error -> zero overshoot) and still capped at
        # vmax; only speeding UP is slew-limited (DIRECT_ACC_UP, default = firmware limit 6.28 rad/s^2).
        self.direct_acc_up = float(os.environ.get("DIRECT_ACC_UP", "6.28"))
        # DIRECT_SMOOTH_ACC > 0 (main.py --smooth): the direct law also limits DECELERATION and changes of direction
        # (vector acceleration limit per arm) and brakes ahead of the target on a sqrt curve, so every move is
        # accelerate -> cruise -> decelerate instead of stopping in ~1 tick. 0 = former direct law (instant stops).
        self.direct_smooth_acc = max(0.0, float(os.environ.get("DIRECT_SMOOTH_ACC", "0")))
        # DIRECT_SMOOTH_TV: velocity response time. Acceleration is proportional to the velocity error
        # ((v_des - v) / TV, capped at DIRECT_SMOOTH_ACC) instead of always slewing at the cap: the capped slew
        # was bang-bang (25-41 % of moving time at +-limit, sign flips 2-3 /s on the real robot = light jerk).
        self.direct_smooth_tv = max(0.0, float(os.environ.get("DIRECT_SMOOTH_TV", "0.06")))
        # DIRECT_OUT_FIR: moving average (ticks) on the commanded positions. Turns the remaining acceleration
        # steps into ramps (S-curve); an average of past plan points never overshoots or exceeds the speed cap.
        self.out_fir_n = max(1, int(os.environ.get("DIRECT_OUT_FIR", "1")))
        self._out_fir = []
        # DIRECT_OUT_FIR2: a second moving average (ticks) after the first. One average turns an acceleration step
        # into a ramp; two in cascade also round the corners of that ramp, i.e. braking builds up and fades out
        # gradually (jerk-limited). Top speed is untouched and the peak deceleration of a normal stop (longer than
        # the two windows) is the same; a stop ends (FIR2 - 1) / 2 ticks later. Still an average of past plan
        # points: no overshoot. Asked for 13/11: the hand rocks about the wrist when the arm stops (strongest near
        # HOME) while the joint sensors show a clean stop, so the excitation is the abruptness of the braking.
        self.out_fir2_n = max(1, int(os.environ.get("DIRECT_OUT_FIR2", "1")))
        self._out_fir2 = []
        if self.law == "direct":
            self.tau = np.ones(14, dtype=np.float64) * max(0.02, float(os.environ.get("DIRECT_TAU", "0.03")))
        elif self.law == "first_order":
            self.tau = np.maximum(0.15, 1.2 * self.max_vel / (2.0 * self.max_acc))
        else:
            self.tau = np.ones(14, dtype=np.float64) * float(os.environ.get("LIMITER_TAU", "0.10"))
        self.tau_val = float(np.max(self.tau))
        self.v_cap = self.max_vel
        # Per-tick step ceiling at the 100 Hz wire rate: 25 % above vmax*10ms, never below the audited 0.016.
        self.step_cap = 0.020 if self.speed_level == 4 else max(0.016, 1.25 * self.max_vel * 0.01)

        self.dt = float(dt)
        self.lim_min = np.array(ARM_LOWER_14, dtype=np.float64)
        self.lim_max = np.array(ARM_UPPER_14, dtype=np.float64)

        init_q = initial_q14 if initial_q14 is not None else HOME
        self.q_plan = np.clip(np.array(init_q, dtype=np.float64), self.lim_min, self.lim_max)
        self.v_plan = np.zeros(14, dtype=np.float64)
        self.a_plan = np.zeros(14, dtype=np.float64)
        self.target_history = [self.q_plan.copy()]
        self._out_fir = []
        self._out_fir2 = []
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
        """Drops active motion and locks plan to the provided posture (bounds widened to include it: no start jump)."""
        self.lim_min, self.lim_max = start_limits(q14)
        self.q_plan = np.clip(np.array(q14, dtype=np.float64), self.lim_min, self.lim_max)
        self.v_plan = np.zeros(14, dtype=np.float64)
        self.a_plan = np.zeros(14, dtype=np.float64)
        self.target_history = [self.q_plan.copy()]
        self._out_fir = []
        self._out_fir2 = []
        self.overshoot_count = 0
        self.settled = np.ones(14, dtype=bool)
        self.settled_target = self.q_plan.copy()
        self.lead_val = 0.0
        self.lead_exceeded = False
        self.backoff_active = False
        self.armor_blocked = False
        self.armor_pair = ""
        self.armor_min_dist = 999.0
        if self.armor_checker is not None:
            self.armor_checker.is_blocked = False
            self.armor_checker.blocked_pair = ""

    def set_far(self, arm: int, far: bool):
        """arm 0 = left, 1 = right. far=True: the joint target of this arm is only a waypoint a few cm ahead of the
        hand on the way to a goal that is still further away (ik_dispatcher._xr_carrot), so the plan must cruise
        through it instead of braking for it. Real robot 16/11: with the 6 cm lead the braking curve held the
        fastest joint at ~0.7 rad/s whatever the speed level (median 0.72 at level 4, 0.82 at level 5).
        The flag expires after 0.1 s without a refresh, so a stale value can never keep the brake off."""
        if not hasattr(self, "_far"): self._far = [(False, 0.0), (False, 0.0)]
        import time as _t
        self._far[arm] = (bool(far), _t.monotonic())

    def _far_goal(self, sl: slice) -> bool:
        far = getattr(self, "_far", None)
        if far is None: return False
        import time as _t
        flag, t = far[0 if sl.start == 0 else 1]
        return flag and (_t.monotonic() - t) < 0.1

    def _smooth_arm_velocity(self, sl: slice, e: np.ndarray, vmax: float, dt: float) -> np.ndarray:
        """Direct law with a vector acceleration limit (one arm). Desired speed (in units of the joint with the
        largest error, all joints scaled together so the hand keeps a straight line) is
        min(vmax, |e|/tau, braking curve); the velocity VECTOR then moves toward it by at most a*dt per tick,
        so speeding up, slowing down and turning are all bounded -> no step-like starts/stops."""
        a = self.direct_smooth_acc
        a_dt = a * dt
        v_prev = self.v_plan[sl]
        peak_e = float(np.max(np.abs(e)))
        if peak_e < 1e-9:
            v_des = np.zeros_like(e)
        else:
            # discrete braking curve: largest speed that can still stop within peak_e at deceleration 0.8*a
            ab = (0.6 if self.direct_smooth_tv > 0.0 else 0.8) * a_dt   # earlier braking leaves room for the TV lag
            s_brake = ab * (np.sqrt(0.25 + 2.0 * peak_e / (ab * dt)) - 0.5)
            if self._far_goal(sl): s_brake = vmax      # target is only a waypoint: do not brake for it (see set_far)
            s_des = min(vmax, peak_e / self.tau[sl][0], s_brake, self.step_cap / dt)
            v_des = e * (s_des / peak_e)
        dv = v_des - v_prev
        if self.direct_smooth_tv > dt:
            dv = dv * (dt / self.direct_smooth_tv)
        dv_peak = float(np.max(np.abs(dv)))
        if dv_peak > a_dt:
            dv = dv * (a_dt / dv_peak)
        v = v_prev + dv
        # Hard ceiling per joint: a braking curve at 2.5x the acceleration limit. Normally inactive (the sqrt curve
        # above already brakes at 0.8x); it only bites when the hand reverses and the target rushes back toward
        # the arm, where it brakes firmly but continuously instead of stopping in one tick, and it still lands on
        # a static target without passing it.
        ah = 2.5 * a_dt
        s_hard = ah * (np.sqrt(0.25 + 2.0 * np.abs(e) / (ah * dt)) - 0.5)
        toward = np.sign(v) == np.sign(e)
        v = np.where(toward, np.sign(v) * np.minimum(np.abs(v), s_hard), v)
        # at rest the plan must be exactly constant (no sub-microradian creep from the proportional decay)
        v = np.where((e == 0.0) & (np.abs(v) < 1e-3), 0.0, v)
        return v

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
        prev_q = self.q_plan.copy()

        # Rest deadband with hysteresis: enter at |e|<=0.002, leave when |e|>0.004 or tgt moved >0.004
        for i in range(14):
            e = target[i] - self.q_plan[i]
            if self.settled[i]:
                if abs(e) > 0.004 or abs(target[i] - self.settled_target[i]) > 0.004:
                    self.settled[i] = False
            elif abs(e) <= 0.002:
                self.settled[i] = True
                self.settled_target[i] = target[i]

            if self.law == "direct":
                continue   # handled per arm below (synchronised scaling)

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
                speed = min(eff_max_vel, abs(e) / self.tau[i])
                if self.law != "first_order":
                    a_dt = self.brake_frac * self.max_acc[i] * dt_eff
                    speed = min(speed, a_dt * (np.sqrt(0.25 + 2.0 * abs(e) / (a_dt * dt_eff)) - 0.5))
                v_des = speed if e >= 0.0 else -speed
                dv = v_des - self.v_plan[i]
                max_dv = self.max_acc[i] * dt_eff
                self.v_plan[i] += np.clip(dv, -max_dv, max_dv)
                dx = np.clip(self.v_plan[i] * dt_eff, -self.step_cap, self.step_cap)
                self.v_plan[i] = dx / dt_eff
                self.q_plan[i] += dx

        if self.law == "direct":
            # Per ARM, all joints are scaled by the same factor (like a joint-space straight line to the target):
            # with independent per-joint clamps the fast joints arrive early and the hand sweeps a curved detour
            # (measured 12-15 cm off the commanded line on a fast up/down swing).
            for sl in (slice(0, 7), slice(7, 14)):
                if self.direct_smooth_acc > 0.0:
                    # rest deadband only once the joint has (almost) stopped, so arrival is not cut to 0 in 1 tick
                    rest = self.settled[sl] & (np.abs(self.v_plan[sl]) < 0.02)
                    e = np.where(rest, 0.0, target[sl] - self.q_plan[sl])
                    if float(np.max(np.abs(e))) < 1e-9 and float(np.max(np.abs(self.v_plan[sl]))) < 1e-6:
                        self.v_plan[sl] = 0.0
                        continue
                    v = self._smooth_arm_velocity(sl, e, eff_max_vel, dt_eff)
                    self.v_plan[sl] = v
                    self.q_plan[sl] = self.q_plan[sl] + v * dt_eff
                    continue
                e = np.where(self.settled[sl], 0.0, target[sl] - self.q_plan[sl])
                v_des = e / self.tau[sl]                                   # first-order pursuit, never overshoots
                peak = float(np.max(np.abs(v_des)))
                if peak < 1e-9:
                    self.v_plan[sl] = 0.0
                    continue
                prev_peak = float(np.max(np.abs(self.v_plan[sl])))
                cap = min(eff_max_vel, prev_peak + self.direct_acc_up * dt_eff, self.step_cap / dt_eff)
                v = v_des * min(1.0, cap / peak)                           # uniform scale: direction preserved
                self.v_plan[sl] = v
                self.q_plan[sl] = self.q_plan[sl] + v * dt_eff

        q_cand = np.clip(self.q_plan, self.lim_min, self.lim_max)

        # Armor self-collision swept check and soft block
        if self.armor_checker is not None:
            q_safe, step_allowed, is_blocked, min_dist, violator = self.armor_checker.filter_step(prev_q, q_cand, goal_q14=target)
            if not step_allowed:
                self.q_plan = prev_q.copy()
                self.v_plan.fill(0.0)
                self.armor_blocked = True
                self.armor_pair = violator
                self.armor_min_dist = float(min_dist)
            else:
                self.q_plan = q_safe
                self.v_plan = (q_safe - prev_q) / dt_eff
                self.armor_blocked = bool(is_blocked)
                self.armor_pair = violator if is_blocked else ""
                self.armor_min_dist = float(min_dist)
        else:
            self.q_plan = q_cand
            self.armor_blocked = False
            self.armor_pair = ""
            self.armor_min_dist = 999.0

        # Log-only overshoot monitor
        self.target_history.append(target.copy())
        if len(self.target_history) > 500:
            self.target_history.pop(0)
        t_min = np.min(self.target_history, axis=0)
        t_max = np.max(self.target_history, axis=0)
        overshot = (self.q_plan < t_min - 0.003) | (self.q_plan > t_max + 0.003)
        if np.any(overshot):
            self.overshoot_count += int(np.sum(overshot))

        if self.law == "direct" and self.direct_smooth_acc > 0.0 and self.out_fir_n > 1:
            self._out_fir.append(self.q_plan.copy())
            if len(self._out_fir) > self.out_fir_n: self._out_fir = self._out_fir[-self.out_fir_n:]
            out = np.mean(self._out_fir, axis=0)
            if self.out_fir2_n > 1:
                self._out_fir2.append(out)
                if len(self._out_fir2) > self.out_fir2_n: self._out_fir2 = self._out_fir2[-self.out_fir2_n:]
                out = np.mean(self._out_fir2, axis=0)
            return out
        return self.q_plan.copy()
