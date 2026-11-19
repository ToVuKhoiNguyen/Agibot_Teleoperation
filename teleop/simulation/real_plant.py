"""
teleop/simulation/real_plant.py
Simulated physical robot plant identified from real hardware telemetry.

Reproduces transport delay, 1st/2nd-order joint dynamics, velocity/accel saturation,
static offsets, deadband and optional noise. Used by:
  * main.py --sim with SIM_PLANT=1 (end-to-end plant: laptop SENT -> laptop ACT,
    config/sim_plant.json), and
  * sil/fake_mc_node.py (MC-only plant: Orin published cmd -> Orin measured state,
    config/mc_plant.json).

The plant always integrates at its native identification rate (metadata.dt_s,
default 0.010 s). step() may be called at any rate: it accumulates wall dt and
runs whole native ticks, so delays expressed in ticks keep their real duration.

Dependencies: numpy + stdlib only (must run on ROS Humble system Python 3.10).
"""

import os
import json
import collections
import numpy as np

NATIVE_DT_DEFAULT = 0.010
MAX_PLAUSIBLE_NOISE_RAD = 0.005  # larger fitted noise means the fit window contained motion


def joint_step(cfg: dict, y: float, v: float, u_del: float, dt: float, apply_limits: bool = True):
    """One native tick of the per-joint discrete model. Shared with tools/identify_plant.py.

    1st_order: y += alpha * (u_del - y), alpha = dt / (tau + dt)   (backward-Euler lag)
    2nd_order: semi-implicit Euler of y'' = wn^2 (u_del - y) - 2 zeta wn y'
    Returns (y_new, v_new). Offset/deadband/noise are output-side and applied by the caller.
    """
    mtype = cfg.get("model_type", "1st_order")
    if mtype == "IDEAL":
        return float(u_del), (float(u_del) - y) / dt

    if mtype == "2nd_order":
        wn = 2.0 * np.pi * float(cfg.get("fn_hz", 5.7))
        zeta = float(cfg.get("zeta", 0.6))
        acc = wn * wn * (u_del - y) - 2.0 * zeta * wn * v
        if apply_limits:
            a_max = float(cfg.get("max_acc_rad_s2", 200.0))
            acc = min(max(acc, -a_max), a_max)
        v_new = v + acc * dt
    else:
        tau = max(1e-4, float(cfg.get("tau_s", 0.11)))
        alpha = dt / (tau + dt)
        v_new = alpha * (u_del - y) / dt
        if apply_limits:
            a_max = float(cfg.get("max_acc_rad_s2", 200.0))
            dv_max = a_max * dt
            v_new = min(max(v_new, v - dv_max), v + dv_max)

    if apply_limits:
        v_max = float(cfg.get("max_vel_rad_s", 3.0))
        v_new = min(max(v_new, -v_max), v_max)
    return y + v_new * dt, v_new


class _VectorPlant:
    """N independent delayed joint models sharing one native clock."""

    def __init__(self, n: int, cfgs: list, native_dt: float, noise_enabled: bool, rng):
        self.n, self.cfgs, self.dt = n, cfgs, native_dt
        self.noise_enabled, self.rng = noise_enabled, rng
        self.delays = [max(0, int(c.get("delay_ticks", 0))) for c in cfgs]
        self.hist = collections.deque(maxlen=max(self.delays + [0]) + 1)
        self.y = np.zeros(n)
        self.v = np.zeros(n)
        self.out = np.zeros(n)

    def reset(self, q0: np.ndarray):
        q0 = np.asarray(q0, dtype=np.float64)
        self.y = q0.copy()
        self.v = np.zeros(self.n)
        self.hist.clear()
        for _ in range(self.hist.maxlen):
            self.hist.append(q0.copy())
        self.out = self._output()

    def tick(self, u: np.ndarray):
        self.hist.append(np.asarray(u, dtype=np.float64).copy())
        for i in range(self.n):
            u_del = self.hist[-1 - self.delays[i]][i]
            self.y[i], self.v[i] = joint_step(self.cfgs[i], self.y[i], self.v[i], u_del, self.dt)
        self.out = self._output()

    def _output(self, y: np.ndarray = None) -> np.ndarray:
        y = self.y if y is None else y
        out = np.empty(self.n)
        for i, c in enumerate(self.cfgs):
            val = y[i] + float(c.get("static_offset_rad", 0.0))
            db = float(c.get("deadband_rad", 0.0))
            if db > 1e-6:
                val = round(val / db) * db
            if self.noise_enabled:
                n_std = float(c.get("noise_std_rad", 0.0))
                if 1e-6 < n_std <= MAX_PLAUSIBLE_NOISE_RAD:
                    val += self.rng.normal(0.0, n_std)
            out[i] = val
        return out


class RealPlantSimulator:
    """Simulates the hardware transfer function SENT -> ACT for 14 arm joints (+ optional 12 hand DoF)."""

    def __init__(self, config_path: str = "config/sim_plant.json", enabled: bool = True):
        self.config_path = config_path
        self.enabled = bool(enabled)
        self.noise_enabled = os.environ.get("SIM_NOISE", "0").strip().lower() in ("1", "true", "yes")
        self.rng = np.random.RandomState(42)

        self.is_ideal = True
        self.plant_data = None
        self.native_dt = NATIVE_DT_DEFAULT
        self.hud_status = "IDEAL SIM, not predictive"
        self.summary_str = "Ideal Sim (Pass-through)"
        self.warnings = []

        self._t_acc = 0.0
        self.prev_u = None
        self._arm = None
        self._hand = None
        self._hand_t_acc = 0.0
        self._last_ideal = np.zeros(14)

        if self.enabled:
            self._load_config()

    # ------------------------------------------------------------------ config
    def _load_config(self):
        if not os.path.exists(self.config_path):
            self.summary_str = f"config not found at {self.config_path} (IDEAL SIM)"
            return
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                self.plant_data = json.load(f)
        except Exception as e:
            self.summary_str = f"Error loading {self.config_path}: {e}"
            return

        meta = self.plant_data.get("metadata", {})
        self.native_dt = float(meta.get("dt_s", NATIVE_DT_DEFAULT))
        joints = self.plant_data.get("joints", {})
        cfgs = []
        for i in range(14):
            c = dict(joints.get(str(i), {"model_type": "IDEAL"}))
            if float(c.get("noise_std_rad", 0.0)) > MAX_PLAUSIBLE_NOISE_RAD:
                self.warnings.append(f"J{i} noise_std {c['noise_std_rad']:.4f} rad implausible -> ignored")
            cfgs.append(c)
        self._arm = _VectorPlant(14, cfgs, self.native_dt, self.noise_enabled, self.rng)

        hcfg = self.plant_data.get("hand", {})
        if hcfg.get("model_type", "IDEAL") != "IDEAL":
            self._hand = _VectorPlant(12, [hcfg] * 12, self.native_dt, False, self.rng)

        self.is_ideal = False
        val_rms = float(meta.get("overall_val_rms_rad") or 0.0)
        validated = bool(meta.get("validated", False))
        tag = f"validated RMS {val_rms*1000:.2f} mrad" if validated else "UNVALIDATED"
        self.hud_status = f"SIM PLANT ({os.path.basename(self.config_path)}): {tag}"
        conf = collections.Counter("IDEAL" if c.get("model_type") == "IDEAL" else c.get("confidence", "?") for c in cfgs)
        conf_s = ", ".join(f"{n} {k}" for k, n in sorted(conf.items()))
        self.summary_str = (f"Sim Plant loaded ({conf_s}, dt={self.native_dt*1000:.0f}ms, {tag})"
                            + (f" | WARN: {'; '.join(self.warnings)}" if self.warnings else ""))

    # ------------------------------------------------------------------ arm
    def reset(self, initial_q14: np.ndarray):
        """Initializes plant state to the specified posture."""
        init = np.asarray(initial_q14, dtype=np.float64)
        self._t_acc = 0.0
        self.prev_u = init.copy()
        self._last_ideal = init.copy()
        if self._arm is not None:
            self._arm.reset(init)

    def step(self, sent_q14: np.ndarray, dt: float = NATIVE_DT_DEFAULT) -> np.ndarray:
        """Advances the plant by wall time dt with setpoint sent_q14 (zero-order hold). Returns ACT."""
        u = np.asarray(sent_q14, dtype=np.float64)
        if self.is_ideal or self._arm is None:
            self._last_ideal = u.copy()
            return u.copy()

        self._t_acc += float(dt)
        while self._t_acc >= self.native_dt - 1e-9:
            self._t_acc -= self.native_dt
            u_tick = u
            mc_cfg = self.plant_data.get("mc_step_reject", {})
            if mc_cfg.get("enabled", False) and self.prev_u is not None:
                thresh = float(mc_cfg.get("threshold_rad_20ms", 0.020)) * (self.native_dt / 0.020)
                if float(np.max(np.abs(u - self.prev_u))) > thresh:
                    u_tick = self.prev_u
            self.prev_u = u_tick.copy()
            self._arm.tick(u_tick)
        return self._arm.out.copy()

    @property
    def velocity(self) -> np.ndarray:
        return self._arm.v.copy() if self._arm is not None else np.zeros(14)

    @property
    def true_state(self) -> np.ndarray:
        """Plant position BEFORE sensor effects (offset/deadband/noise) - what a physics servo should track."""
        return self._arm.y.copy() if self._arm is not None else self._last_ideal.copy()

    def measure(self, q_true: np.ndarray) -> np.ndarray:
        """Applies the identified sensor effects (static offset, quantization, noise) to a true position."""
        q = np.asarray(q_true, dtype=np.float64)
        return self._arm._output(q) if self._arm is not None else q.copy()

    # ------------------------------------------------------------------ hand
    def reset_hand(self, initial_h12: np.ndarray):
        self._hand_t_acc = 0.0
        if self._hand is not None:
            self._hand.reset(np.asarray(initial_h12, dtype=np.float64))

    def step_hand(self, sent_h12: np.ndarray, dt: float = NATIVE_DT_DEFAULT) -> np.ndarray:
        """Hand plant in raw units (0..2000). IDEAL pass-through unless config 'hand' is identified."""
        h = np.asarray(sent_h12, dtype=np.float64)
        if self._hand is None:
            return h.copy()
        self._hand_t_acc += float(dt)
        while self._hand_t_acc >= self.native_dt - 1e-9:
            self._hand_t_acc -= self.native_dt
            self._hand.tick(h)
        return self._hand.out.copy()
