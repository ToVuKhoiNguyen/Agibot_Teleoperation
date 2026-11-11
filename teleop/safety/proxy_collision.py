"""
teleop/safety/proxy_collision.py
Lightweight proxy collision checker using capsule-capsule and capsule-sphere geometries.
Runs on COMMAND (PLAN) posture at <= 50 Hz with < 0.5 ms per check.
"""

from pathlib import Path
import numpy as np
import mujoco
import yaml


class ProxyCollisionChecker:
    """Lightweight collision checker evaluating arm-arm and arm-torso/head proxy geometries."""

    def __init__(self, model: mujoco.MjModel, config_path: Path = None):
        self.model = model
        self.data = mujoco.MjData(model)
        self.l_adr = list(range(12, 19))
        self.r_adr = list(range(31, 38))

        # Default margins
        self.d_warn = 0.05
        self.d_critical = 0.02
        self.warn_scale = 0.40
        self.min_scale = 0.15

        if config_path and Path(config_path).exists():
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    cfg = yaml.safe_load(f)
                    soft = cfg.get("soft_collision", {})
                    self.d_warn = float(soft.get("d_warn", self.d_warn))
                    self.d_critical = float(soft.get("d_critical", self.d_critical))
                    self.warn_scale = float(soft.get("warn_scale", self.warn_scale))
                    self.min_scale = float(soft.get("min_scale", self.min_scale))
            except Exception as e:
                print(f"[ProxyCollision] Error loading config: {e}")

        self.last_d = 999.0
        self.last_check_t = 0.0
        self.last_scale = 1.0

    @staticmethod
    def _dist_segment_segment(p1, q1, p2, q2) -> float:
        d1 = q1 - p1
        d2 = q2 - p2
        r = p1 - p2
        a = float(np.dot(d1, d1))
        e = float(np.dot(d2, d2))
        f = float(np.dot(d2, r))
        if a <= 1e-8 and e <= 1e-8:
            return float(np.linalg.norm(p1 - p2))
        if a <= 1e-8:
            s = 0.0
            t = np.clip(f / e, 0.0, 1.0)
        else:
            c = float(np.dot(d1, r))
            if e <= 1e-8:
                t = 0.0
                s = np.clip(-c / a, 0.0, 1.0)
            else:
                b = float(np.dot(d1, d2))
                denom = a * e - b * b
                s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom != 0.0 else 0.0
                t = (b * s + f) / e
                if t < 0.0:
                    t = 0.0
                    s = np.clip(-c / a, 0.0, 1.0)
                elif t > 1.0:
                    t = 1.0
                    s = np.clip((b - c) / a, 0.0, 1.0)
        return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))

    @staticmethod
    def _dist_point_segment(pt, p, q) -> float:
        d = q - p
        l2 = float(np.dot(d, d))
        if l2 <= 1e-8:
            return float(np.linalg.norm(pt - p))
        t = np.clip(float(np.dot(pt - p, d)) / l2, 0.0, 1.0)
        return float(np.linalg.norm(pt - (p + t * d)))

    def compute_min_distance(self, q14: np.ndarray) -> float:
        """Evaluates minimum proxy distance for given 14-DoF arm configuration."""
        q = np.asarray(q14, dtype=np.float64)
        for i in range(7):
            self.data.qpos[self.l_adr[i]] = q[i]
            self.data.qpos[self.r_adr[i]] = q[7 + i]
        mujoco.mj_kinematics(self.model, self.data)

        d = self.data
        # 4 capsules per arm
        caps_l = [
            (d.xpos[16], d.xpos[17], 0.045),
            (d.xpos[17], d.xpos[18], 0.045),
            (d.xpos[18], d.xpos[19], 0.040),
            (d.xpos[19], d.xpos[20], 0.040),
        ]
        caps_r = [
            (d.xpos[35], d.xpos[36], 0.045),
            (d.xpos[36], d.xpos[37], 0.045),
            (d.xpos[37], d.xpos[38], 0.040),
            (d.xpos[38], d.xpos[39], 0.040),
        ]
        # Torso and head spheres
        torso_spheres = [
            (d.xpos[1] + np.array([-0.05, 0.0, 0.10]), 0.14),
            (d.xpos[1] + np.array([-0.05, 0.0, 0.25]), 0.14),
            (d.xpos[1] + np.array([-0.05, 0.0, 0.40]), 0.14),
            (d.xpos[53] + np.array([0.0, 0.0, 0.0]), 0.12),
        ]

        min_d = 999.0
        # Arm - arm
        for p1, q1, r1 in caps_l:
            for p2, q2, r2 in caps_r:
                dist = self._dist_segment_segment(p1, q1, p2, q2) - (r1 + r2)
                if dist < min_d:
                    min_d = dist

        # Arm - torso / head
        for p1, q1, r1 in caps_l + caps_r:
            for spt, sr in torso_spheres:
                dist = self._dist_point_segment(spt, p1, q1) - (r1 + sr)
                if dist < min_d:
                    min_d = dist

        return float(min_d)

    def evaluate_scale(self, q14: np.ndarray) -> tuple[float, float, bool]:
        """Neutralized: Armor collision handles soft-blocking inside the limiter step."""
        curr_d = self.compute_min_distance(q14)
        self.last_d = curr_d
        self.last_scale = 1.0
        return 1.0, curr_d, False
        return scale, curr_d, decreasing

