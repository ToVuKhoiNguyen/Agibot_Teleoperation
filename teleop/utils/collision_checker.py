"""
teleop/utils/collision_checker.py
Comprehensive self-collision detection and avoidance for AGIBOT A2 Ultra.
Provides:
- mj_geomDistance-based multi-pair exact signed distance computation.
- Scratch MjData forward kinematics validation.
- Interpolated trajectory sub-stepping to prevent high-velocity tunneling.
- Line search / step-back projection to keep robot in valid free-space.
- Pairwise classification (torso-arm, torso-hand, head-arm, arm-arm, hand-hand).
"""

import os
import yaml
import numpy as np
import mujoco

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "safety.yaml")


class CollisionChecker:
    def __init__(self, model: mujoco.MjModel, config_path: str = None, enabled: bool = True):
        self.model = model
        self.scratch_data = mujoco.MjData(model)
        self.enabled = enabled
        self.fromto = np.zeros(6, dtype=np.float64)

        # Load config
        self.config_path = config_path or CONFIG_PATH
        self.safety_margin = 0.02
        self.real_robot_margin = 0.03
        self.substeps = 5
        self.dist_max = 0.50
        self.pair_margins = {}

        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r") as f:
                    cfg = yaml.safe_load(f) or {}
                sc_cfg = cfg.get("self_collision", {})
                self.safety_margin = float(sc_cfg.get("safety_margin", 0.02))
                self.substeps = int(sc_cfg.get("interpolation_substeps", 5))
                self.dist_max = float(sc_cfg.get("dist_max", 0.50))
                self.pair_margins = cfg.get("pair_margins", {})
            except Exception as e:
                print(f"[WARN] Failed to load safety.yaml: {e}")

        self._discover_collision_geoms()
        self._build_collision_pairs()
        self.last_min_dist = 999.0
        self.last_min_pair = (None, None, "none")
        self.step_idx = 0

    def _discover_collision_geoms(self):
        """Discovers visual/hull collision geoms for torso, head, arms, and hands."""
        self.torso_geoms = []
        self.head_geoms = []
        self.left_arm_geoms = []
        self.left_hand_geoms = []
        self.right_arm_geoms = []
        self.right_hand_geoms = []

        # Hand body prefixes
        l_hand_prefixes = ("L_thumb", "L_index", "L_middle", "L_ring", "L_pinky")
        r_hand_prefixes = ("R_thumb", "R_index", "R_middle", "R_ring", "R_pinky")

        for gid in range(self.model.ngeom):
            # Only consider group 0 (collision geoms/hulls) or explicit hull mesh
            bid = self.model.geom_bodyid[gid]
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
            gname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            mid = self.model.geom_dataid[gid]
            mname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_MESH, mid) if self.model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH else ""

            # Check if this geom is a collision hull (group 0 or hull mesh)
            is_hull = (self.model.geom_group[gid] == 0) or ("hull" in mname) or ("hull" in gname)
            if not is_hull:
                continue

            if bname == "base_link":
                self.torso_geoms.append(gid)
            elif "head" in bname:
                self.head_geoms.append(gid)
            elif bname in ("left_arm_link03", "left_arm_link04", "left_arm_link05", "left_arm_link06", "left_arm_link07"):
                self.left_arm_geoms.append(gid)
            elif bname in ("right_arm_link03", "right_arm_link04", "right_arm_link05", "right_arm_link06", "right_arm_link07"):
                self.right_arm_geoms.append(gid)
            elif any(bname.startswith(p) for p in l_hand_prefixes):
                self.left_hand_geoms.append(gid)
            elif any(bname.startswith(p) for p in r_hand_prefixes):
                self.right_hand_geoms.append(gid)

    def _build_collision_pairs(self):
        """Constructs list of (geom1, geom2, margin_category)."""
        self.pairs = []

        # 1. Torso vs Left Arm
        for tg in self.torso_geoms:
            for ag in self.left_arm_geoms:
                self.pairs.append((tg, ag, "torso_arm"))
            for hg in self.left_hand_geoms:
                self.pairs.append((tg, hg, "torso_hand"))

        # 2. Torso vs Right Arm
        for tg in self.torso_geoms:
            for ag in self.right_arm_geoms:
                self.pairs.append((tg, ag, "torso_arm"))
            for hg in self.right_hand_geoms:
                self.pairs.append((tg, hg, "torso_hand"))

        # 3. Head vs Left Arm & Hand
        for hg in self.head_geoms:
            for ag in self.left_arm_geoms:
                self.pairs.append((hg, ag, "head_arm"))
            for lh in self.left_hand_geoms:
                self.pairs.append((hg, lh, "head_hand"))

        # 4. Head vs Right Arm & Hand
        for hg in self.head_geoms:
            for ag in self.right_arm_geoms:
                self.pairs.append((hg, ag, "head_arm"))
            for rh in self.right_hand_geoms:
                self.pairs.append((hg, rh, "head_hand"))

        # 5. Left Arm/Hand vs Right Arm/Hand
        for lg in self.left_arm_geoms:
            for rg in self.right_arm_geoms:
                self.pairs.append((lg, rg, "arm_arm"))
            for rh in self.right_hand_geoms:
                self.pairs.append((lg, rh, "arm_arm"))

        for lh in self.left_hand_geoms:
            for rg in self.right_arm_geoms:
                self.pairs.append((lh, rg, "arm_arm"))
            for rh in self.right_hand_geoms:
                self.pairs.append((lh, rh, "hand_hand"))

    def compute_min_distance(self, data: mujoco.MjData = None, force: bool = False) -> tuple[float, tuple[int, int, str]]:
        """
        Computes minimum signed distance across all monitored collision pairs.
        Returns (min_dist, (geom1, geom2, category)).
        Optimized with early-out when previous min_distance > 5 cm.
        """
        self.step_idx += 1
        if not force and self.last_min_dist > 0.05 and (self.step_idx % 2 != 0):
            return float(self.last_min_dist), self.last_min_pair

        d = data if data is not None else self.scratch_data
        min_dist = 999.0
        min_pair = (None, None, "none")

        for g1, g2, cat in self.pairs:
            dist = mujoco.mj_geomDistance(self.model, d, g1, g2, self.dist_max, self.fromto)
            if dist < min_dist:
                min_dist = dist
                min_pair = (g1, g2, cat)
                if min_dist < 0.005:  # Deep boundary early breach
                    break

        self.last_min_dist = float(min_dist)
        self.last_min_pair = min_pair
        return float(min_dist), min_pair

    def check_configuration(self, qpos: np.ndarray, data: mujoco.MjData = None) -> tuple[bool, float, tuple]:
        """
        Checks whether a given full qpos vector is safe (no self-collision within safety margin).
        Returns (is_safe, min_dist, min_pair).
        """
        if not self.enabled:
            return True, 1.0, (None, None, "disabled")

        d = data if data is not None else self.scratch_data
        d.qpos[:] = qpos
        mujoco.mj_forward(self.model, d)

        min_dist, min_pair = self.compute_min_distance(d)
        req_margin = self.pair_margins.get(min_pair[2], self.safety_margin)
        is_safe = (min_dist >= req_margin)

        return is_safe, min_dist, min_pair

    def filter_step(
        self,
        current_qpos: np.ndarray,
        target_qpos: np.ndarray,
        data: mujoco.MjData = None
    ) -> tuple[np.ndarray, bool, float]:
        """
        Validates target_qpos and interpolates 5 sub-steps between current_qpos and target_qpos.
        If a collision is detected along the path or at the target:
        - Performs line search to find the farthest collision-free configuration along the segment.
        - Returns (safe_qpos, collision_blocked, min_dist).
        """
        if not self.enabled:
            return target_qpos.copy(), False, 1.0

        # Substep path validation (anti-tunneling)
        # At >= 100-200 Hz, deltas < 0.08 rad are continuous; test target directly unless large step jump
        delta_norm = float(np.max(np.abs(target_qpos - current_qpos)))
        if delta_norm < 0.08:
            steps = 1
            alphas = [1.0]
        else:
            steps = max(1, self.substeps)
            alphas = np.linspace(0.0, 1.0, steps + 1)[1:] # sub-steps for step jumps

        best_qpos = current_qpos.copy()
        blocked = False
        lowest_dist = 999.0

        for a in alphas:
            interp_qpos = current_qpos + a * (target_qpos - current_qpos)
            safe, d_val, _ = self.check_configuration(interp_qpos, data)
            if d_val < lowest_dist:
                lowest_dist = d_val

            if safe:
                best_qpos = interp_qpos.copy()
            else:
                blocked = True
                # Binary search / fine line search between best_qpos and interp_qpos
                low_a = a - (1.0 / steps)
                high_a = a
                for _ in range(4):
                    mid_a = 0.5 * (low_a + high_a)
                    test_q = current_qpos + mid_a * (target_qpos - current_qpos)
                    s_mid, d_mid, _ = self.check_configuration(test_q, data)
                    if d_mid < lowest_dist:
                        lowest_dist = d_mid
                    if s_mid:
                        best_qpos = test_q.copy()
                        low_a = mid_a
                    else:
                        high_a = mid_a
                break

        _, safe_dist, _ = self.check_configuration(best_qpos, data)
        return best_qpos, blocked, safe_dist

    def project_target_tangent(
        self,
        current_ee_pos: np.ndarray,
        target_ee_pos: np.ndarray,
        data: mujoco.MjData = None,
        safety_buffer: float = None
    ) -> tuple[np.ndarray, bool]:
        """
        Projects target_ee_pos onto the contact tangent if moving towards a penetrating boundary.
        Removes only the normal component directed into the obstacle, allowing smooth surface sliding.
        Returns (projected_pos, was_projected).
        """
        if not self.enabled:
            return target_ee_pos.copy(), False

        d = data if data is not None else self.scratch_data
        min_dist, (g1, g2, cat) = self.compute_min_distance(d)
        buf = safety_buffer if safety_buffer is not None else self.pair_margins.get(cat, self.safety_margin)

        # Only activate when close to or breaching margin
        if min_dist > buf + 0.015:
            return target_ee_pos.copy(), False

        p1 = self.fromto[0:3]
        p2 = self.fromto[3:6]
        vec = p2 - p1
        norm_v = np.linalg.norm(vec)
        if norm_v < 1e-6:
            n = current_ee_pos - np.array([0.0, 0.0, 1.1])
            norm_n = np.linalg.norm(n)
            n = n / norm_n if norm_n > 1e-4 else np.array([1.0, 0.0, 0.0])
        else:
            n = vec / norm_v

        delta_p = target_ee_pos - current_ee_pos
        normal_component = np.dot(delta_p, n)

        # If moving INTO obstacle (normal_component < 0) or already penetrating (min_dist < buf)
        if normal_component < 0.0:
            delta_tangent = delta_p - normal_component * n
            if min_dist < buf:
                restore = (buf - min_dist) * n
                delta_tangent = delta_tangent + restore
            projected = current_ee_pos + delta_tangent
            return projected, True
        elif min_dist < buf:
            restore = (buf - min_dist) * n
            return target_ee_pos + restore, True

        return target_ee_pos.copy(), False
