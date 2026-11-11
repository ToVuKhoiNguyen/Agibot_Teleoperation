"""
teleop/safety/armor_collision.py
Armor collision check and soft-block avoidance for AGIBOT A2 Ultra.
Inflates collision links by shell_m (0.01 m, threshold = 2 * shell_m = 0.02 m).
Soft block inside limiter step (holds pose with v=0, no FAULT, no banner).
Latches blocked state with >= 2 mm clearance improvement hysteresis.
"""

import os
import yaml
import numpy as np
import mujoco


class ArmorCollisionChecker:
    """Evaluates swept distance across candidate steps and soft-blocks on shell contact."""

    def __init__(self, model: mujoco.MjModel, config_path: str = None, enabled: bool = True):
        self.model = model
        self.enabled = enabled
        self.scratch_data = mujoco.MjData(model)
        self.fromto = np.zeros(6, dtype=np.float64)

        # Load config
        self.shell_m = 0.01
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    cfg = yaml.safe_load(f) or {}
                ac_cfg = cfg.get("armor_collision", {})
                self.shell_m = float(ac_cfg.get("shell_m", 0.01))
                self.enabled = bool(ac_cfg.get("enabled", enabled))
            except Exception as e:
                print(f"[ArmorCollision] Error loading config: {e}")

        self.threshold_m = 2.0 * self.shell_m # 0.02 m

        # Discover joint qpos addresses
        left_jnts = [f"idx{13+i}_left_arm_joint{i+1}" for i in range(7)]
        right_jnts = [f"idx{20+i}_right_arm_joint{i+1}" for i in range(7)]
        self.l_adrs = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in left_jnts]
        self.r_adrs = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in right_jnts]

        # Discover geoms and build pairs
        self._build_candidate_pairs()

        # Block state tracking
        self.is_blocked = False
        self.blocked_pair = ""
        self.blocked_dist = 0.0
        self.last_min_dist = 999.0
        self.last_log_t = 0.0

        print(
            f"[ArmorCollision] Discovered {len(self.pairs)} candidate collision pairs "
            f"(shell_m: {self.shell_m:.3f}m, threshold: {self.threshold_m:.3f}m)"
        )

    def _build_candidate_pairs(self):
        """Constructs candidate pairs: arm-arm, arm-torso/head/pelvis, intra-arm non-adjacent."""
        torso_geoms = []
        head_geoms = []
        pelvis_geoms = []
        l_arm_geoms = []
        r_arm_geoms = []

        l_bodies = {"left_arm_link03", "left_arm_link04", "left_arm_link05", "left_arm_link06", "left_arm_link07"}
        r_bodies = {"right_arm_link03", "right_arm_link04", "right_arm_link05", "right_arm_link06", "right_arm_link07"}
        pelvis_bodies = {"left_hip_roll", "left_hip_yaw", "left_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_hip_pitch"}

        for gid in range(self.model.ngeom):
            if self.model.geom_group[gid] != 0 and self.model.geom_contype[gid] == 0:
                continue
            bid = self.model.geom_bodyid[gid]
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""

            # Ignore finger links
            if any(bname.startswith(p) for p in ("L_thumb", "L_index", "L_middle", "L_ring", "L_pinky", "R_thumb", "R_index", "R_middle", "R_ring", "R_pinky")):
                continue

            if bname == "base_link":
                torso_geoms.append(gid)
            elif "head" in bname:
                head_geoms.append(gid)
            elif bname in pelvis_bodies:
                pelvis_geoms.append(gid)
            elif bname in l_bodies:
                l_arm_geoms.append(gid)
            elif bname in r_bodies:
                r_arm_geoms.append(gid)

        fixed_geoms = torso_geoms + head_geoms + pelvis_geoms
        pairs = []

        # 1. Arm vs Torso/Head/Pelvis
        for fg in fixed_geoms:
            for lg in l_arm_geoms:
                pairs.append((fg, lg))
            for rg in r_arm_geoms:
                pairs.append((fg, rg))

        # 2. Left Arm vs Right Arm
        for lg in l_arm_geoms:
            for rg in r_arm_geoms:
                pairs.append((lg, rg))

        # 3. Intra-arm non-adjacent links
        for arm_geoms in (l_arm_geoms, r_arm_geoms):
            by_bname = {}
            for g in arm_geoms:
                bid = self.model.geom_bodyid[g]
                bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
                by_bname.setdefault(bname, []).append(g)

            prefix = "left_arm" if arm_geoms is l_arm_geoms else "right_arm"
            l3 = by_bname.get(f"{prefix}_link03", [])
            l4 = by_bname.get(f"{prefix}_link04", [])
            l5 = by_bname.get(f"{prefix}_link05", [])
            l6 = by_bname.get(f"{prefix}_link06", [])
            l7 = by_bname.get(f"{prefix}_link07", [])

            for g3 in l3:
                for g5 in l5: pairs.append((g3, g5))
                for g6 in l6: pairs.append((g3, g6))
                for g7 in l7: pairs.append((g3, g7))
            for g4 in l4:
                for g6 in l6: pairs.append((g4, g6))
                for g7 in l7: pairs.append((g4, g7))

        self.pairs = pairs
        self.p_g1 = np.array([p[0] for p in pairs], dtype=np.int32)
        self.p_g2 = np.array([p[1] for p in pairs], dtype=np.int32)
        r1 = self.model.geom_rbound[self.p_g1]
        r2 = self.model.geom_rbound[self.p_g2]
        self.r_sums_sq = (r1 + r2 + 0.15) ** 2

    def compute_min_distance(self, q14: np.ndarray) -> tuple[float, str, tuple[int, int]]:
        """Computes minimum pairwise distance across all candidate pairs at q14 with coarse pre-filter."""
        self.scratch_data.qpos[self.l_adrs] = q14[:7]
        self.scratch_data.qpos[self.r_adrs] = q14[7:]
        mujoco.mj_kinematics(self.model, self.scratch_data)

        min_d = 999.0
        min_g1, min_g2 = -1, -1

        gxpos = self.scratch_data.geom_xpos
        diff = gxpos[self.p_g1] - gxpos[self.p_g2]
        d_sq = np.einsum("ij,ij->i", diff, diff)
        active_indices = np.where(d_sq <= self.r_sums_sq)[0]

        for idx in active_indices:
            g1, g2 = int(self.p_g1[idx]), int(self.p_g2[idx])
            d = mujoco.mj_geomDistance(self.model, self.scratch_data, g1, g2, 0.15, self.fromto)
            if d < min_d:
                min_d = d
                min_g1, min_g2 = g1, g2

        pair_name = ""
        if min_g1 >= 0:
            b1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.model.geom_bodyid[min_g1])
            b2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.model.geom_bodyid[min_g2])
            pair_name = f"{b1} / {b2}"

        self.last_min_dist = float(min_d)
        return float(min_d), pair_name, (min_g1, min_g2)

    def filter_step(self, current_q14, candidate_q14) -> tuple[np.ndarray, bool, bool, float, str]:
        """Evaluates swept candidate step (1/3, 2/3, 1), bisects on collision, holds with v=0 if blocked."""
        curr_q = np.asarray(current_q14, dtype=np.float64)
        cand_q = np.asarray(candidate_q14, dtype=np.float64)
        if not self.enabled:
            return cand_q.copy(), True, False, 999.0, ""

        dq = cand_q - curr_q
        curr_d, curr_pair, _ = self.compute_min_distance(curr_q)

        if float(np.max(np.abs(dq))) < 1e-6:
            if curr_d <= self.threshold_m:
                self.is_blocked = True
                self.blocked_pair = curr_pair
                self.blocked_dist = curr_d
            elif self.is_blocked and curr_d >= self.blocked_dist + 0.002:
                self.is_blocked = False
                self.blocked_pair = ""
            return curr_q.copy(), True, self.is_blocked, curr_d, self.blocked_pair

        violator_pair = curr_pair

        # Swept check with bisection: alpha in [1.0, 0.5, 0.25, 0.125]
        for alpha in (1.0, 0.5, 0.25, 0.125):
            p1 = curr_q + dq * (alpha / 3.0)
            p2 = curr_q + dq * (alpha * 2.0 / 3.0)
            p3 = curr_q + dq * alpha

            d1, pair1, _ = self.compute_min_distance(p1)
            d2, pair2, _ = self.compute_min_distance(p2)
            d3, pair3, _ = self.compute_min_distance(p3)

            d_min_step = min(d1, d2, d3)
            if d_min_step == d1: violator_pair = pair1
            elif d_min_step == d2: violator_pair = pair2
            else: violator_pair = pair3

            if d_min_step > self.threshold_m:
                if self.is_blocked and d3 >= self.blocked_dist + 0.002:
                    self.is_blocked = False
                    self.blocked_pair = ""
                return p3.copy(), True, self.is_blocked, d3, self.blocked_pair
            else:
                # Step touches shell: allowed if increasing clearance
                if d3 > curr_d + 1e-5 and d_min_step >= curr_d - 1e-5:
                    if self.is_blocked and d3 >= self.blocked_dist + 0.002:
                        self.is_blocked = False
                        self.blocked_pair = ""
                    return p3.copy(), True, self.is_blocked, d3, self.blocked_pair

        # All bisections rejected -> Soft block!
        self.is_blocked = True
        self.blocked_dist = curr_d
        self.blocked_pair = curr_pair or violator_pair
        return curr_q.copy(), False, True, curr_d, self.blocked_pair
