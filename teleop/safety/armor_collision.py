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

        # Initialize scratch data with standing home posture
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.scratch_data, key_id)

        # Load config
        self.shell_m = 0.01
        # Torso is a single convex hull (fills concavities at the shoulders), so it is already conservative:
        # pairs touching it use this smaller trigger distance instead of 2 * shell_m.
        self.torso_threshold_m = 0.005
        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, "r", encoding="utf-8") as f:
                    cfg = yaml.safe_load(f) or {}
                ac_cfg = cfg.get("armor_collision", {})
                self.shell_m = float(ac_cfg.get("shell_m", 0.01))
                self.torso_threshold_m = float(ac_cfg.get("torso_threshold_m", self.torso_threshold_m))
                self.enabled = bool(ac_cfg.get("enabled", enabled))
            except Exception as e:
                print(f"[ArmorCollision] Error loading config: {e}")

        self.threshold_m = 2.0 * self.shell_m  # 0.020 m
        self.torso_threshold_m = min(self.torso_threshold_m, self.threshold_m)

        # Discover joint qpos addresses
        left_jnts = [f"idx{13+i}_left_arm_joint{i+1}" for i in range(7)]
        right_jnts = [f"idx{20+i}_right_arm_joint{i+1}" for i in range(7)]
        self.l_adrs = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in left_jnts]
        self.r_adrs = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in right_jnts]

        # Discover geoms and build pairs
        self._build_candidate_pairs()
        self.levers = self._compute_joint_levers()

        # Block state tracking
        self.is_blocked = False
        self.blocked_pair = ""
        self.blocked_dist = 0.0
        self.last_min_dist = 999.0

    def _compute_joint_levers(self, n_samples: int = 300, pad_m: float = 0.02) -> np.ndarray:
        """Per-joint upper bound (m) on distance from the joint axis anchor to any collision point it moves.

        Max over random in-range arm configurations of |geom_xpos - xanchor| + geom_rbound for all
        collision geoms in the joint's subtree, plus pad_m. Used for the Lipschitz fast path.
        """
        jids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{13+i}_left_arm_joint{i+1}") for i in range(7)] + \
               [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{20+i}_right_arm_joint{i+1}") for i in range(7)]
        adrs = list(self.l_adrs) + list(self.r_adrs)
        col = [g for g in range(self.model.ngeom) if self.model.geom_group[g] == 0 or self.model.geom_contype[g] != 0]
        subtree = []
        for jid in jids:
            root = self.model.jnt_bodyid[jid]
            geoms = []
            for g in col:
                b = self.model.geom_bodyid[g]
                while b > 0 and b != root:
                    b = self.model.body_parentid[b]
                if b == root:
                    geoms.append(g)
            subtree.append(np.array(geoms, dtype=np.int32))
        d = mujoco.MjData(self.model)
        d.qpos[:] = self.scratch_data.qpos
        rng = np.random.RandomState(0)
        lo, hi = self.model.jnt_range[jids, 0], self.model.jnt_range[jids, 1]
        levers = np.zeros(14)
        for k in range(n_samples):
            d.qpos[adrs] = d.qpos[adrs] if k == 0 else rng.uniform(lo, hi)
            mujoco.mj_kinematics(self.model, d)
            for i, jid in enumerate(jids):
                gs = subtree[i]
                if len(gs):
                    r = np.linalg.norm(d.geom_xpos[gs] - d.xanchor[jid], axis=1) + self.model.geom_rbound[gs]
                    levers[i] = max(levers[i], float(np.max(r)))
        return levers + pad_m

    def _build_candidate_pairs(self):
        """Constructs candidate pairs: arm-arm, arm-torso, arm-head, arm-pelvis, arm-thigh/shin, hand-body, in-arm."""
        def is_col(g):
            return self.model.geom_group[g] == 0 or self.model.geom_contype[g] != 0

        torso, head, pelvis, thigh_shin = [], [], [], []
        l_arm, r_arm, l_hand, r_hand = [], [], [], []

        for gid in range(self.model.ngeom):
            if not is_col(gid):
                continue
            bid = self.model.geom_bodyid[gid]
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""

            if bname == "base_link":
                torso.append(gid)
            elif "head" in bname:
                head.append(gid)
            elif any(p in bname for p in ("hip_roll", "hip_yaw", "hip_pitch")):
                pelvis.append(gid)
            elif any(p in bname for p in ("tarsus", "toe")):
                thigh_shin.append(gid)
            elif any(f"left_arm_link0{i}" in bname for i in range(3, 8)):
                l_arm.append(gid)
            elif any(f"right_arm_link0{i}" in bname for i in range(3, 8)):
                r_arm.append(gid)
            elif bname.startswith("L_"):
                l_hand.append(gid)
            elif bname.startswith("R_"):
                r_hand.append(gid)

        body_fixed = torso + head + pelvis + thigh_shin
        p_arm_arm = [(la, ra) for la in l_arm for ra in r_arm]
        p_arm_torso = [(a, g) for a in l_arm + r_arm for g in torso]
        p_arm_head = [(a, g) for a in l_arm + r_arm for g in head]
        p_arm_pelvis = [(a, g) for a in l_arm + r_arm for g in pelvis]
        p_arm_thigh_shin = [(a, g) for a in l_arm + r_arm for g in thigh_shin]

        p_hand_body = (
            [(h, g) for h in l_hand + r_hand for g in body_fixed]
            + [(lh, ra) for lh in l_hand for ra in r_arm]
            + [(rh, la) for rh in r_hand for la in l_arm]
            + [(lh, rh) for lh in l_hand for rh in r_hand]
        )

        p_in_arm = []
        for arm_geoms in (l_arm, r_arm):
            by_link = {}
            for g in arm_geoms:
                bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.model.geom_bodyid[g])
                by_link.setdefault(bname, []).append(g)
            prefix = "left_arm" if arm_geoms is l_arm else "right_arm"
            for g3 in by_link.get(f"{prefix}_link03", []):
                for g5 in by_link.get(f"{prefix}_link05", []): p_in_arm.append((g3, g5))
                for g6 in by_link.get(f"{prefix}_link06", []): p_in_arm.append((g3, g6))
                for g7 in by_link.get(f"{prefix}_link07", []): p_in_arm.append((g3, g7))
            for g4 in by_link.get(f"{prefix}_link04", []):
                for g6 in by_link.get(f"{prefix}_link06", []): p_in_arm.append((g4, g6))
                for g7 in by_link.get(f"{prefix}_link07", []): p_in_arm.append((g4, g7))

        all_pairs = list(set(tuple(sorted(p)) for p in (
            p_arm_arm + p_arm_torso + p_arm_head + p_arm_pelvis + p_arm_thigh_shin + p_hand_body + p_in_arm
        )))

        self.pairs = all_pairs
        self.p_g1 = np.array([p[0] for p in all_pairs], dtype=np.int32)
        self.p_g2 = np.array([p[1] for p in all_pairs], dtype=np.int32)
        torso_set = set(torso)
        # Effective distance = d + offset, so every pair is compared against the single threshold_m:
        # d_eff <= threshold_m  <=>  d <= that pair's own threshold.
        self.p_offset = np.array(
            [(self.threshold_m - self.torso_threshold_m) if (p[0] in torso_set or p[1] in torso_set) else 0.0 for p in all_pairs],
            dtype=np.float64,
        )
        r1, r2 = self.model.geom_rbound[self.p_g1], self.model.geom_rbound[self.p_g2]
        self.r_sums_sq = (r1 + r2 + 0.15) ** 2

        print(
            f"[ArmorCollision] Group counts: arm-arm: {len(p_arm_arm)}, arm-torso: {len(p_arm_torso)}, "
            f"arm-head: {len(p_arm_head)}, arm-pelvis: {len(p_arm_pelvis)}, arm-thigh/shin: {len(p_arm_thigh_shin)}, "
            f"hand-body: {len(p_hand_body)}, in-arm: {len(p_in_arm)} | Total: {len(all_pairs)} pairs (threshold: {self.threshold_m*1000:.0f}mm)"
        )

    def compute_min_distance(self, q14: np.ndarray) -> tuple[float, str, tuple[int, int]]:
        """Minimum pairwise effective distance (torso pairs shifted by p_offset) at q14, with coarse pre-filter."""
        self.scratch_data.qpos[self.l_adrs] = q14[:7]
        self.scratch_data.qpos[self.r_adrs] = q14[7:]
        mujoco.mj_kinematics(self.model, self.scratch_data)

        gxpos = self.scratch_data.geom_xpos
        diff = gxpos[self.p_g1] - gxpos[self.p_g2]
        d_sq = np.einsum("ij,ij->i", diff, diff)
        active_indices = np.where(d_sq <= self.r_sums_sq)[0]

        min_d, min_g1, min_g2 = 999.0, -1, -1
        for idx in active_indices:
            g1, g2 = int(self.p_g1[idx]), int(self.p_g2[idx])
            d = mujoco.mj_geomDistance(self.model, self.scratch_data, g1, g2, 0.15, self.fromto) + self.p_offset[idx]
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

    def filter_step(self, current_q14, candidate_q14, goal_q14=None) -> tuple[np.ndarray, bool, bool, float, str]:
        """Evaluates swept candidate step (1/3, 2/3, 1), bisects on collision, holds with v=0 if blocked."""
        curr_q = np.asarray(current_q14, dtype=np.float64)
        cand_q = np.asarray(candidate_q14, dtype=np.float64)
        if not self.enabled:
            return cand_q.copy(), True, False, 999.0, ""

        dq = cand_q - curr_q
        max_dq = float(np.max(np.abs(dq)))
        if max_dq < 1e-6:
            curr_d, curr_pair, _ = self.compute_min_distance(curr_q)
            if curr_d <= self.threshold_m:
                self.is_blocked = True
                self.blocked_pair = curr_pair
                self.blocked_dist = curr_d
            elif self.is_blocked and curr_d >= max(self.threshold_m + 0.001, self.blocked_dist + 0.002):
                self.is_blocked = False
                self.blocked_pair = ""
            return curr_q.copy(), True, self.is_blocked, curr_d, self.blocked_pair

        # Fast path: check endpoint candidate first if safely clear.
        # Lipschitz bound: a joint step dq moves any collision point by at most
        # sum_i |dq_i| * lever_i (metres; lever_i from _compute_joint_levers). If the endpoint
        # clearance exceeds threshold + that bound, the whole straight segment is clear.
        d3, pair3, _ = self.compute_min_distance(cand_q)
        safe_margin = max(0.005, float(np.dot(np.abs(dq), self.levers)))
        if d3 > self.threshold_m + safe_margin and not self.is_blocked:
            return cand_q.copy(), True, False, d3, ""

        curr_d, curr_pair, _ = self.compute_min_distance(curr_q)
        violator_pair = curr_pair

        # Second exact fast path: every point of the segment lies within half the step of one
        # endpoint, so clearance along it >= min(d_curr, d_cand) - safe_margin / 2.
        if not self.is_blocked and min(curr_d, d3) > self.threshold_m + 0.5 * safe_margin:
            return cand_q.copy(), True, False, d3, ""

        # Swept check with bisection: alpha in [1.0, 0.5, 0.25, 0.125]
        for alpha in (1.0, 0.5, 0.25, 0.125):
            p1 = curr_q + dq * (alpha / 3.0)
            p2 = curr_q + dq * (alpha * 2.0 / 3.0)
            p3 = curr_q + dq * alpha

            d1, pair1, _ = self.compute_min_distance(p1)
            d2, pair2, _ = self.compute_min_distance(p2)
            d3, pair3, _ = (d3, pair3, None) if alpha == 1.0 else self.compute_min_distance(p3)

            d_min_step = min(d1, d2, d3)
            violator_pair = pair1 if d_min_step == d1 else (pair2 if d_min_step == d2 else pair3)

            if d_min_step > self.threshold_m:
                if self.is_blocked and d3 >= max(self.threshold_m + 0.001, self.blocked_dist + 0.002):
                    self.is_blocked = False
                    self.blocked_pair = ""
                return p3.copy(), True, self.is_blocked, d3, self.blocked_pair
            else:
                # Step touches shell: allowed if increasing clearance
                if d3 >= curr_d and d_min_step >= curr_d - 1e-5:
                    if self.is_blocked and d3 >= max(self.threshold_m + 0.001, self.blocked_dist + 0.002):
                        self.is_blocked = False
                        self.blocked_pair = ""
                    return p3.copy(), True, self.is_blocked, d3, self.blocked_pair

        # All bisections rejected -> Soft block. One arm touching the shell must not freeze the other arm,
        # so each arm is retried on its own. If the commanded goal itself is clear of the shell (the
        # operator is steering OUT, not pushing in), single joints may also keep moving as long as they do
        # not reduce clearance; otherwise the straight joint-space path to a legal goal can stay blocked forever.
        self.is_blocked = True
        self.blocked_dist = curr_d
        self.blocked_pair = curr_pair or violator_pair
        per_joint = False
        if goal_q14 is not None:
            d_goal, _, _ = self.compute_min_distance(np.asarray(goal_q14, dtype=np.float64))
            per_joint = d_goal > self.threshold_m + 0.001
        q_slide, d_slide, moved = self._slide_step(curr_q, dq, curr_d, per_joint)
        if moved:
            if d_slide >= max(self.threshold_m + 0.001, self.blocked_dist + 0.002):
                self.is_blocked = False
                self.blocked_pair = ""
            return q_slide, True, self.is_blocked, d_slide, self.blocked_pair
        return curr_q.copy(), False, True, curr_d, self.blocked_pair

    def _slide_step(self, curr_q: np.ndarray, dq: np.ndarray, curr_d: float, per_joint: bool = False):
        """Per-arm partial step: an arm whose own move keeps clearance above the threshold, or does not
        reduce it, keeps moving while the other arm is blocked. Small steps only (endpoint check, <= 1 mm)."""
        q = curr_q.copy()
        d_now = curr_d
        moved = False

        def try_apply(idx):
            nonlocal q, d_now, moved
            step = np.zeros(14)
            step[idx] = dq[idx]
            if float(np.max(np.abs(step))) < 1e-9:
                return True
            if float(np.dot(np.abs(step), self.levers)) > 0.001:
                step *= 0.001 / float(np.dot(np.abs(step), self.levers))
            cand = q + step
            d_c, _, _ = self.compute_min_distance(cand)
            if d_c > self.threshold_m or d_c >= d_now:
                q, d_now, moved = cand, d_c, True
                return True
            return False

        for arm in (list(range(7)), list(range(7, 14))):
            if not try_apply(arm) and per_joint:
                for j in arm:
                    try_apply([j])
        self.last_min_dist = float(d_now)
        return q, d_now, moved
