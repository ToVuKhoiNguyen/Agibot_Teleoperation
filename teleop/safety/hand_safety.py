"""
teleop/safety/hand_safety.py
Intra-hand self-collision checking and line-search back-off:
- check_hand_collision: Forward kinematics on active/mimic joints + distance query
- filter_hand_step: Binary line search on closure along trajectory to enforce safe margin
"""

import mujoco
import numpy as np


def check_hand_collision_impl(
    model: mujoco.MjModel,
    scratch_data: mujoco.MjData,
    hand_pairs: dict,
    hand_active_jnt_adrs: dict,
    hand_mimic_jnt_adrs: dict,
    side: str,
    active_qpos: np.ndarray,
    margin: float = 0.001,
    data: mujoco.MjData = None,
    dist_max: float = 0.15,
    fromto: np.ndarray = None,
    enabled: bool = True,
) -> tuple[bool, float, tuple]:
    """Forward-kinematics check of candidate active hand command (6 active joints)."""
    if not enabled:
        return True, 1.0, (None, None, "disabled")

    d = data if data is not None else scratch_data
    side = "left" if side == "left" else "right"
    q_act = np.asarray(active_qpos, dtype=np.float64)
    if len(q_act) < 6:
        return True, 1.0, (None, None, "insufficient_dims")

    # Set active joints
    act_adrs = hand_active_jnt_adrs[side]
    for idx, val in zip(act_adrs, q_act):
        d.qpos[idx] = val

    # Set mimic joints from active joints
    mim_adrs = hand_mimic_jnt_adrs[side]
    d.qpos[mim_adrs[0]] = 0.40 * q_act[1]  # thumb_2
    d.qpos[mim_adrs[1]] = 0.60 * q_act[1]  # thumb_3
    d.qpos[mim_adrs[2]] = 1.00 * q_act[2]  # index_2
    d.qpos[mim_adrs[3]] = 1.00 * q_act[3]  # middle_2
    d.qpos[mim_adrs[4]] = 1.00 * q_act[4]  # ring_2
    d.qpos[mim_adrs[5]] = 1.00 * q_act[5]  # pinky_2

    mujoco.mj_forward(model, d)

    min_dist = 999.0
    min_pair = (None, None, "none")
    ft = fromto if fromto is not None else np.zeros(6, dtype=np.float64)

    for g1, g2, pname in hand_pairs[side]:
        dist = mujoco.mj_geomDistance(model, d, g1, g2, dist_max, ft)
        if dist < min_dist:
            min_dist = dist
            min_pair = (g1, g2, pname)
            if min_dist < 0.0:
                break

    is_safe = (min_dist >= margin)
    return is_safe, float(min_dist), min_pair


def filter_hand_step_impl(
    model: mujoco.MjModel,
    scratch_data: mujoco.MjData,
    hand_pairs: dict,
    hand_active_jnt_adrs: dict,
    hand_mimic_jnt_adrs: dict,
    side: str,
    current_active_qpos: np.ndarray,
    target_active_qpos: np.ndarray,
    margin: float = 0.001,
    data: mujoco.MjData = None,
    dist_max: float = 0.15,
    fromto: np.ndarray = None,
    enabled: bool = True,
) -> tuple[np.ndarray, bool, float]:
    """Validates target_active_qpos and performs binary line search back-off if necessary."""
    if not enabled:
        return target_active_qpos.copy(), False, 1.0

    curr = np.asarray(current_active_qpos, dtype=np.float64) if current_active_qpos is not None else np.zeros(6)
    tgt = np.asarray(target_active_qpos, dtype=np.float64)

    if np.max(np.abs(tgt - curr)) < 1e-4:
        return curr.copy(), False, 0.005

    def _chk(q):
        return check_hand_collision_impl(
            model, scratch_data, hand_pairs, hand_active_jnt_adrs, hand_mimic_jnt_adrs,
            side, q, margin=margin, data=data, dist_max=dist_max, fromto=fromto, enabled=enabled,
        )

    safe_tgt, dist_tgt, _ = _chk(target_active_qpos)
    if safe_tgt:
        return target_active_qpos.copy(), False, dist_tgt

    # Target violates margin -> perform binary line search backoff
    blocked = True
    safe_curr, dist_curr, _ = _chk(curr)
    if not safe_curr:
        curr = np.zeros(6, dtype=np.float64)
        safe_curr, dist_curr, _ = _chk(curr)

    low_a = 0.0
    high_a = 1.0
    best_cmd = curr.copy()
    best_dist = dist_curr if safe_curr else margin

    for _ in range(8):
        mid_a = 0.5 * (low_a + high_a)
        cand = curr + mid_a * (tgt - curr)
        s_cand, d_cand, _ = _chk(cand)
        if s_cand:
            best_cmd = cand.copy()
            best_dist = d_cand
            low_a = mid_a
        else:
            high_a = mid_a

    # Per-finger refinement: allow non-colliding fingers to flex up to target
    for j in (3, 4, 5, 2, 0, 1):
        if abs(tgt[j] - best_cmd[j]) < 1e-3:
            continue
        c = best_cmd.copy()
        c[j] = tgt[j]
        s_cand, d_cand, _ = _chk(c)
        if s_cand:
            best_cmd[j] = tgt[j]
            best_dist = min(best_dist, d_cand)
        else:
            low_j, high_j = best_cmd[j], tgt[j]
            for _ in range(4):
                mid_j = 0.5 * (low_j + high_j)
                c[j] = mid_j
                s_mid, d_mid, _ = _chk(c)
                if s_mid:
                    best_cmd[j] = mid_j
                    best_dist = min(best_dist, d_mid)
                    low_j = mid_j
                else:
                    high_j = mid_j

    return best_cmd, blocked, float(best_dist)
