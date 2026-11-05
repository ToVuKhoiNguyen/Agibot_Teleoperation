"""
teleop/safety/arm_safety.py
Arm and torso collision avoidance routines:
- compute_min_distance_impl: Evaluates distance across active collision pairs
- check_configuration_impl: Validates forward kinematics for whole-body qpos
- filter_step_impl: Interpolates trajectory and performs binary line search backoff
- project_target_tangent_impl: Projects EE target tangent to contact surface
"""

import mujoco
import numpy as np


def compute_min_distance_impl(
    model: mujoco.MjModel,
    pairs: list,
    data: mujoco.MjData,
    dist_max: float,
    fromto: np.ndarray,
    last_min_dist: float = 0.0,
    last_min_pair: tuple = (None, None, "none"),
    step_idx: int = 0,
    force: bool = False,
) -> tuple[float, tuple]:
    """Computes minimum signed distance across all monitored collision pairs."""
    if not force and last_min_dist > 0.05 and (step_idx % 2 != 0):
        return float(last_min_dist), last_min_pair

    min_dist = 999.0
    min_pair = (None, None, "none")

    for g1, g2, cat in pairs:
        dist = mujoco.mj_geomDistance(model, data, g1, g2, dist_max, fromto)
        if dist < min_dist:
            min_dist = dist
            min_pair = (g1, g2, cat)
            if min_dist < 0.005:  # Deep boundary early breach
                break

    return float(min_dist), min_pair


def check_configuration_impl(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pairs: list,
    pair_margins: dict,
    safety_margin: float,
    dist_max: float,
    fromto: np.ndarray,
    qpos: np.ndarray,
    enabled: bool = True,
) -> tuple[bool, float, tuple]:
    """Checks whether a given full qpos vector is safe."""
    if not enabled:
        return True, 1.0, (None, None, "disabled")

    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)

    min_dist, min_pair = compute_min_distance_impl(
        model, pairs, data, dist_max, fromto, force=True
    )
    req_margin = pair_margins.get(min_pair[2], safety_margin)
    is_safe = (min_dist >= req_margin)

    return is_safe, min_dist, min_pair


def filter_step_impl(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pairs: list,
    pair_margins: dict,
    safety_margin: float,
    dist_max: float,
    fromto: np.ndarray,
    substeps: int,
    current_qpos: np.ndarray,
    target_qpos: np.ndarray,
    enabled: bool = True,
) -> tuple[np.ndarray, bool, float]:
    """Validates target_qpos and interpolates sub-steps to find collision-free path."""
    if not enabled:
        return target_qpos.copy(), False, 1.0

    delta_norm = float(np.max(np.abs(target_qpos - current_qpos)))
    if delta_norm < 0.08:
        steps = 1
        alphas = [1.0]
    else:
        steps = max(1, substeps)
        alphas = np.linspace(0.0, 1.0, steps + 1)[1:]

    best_qpos = current_qpos.copy()
    blocked = False
    lowest_dist = 999.0

    for a in alphas:
        interp_qpos = current_qpos + a * (target_qpos - current_qpos)
        safe, d_val, _ = check_configuration_impl(
            model, data, pairs, pair_margins, safety_margin, dist_max, fromto, interp_qpos, enabled=enabled
        )
        if d_val < lowest_dist:
            lowest_dist = d_val

        if safe:
            best_qpos = interp_qpos.copy()
        else:
            blocked = True
            low_a = a - (1.0 / steps)
            high_a = a
            for _ in range(4):
                mid_a = 0.5 * (low_a + high_a)
                test_q = current_qpos + mid_a * (target_qpos - current_qpos)
                s_mid, d_mid, _ = check_configuration_impl(
                    model, data, pairs, pair_margins, safety_margin, dist_max, fromto, test_q, enabled=enabled
                )
                if d_mid < lowest_dist:
                    lowest_dist = d_mid
                if s_mid:
                    best_qpos = test_q.copy()
                    low_a = mid_a
                else:
                    high_a = mid_a
            break

    _, safe_dist, _ = check_configuration_impl(
        model, data, pairs, pair_margins, safety_margin, dist_max, fromto, best_qpos, enabled=enabled
    )
    return best_qpos, blocked, safe_dist


def project_target_tangent_impl(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pairs: list,
    pair_margins: dict,
    safety_margin: float,
    dist_max: float,
    fromto: np.ndarray,
    current_ee_pos: np.ndarray,
    target_ee_pos: np.ndarray,
    safety_buffer: float = None,
    enabled: bool = True,
) -> tuple[np.ndarray, bool]:
    """Projects target_ee_pos onto contact tangent if moving towards boundary."""
    if not enabled:
        return target_ee_pos.copy(), False

    min_dist, (g1, g2, cat) = compute_min_distance_impl(
        model, pairs, data, dist_max, fromto, force=True
    )
    buf = safety_buffer if safety_buffer is not None else pair_margins.get(cat, safety_margin)

    if min_dist > buf + 0.015:
        return target_ee_pos.copy(), False

    p1 = fromto[0:3]
    p2 = fromto[3:6]
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
