"""
teleop/kinematics/reach_projection.py
Cartesian workspace reachability projection for AGIBOT A2 Ultra dual-arm teleoperation.
Projects unreachable Cartesian targets onto the reachable sphere around each shoulder frame,
maintaining the commanded direction and orientation, preventing kinematic singularity chatter.
"""

import os
import yaml
import numpy as np

# Shoulder positions in base frame (from URDF / MJCF geometry)
SHOULDER_ORIGIN_LEFT = np.array([-0.03779, 0.19100, 1.27210], dtype=np.float64)
SHOULDER_ORIGIN_RIGHT = np.array([-0.03779, -0.19100, 1.27210], dtype=np.float64)

# Derived from URDF kinematic chain (shoulder joint1 anchor to left_arm_ee site):
# Segments: link02 (0.1540m) + link03 (0.0930m) + link04 (0.1030m) + link05 (0.1598m) = 0.5098m
# Safety margin from config/safety.yaml (0.0098m) -> R_max = 0.5098 - 0.0098 = 0.500m
URDF_CHAIN_LENGTH_M = 0.5098
DEFAULT_SAFETY_MARGIN_M = 0.0098
DEFAULT_MAX_REACH_M = URDF_CHAIN_LENGTH_M - DEFAULT_SAFETY_MARGIN_M # 0.500 m
# Inner limit: with the elbow fully bent (J4 limit 2.0 rad) the hand is still 0.27-0.28 m from the shoulder (FK over
# 20000 random shoulder poses), so a target closer than that cannot be reached by any posture. Real robot 17/11,
# trigger-only motion: the target was inside this zone 20 % of the time (IK error 40-100 mm), and there the IK
# solution swung between postures (joint target leaps of 0.5-0.95 rad in one tick) = the arm changing posture
# abruptly near the shoulder. Such a target is pushed out onto this sphere. MIN_REACH_M=0 disables.
DEFAULT_MIN_REACH_M = float(os.environ.get("MIN_REACH_M", "0.29"))
_INNER_FALLBACK_DIR = np.array([0.8, 0.0, -0.6])   # target exactly on the shoulder: forward and down


def load_max_reach_from_config(config_path: str = "config/safety.yaml") -> float:
    """Loads max reach derived from URDF chain length minus safety margin."""
    if os.path.exists(config_path):
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f) or {}
            rcfg = cfg.get("reach_projection", {})
            chain_len = float(rcfg.get("chain_length_m", URDF_CHAIN_LENGTH_M))
            margin = float(rcfg.get("safety_margin_m", DEFAULT_SAFETY_MARGIN_M))
            return float(rcfg.get("max_reach_m", chain_len - margin))
        except Exception:
            pass
    return DEFAULT_MAX_REACH_M


def reach_message(d_beyond_m: float) -> str:
    """Operator text for a clamped target (see project_to_reachable_workspace d_beyond_m)."""
    if d_beyond_m < 0.0:
        return f"Target too close to the shoulder ({-d_beyond_m*100:.0f} cm inside)"
    return f"Target out of reach ({d_beyond_m*100:.0f} cm beyond)"


def project_to_reachable_workspace(
    target_pos: np.ndarray,
    shoulder_pos: np.ndarray = None,
    side: str = "left",
    max_reach: float = DEFAULT_MAX_REACH_M,
) -> tuple[np.ndarray, bool, float]:
    """Projects a 3D Cartesian target position onto the reachable sphere around the shoulder.

    Returns:
        projected_pos: 3D position (bit-identical copy if target is within reach).
        is_clamped: True if target was outside reachable workspace.
        d_beyond_m: Distance in meters target was outside max_reach (0.0 if within reach);
            NEGATIVE = the target was that far inside the inner limit (too close to the shoulder).
    """
    if shoulder_pos is None:
        shldr = SHOULDER_ORIGIN_LEFT if side.lower() == "left" else SHOULDER_ORIGIN_RIGHT
    else:
        shldr = np.asarray(shoulder_pos, dtype=np.float64)

    pos = np.asarray(target_pos, dtype=np.float64)
    v = pos - shldr
    dist = float(np.linalg.norm(v))

    if dist > max_reach and dist > 1e-6:
        proj_pos = shldr + v * (max_reach / dist)
        d_beyond = dist - max_reach
        return proj_pos, True, float(d_beyond)
    elif dist < DEFAULT_MIN_REACH_M:
        u = v / dist if dist > 1e-6 else _INNER_FALLBACK_DIR
        return shldr + u * DEFAULT_MIN_REACH_M, True, float(dist - DEFAULT_MIN_REACH_M)
    else:
        # Strict identity within reachable set
        return pos.copy(), False, 0.0

