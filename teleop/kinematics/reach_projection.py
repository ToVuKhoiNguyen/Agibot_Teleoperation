"""
teleop/kinematics/reach_projection.py
Cartesian workspace reachability projection for AGIBOT A2 Ultra dual-arm teleoperation.
Projects unreachable Cartesian targets onto the reachable sphere around each shoulder frame,
maintaining the commanded direction and orientation, preventing kinematic singularity chatter.
"""

import numpy as np

# Shoulder positions in base frame (from URDF / MJCF geometry)
SHOULDER_ORIGIN_LEFT = np.array([-0.03779, 0.19100, 1.27210], dtype=np.float64)
SHOULDER_ORIGIN_RIGHT = np.array([-0.03779, -0.19100, 1.27210], dtype=np.float64)
DEFAULT_MAX_REACH_M = 0.500 # True physical arm length from shoulder to EE


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
        d_beyond_m: Distance in meters target was outside max_reach (0.0 if within reach).
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
    else:
        # Strict identity within reachable set
        return pos.copy(), False, 0.0

