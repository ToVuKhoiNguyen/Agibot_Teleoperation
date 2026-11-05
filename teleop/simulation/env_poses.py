"""
teleop/simulation/env_poses.py
Pose extraction methods for AgibotSimEnv:
- extract_ee_poses: site/body world translation and 3x3 orientation
- extract_visual_body_poses: world-frame geom transforms aligned to mesh coordinates
"""

import mujoco
import numpy as np


def extract_ee_poses(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    left_ee_site_id: int,
    left_ee_body_id: int,
    right_ee_site_id: int,
    right_ee_body_id: int,
) -> dict:
    """Returns EE translation (x,y,z) and rotation matrix (3x3) for both wrists."""
    mujoco.mj_kinematics(model, data)
    poses = {}

    if left_ee_site_id >= 0:
        l_pos = data.site_xpos[left_ee_site_id].copy()
        l_mat = data.site_xmat[left_ee_site_id].reshape(3, 3).copy()
    else:
        l_pos = data.xpos[left_ee_body_id].copy()
        l_mat = data.xmat[left_ee_body_id].reshape(3, 3).copy()

    if right_ee_site_id >= 0:
        r_pos = data.site_xpos[right_ee_site_id].copy()
        r_mat = data.site_xmat[right_ee_site_id].reshape(3, 3).copy()
    else:
        r_pos = data.xpos[right_ee_body_id].copy()
        r_mat = data.xmat[right_ee_body_id].reshape(3, 3).copy()

    poses["left"] = {"pos": l_pos, "rot": l_mat}
    poses["right"] = {"pos": r_pos, "rot": r_mat}
    return poses


def extract_visual_body_poses(
    data: mujoco.MjData,
    visual_geom_precomp: list,
    visual_body_names: list,
    visual_body_ids: list,
) -> dict:
    """
    Returns a dictionary mapping geom ID string -> [x, y, z, qx, qy, qz, qw]
    for all visual geoms with mesh frames aligned, plus body-name keys.
    """
    poses = {}
    for gid, mid, r_mq_T, mpos in visual_geom_precomp:
        gxmat = data.geom_xmat[gid].reshape(3, 3)
        gxpos = data.geom_xpos[gid]
        R_mesh_world = gxmat @ r_mq_T
        P_mesh_world = gxpos - R_mesh_world @ mpos

        m = R_mesh_world
        tr = m[0, 0] + m[1, 1] + m[2, 2]
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * S
            qx = (m[2, 1] - m[1, 2]) / S
            qy = (m[0, 2] - m[2, 0]) / S
            qz = (m[1, 0] - m[0, 1]) / S
        elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            qw = (m[2, 1] - m[1, 2]) / S
            qx = 0.25 * S
            qy = (m[0, 1] + m[1, 0]) / S
            qz = (m[0, 2] + m[2, 0]) / S
        elif m[1, 1] > m[2, 2]:
            S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            qw = (m[0, 2] - m[2, 0]) / S
            qx = (m[0, 1] + m[1, 0]) / S
            qy = 0.25 * S
            qz = (m[1, 2] + m[2, 1]) / S
        else:
            S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            qw = (m[1, 0] - m[0, 1]) / S
            qx = (m[0, 2] + m[2, 0]) / S
            qy = (m[1, 2] + m[2, 1]) / S
            qz = 0.25 * S

        poses[str(gid)] = [
            round(float(P_mesh_world[0]), 5),
            round(float(P_mesh_world[1]), 5),
            round(float(P_mesh_world[2]), 5),
            round(float(qx), 5),
            round(float(qy), 5),
            round(float(qz), 5),
            round(float(qw), 5),
        ]

    for bname, bid in zip(visual_body_names, visual_body_ids):
        poses[bname] = [
            round(float(data.xpos[bid][0]), 4),
            round(float(data.xpos[bid][1]), 4),
            round(float(data.xpos[bid][2]), 4),
            round(float(data.xquat[bid][0]), 4),
            round(float(data.xquat[bid][1]), 4),
            round(float(data.xquat[bid][2]), 4),
            round(float(data.xquat[bid][3]), 4),
        ]
    return poses
