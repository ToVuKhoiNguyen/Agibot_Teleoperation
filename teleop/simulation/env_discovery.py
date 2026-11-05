"""
teleop/simulation/env_discovery.py
Model structure discovery and precomputations for AgibotSimEnv:
- Joint categorization (arms, hands active/mimic, legs, head)
- Actuator mapping
- Visual geom coordinate alignment precomputation
"""

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R


def discover_joints(model: mujoco.MjModel):
    """Categorizes all revolute joints by name and maps indices."""
    left_arm_joints = []
    right_arm_joints = []
    left_hand_active_joints = []
    right_hand_active_joints = []
    left_hand_mimic_joints = []
    right_hand_mimic_joints = []
    leg_joints = []
    head_joints = []

    joint_limits = {}
    joint_id_map = {}
    qpos_idx_map = {}
    qvel_idx_map = {}

    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        rng = model.jnt_range[i]
        qpos_adr = model.jnt_qposadr[i]
        qvel_adr = model.jnt_dofadr[i]

        joint_id_map[name] = i
        qpos_idx_map[name] = qpos_adr
        qvel_idx_map[name] = qvel_adr
        joint_limits[name] = (float(rng[0]), float(rng[1]))

        if "left_arm" in name:
            left_arm_joints.append(name)
        elif "right_arm" in name:
            right_arm_joints.append(name)
        elif name.startswith("L_"):
            if name.endswith("_2_joint") or name.endswith("_3_joint"):
                left_hand_mimic_joints.append(name)
            else:
                left_hand_active_joints.append(name)
        elif name.startswith("R_"):
            if name.endswith("_2_joint") or name.endswith("_3_joint"):
                right_hand_mimic_joints.append(name)
            else:
                right_hand_active_joints.append(name)
        elif "head" in name:
            head_joints.append(name)
        elif "hip" in name or "tarsus" in name or "toe" in name:
            leg_joints.append(name)

    # Sort for deterministic index matching (arms alphabetically, hands canonically)
    left_arm_joints.sort()
    right_arm_joints.sort()
    hand_order = ["thumb_swing", "thumb_1", "index_1", "middle_1", "ring_1", "pinky_1"]
    left_hand_active_joints.sort(key=lambda j: next((i for i, k in enumerate(hand_order) if k in j), 99))
    right_hand_active_joints.sort(key=lambda j: next((i for i, k in enumerate(hand_order) if k in j), 99))

    return {
        "left_arm_joints": left_arm_joints,
        "right_arm_joints": right_arm_joints,
        "left_hand_active_joints": left_hand_active_joints,
        "right_hand_active_joints": right_hand_active_joints,
        "left_hand_mimic_joints": left_hand_mimic_joints,
        "right_hand_mimic_joints": right_hand_mimic_joints,
        "leg_joints": leg_joints,
        "head_joints": head_joints,
        "joint_limits": joint_limits,
        "joint_id_map": joint_id_map,
        "qpos_idx_map": qpos_idx_map,
        "qvel_idx_map": qvel_idx_map,
    }


def discover_actuators(model: mujoco.MjModel) -> dict:
    """Maps actuator names and targets to control vector indices."""
    actuator_map = {}
    for i in range(model.nu):
        trnid = model.actuator_trnid[i, 0]
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, trnid)
        actuator_map[jname] = i
    return actuator_map


def precompute_visual_geoms(model: mujoco.MjModel):
    """Discovers visual geoms and precomputes mesh orientation alignment matrices."""
    visual_geom_ids = []
    visual_mesh_ids = []
    visual_geom_precomp = []  # (gid, mid, r_mq_T, mesh_pos)
    visual_body_names = []

    for i in range(model.ngeom):
        if model.geom_group[i] == 1 and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
            bid = model.geom_bodyid[i]
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
            if bname not in visual_body_names:
                visual_body_names.append(bname)

            mid = model.geom_dataid[i]
            mq = model.mesh_quat[mid]
            r_mq = R.from_quat([mq[1], mq[2], mq[3], mq[0]]).as_matrix()
            visual_geom_ids.append(i)
            visual_mesh_ids.append(mid)
            visual_geom_precomp.append((i, mid, r_mq.T, model.mesh_pos[mid]))

    visual_body_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b) for b in visual_body_names]
    return {
        "visual_geom_ids": visual_geom_ids,
        "visual_mesh_ids": visual_mesh_ids,
        "visual_geom_precomp": visual_geom_precomp,
        "visual_body_names": visual_body_names,
        "visual_body_ids": visual_body_ids,
    }


def propagate_mimics(data: mujoco.MjData, qpos_idx_map: dict, prefix: str, mimic_joints: list):
    """Propagates active joint positions to coupled mimic finger joints."""
    for mimic_name in mimic_joints:
        if "thumb" in mimic_name:
            active_name = f"{prefix}_thumb_1_joint"
            ratio = 0.40
        elif "index" in mimic_name:
            active_name = f"{prefix}_index_1_joint"
            ratio = 1.00
        elif "middle" in mimic_name:
            active_name = f"{prefix}_middle_1_joint"
            ratio = 1.00
        elif "ring" in mimic_name:
            active_name = f"{prefix}_ring_1_joint"
            ratio = 1.00
        elif "pinky" in mimic_name:
            active_name = f"{prefix}_pinky_1_joint"
            ratio = 1.00
        else:
            continue
        act_val = data.qpos[qpos_idx_map[active_name]]
        data.qpos[qpos_idx_map[mimic_name]] = act_val * ratio

