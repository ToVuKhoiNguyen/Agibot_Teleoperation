"""
teleop/simulation/mujoco_env.py
MuJoCo Simulation Environment for AGIBOT A2 Ultra.
Provides joint discovery, position control for arms and dexterous hands,
joint limit enforcement, reset functionality, and end-effector pose tracking.
"""

import os
import time
import numpy as np
import mujoco

try:
    import mujoco.viewer as mjviewer
except ImportError:
    mjviewer = None


class AgibotSimEnv:
    """
    Simulation environment interface for AGIBOT A2 Ultra in MuJoCo.
    """

    def __init__(self, xml_path: str, render_gui: bool = False):
        self.xml_path = xml_path
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"Model file not found: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.dt = self.model.opt.timestep
        self.render_gui = render_gui
        self.viewer = None

        # Joint discovery
        self._discover_joints()

        # Actuator discovery
        self._discover_actuators()

        # End effector sites / bodies
        self.left_ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_arm_ee")
        self.right_ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_arm_ee")
        self.left_ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_hand")
        self.right_ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_hand")

        # Discover visual geoms and bodies for accurate 3D world pose streaming
        self.visual_geom_ids = []
        self.visual_mesh_ids = []
        self.visual_geom_precomp = [] # (gid, mid, r_mq_T, mesh_pos)
        self.visual_body_names = []

        from scipy.spatial.transform import Rotation as _R
        for i in range(self.model.ngeom):
            if self.model.geom_group[i] == 1 and self.model.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
                bid = self.model.geom_bodyid[i]
                bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid)
                if bname not in self.visual_body_names:
                    self.visual_body_names.append(bname)
                
                mid = self.model.geom_dataid[i]
                mq = self.model.mesh_quat[mid]
                r_mq = _R.from_quat([mq[1], mq[2], mq[3], mq[0]]).as_matrix()
                self.visual_geom_ids.append(i)
                self.visual_mesh_ids.append(mid)
                self.visual_geom_precomp.append((i, mid, r_mq.T, self.model.mesh_pos[mid]))

        self.visual_body_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, b) for b in self.visual_body_names]

        # Nominal rest postures (arms bent forward in front of chest, hands open, legs standing)
        self.default_arm_qpos_left = np.array([0.35, 0.35, 0.0, -1.20, 0.0, 0.15, 0.0], dtype=np.float64)
        self.default_arm_qpos_right = np.array([0.35, -0.35, 0.0, 1.20, 0.0, 0.15, 0.0], dtype=np.float64)

        # Non-arm, non-hand joint indices to hold fixed at home
        self.non_manip_qpos_indices = []
        self.non_manip_qvel_indices = []
        for jname in self.leg_joints + self.head_joints:
            self.non_manip_qpos_indices.append(self.qpos_idx_map[jname])
            self.non_manip_qvel_indices.append(self.qvel_idx_map[jname])

        self.arm_qpos_indices = []
        self.arm_qvel_indices = []
        for jname in self.left_arm_joints + self.right_arm_joints:
            self.arm_qpos_indices.append(self.qpos_idx_map[jname])
            self.arm_dof_indices = getattr(self, "arm_dof_indices", [])
            self.arm_dof_indices.append(self.qvel_idx_map[jname])

        # Reset to home keyframe at startup
        self.reset()

        if self.render_gui and mjviewer is not None:
            has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
            if has_display:
                try:
                    self.viewer = mjviewer.launch_passive(self.model, self.data)
                    self.viewer.cam.distance = 2.4
                    self.viewer.cam.azimuth = -135
                    self.viewer.cam.elevation = -18
                    self.viewer.cam.lookat[:] = [0, 0, 0.9]
                except Exception as e:
                    print(f"[WARN] Failed to launch MuJoCo passive viewer: {e}")
                    self.viewer = None
            else:
                print("[WARN] DISPLAY / WAYLAND_DISPLAY not found in environment. MuJoCo passive viewer cannot be displayed.")

    def _discover_joints(self):
        """Automatically categorizes all revolute joints by name."""
        self.left_arm_joints = []
        self.right_arm_joints = []
        self.left_hand_active_joints = []
        self.right_hand_active_joints = []
        self.left_hand_mimic_joints = []
        self.right_hand_mimic_joints = []
        self.leg_joints = []
        self.head_joints = []

        self.joint_limits = {}
        self.joint_id_map = {}
        self.qpos_idx_map = {}
        self.qvel_idx_map = {}

        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            rng = self.model.jnt_range[i]
            qpos_adr = self.model.jnt_qposadr[i]
            qvel_adr = self.model.jnt_dofadr[i]

            self.joint_id_map[name] = i
            self.qpos_idx_map[name] = qpos_adr
            self.qvel_idx_map[name] = qvel_adr
            self.joint_limits[name] = (float(rng[0]), float(rng[1]))

            if "left_arm" in name:
                self.left_arm_joints.append(name)
            elif "right_arm" in name:
                self.right_arm_joints.append(name)
            elif name.startswith("L_"):
                if name.endswith("_2_joint") or name.endswith("_3_joint"):
                    self.left_hand_mimic_joints.append(name)
                else:
                    self.left_hand_active_joints.append(name)
            elif name.startswith("R_"):
                if name.endswith("_2_joint") or name.endswith("_3_joint"):
                    self.right_hand_mimic_joints.append(name)
                else:
                    self.right_hand_active_joints.append(name)
            elif "head" in name:
                self.head_joints.append(name)
            elif "hip" in name or "tarsus" in name or "toe" in name:
                self.leg_joints.append(name)

        # Sort for deterministic index matching
        self.left_arm_joints.sort()
        self.right_arm_joints.sort()
        self.left_hand_active_joints.sort()
        self.right_hand_active_joints.sort()

    def _discover_actuators(self):
        """Maps actuator names and targets to control vector indices."""
        self.actuator_map = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            trnid = self.model.actuator_trnid[i, 0]
            jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, trnid)
            self.actuator_map[jname] = i

    def reset(self):
        """Resets simulation data and sets default posture using 'home' keyframe."""
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        else:
            mujoco.mj_resetData(self.model, self.data)
            self.set_left_arm_qpos(self.default_arm_qpos_left)
            self.set_right_arm_qpos(self.default_arm_qpos_right)

        # Hold all non-manipulation joints (legs, head) strictly at home (0.0)
        if hasattr(self, "non_manip_qpos_indices"):
            self.data.qpos[self.non_manip_qpos_indices] = 0.0
            self.data.qvel[self.non_manip_qvel_indices] = 0.0

        # Sync actuator targets to home posture
        for jname in self.left_arm_joints + self.right_arm_joints:
            if jname in self.actuator_map:
                act_id = self.actuator_map[jname]
                self.data.ctrl[act_id] = self.data.qpos[self.qpos_idx_map[jname]]

        # Hands open (0 radians)
        for jname in self.left_hand_active_joints + self.right_hand_active_joints:
            if jname in self.actuator_map:
                act_id = self.actuator_map[jname]
                self.data.ctrl[act_id] = 0.0

        # Non-manipulation joints hold zero position
        for jname in self.leg_joints + self.head_joints:
            if jname in self.actuator_map:
                self.data.ctrl[self.actuator_map[jname]] = 0.0

        # Run forward kinematics to update positions
        mujoco.mj_forward(self.model, self.data)

        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def set_arm_commands(self, left_qpos: np.ndarray = None, right_qpos: np.ndarray = None):
        """
        Sends position control targets to bilateral arms with joint limit enforcement.
        """
        if left_qpos is not None:
            left_qpos = np.asarray(left_qpos, dtype=np.float64)
            for i, jname in enumerate(self.left_arm_joints):
                if i >= len(left_qpos):
                    break
                lo, hi = self.joint_limits[jname]
                clamped_val = float(np.clip(left_qpos[i], lo, hi))
                if jname in self.actuator_map:
                    self.data.ctrl[self.actuator_map[jname]] = clamped_val

        if right_qpos is not None:
            right_qpos = np.asarray(right_qpos, dtype=np.float64)
            for i, jname in enumerate(self.right_arm_joints):
                if i >= len(right_qpos):
                    break
                lo, hi = self.joint_limits[jname]
                clamped_val = float(np.clip(right_qpos[i], lo, hi))
                if jname in self.actuator_map:
                    self.data.ctrl[self.actuator_map[jname]] = clamped_val

    def set_hand_commands(self, left_hand: np.ndarray = None, right_hand: np.ndarray = None):
        """
        Sends position control targets to the 6 active joints per hand with joint limit enforcement.
        Coupled mimic joints follow automatically via MuJoCo equality constraints.
        Expected order: [thumb_swing, thumb_1, index_1, middle_1, ring_1, pinky_1]
        """
        if left_hand is not None:
            left_hand = np.asarray(left_hand, dtype=np.float64)
            for i, jname in enumerate(self.left_hand_active_joints):
                if i >= len(left_hand):
                    break
                lo, hi = self.joint_limits[jname]
                clamped_val = float(np.clip(left_hand[i], lo, hi))
                if jname in self.actuator_map:
                    self.data.ctrl[self.actuator_map[jname]] = clamped_val

        if right_hand is not None:
            right_hand = np.asarray(right_hand, dtype=np.float64)
            for i, jname in enumerate(self.right_hand_active_joints):
                if i >= len(right_hand):
                    break
                lo, hi = self.joint_limits[jname]
                clamped_val = float(np.clip(right_hand[i], lo, hi))
                if jname in self.actuator_map:
                    self.data.ctrl[self.actuator_map[jname]] = clamped_val

    def set_left_arm_qpos(self, qpos: np.ndarray):
        """Directly writes left arm qpos (used for kinematic initialization)."""
        for i, jname in enumerate(self.left_arm_joints):
            lo, hi = self.joint_limits[jname]
            val = float(np.clip(qpos[i], lo, hi))
            self.data.qpos[self.qpos_idx_map[jname]] = val
            self.data.qvel[self.qvel_idx_map[jname]] = 0.0

    def set_right_arm_qpos(self, qpos: np.ndarray):
        """Directly writes right arm qpos (used for kinematic initialization)."""
        for i, jname in enumerate(self.right_arm_joints):
            lo, hi = self.joint_limits[jname]
            val = float(np.clip(qpos[i], lo, hi))
            self.data.qpos[self.qpos_idx_map[jname]] = val
            self.data.qvel[self.qvel_idx_map[jname]] = 0.0

    def get_left_arm_qpos(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qpos_idx_map[j]] for j in self.left_arm_joints])

    def get_right_arm_qpos(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qpos_idx_map[j]] for j in self.right_arm_joints])

    def get_left_hand_qpos(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qpos_idx_map[j]] for j in self.left_hand_active_joints])

    def get_right_hand_qpos(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qpos_idx_map[j]] for j in self.right_hand_active_joints])

    def get_ee_poses(self) -> dict:
        """
        Returns end-effector translation (x,y,z) and rotation matrix (3x3)
        for both left and right wrists/hands.
        """
        mujoco.mj_kinematics(self.model, self.data)
        poses = {}

        if self.left_ee_site_id >= 0:
            l_pos = self.data.site_xpos[self.left_ee_site_id].copy()
            l_mat = self.data.site_xmat[self.left_ee_site_id].reshape(3, 3).copy()
        else:
            l_pos = self.data.xpos[self.left_ee_body_id].copy()
            l_mat = self.data.xmat[self.left_ee_body_id].reshape(3, 3).copy()

        if self.right_ee_site_id >= 0:
            r_pos = self.data.site_xpos[self.right_ee_site_id].copy()
            r_mat = self.data.site_xmat[self.right_ee_site_id].reshape(3, 3).copy()
        else:
            r_pos = self.data.xpos[self.right_ee_body_id].copy()
            r_mat = self.data.xmat[self.right_ee_body_id].reshape(3, 3).copy()

        poses["left"] = {"pos": l_pos, "rot": l_mat}
        poses["right"] = {"pos": r_pos, "rot": r_mat}
        return poses

    def get_visual_body_poses(self) -> dict:
        """
        Returns a dictionary mapping geom ID string -> [x, y, z, qx, qy, qz, qw]
        for all visual geoms in MuJoCo world frame with mesh coordinate frames aligned.
        Also retains body name keys for backwards-compatibility.
        """
        poses = {}
        for gid, mid, r_mq_T, mpos in self.visual_geom_precomp:
            gxmat = self.data.geom_xmat[gid].reshape(3, 3)
            gxpos = self.data.geom_xpos[gid]
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
                round(float(qw), 5)
            ]

        # Retain body poses for any body-level consumers
        for bname, bid in zip(self.visual_body_names, self.visual_body_ids):
            poses[bname] = [
                round(float(self.data.xpos[bid][0]), 4),
                round(float(self.data.xpos[bid][1]), 4),
                round(float(self.data.xpos[bid][2]), 4),
                round(float(self.data.xquat[bid][0]), 4),
                round(float(self.data.xquat[bid][1]), 4),
                round(float(self.data.xquat[bid][2]), 4),
                round(float(self.data.xquat[bid][3]), 4)
            ]
        return poses

    def step(self, num_substeps: int = 5, hold_arms: bool = False, hold_left_qpos: np.ndarray = None, hold_right_qpos: np.ndarray = None):
        """Steps dynamic simulation forward while holding non-manipulation joints at home.
        If hold_arms is True, rigidly freezes arm joint positions to prevent sagging under gravity."""
        if hold_arms:
            if hold_left_qpos is not None:
                self.set_left_arm_qpos(hold_left_qpos)
            if hold_right_qpos is not None:
                self.set_right_arm_qpos(hold_right_qpos)
            # Sync actuators and clear velocities
            for jname in self.left_arm_joints + self.right_arm_joints:
                if jname in self.actuator_map:
                    self.data.ctrl[self.actuator_map[jname]] = self.data.qpos[self.qpos_idx_map[jname]]
                self.data.qvel[self.qvel_idx_map[jname]] = 0.0

            if hasattr(self, "non_manip_qpos_indices"):
                self.data.qpos[self.non_manip_qpos_indices] = 0.0
                self.data.qvel[self.non_manip_qvel_indices] = 0.0
            mujoco.mj_forward(self.model, self.data)
            if self.viewer is not None and self.viewer.is_running():
                self.viewer.sync()
            return

        for _ in range(num_substeps):
            # Hold non-arm/non-hand joints strictly at neutral standing pose
            if hasattr(self, "non_manip_qpos_indices"):
                self.data.qpos[self.non_manip_qpos_indices] = 0.0
                self.data.qvel[self.non_manip_qvel_indices] = 0.0
            for jname in self.leg_joints + self.head_joints:
                if jname in self.actuator_map:
                    self.data.ctrl[self.actuator_map[jname]] = 0.0

            mujoco.mj_step(self.model, self.data)

            if hasattr(self, "non_manip_qpos_indices"):
                self.data.qpos[self.non_manip_qpos_indices] = 0.0
                self.data.qvel[self.non_manip_qvel_indices] = 0.0

        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

