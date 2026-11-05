"""
teleop/simulation/mujoco_env.py
MuJoCo simulation wrapper for AGIBOT A2 Ultra:
- Loads official MJCF scene model
- Handles joint actuation and mimic-coupling for 5-finger hands
- Exposes end-effector and visual geom transforms
"""

import os
import mujoco
import numpy as np

try:
    from mujoco import viewer as mjviewer
except ImportError:
    mjviewer = None

from .env_discovery import discover_joints, discover_actuators, precompute_visual_geoms, propagate_mimics
from .env_poses import extract_ee_poses, extract_visual_body_poses


class AgibotSimEnv:
    """Wrapper around MuJoCo model and data for AGIBOT A2 Ultra teleoperation."""

    def __init__(self, xml_path: str, render_gui: bool = False):
        self.xml_path = xml_path
        self.render_gui = render_gui

        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"MuJoCo XML model file not found at: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        for k, v in discover_joints(self.model).items():
            setattr(self, k, v)
        self.actuator_map = discover_actuators(self.model)

        # End effector sites / bodies
        self.left_ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_arm_ee")
        self.right_ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_arm_ee")
        self.left_ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_hand")
        self.right_ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_hand")

        for k, v in precompute_visual_geoms(self.model).items():
            setattr(self, k, v)

        # Physics options for held standing base
        self.model.opt.iterations = 25
        self.model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_SENSOR

        # Nominal rest postures (arms bent forward in front of chest, hands open, legs standing)
        self.default_arm_qpos_left = np.array([0.35, 0.35, 0.0, -1.20, 0.0, 0.15, 0.0], dtype=np.float64)
        self.default_arm_qpos_right = np.array([0.35, -0.35, 0.0, 1.20, 0.0, 0.15, 0.0], dtype=np.float64)

        # Non-arm, non-hand joint indices to hold fixed at home
        self.non_manip_qpos_indices = [self.qpos_idx_map[j] for j in self.leg_joints + self.head_joints]
        self.non_manip_qvel_indices = [self.qvel_idx_map[j] for j in self.leg_joints + self.head_joints]

        self.arm_qpos_indices = []
        self.arm_dof_indices = []
        for jname in self.left_arm_joints + self.right_arm_joints:
            self.arm_qpos_indices.append(self.qpos_idx_map[jname])
            self.arm_dof_indices.append(self.qvel_idx_map[jname])

        self.viewer = None

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
            else:
                print("[WARN] DISPLAY / WAYLAND_DISPLAY not found in environment. MuJoCo passive viewer cannot be displayed.")

    def reset(self):
        """Resets simulation data and sets default posture using 'home' keyframe."""
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        else:
            mujoco.mj_resetData(self.model, self.data)
            for i, jname in enumerate(self.left_arm_joints):
                self.data.qpos[self.qpos_idx_map[jname]] = self.default_arm_qpos_left[i]
            for i, jname in enumerate(self.right_arm_joints):
                self.data.qpos[self.qpos_idx_map[jname]] = self.default_arm_qpos_right[i]

        self.cached_home_qpos = self.data.qpos.copy()
        for jname in self.left_arm_joints + self.right_arm_joints:
            act_idx = self.actuator_map.get(jname)
            if act_idx is not None:
                self.data.ctrl[act_idx] = self.data.qpos[self.qpos_idx_map[jname]]

        for jname in self.left_hand_active_joints + self.right_hand_active_joints:
            act_idx = self.actuator_map.get(jname)
            if act_idx is not None:
                self.data.ctrl[act_idx] = self.data.qpos[self.qpos_idx_map[jname]]

        mujoco.mj_forward(self.model, self.data)
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def set_arm_commands(self, left_qpos: np.ndarray = None, right_qpos: np.ndarray = None):
        """Commands 7-DoF joint position targets to the position-controlled arm actuators."""
        if left_qpos is not None:
            for i, jname in enumerate(self.left_arm_joints):
                act_idx = self.actuator_map.get(jname)
                if act_idx is not None:
                    low, high = self.joint_limits[jname]
                    target = float(np.clip(left_qpos[i], low, high))
                    self.data.ctrl[act_idx] = target
                    self.data.qpos[self.qpos_idx_map[jname]] = target

        if right_qpos is not None:
            for i, jname in enumerate(self.right_arm_joints):
                act_idx = self.actuator_map.get(jname)
                if act_idx is not None:
                    low, high = self.joint_limits[jname]
                    target = float(np.clip(right_qpos[i], low, high))
                    self.data.ctrl[act_idx] = target
                    self.data.qpos[self.qpos_idx_map[jname]] = target

    def set_hand_commands(self, left_hand: np.ndarray = None, right_hand: np.ndarray = None):
        """Commands 6 active joints per hand and propagates coupled mimic joints."""
        if left_hand is not None:
            for i, jname in enumerate(self.left_hand_active_joints):
                act_idx = self.actuator_map.get(jname)
                if act_idx is not None and i < len(left_hand):
                    low, high = self.joint_limits[jname]
                    val = float(np.clip(left_hand[i], low, high))
                    self.data.ctrl[act_idx] = val
                    self.data.qpos[self.qpos_idx_map[jname]] = val
            propagate_mimics(self.data, self.qpos_idx_map, "L", self.left_hand_mimic_joints)

        if right_hand is not None:
            for i, jname in enumerate(self.right_hand_active_joints):
                act_idx = self.actuator_map.get(jname)
                if act_idx is not None and i < len(right_hand):
                    low, high = self.joint_limits[jname]
                    val = float(np.clip(right_hand[i], low, high))
                    self.data.ctrl[act_idx] = val
                    self.data.qpos[self.qpos_idx_map[jname]] = val
            propagate_mimics(self.data, self.qpos_idx_map, "R", self.right_hand_mimic_joints)

    def set_left_arm_qpos(self, qpos: np.ndarray):
        for i, jname in enumerate(self.left_arm_joints):
            low, high = self.joint_limits[jname]
            val = float(np.clip(qpos[i], low, high))
            self.data.qpos[self.qpos_idx_map[jname]] = val
            act_idx = self.actuator_map.get(jname)
            if act_idx is not None:
                self.data.ctrl[act_idx] = val

    def set_right_arm_qpos(self, qpos: np.ndarray):
        for i, jname in enumerate(self.right_arm_joints):
            low, high = self.joint_limits[jname]
            val = float(np.clip(qpos[i], low, high))
            self.data.qpos[self.qpos_idx_map[jname]] = val
            act_idx = self.actuator_map.get(jname)
            if act_idx is not None:
                self.data.ctrl[act_idx] = val

    def get_left_arm_qpos(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qpos_idx_map[j]] for j in self.left_arm_joints], dtype=np.float64)

    def get_right_arm_qpos(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qpos_idx_map[j]] for j in self.right_arm_joints], dtype=np.float64)

    def get_left_hand_qpos(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qpos_idx_map[j]] for j in self.left_hand_active_joints], dtype=np.float64)

    def get_right_hand_qpos(self) -> np.ndarray:
        return np.array([self.data.qpos[self.qpos_idx_map[j]] for j in self.right_hand_active_joints], dtype=np.float64)

    def get_ee_poses(self) -> dict:
        return extract_ee_poses(
            self.model,
            self.data,
            self.left_ee_site_id,
            self.left_ee_body_id,
            self.right_ee_site_id,
            self.right_ee_body_id,
        )

    def get_visual_body_poses(self) -> dict:
        return extract_visual_body_poses(
            self.data,
            self.visual_geom_precomp,
            self.visual_body_names,
            self.visual_body_ids,
        )

    def step(self, num_substeps: int = 5, hold_arms: bool = False, hold_left_qpos: np.ndarray = None, hold_right_qpos: np.ndarray = None):
        """Steps dynamic simulation forward while holding non-manipulation joints at home.
        If hold_arms is True, rigidly freezes arm joint positions to prevent sagging under gravity."""
        if hold_arms:
            if hold_left_qpos is not None:
                self.set_left_arm_qpos(hold_left_qpos)
            if hold_right_qpos is not None:
                self.set_right_arm_qpos(hold_right_qpos)
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
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None
