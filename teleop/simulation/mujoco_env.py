"""
teleop/simulation/mujoco_env.py - MuJoCo simulation wrapper for AGIBOT A2 Ultra.
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
from teleop.robot.aimdk_map import HOME
from .task_scene import TaskScene


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
        self.default_arm_qpos_left = np.array(HOME[:7], dtype=np.float64)
        self.default_arm_qpos_right = np.array(HOME[7:], dtype=np.float64)

        # Non-arm, non-hand joint indices to hold fixed at standing home
        self.non_manip_qpos_indices = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in self.leg_joints + self.head_joints]
        self.non_manip_qvel_indices = [self.qvel_idx_map[j] for j in self.leg_joints + self.head_joints]
        st_legs = {"idx03_left_hip_pitch": -0.30, "idx04_left_tarsus": 0.60, "idx05_left_toe_pitch": -0.30, "idx09_right_hip_pitch": -0.30, "idx10_right_tarsus": 0.60, "idx11_right_toe_pitch": -0.30}
        self.non_manip_home_qpos = np.array([st_legs.get(j, 0.0) for j in self.leg_joints + self.head_joints], dtype=np.float64)

        self.arm_qpos_indices = []
        self.arm_dof_indices = []
        for jname in self.left_arm_joints + self.right_arm_joints:
            self.arm_qpos_indices.append(self.qpos_idx_map[jname])
            self.arm_dof_indices.append(self.qvel_idx_map[jname])

        # Table collision solver structures
        self._table_geoms = set([
            g for g in range(self.model.ngeom)
            if "task_table" in mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.model.geom_bodyid[g])
        ])
        self._arm_geoms = set([
            g for g in range(self.model.ngeom)
            if any(k in mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, self.model.geom_bodyid[g])
                   for k in ["arm", "thumb", "index", "middle", "ring", "pinky"])
        ])
        self._jacp_buf = np.zeros((3, self.model.nv))

        self.viewer = None
        self.task = TaskScene.detect(self.model)     # practice scene with task_obj_* bodies (scene_pick_bin.xml), else None
        if self.task is not None: print(f"[TASK] practice scene: {len(self.task.objs)} objects, grasp assist on", flush=True)

        # Offscreen camera renderer for simulated head camera
        self.camera_name = "head_front_cam" if mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "head_front_cam") >= 0 else None
        self._renderer = None
        self._cam_seq = 0

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

        if hasattr(self, "non_manip_qpos_indices"):
            self.data.qpos[self.non_manip_qpos_indices] = self.non_manip_home_qpos
        if getattr(self, "task", None) is not None: self.task.reset(self.data)
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

    def reset_objects(self):
        """Resets practice scene task objects back to their initial positions without resetting robot joints."""
        if getattr(self, "task", None) is not None:
            self.task.reset(self.data)
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
        mujoco.mj_forward(self.model, self.data)

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
        mujoco.mj_forward(self.model, self.data)

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

    def set_head_qpos(self, yaw: float = 0.0, pitch: float = 0.0):
        """Sets head yaw (idx27) and pitch (idx28) in radians and updates hold state."""
        combined = getattr(self, "leg_joints", []) + getattr(self, "head_joints", [])
        if "idx27_head_joint1" in self.qpos_idx_map:
            low, high = self.joint_limits.get("idx27_head_joint1", (-0.785, 0.785))
            val = float(np.clip(yaw, low, high))
            if "idx27_head_joint1" in combined and hasattr(self, "non_manip_home_qpos"):
                self.non_manip_home_qpos[combined.index("idx27_head_joint1")] = val
            self.data.qpos[self.qpos_idx_map["idx27_head_joint1"]] = val
            act_idx = self.actuator_map.get("idx27_head_joint1")
            if act_idx is not None:
                self.data.ctrl[act_idx] = val
        if "idx28_head_joint2" in self.qpos_idx_map:
            low, high = self.joint_limits.get("idx28_head_joint2", (-0.401, 0.401))
            val = float(np.clip(pitch, low, high))
            if "idx28_head_joint2" in combined and hasattr(self, "non_manip_home_qpos"):
                self.non_manip_home_qpos[combined.index("idx28_head_joint2")] = val
            self.data.qpos[self.qpos_idx_map["idx28_head_joint2"]] = val
            act_idx = self.actuator_map.get("idx28_head_joint2")
            if act_idx is not None:
                self.data.ctrl[act_idx] = val
        mujoco.mj_forward(self.model, self.data)
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

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

    def resolve_table_penetration(self, max_iters: int = 8, margin_m: float = 0.001):
        """Resolves any penetration between table surface and arm/hand geoms by projecting joint positions."""
        if not self._table_geoms or not self._arm_geoms:
            return
        for _ in range(max_iters):
            mujoco.mj_forward(self.model, self.data)
            worst_c = None
            min_dist = 0.0
            for i in range(self.data.ncon):
                c = self.data.contact[i]
                if (c.geom1 in self._table_geoms and c.geom2 in self._arm_geoms) or (c.geom2 in self._table_geoms and c.geom1 in self._arm_geoms):
                    if c.dist < min_dist:
                        min_dist = c.dist
                        worst_c = c
            if worst_c is None or min_dist >= -0.0005:
                break
            robot_geom = worst_c.geom2 if worst_c.geom1 in self._table_geoms else worst_c.geom1
            robot_body = self.model.geom_bodyid[robot_geom]
            mujoco.mj_jac(self.model, self.data, self._jacp_buf, None, worst_c.pos, robot_body)
            # Dofs corresponding to the arm
            bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, robot_body)
            is_right = ("right" in bname) or ("R_" in bname)
            active_dofs = self.arm_dof_indices[7:] if is_right else self.arm_dof_indices[:7]
            active_qpos = self.arm_qpos_indices[7:] if is_right else self.arm_qpos_indices[:7]

            J_z = self._jacp_buf[2, active_dofs]
            denom = float(np.dot(J_z, J_z)) + 1e-6
            delta_z = -min_dist + margin_m
            dq = J_z * (delta_z / denom)
            for j_idx, q_adr in enumerate(active_qpos):
                self.data.qpos[q_adr] += dq[j_idx]

        mujoco.mj_forward(self.model, self.data)

    def step(self, num_substeps: int = 5, hold_arms: bool = False, hold_left_qpos: np.ndarray = None, hold_right_qpos: np.ndarray = None):
        """Steps dynamic simulation forward while holding non-manipulation joints at home.
        If hold_arms is True, rigidly freezes arm joint positions to prevent sagging under gravity."""
        if hold_arms:
            if hold_left_qpos is not None:
                self.set_left_arm_qpos(hold_left_qpos)
            if hold_right_qpos is not None:
                self.set_right_arm_qpos(hold_right_qpos)
            self.resolve_table_penetration()
            for jname in self.left_arm_joints + self.right_arm_joints:
                if jname in self.actuator_map:
                    self.data.ctrl[self.actuator_map[jname]] = self.data.qpos[self.qpos_idx_map[jname]]
                self.data.qvel[self.qvel_idx_map[jname]] = 0.0

            if hasattr(self, "non_manip_qpos_indices"):
                self.data.qpos[self.non_manip_qpos_indices] = self.non_manip_home_qpos
                self.data.qvel[self.non_manip_qvel_indices] = 0.0
            if self.task is not None: self.task.step(self.data)     # objects: grasp assist + real-time physics
            else: mujoco.mj_forward(self.model, self.data)
            if self.viewer is not None and self.viewer.is_running():
                self.viewer.sync()
            return

        for _ in range(num_substeps):
            if hasattr(self, "non_manip_qpos_indices"):
                self.data.qpos[self.non_manip_qpos_indices] = self.non_manip_home_qpos
                self.data.qvel[self.non_manip_qvel_indices] = 0.0
            for i, jname in enumerate(self.leg_joints + self.head_joints):
                if jname in self.actuator_map:
                    self.data.ctrl[self.actuator_map[jname]] = self.non_manip_home_qpos[i]

            mujoco.mj_step(self.model, self.data)

            if hasattr(self, "non_manip_qpos_indices"):
                self.data.qpos[self.non_manip_qpos_indices] = self.non_manip_home_qpos
                self.data.qvel[self.non_manip_qvel_indices] = 0.0

        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def render_head_camera(self, width: int = 640, height: int = 480):
        """Renders RGB and depth images from head_front_cam if available.
        Returns: (rgb_jpeg_bytes, depth_png_bytes, seq) or (None, None, 0)
        """
        if not self.camera_name:
            return None, None, 0
        try:
            import cv2
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, height=height, width=width)
            self._renderer.disable_depth_rendering()
            self._renderer.update_scene(self.data, camera=self.camera_name)
            rgb = self._renderer.render()

            self._renderer.enable_depth_rendering()
            self._renderer.update_scene(self.data, camera=self.camera_name)
            depth_m = self._renderer.render()

            # Encode RGB as JPEG
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            ok_rgb, rgb_bytes = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])

            # Encode depth in mm as 16-bit PNG
            depth_mm = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
            ok_depth, depth_bytes = cv2.imencode(".png", depth_mm)

            self._cam_seq += 1
            seq = self._cam_seq
            return (rgb_bytes.tobytes() if ok_rgb else None,
                    depth_bytes.tobytes() if ok_depth else None,
                    seq)
        except Exception as e:
            return None, None, 0

    def close(self):
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None
