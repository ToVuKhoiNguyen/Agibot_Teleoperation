"""
teleop/utils/collision_checker.py
Comprehensive self-collision detection and avoidance for AGIBOT A2 Ultra.
Facade coordinating safety checking across:
- teleop.safety.pair_builder (geom discovery and pair pairing)
- teleop.safety.arm_safety (whole-body FK distance check and path filtering)
- teleop.safety.hand_safety (intra-hand FK distance check and line search)
"""

import os
import yaml
import numpy as np
import mujoco

from teleop.safety.pair_builder import (
    discover_collision_geoms,
    build_collision_pairs,
    build_hand_collision_pairs,
)
from teleop.safety.arm_safety import (
    compute_min_distance_impl,
    check_configuration_impl,
    filter_step_impl,
    project_target_tangent_impl,
)
from teleop.safety.hand_safety import (
    check_hand_collision_impl,
    filter_hand_step_impl,
)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "safety.yaml")


class CollisionChecker:
    def __init__(self, model: mujoco.MjModel, config_path: str = None, enabled: bool = True):
        self.model = model
        self.scratch_data = mujoco.MjData(model)
        self.enabled = enabled
        self.fromto = np.zeros(6, dtype=np.float64)

        # Load config
        self.config_path = config_path or CONFIG_PATH
        self.safety_margin = 0.02
        self.real_robot_margin = 0.03
        self.substeps = 5
        self.dist_max = 0.50
        self.pair_margins = {}

        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r") as f:
                    cfg = yaml.safe_load(f) or {}
                sc_cfg = cfg.get("self_collision", {})
                self.safety_margin = float(sc_cfg.get("safety_margin", 0.02))
                self.substeps = int(sc_cfg.get("interpolation_substeps", 5))
                self.dist_max = float(sc_cfg.get("dist_max", 0.50))
                self.pair_margins = cfg.get("pair_margins", {})
                sd_cfg = cfg.get("shadow_check", {})
                self.shadow_horizon = int(sd_cfg.get("horizon_steps", 10))
                self.shadow_inflation_margin = float(sd_cfg.get("inflation_margin", 0.03))
                self.shadow_check_enabled = bool(sd_cfg.get("enabled", True))
            except Exception as e:
                print(f"[WARN] Failed to load safety.yaml: {e}")

        self._discover_collision_geoms()
        self._build_collision_pairs()
        self._build_hand_collision_pairs()
        self.last_min_dist = 999.0
        self.last_min_pair = (None, None, "none")
        self.step_idx = 0

        # Discover hand joint addresses for FK checks
        self.hand_active_jnt_adrs = {}
        self.hand_mimic_jnt_adrs = {}
        for s in ("left", "right"):
            p = "L_" if s == "left" else "R_"
            act_names = [f"{p}thumb_swing_joint", f"{p}thumb_1_joint", f"{p}index_1_joint", f"{p}middle_1_joint", f"{p}ring_1_joint", f"{p}pinky_1_joint"]
            mim_names = [f"{p}thumb_2_joint", f"{p}thumb_3_joint", f"{p}index_2_joint", f"{p}middle_2_joint", f"{p}ring_2_joint", f"{p}pinky_2_joint"]
            self.hand_active_jnt_adrs[s] = [
                self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)]
                for j in act_names
            ]
            self.hand_mimic_jnt_adrs[s] = [
                self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)]
                for j in mim_names
            ]

    def _discover_collision_geoms(self):
        d = discover_collision_geoms(self.model)
        self.torso_geoms = d["torso_geoms"]
        self.head_geoms = d["head_geoms"]
        self.left_arm_geoms = d["left_arm_geoms"]
        self.left_hand_geoms = d["left_hand_geoms"]
        self.right_arm_geoms = d["right_arm_geoms"]
        self.right_hand_geoms = d["right_hand_geoms"]

    def _build_collision_pairs(self):
        self.pairs = build_collision_pairs(
            self.torso_geoms,
            self.head_geoms,
            self.left_arm_geoms,
            self.left_hand_geoms,
            self.right_arm_geoms,
            self.right_hand_geoms,
        )

    def _build_hand_collision_pairs(self):
        self.hand_pairs = build_hand_collision_pairs(self.model)

    def compute_min_distance(self, data: mujoco.MjData = None, force: bool = False) -> tuple[float, tuple[int, int, str]]:
        self.step_idx += 1
        d = data if data is not None else self.scratch_data
        min_dist, min_pair = compute_min_distance_impl(
            self.model,
            self.pairs,
            d,
            self.dist_max,
            self.fromto,
            last_min_dist=self.last_min_dist,
            last_min_pair=self.last_min_pair,
            step_idx=self.step_idx,
            force=force,
        )
        self.last_min_dist = float(min_dist)
        self.last_min_pair = min_pair
        return float(min_dist), min_pair

    def check_configuration(self, qpos: np.ndarray, data: mujoco.MjData = None) -> tuple[bool, float, tuple]:
        d = data if data is not None else self.scratch_data
        return check_configuration_impl(
            self.model,
            d,
            self.pairs,
            self.pair_margins,
            self.safety_margin,
            self.dist_max,
            self.fromto,
            qpos,
            enabled=self.enabled,
        )

    def predict_clearance(self, future_poses_q14: list, left_adr: list, right_adr: list) -> tuple[float, tuple]:
        """Evaluates minimum predicted distance across N future forward-simulated poses."""
        min_pred_dist, min_pred_pair = 999.0, (None, None, "none")
        for q14 in future_poses_q14:
            self.scratch_data.qpos[left_adr] = q14[:7]
            self.scratch_data.qpos[right_adr] = q14[7:]
            mujoco.mj_forward(self.model, self.scratch_data)
            d, pair = self.compute_min_distance(data=self.scratch_data, force=True)
            if d < min_pred_dist:
                min_pred_dist, min_pred_pair = d, pair
                if min_pred_dist < 0.005: break
        return float(min_pred_dist), min_pred_pair

    def filter_step(
        self,
        current_qpos: np.ndarray,
        target_qpos: np.ndarray,
        data: mujoco.MjData = None
    ) -> tuple[np.ndarray, bool, float]:
        d = data if data is not None else self.scratch_data
        return filter_step_impl(
            self.model,
            d,
            self.pairs,
            self.pair_margins,
            self.safety_margin,
            self.dist_max,
            self.fromto,
            self.substeps,
            current_qpos,
            target_qpos,
            enabled=self.enabled,
        )

    def project_target_tangent(
        self,
        current_ee_pos: np.ndarray,
        target_ee_pos: np.ndarray,
        data: mujoco.MjData = None,
        safety_buffer: float = None
    ) -> tuple[np.ndarray, bool]:
        d = data if data is not None else self.scratch_data
        return project_target_tangent_impl(
            self.model,
            d,
            self.pairs,
            self.pair_margins,
            self.safety_margin,
            self.dist_max,
            self.fromto,
            current_ee_pos,
            target_ee_pos,
            safety_buffer=safety_buffer,
            enabled=self.enabled,
        )

    def check_hand_collision(
        self,
        side: str,
        active_qpos: np.ndarray,
        margin: float = 0.001,
        data: mujoco.MjData = None
    ) -> tuple[bool, float, tuple]:
        return check_hand_collision_impl(
            self.model,
            self.scratch_data,
            self.hand_pairs,
            self.hand_active_jnt_adrs,
            self.hand_mimic_jnt_adrs,
            side,
            active_qpos,
            margin=margin,
            data=data,
            dist_max=self.dist_max,
            fromto=self.fromto,
            enabled=self.enabled,
        )

    def filter_hand_step(
        self,
        side: str,
        current_active_qpos: np.ndarray,
        target_active_qpos: np.ndarray,
        margin: float = 0.001,
        data: mujoco.MjData = None
    ) -> tuple[np.ndarray, bool, float]:
        return filter_hand_step_impl(
            self.model,
            self.scratch_data,
            self.hand_pairs,
            self.hand_active_jnt_adrs,
            self.hand_mimic_jnt_adrs,
            side,
            current_active_qpos,
            target_active_qpos,
            margin=margin,
            data=data,
            dist_max=self.dist_max,
            fromto=self.fromto,
            enabled=self.enabled,
        )
