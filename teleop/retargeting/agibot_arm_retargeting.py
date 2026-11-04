"""
teleop/retargeting/agibot_arm_retargeting.py
Retargets human operator arm poses from WebXR coordinate frame to AGIBOT A2 Ultra robot workspace.
Handles scale adjustment, reference shoulder origin anchoring, and pose smoothing.
"""

import numpy as np
from teleop.utils.weighted_moving_filter import WeightedMovingFilter


class AgibotArmRetargeting:
    """
    Retargets human wrist pose to AGIBOT A2 Ultra bilateral arm workspace.
    """

    def __init__(self, arm: str = "left", scale: float = 1.0, filter_weights=(0.5, 0.3, 0.2)):
        assert arm in ["left", "right"], f"Arm must be 'left' or 'right', got {arm}"
        self.arm = arm
        self.scale = scale

        # Robot shoulder anchors in world frame (from scene.xml base at z=0.93m)
        if self.arm == "left":
            self.shoulder_origin = np.array([-0.028, 0.22, 1.35], dtype=np.float64)
            self.nominal_pos = np.array([0.30, 0.35, 1.15], dtype=np.float64)
        else:
            self.shoulder_origin = np.array([-0.028, -0.22, 1.35], dtype=np.float64)
            self.nominal_pos = np.array([0.30, -0.35, 1.15], dtype=np.float64)

        # Filters for position (3) and rotation quaternion (4)
        self.pos_filter = WeightedMovingFilter(weights=filter_weights, data_size=3)
        self.rot_filter = WeightedMovingFilter(weights=filter_weights, data_size=4)
        self.initialized = False

    def reset(self):
        self.pos_filter.reset()
        self.rot_filter.reset()
        self.initialized = False

    def retarget_pose(
        self,
        xr_wrist_pos: np.ndarray,
        xr_wrist_rot_mat: np.ndarray,
        xr_head_pos: np.ndarray = None
    ) -> tuple:
        """
        Transforms XR wrist pose (Y-up, -Z forward, X-right) to Robot frame (Z-up, X-forward, Y-left).
        Returns:
            target_pos: np.ndarray (3,) in MuJoCo world frame
            target_rot: np.ndarray (3, 3) rotation matrix
        """
        xr_wrist_pos = np.asarray(xr_wrist_pos, dtype=np.float64)
        xr_wrist_rot_mat = np.asarray(xr_wrist_rot_mat, dtype=np.float64)

        # XR to Robot frame conversion matrix R_xr_to_robot:
        # robot_x = -xr_z (forward)
        # robot_y = -xr_x (left)
        # robot_z =  xr_y (up)
        T_mat = np.array([
            [ 0.0,  0.0, -1.0],
            [-1.0,  0.0,  0.0],
            [ 0.0,  1.0,  0.0]
        ], dtype=np.float64)

        if xr_head_pos is not None:
            rel_wrist = xr_wrist_pos - np.asarray(xr_head_pos, dtype=np.float64)
        else:
            rel_wrist = xr_wrist_pos

        robot_rel_pos = T_mat @ rel_wrist * self.scale
        target_pos_raw = self.shoulder_origin + robot_rel_pos

        # Workspace bounding box safety clamping
        target_pos_raw[0] = np.clip(target_pos_raw[0], -0.10, 0.75)   # x (front)
        if self.arm == "left":
            target_pos_raw[1] = np.clip(target_pos_raw[1], 0.10, 0.70)  # y (left)
        else:
            target_pos_raw[1] = np.clip(target_pos_raw[1], -0.70, -0.10) # y (right)
        target_pos_raw[2] = np.clip(target_pos_raw[2], 0.75, 1.65)   # z (height)

        filtered_pos = self.pos_filter.add_data(target_pos_raw)

        # Base rotation transformation
        # Converts operator wrist relative orientation into robot arm workspace
        if self.arm == "left":
            # Left arm mirrored basis
            R_basis = np.array([
                [-0.841,  0.000, -0.540],
                [-0.498,  0.389,  0.775],
                [ 0.210,  0.921, -0.328]
            ], dtype=np.float64)
        else:
            # Right arm basis
            R_basis = np.array([
                [ 0.841,  0.000, -0.540],
                [-0.498, -0.389, -0.775],
                [-0.210,  0.921, -0.328]
            ], dtype=np.float64)

        target_rot_raw = T_mat @ xr_wrist_rot_mat @ T_mat.T @ R_basis

        # Convert rotation to quaternion for smoothing
        tr = np.trace(target_rot_raw)
        if tr > 0.0:
            s = 0.5 / np.sqrt(tr + 1.0)
            w = 0.25 / s
            x = (target_rot_raw[2, 1] - target_rot_raw[1, 2]) * s
            y = (target_rot_raw[0, 2] - target_rot_raw[2, 0]) * s
            z = (target_rot_raw[1, 0] - target_rot_raw[0, 1]) * s
        else:
            w, x, y, z = 1.0, 0.0, 0.0, 0.0

        quat = np.array([w, x, y, z], dtype=np.float64)
        norm_q = np.linalg.norm(quat)
        if norm_q > 1e-6:
            quat /= norm_q

        filtered_quat = self.rot_filter.add_data(quat)
        filtered_quat /= np.linalg.norm(filtered_quat)

        # Convert quaternion back to rotation matrix
        qw, qx, qy, qz = filtered_quat
        filtered_rot = np.array([
            [1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw), 2.0*(qx*qz + qy*qw)],
            [2.0*(qx*qy + qz*qw), 1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw)],
            [2.0*(qx*qz - qy*qw), 2.0*(qy*qz + qx*qw), 1.0 - 2.0*(qx*qx + qy*qy)]
        ], dtype=np.float64)

        return filtered_pos, filtered_rot
