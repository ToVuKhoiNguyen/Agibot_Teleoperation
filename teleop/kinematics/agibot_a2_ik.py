"""
teleop/kinematics/agibot_a2_ik.py
Damped Least Squares (DLS) Inverse Kinematics for AGIBOT A2 Ultra Bilateral Arms (7-DoF each).
Features null-space posture regularization, joint limit clamping, and high-frequency convergence (>100 Hz).
"""

import os
import time
import numpy as np
import mujoco


class AgibotA2IK:
    """
    Inverse Kinematics solver for AGIBOT A2 Ultra left and right arms.
    """

    def __init__(self, xml_path: str, arm: str = "left"):
        assert arm in ["left", "right"], f"Arm must be 'left' or 'right', got {arm}"
        self.arm = arm
        self.xml_path = xml_path

        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"Model XML not found: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # Setup arm joints and site
        if self.arm == "left":
            self.joint_names = [f"idx{i}_left_arm_joint{i-12}" for i in range(13, 20)]
            self.site_name = "left_arm_ee"
            self.nominal_qpos = np.array([0.35, 0.35, 0.0, -1.20, 0.0, 0.15, 0.0], dtype=np.float64)
        else:
            self.joint_names = [f"idx{i}_right_arm_joint{i-19}" for i in range(20, 27)]
            self.site_name = "right_arm_ee"
            self.nominal_qpos = np.array([0.35, -0.35, 0.0, 1.20, 0.0, 0.15, 0.0], dtype=np.float64)

        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)
        if self.site_id < 0:
            raise ValueError(f"Site '{self.site_name}' not found in model {xml_path}")

        self.qpos_indices = []
        self.dof_indices = []
        self.limits = []

        for j in self.joint_names:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            if jid < 0:
                raise ValueError(f"Joint '{j}' not found in model {xml_path}")
            self.qpos_indices.append(self.model.jnt_qposadr[jid])
            self.dof_indices.append(self.model.jnt_dofadr[jid])
            rng = self.model.jnt_range[jid]
            self.limits.append([float(rng[0]), float(rng[1])])

        self.limits = np.array(self.limits, dtype=np.float64)
        self.jac_pos = np.zeros((3, self.model.nv), dtype=np.float64)
        self.jac_rot = np.zeros((3, self.model.nv), dtype=np.float64)

    def forward_kinematics(self, qpos: np.ndarray):
        """Computes end-effector position (3,) and rotation matrix (3,3) for given arm qpos."""
        for idx, val in zip(self.qpos_indices, qpos):
            self.data.qpos[idx] = val
        mujoco.mj_forward(self.model, self.data)
        pos = self.data.site_xpos[self.site_id].copy()
        rot = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
        return pos, rot

    def solve(
        self,
        target_pos: np.ndarray,
        target_rot: np.ndarray,
        init_qpos: np.ndarray = None,
        max_iters: int = 40,
        pos_tol: float = 0.005,      # 5 mm
        rot_tol: float = 0.0349,     # 2 degrees
        damping: float = 0.02,
        step_size: float = 0.7,
        null_space_weight: float = 0.05
    ):
        """
        Solves IK for target position and rotation matrix.
        Returns:
            q_sol: np.ndarray (7,)
            success: bool
            pos_err_norm: float (meters)
            rot_err_deg: float (degrees)
            iters: int
        """
        target_pos = np.asarray(target_pos, dtype=np.float64)
        target_rot = np.asarray(target_rot, dtype=np.float64)

        if init_qpos is not None:
            q_cur = np.asarray(init_qpos, dtype=np.float64).copy()
        else:
            q_cur = self.nominal_qpos.copy()

        # Clamp initial state to limits
        q_cur = np.clip(q_cur, self.limits[:, 0], self.limits[:, 1])

        pos_err_norm = 1.0
        rot_err_deg = 180.0
        success = False

        for it in range(max_iters):
            # Update robot state in local data
            for idx, val in zip(self.qpos_indices, q_cur):
                self.data.qpos[idx] = val
            mujoco.mj_forward(self.model, self.data)

            cur_pos = self.data.site_xpos[self.site_id]
            cur_rot = self.data.site_xmat[self.site_id].reshape(3, 3)

            # Translation error
            e_pos = target_pos - cur_pos

            # Orientation error in axis-angle form
            r_err_mat = target_rot @ cur_rot.T
            trace = float(np.trace(r_err_mat))
            angle = float(np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0)))

            if angle < 1e-6:
                e_rot = np.zeros(3, dtype=np.float64)
            else:
                e_rot = (angle / (2.0 * np.sin(angle))) * np.array([
                    r_err_mat[2, 1] - r_err_mat[1, 2],
                    r_err_mat[0, 2] - r_err_mat[2, 0],
                    r_err_mat[1, 0] - r_err_mat[0, 1]
                ], dtype=np.float64)

            pos_err_norm = float(np.linalg.norm(e_pos))
            rot_err_norm = float(np.linalg.norm(e_rot))
            rot_err_deg = float(np.degrees(rot_err_norm))

            # Acceptance check
            if pos_err_norm <= pos_tol and rot_err_norm <= rot_tol:
                success = True
                return q_cur, success, pos_err_norm, rot_err_deg, it + 1

            # Compute Jacobian
            mujoco.mj_jacSite(self.model, self.data, self.jac_pos, self.jac_rot, self.site_id)
            J_pos = self.jac_pos[:, self.dof_indices]
            J_rot = self.jac_rot[:, self.dof_indices]
            J = np.vstack([J_pos, J_rot])  # 6x7

            e = np.concatenate([e_pos, e_rot])  # 6

            # Damped Least Squares
            JJt = J @ J.T
            diag_damp = (damping ** 2) * np.eye(6)
            J_dls = J.T @ np.linalg.solve(JJt + diag_damp, np.eye(6))

            # Null-space projection toward natural posture
            null_bias = null_space_weight * (self.nominal_qpos - q_cur)
            dq_primary = J_dls @ e
            dq_null = (np.eye(7) - J_dls @ J) @ null_bias

            dq = dq_primary + dq_null
            dq = np.clip(dq, -0.4, 0.4)

            q_cur += step_size * dq
            q_cur = np.clip(q_cur, self.limits[:, 0], self.limits[:, 1])

        # Final evaluation
        return q_cur, success, pos_err_norm, rot_err_deg, max_iters


AgibotA2ArmIK = AgibotA2IK
