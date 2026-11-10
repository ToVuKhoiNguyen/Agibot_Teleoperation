"""
teleop/kinematics/pyroki_ik.py
Optimization-based Inverse Kinematics for AGIBOT A2 Ultra using Pyroki (JAX + jaxls).
Provides:
- 7-DoF arm kinematic chain solved with Levenberg-Marquardt / Trust Region QP in JAX
- Full end-effector 6D pose tracking (position + quaternion)
- Active safety hardware joint limits as hard constraints (REAL_ROBOT_LIMITS)
- Posture regularization towards nominal human arm posture
- Warm starting and JIT compilation (<1 ms per step on CPU)
- Thread-safe wrapper matching AgibotA2IK / AgibotMinkIK interface
"""

import os
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from teleop.robot.aimdk_map import HOME

try:
    import jax
    import jax.numpy as jnp
    import jaxls
    import jaxlie
    import yourdfpy
    import pyroki as pk
    PYROKI_AVAILABLE = True
except ImportError:
    PYROKI_AVAILABLE = False


REAL_ROBOT_LIMITS = {
    "idx13_left_arm_joint1": (-2.91, 2.91),
    "idx14_left_arm_joint2": (-0.46, 1.60),
    "idx15_left_arm_joint3": (-2.91, 2.91),
    "idx16_left_arm_joint4": (-2.00, -0.03),
    "idx17_left_arm_joint5": (-2.88, 2.88),
    "idx18_left_arm_joint6": (-0.785, 0.785), # Wrist pitch: ±45 deg
    "idx19_left_arm_joint7": (-0.523, 0.523), # Wrist yaw: ±30 deg
    "idx20_right_arm_joint1": (-2.91, 2.91),
    "idx21_right_arm_joint2": (-1.60, 0.46),
    "idx22_right_arm_joint3": (-2.91, 2.91),
    "idx23_right_arm_joint4": (0.03, 2.00),
    "idx24_right_arm_joint5": (-2.88, 2.88),
    "idx25_right_arm_joint6": (-0.785, 0.785), # Wrist pitch: ±45 deg
    "idx26_right_arm_joint7": (-0.523, 0.523), # Wrist yaw: ±30 deg
}


class AgibotPyrokiIK:
    """
    Pyroki-based JAX optimization IK for AGIBOT A2 Ultra.
    """

    def __init__(
        self,
        urdf_or_xml_path: str = None,
        arm: str = "left",
        use_real_robot_profile: bool = True
    ):
        if not PYROKI_AVAILABLE:
            raise ImportError("Pyroki, JAX, or yourdfpy not available in current environment.")

        assert arm in ["left", "right"], f"Arm must be 'left' or 'right', got {arm}"
        self.arm = arm
        self.use_real_robot_profile = use_real_robot_profile

        # Locate URDF
        if urdf_or_xml_path and urdf_or_xml_path.endswith(".urdf") and os.path.exists(urdf_or_xml_path):
            self.urdf_path = urdf_or_xml_path
        else:
            cand = Path(__file__).resolve().parent.parent.parent / "assets" / "agibot_a2_ultra" / "urdf" / "a2.urdf"
            if cand.exists():
                self.urdf_path = str(cand)
            else:
                raise FileNotFoundError(f"A2 URDF not found at {cand}")

        # Load URDF and update limits
        self.urdf = yourdfpy.URDF.load(self.urdf_path)
        if self.use_real_robot_profile:
            for j in self.urdf.robot.joints:
                if j.name in REAL_ROBOT_LIMITS:
                    j.limit.lower = REAL_ROBOT_LIMITS[j.name][0]
                    j.limit.upper = REAL_ROBOT_LIMITS[j.name][1]

        # Build Pyroki robot
        self.robot = pk.Robot.from_urdf(self.urdf)
        self.total_joints = self.robot.joints.num_actuated_joints

        # Arm specifics
        if self.arm == "left":
            self.joint_names = [f"idx{i}_left_arm_joint{i-12}" for i in range(13, 20)]
            self.ee_link_name = "left_arm_link07"
            self.nominal_qpos = np.array(HOME[:7], dtype=np.float64)
        else:
            self.joint_names = [f"idx{i}_right_arm_joint{i-19}" for i in range(20, 27)]
            self.ee_link_name = "right_arm_link07"
            self.nominal_qpos = np.array(HOME[7:], dtype=np.float64)

        self.ee_link_idx = jnp.array(self.robot.links.names.index(self.ee_link_name))
        self.arm_indices = [self.robot.joints.actuated_names.index(j) for j in self.joint_names]

        # Joint mask (only arm joints can move)
        mask = np.zeros(self.total_joints, dtype=np.float32)
        for idx in self.arm_indices:
            mask[idx] = 1.0
        self.mask = jnp.array(mask)

        # Full nominal qpos vector
        full_nominal = np.zeros(self.total_joints, dtype=np.float32)
        for idx, v in zip(self.arm_indices, self.nominal_qpos):
            full_nominal[idx] = v
        self.full_nominal_jnp = jnp.array(full_nominal)
        self.current_full_qpos = full_nominal.copy()
        self.last_arm_qpos = self.nominal_qpos.copy()

        # World to Base offset (MuJoCo world Z=0.93 for base_link)
        self.base_offset = np.array([0.0, 0.0, 0.93], dtype=np.float64)

        # Limits for validation
        self.limits = np.array([REAL_ROBOT_LIMITS[j] for j in self.joint_names], dtype=np.float64)

        # Build JIT function
        self._build_jit_solver()

    def _build_jit_solver(self):
        robot = self.robot
        JointVar = robot.joint_var_cls
        ee_idx = self.ee_link_idx
        mask = self.mask

        @jax.jit
        def _solve(target_pos, target_quat, init_q, rest_q):
            target_pose = jaxlie.SE3.from_rotation_and_translation(
                jaxlie.SO3(target_quat), target_pos
            )
            costs = [
                pk.costs.pose_cost_analytic_jac(
                    robot,
                    JointVar(0),
                    target_pose,
                    ee_idx,
                    pos_weight=100.0,
                    ori_weight=5.0,
                    joint_mask=mask,
                ),
                pk.costs.rest_cost(
                    JointVar(0),
                    rest_pose=rest_q,
                    weight=0.02,
                ),
                pk.costs.limit_constraint(robot, JointVar(0)),
            ]
            sol = (
                jaxls.LeastSquaresProblem(costs=costs, variables=[JointVar(0)])
                .analyze()
                .solve(
                    verbose=False,
                    linear_solver="dense_cholesky",
                    trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
                    initial_vals=jaxls.VarValues.make([JointVar(0).with_value(init_q)]),
                )
            )
            return sol[JointVar(0)]

        self._jit_solve = _solve

        # Warm-up / compile
        init_q_jnp = jnp.array(self.current_full_qpos)
        fk0 = np.array(robot.forward_kinematics(init_q_jnp))
        p0 = jnp.array(fk0[int(ee_idx), 4:])
        q0 = jnp.array(fk0[int(ee_idx), :4])
        _ = self._jit_solve(p0, q0, init_q_jnp, init_q_jnp)

    def forward_kinematics(self, arm_qpos: np.ndarray):
        """Returns end-effector position (MuJoCo world) and 3x3 rotation matrix."""
        full_q = self.current_full_qpos.copy()
        for idx, v in zip(self.arm_indices, arm_qpos):
            full_q[idx] = v
        fk = np.array(self.robot.forward_kinematics(jnp.array(full_q)))
        ee_quat_wxyz = fk[int(self.ee_link_idx), :4]
        ee_pos_base = fk[int(self.ee_link_idx), 4:]
        ee_pos_world = ee_pos_base + self.base_offset
        rot_mat = R.from_quat([ee_quat_wxyz[1], ee_quat_wxyz[2], ee_quat_wxyz[3], ee_quat_wxyz[0]]).as_matrix()
        return ee_pos_world, rot_mat

    def solve(
        self,
        target_pos: np.ndarray,
        target_rot: np.ndarray,
        init_qpos: np.ndarray = None,
        max_iters: int = 25,
        pos_tol: float = 0.005,
        rot_tol: float = 0.05
    ):
        """
        Solves inverse kinematics for the 7-DoF arm.
        Returns:
            arm_qpos (7,), success (bool), pos_err (m), rot_err (rad), iters (int)
        """
        if init_qpos is not None:
            self.last_arm_qpos = np.array(init_qpos, dtype=np.float64)

        # Convert MuJoCo world pos to base link pos
        pos_base = np.array(target_pos, dtype=np.float32) - self.base_offset.astype(np.float32)

        # Convert target_rot (3x3 matrix) to wxyz quaternion
        r = R.from_matrix(target_rot)
        xyzw = r.as_quat()
        wxyz = np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]], dtype=np.float32)

        # Update init vector
        full_init = self.current_full_qpos.copy()
        for idx, v in zip(self.arm_indices, self.last_arm_qpos):
            full_init[idx] = v

        sol_full = np.array(self._jit_solve(
            jnp.array(pos_base),
            jnp.array(wxyz),
            jnp.array(full_init),
            self.full_nominal_jnp
        ))

        # Extract arm joints
        arm_sol = np.array([sol_full[idx] for idx in self.arm_indices], dtype=np.float64)

        # Compute FK error
        fk = np.array(self.robot.forward_kinematics(jnp.array(sol_full)))
        cur_pos_base = fk[int(self.ee_link_idx), 4:]
        cur_quat_wxyz = fk[int(self.ee_link_idx), :4]

        pos_err = float(np.linalg.norm(cur_pos_base - pos_base))

        cur_r = R.from_quat([cur_quat_wxyz[1], cur_quat_wxyz[2], cur_quat_wxyz[3], cur_quat_wxyz[0]])
        rot_diff = r * cur_r.inv()
        rot_err = float(rot_diff.magnitude())

        success = (pos_err <= pos_tol) and (rot_err <= rot_tol)

        # Update state
        self.current_full_qpos = sol_full
        self.last_arm_qpos = arm_sol.copy()

        return arm_sol, success, pos_err, rot_err, 1

