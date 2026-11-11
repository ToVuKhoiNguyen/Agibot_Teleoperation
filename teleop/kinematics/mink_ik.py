"""
teleop/kinematics/mink_ik.py
Differential Inverse Kinematics for AGIBOT A2 Ultra using Mink (Quadratic Programming).
Supports:
- State-preserving warm-started differential QP solver
- Real robot hardware velocity and position limits
- Natural posture regularization and human elbow hints
- Singularity handling with automatic orientation relaxation
- Self-collision avoidance limit (optional)
"""

import os
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R

try:
    import mink
except ImportError:
    mink = None

from teleop.robot.aimdk_map import HOME


# AimDK & hardware joint limits profile (rad) from physical manual Sec 3.4
REAL_ROBOT_LIMITS = {
    "idx13_left_arm_joint1": (-2.91, 2.91),
    "idx14_left_arm_joint2": (-0.46, 1.60),
    "idx15_left_arm_joint3": (-2.91, 2.91),
    "idx16_left_arm_joint4": (-2.00, -0.03),
    "idx17_left_arm_joint5": (-2.88, 2.88),
    "idx18_left_arm_joint6": (-0.785, 0.785), # Wrist pitch: ±45 deg (physical limit)
    "idx19_left_arm_joint7": (-0.523, 0.523), # Wrist yaw: ±30 deg (physical limit)
    "idx20_right_arm_joint1": (-2.91, 2.91),
    "idx21_right_arm_joint2": (-1.60, 0.46),
    "idx22_right_arm_joint3": (-2.91, 2.91),
    "idx23_right_arm_joint4": (0.03, 2.00),
    "idx24_right_arm_joint5": (-2.88, 2.88),
    "idx25_right_arm_joint6": (-0.785, 0.785), # Wrist pitch: ±45 deg (physical limit)
    "idx26_right_arm_joint7": (-0.523, 0.523), # Wrist yaw: ±30 deg (physical limit)
}


class AgibotMinkIK:
    """
    Mink-based QP Differential Inverse Kinematics for AGIBOT A2 Ultra 7-DoF Arm.
    """

    def __init__(
        self,
        xml_path: str,
        arm: str = "left",
        solver: str = "daqp",
        dt: float = 0.02,
        use_collision_avoidance: bool = False,
        use_real_robot_profile: bool = True
    ):
        assert arm in ["left", "right"], f"Arm must be 'left' or 'right', got {arm}"
        self.arm = arm
        self.xml_path = xml_path
        self.solver = solver
        self.dt = dt
        self.use_collision_avoidance = use_collision_avoidance
        self.use_real_robot_profile = use_real_robot_profile

        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"Model XML not found: {xml_path}")

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # Arm joints and site
        if self.arm == "left":
            self.joint_names = [f"idx{i}_left_arm_joint{i-12}" for i in range(13, 20)]
            self.site_name = "left_arm_ee"
            self.elbow_site_or_body = "left_arm_link04"
            self.nominal_qpos = np.array(HOME[:7], dtype=np.float64)
        else:
            self.joint_names = [f"idx{i}_right_arm_joint{i-19}" for i in range(20, 27)]
            self.site_name = "right_arm_ee"
            self.elbow_site_or_body = "right_arm_link04"
            self.nominal_qpos = np.array(HOME[7:], dtype=np.float64)

        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)
        self.qpos_indices = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in self.joint_names]
        self.dof_indices = [self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in self.joint_names]

        # Extract joint limits
        self.limits = []
        for jname in self.joint_names:
            if self.use_real_robot_profile and jname in REAL_ROBOT_LIMITS:
                self.limits.append(REAL_ROBOT_LIMITS[jname])
            else:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                rng = self.model.jnt_range[jid]
                self.limits.append((float(rng[0]), float(rng[1])))
        self.limits = np.array(self.limits, dtype=np.float64)

        # Mink configuration setup
        if mink is not None:
            self.conf = mink.Configuration(self.model)
            # Freeze all non-arm DOFs
            frozen_dofs = [i for i in range(self.model.nv) if i not in self.dof_indices]
            self.freeze_constraint = mink.DofFreezingTask(self.model, dof_indices=frozen_dofs)

            # Frame task for end-effector
            self.frame_task = mink.FrameTask(
                frame_name=self.site_name,
                frame_type="site",
                position_cost=30.0,
                orientation_cost=3.0,
                lm_damping=0.10
            )

            # Posture task for elbow redundancy and natural posture
            self.posture_task = mink.PostureTask(self.model, cost=1.00)
            self._set_posture_target(self.nominal_qpos)

            # Hardware limits as hard constraints
            vel_limit_dict = {j: 3.0 for j in self.joint_names}
            self.vel_limit = mink.VelocityLimit(self.model, vel_limit_dict)

            # Configuration limits (using active safety profile)
            if self.use_real_robot_profile:
                # Update model joint ranges for configuration limits
                for jname, (l_min, l_max) in zip(self.joint_names, self.limits):
                    jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                    self.model.jnt_range[jid] = [l_min, l_max]
            self.config_limit = mink.ConfigurationLimit(self.model)

            self.limits_list = [self.config_limit, self.vel_limit]
            if self.use_collision_avoidance and hasattr(mink, "CollisionAvoidanceLimit"):
                try:
                    from teleop.utils.collision_checker import CollisionChecker
                    checker = CollisionChecker(self.model)
                    arm_geoms = (checker.left_arm_geoms + checker.left_hand_geoms) if self.arm == "left" else (checker.right_arm_geoms + checker.right_hand_geoms)
                    geom_pairs = []
                    if checker.torso_geoms:
                        geom_pairs.append((checker.torso_geoms, arm_geoms))
                    if checker.head_geoms:
                        geom_pairs.append((checker.head_geoms, arm_geoms))
                    self.collision_limit = mink.CollisionAvoidanceLimit(
                        self.model,
                        geom_pairs=geom_pairs,
                        minimum_distance_from_collisions=0.005,
                        collision_detection_distance=0.025
                    )
                    self.limits_list.append(self.collision_limit)
                except Exception as e:
                    print(f"[WARN] Failed to initialize mink CollisionAvoidanceLimit: {e}")
        else:
            self.conf = None

        self.last_qpos = self.nominal_qpos.copy()
        self.clamped_lower = np.zeros(len(self.joint_names), dtype=bool)
        self.clamped_upper = np.zeros(len(self.joint_names), dtype=bool)

    def _set_posture_target(self, arm_qpos: np.ndarray):
        """Sets target for posture task with arm joints at arm_qpos and all others at home."""
        if mink is None:
            return
        # Set arm joints
        for idx, val in zip(self.qpos_indices, arm_qpos):
            self.conf.data.qpos[idx] = val
        self.posture_task.set_target_from_configuration(self.conf)

    def set_elbow_hint(self, elbow_target_qpos: np.ndarray):
        """Soft elbow hint for 7-DoF arm posture."""
        self._set_posture_target(elbow_target_qpos)

    def solve(
        self,
        target_pos: np.ndarray,
        target_rot: np.ndarray,
        init_qpos: np.ndarray = None,
        max_iters: int = 25,
        pos_tol: float = 0.005,
        rot_tol: float = 0.035,
        elbow_hint: np.ndarray = None,
        **kwargs
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
            q_cur = self.last_qpos.copy()

        # Enforce hard limits on warm start
        q_cur = np.clip(q_cur, self.limits[:, 0], self.limits[:, 1])

        if mink is None:
            # Fallback if mink unavailable
            return q_cur, False, 0.0, 0.0, 0

        # Sync warm start into Mink configuration
        for idx, val in zip(self.qpos_indices, q_cur):
            self.conf.data.qpos[idx] = val
        self.conf.update()

        # Update posture task target
        if elbow_hint is not None:
            self._set_posture_target(elbow_hint)
        else:
            self._set_posture_target(self.nominal_qpos)

        # Convert target to Mink SE3
        r_scipy = R.from_matrix(target_rot)
        target_quat = r_scipy.as_quat() # [x, y, z, w]
        mink_quat = np.array([target_quat[3], target_quat[0], target_quat[1], target_quat[2]]) # [w, x, y, z]
        target_se3 = mink.SE3.from_rotation_and_translation(mink.SO3(mink_quat), target_pos)

        # Automatic relaxation of orientation weight when target is stretched / near-unreachable
        ee_home_dist = float(np.linalg.norm(target_pos - np.array([0.0, 0.0, 1.25])))
        if ee_home_dist > 0.70:
            # Stretched arm: prioritize reaching target position smoothly over orientation rigidity
            self.frame_task.position_cost = 40.0
            self.frame_task.orientation_cost = 0.5
        else:
            self.frame_task.position_cost = 30.0
            self.frame_task.orientation_cost = 3.0

        self.frame_task.set_target(target_se3)

        # Solve differential QP steps
        iters_done = 0
        success = False
        pos_err = 1.0
        rot_err_deg = 180.0

        tasks = [self.frame_task, self.posture_task]
        constraints = [self.freeze_constraint]

        for it in range(max_iters):
            iters_done += 1
            try:
                vel = mink.solve_ik(
                    configuration=self.conf,
                    tasks=tasks,
                    limits=self.limits_list,
                    constraints=constraints,
                    dt=self.dt,
                    solver=self.solver,
                    damping=0.05
                )
            except Exception:
                # Fallback solver if primary encountered an issue
                try:
                    vel = mink.solve_ik(
                        configuration=self.conf,
                        tasks=tasks,
                        limits=self.limits_list,
                        constraints=constraints,
                        dt=self.dt,
                        solver="osqp",
                        damping=0.1
                    )
                except Exception:
                    break

            self.conf.integrate_inplace(vel, self.dt)

            # Check convergence
            ee_trans = self.conf.get_transform_frame_to_world(self.site_name, "site")
            pos_err = float(np.linalg.norm(ee_trans.translation() - target_pos))
            # Orientation error
            cur_rot_mat = ee_trans.rotation().as_matrix()
            rot_err_mat = target_rot @ cur_rot_mat.T
            trace = float(np.trace(rot_err_mat))
            angle = float(np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0)))
            rot_err_deg = float(np.degrees(angle))

            if pos_err <= pos_tol and angle <= rot_tol:
                success = True
                break

        if not success:
            # Hold last valid solution bit-identical on failure; reset configuration drift
            q_sol = np.array(init_qpos, dtype=np.float64).copy()
            for idx, val in zip(self.qpos_indices, q_sol):
                self.conf.data.qpos[idx] = val
            self.last_qpos = q_sol.copy()
            return q_sol, False, pos_err, rot_err_deg, iters_done

        # Extract solved arm joints
        q_sol = np.array([float(self.conf.data.qpos[idx]) for idx in self.qpos_indices], dtype=np.float64)

        # Joint-limit clamping with hysteresis (enter at limit, leave when > 0.005 rad inside)
        for i in range(len(q_sol)):
            l_min, l_max = self.limits[i, 0], self.limits[i, 1]
            if self.clamped_lower[i]:
                if q_sol[i] >= l_min + 0.005: self.clamped_lower[i] = False
                else: q_sol[i] = l_min
            elif q_sol[i] <= l_min:
                self.clamped_lower[i] = True
                q_sol[i] = l_min

            if self.clamped_upper[i]:
                if q_sol[i] <= l_max - 0.005: self.clamped_upper[i] = False
                else: q_sol[i] = l_max
            elif q_sol[i] >= l_max:
                self.clamped_upper[i] = True
                q_sol[i] = l_max

        self.last_qpos = q_sol.copy()
        return q_sol, success, pos_err, rot_err_deg, iters_done
