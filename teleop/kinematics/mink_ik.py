"""
teleop/kinematics/mink_ik.py
Differential Inverse Kinematics for AGIBOT A2 Ultra using Mink (Quadratic Programming).
Supports:
- State-preserving warm-started QP solver with position-first orientation relaxation
- Real hardware limits, nullspace posture regularizer (cost=0.01), best-effort return
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

REAL_ROBOT_LIMITS = {
    "idx13_left_arm_joint1": (-2.91, 2.91), "idx14_left_arm_joint2": (-0.46, 1.60),
    "idx15_left_arm_joint3": (-2.91, 2.91), "idx16_left_arm_joint4": (-2.00, -0.03),
    "idx17_left_arm_joint5": (-2.88, 2.88), "idx18_left_arm_joint6": (-0.785, 0.785),
    "idx19_left_arm_joint7": (-0.523, 0.523), "idx20_right_arm_joint1": (-2.91, 2.91),
    "idx21_right_arm_joint2": (-1.60, 0.46), "idx22_right_arm_joint3": (-2.91, 2.91),
    "idx23_right_arm_joint4": (0.03, 2.00), "idx24_right_arm_joint5": (-2.88, 2.88),
    "idx25_right_arm_joint6": (-0.785, 0.785), "idx26_right_arm_joint7": (-0.523, 0.523),
}


class AgibotMinkIK:
    def __init__(self, xml_path: str, arm: str = "left", solver: str = "daqp", dt: float = 0.02,
                 use_collision_avoidance: bool = False, use_real_robot_profile: bool = True):
        assert arm in ["left", "right"], f"Arm must be 'left' or 'right', got {arm}"
        self.arm, self.solver, self.dt = arm, solver, dt
        self.use_collision_avoidance = use_collision_avoidance
        self.use_real_robot_profile = use_real_robot_profile
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"Model XML not found: {xml_path}")
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        if self.arm == "left":
            self.joint_names = [f"idx{i}_left_arm_joint{i-12}" for i in range(13, 20)]
            self.site_name, self.nominal_qpos = "left_arm_ee", np.array(HOME[:7], dtype=np.float64)
        else:
            self.joint_names = [f"idx{i}_right_arm_joint{i-19}" for i in range(20, 27)]
            self.site_name, self.nominal_qpos = "right_arm_ee", np.array(HOME[7:], dtype=np.float64)

        self.site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)
        self.qpos_indices = [self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in self.joint_names]
        self.dof_indices = [self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in self.joint_names]

        self.limits = []
        for jname in self.joint_names:
            if self.use_real_robot_profile and jname in REAL_ROBOT_LIMITS:
                self.limits.append(REAL_ROBOT_LIMITS[jname])
            else:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                rng = self.model.jnt_range[jid]
                self.limits.append((float(rng[0]), float(rng[1])))
        self.limits = np.array(self.limits, dtype=np.float64)

        if mink is not None:
            self.conf = mink.Configuration(self.model)
            frozen_dofs = [i for i in range(self.model.nv) if i not in self.dof_indices]
            self.freeze_constraint = mink.DofFreezingTask(self.model, dof_indices=frozen_dofs)
            self.frame_task = mink.FrameTask(frame_name=self.site_name, frame_type="site",
                                             position_cost=40.0, orientation_cost=3.0, lm_damping=0.10)
            self.posture_task = mink.PostureTask(self.model, cost=0.01)
            self._set_posture_target(self.nominal_qpos)
            self.vel_limit = mink.VelocityLimit(self.model, {j: 3.0 for j in self.joint_names})
            if self.use_real_robot_profile:
                for jname, (l_min, l_max) in zip(self.joint_names, self.limits):
                    jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                    self.model.jnt_range[jid] = [l_min, l_max]
            self.config_limit = mink.ConfigurationLimit(self.model)
            self.limits_list = [self.config_limit, self.vel_limit]
        else:
            self.conf = None

        self.last_qpos = self.nominal_qpos.copy()
        self.consecutive_failures = 0

    def _set_posture_target(self, arm_qpos: np.ndarray):
        if mink is None: return
        for idx, val in zip(self.qpos_indices, arm_qpos):
            self.conf.data.qpos[idx] = val
        self.posture_task.set_target_from_configuration(self.conf)

    def set_elbow_hint(self, elbow_target_qpos: np.ndarray):
        self._set_posture_target(elbow_target_qpos)

    def _eval_candidate(self, target_pos: np.ndarray, target_rot: np.ndarray):
        ee_trans = self.conf.get_transform_frame_to_world(self.site_name, "site")
        p_err = float(np.linalg.norm(ee_trans.translation() - target_pos))
        cur_rot = ee_trans.rotation().as_matrix()
        rot_err = target_rot @ cur_rot.T
        angle = float(np.arccos(np.clip((np.trace(rot_err) - 1.0) / 2.0, -1.0, 1.0)))
        return p_err, float(np.degrees(angle))

    def solve(self, target_pos: np.ndarray, target_rot: np.ndarray, init_qpos: np.ndarray = None,
              max_iters: int = 25, pos_tol: float = 0.005, rot_tol: float = 0.040,
              elbow_hint: np.ndarray = None, **kwargs):
        target_pos = np.asarray(target_pos, dtype=np.float64)
        target_rot = np.asarray(target_rot, dtype=np.float64)
        q_start = np.clip(np.asarray(init_qpos if init_qpos is not None else self.last_qpos, dtype=np.float64),
                          self.limits[:, 0], self.limits[:, 1])
        if mink is None:
            self.last_ori_relaxed = False
            return q_start, False, 0.0, 0.0, 0

        r_scipy = R.from_matrix(target_rot)
        tq = r_scipy.as_quat()
        target_se3 = mink.SE3.from_rotation_and_translation(
            mink.SO3(np.array([tq[3], tq[0], tq[1], tq[2]])), target_pos
        )
        self.frame_task.set_target(target_se3)
        self._set_posture_target(elbow_hint if elbow_hint is not None else self.nominal_qpos)

        # Orientation relaxation ladder: nominal (3.0), mild (0.9), loose (0.15), zero (0.0)
        ori_schedule = [3.0, 0.9, 0.15, 0.0]
        best_q = q_start.copy()
        best_p_err = 999.0
        best_rot_err = 180.0
        iters_done = 0
        success = False
        ori_relaxed = False

        for ori_w in ori_schedule:
            self.frame_task.set_position_cost(40.0)
            self.frame_task.set_orientation_cost(ori_w)

            # Sync warm start into configuration
            for idx, val in zip(self.qpos_indices, q_start):
                self.conf.data.qpos[idx] = val
            self.conf.update()

            tasks = [self.frame_task, self.posture_task]
            constraints = [self.freeze_constraint]
            iters_pass = min(max_iters, 15)

            for it in range(iters_pass):
                iters_done += 1
                try:
                    vel = mink.solve_ik(self.conf, tasks, self.dt, self.solver,
                                        damping=0.05, limits=self.limits_list, constraints=constraints)
                except Exception:
                    try:
                        vel = mink.solve_ik(self.conf, tasks, self.dt, "osqp",
                                            damping=0.1, limits=self.limits_list, constraints=constraints)
                    except Exception:
                        break
                self.conf.integrate_inplace(vel, self.dt)
                p_err, r_err_deg = self._eval_candidate(target_pos, target_rot)
                if p_err < best_p_err:
                    best_p_err = p_err
                    best_rot_err = r_err_deg
                    best_q = np.array([float(self.conf.data.qpos[idx]) for idx in self.qpos_indices])
                    best_q = np.clip(best_q, self.limits[:, 0], self.limits[:, 1])
                    ori_relaxed = (ori_w < 3.0)

                if p_err <= pos_tol and (ori_w == 0.0 or np.radians(r_err_deg) <= rot_tol):
                    success = True
                    break

            if success or best_p_err <= pos_tol:
                success = (best_p_err <= pos_tol)
                break

        # Consecutive failure watchdog: re-solve from nominal if stuck
        if best_p_err > 0.020:
            self.consecutive_failures += 1
            if self.consecutive_failures >= 5:
                # Try cold solve from nominal posture
                for idx, val in zip(self.qpos_indices, self.nominal_qpos):
                    self.conf.data.qpos[idx] = val
                self.conf.update()
                for it in range(20):
                    try:
                        vel = mink.solve_ik(self.conf, tasks, self.dt, self.solver,
                                            damping=0.05, limits=self.limits_list, constraints=constraints)
                        self.conf.integrate_inplace(vel, self.dt)
                    except Exception:
                        break
                alt_p_err, alt_r_err = self._eval_candidate(target_pos, target_rot)
                alt_q = np.clip(np.array([float(self.conf.data.qpos[idx]) for idx in self.qpos_indices]),
                                self.limits[:, 0], self.limits[:, 1])
                # Only accept cold solution if step change is safe (< 0.20 rad)
                if alt_p_err < best_p_err and float(np.max(np.abs(alt_q - q_start))) < 0.20:
                    best_q = alt_q
                    best_p_err = alt_p_err
                    best_rot_err = alt_r_err
                self.consecutive_failures = 0
        else:
            self.consecutive_failures = 0

        self.last_qpos = best_q.copy()
        self.last_ori_relaxed = bool(ori_relaxed)
        return best_q, success, best_p_err, best_rot_err, iters_done
