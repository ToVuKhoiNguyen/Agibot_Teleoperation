"""
teleop/kinematics/mink_ik.py
Differential Inverse Kinematics for AGIBOT A2 Ultra using Mink (Quadratic Programming).
Supports:
- State-preserving warm-started QP solver with position-first orientation relaxation
- Real hardware limits, nullspace posture regularizer (cost=0.01), best-effort return
"""

import os
import math
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


# Natural rest pose for the null-space (PostureTask) bias = HOME: elbow 23 cm below and 9 cm outside the shoulder,
# forearm forward. To move the elbow further out, change joint 2 (shoulder roll) of the pose used as nominal_qpos.
# Rest-pose bias weights per arm joint (shoulder pitch, shoulder roll, shoulder yaw, elbow, wrist x3)
REST_WEIGHTS = (0.3, 0.5, 0.4, 0.5, 0.02, 0.02, 0.02)
REST_SCALE = 0.1
# Elbow-out preference (one-sided): when the elbow (arm_link04) is closer than ELBOW_OUT_MIN to the body side of
# the shoulder, a weak spring pulls it toward a point outside and below the shoulder, harder the more it is tucked
# in; at ELBOW_OUT_MIN or further out the spring is off, so postures that are already open (HOME: 9 cm) are left
# alone. Among all arm postures reaching the same hand pose the IK therefore avoids the ones with the elbow against
# the torso. The spring is far weaker than the hand task (position cost 40): the hand target is not traded for it.
# Strength of the pull = COST^2 * GAIN; speed of the resulting self-motion ~ GAIN per IK step (small = gentle).
# Measured on 187 s of real controller motion (13/11): elbow closer than 5 cm to the body side of the shoulder
# 16 % / 21 % of the time (left / right) -> 2 % / 10 %; hand error, per-tick joint step and joint travel unchanged.
# With MIN 0.15 (user choice 13/11, "about 15 cm"): median elbow offset 10 -> 14 cm, tucked time -> 1 % / 8 %, and
# an arm steered from HOME opens to about 12 cm (upper arm turns ~0.6 rad); the Home button's joint pose is unchanged.
# COST 8.5 / GAIN 0.01 = same pull as 6.0 / 0.02 but half the swing speed: from a fully tucked posture the hand then
# wanders 9 mm instead of 20 mm while the limiter follows the elbow out (about 1.5 s).
ELBOW_OUT_COST = 8.5                       # 0 disables
ELBOW_OUT_GAIN = 0.01
ELBOW_OUT_MIN = 0.15                       # m, lateral elbow offset from the shoulder below which the pull acts
                                           # (main.py --elbow-out, cm; 9 = leave HOME-like postures alone)
ELBOW_OUT_OFFSET = (-0.05, 0.32, -0.10)    # pull point from the shoulder: x back, y outward (mirrored on the right), z down


# Soft orientation (translation-only inputs: XYZ nudge, gizmo translate, controller TRANSLATE clutch):
# a person moving the hand somewhere lets the hand turn with the forearm instead of holding its world orientation,
# so the wrist stays straight. Holding the orientation rigidly makes the two small wrist joints (real range only
# +-0.30 / +-0.20 rad) bend to their stops to cancel every rotation of the arm (measured: J7 0.32-0.52 rad after
# 24 cm of nudging from HOME). In soft mode the orientation target is only a weak preference and a wrist-neutral
# task dominates it; position accuracy is unchanged (position cost 40).
SOFT_ORI_COST = 0.1
SOFT_ORI_FULL_PASS = False   # True only when IK is NOT stepped every tick after a step (ik_dispatcher sets it)
WRIST_NEUTRAL_COST = 1.5
WRIST_FREEZE_SOFT = os.environ.get("WRIST_FREEZE_SOFT", "1") != "0"


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
            # Soft-orientation (translation-only) solves also freeze the two wrist joints at the warm start: the hand
            # is placed with J1-J5 and the wrist does not move at all. With the wrist only softly pulled toward a
            # stored reference it kept being commanded (real robot 16/11: wrist command moving 20-74 % of the
            # arm-moving time, measured wrist shake 10x its command), and after Home the stale reference swung the
            # hand ~17 deg on the next trigger press. WRIST_FREEZE_SOFT=0 restores the soft pull.
            self.freeze_wrist_constraint = mink.DofFreezingTask(
                self.model, dof_indices=sorted(frozen_dofs + list(self.dof_indices[5:])))
            self.frame_task = mink.FrameTask(frame_name=self.site_name, frame_type="site",
                                             position_cost=40.0, orientation_cost=3.0, lm_damping=0.10)
            self.posture_task = mink.PostureTask(self.model, cost=0.01)
            self._set_posture_target(self.nominal_qpos)
            # Rest-pose (null-space) bias: the arm has 7 joints for a 6-DOF hand target, so the elbow can swivel
            # while the hand stays on target. Without a fixed reference it drifts (measured: 6.3 cm elbow shift
            # after three identical hand loops). Pull shoulder/elbow softly toward the natural rest pose (HOME:
            # elbow down, slightly out); wrists stay almost free so hand orientation is not fought.
            # Scale 0.1 x [0.3, 0.5, 0.4, 0.5, 0.02, 0.02, 0.02]: drift 0.8 cm, hand error unchanged (0.2 mm);
            # 0.2-0.3 x doubled the solution jumps near the reach limit, so do not raise it without re-testing.
            rest_cost = np.full(self.model.nv, 1e-4)
            for dof, w in zip(self.dof_indices, REST_WEIGHTS): rest_cost[dof] = REST_SCALE * w
            self.rest_task = mink.PostureTask(self.model, cost=rest_cost)
            self.rest_task.set_target_from_configuration(self.conf)   # conf holds the nominal (HOME) arm pose here
            wrist_cost = np.zeros(self.model.nv)
            for dof in self.dof_indices[5:]: wrist_cost[dof] = WRIST_NEUTRAL_COST
            self.wrist_task = mink.PostureTask(self.model, cost=wrist_cost)
            self.wrist_task.set_target_from_configuration(self.conf)  # nominal (HOME) wrist = straight
            self.elbow_task = None
            self.conf.update()
            self._side = 1.0 if self.arm == "left" else -1.0
            self._elbow_bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"{self.arm}_arm_link04")
            sh = self.conf.data.xpos[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"{self.arm}_arm_link02")].copy()
            self._shoulder_y = float(sh[1])
            off = np.array(ELBOW_OUT_OFFSET, dtype=np.float64) * np.array([1.0, self._side, 1.0])
            self.elbow_task = mink.FrameTask(frame_name=f"{self.arm}_arm_link04", frame_type="body",
                                             position_cost=max(ELBOW_OUT_COST, 1e-6), orientation_cost=0.0,
                                             gain=ELBOW_OUT_GAIN, lm_damping=0.0)
            self.elbow_task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3.identity(), sh + off))
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

    def set_wrist_reference(self, wrist_q2: np.ndarray):
        """Wrist angles (J6, J7) the soft-orientation mode holds: 0 = straight, or the bend the user last set."""
        if mink is None or getattr(self, "wrist_task", None) is None: return
        for idx, val, (lo, hi) in zip(self.qpos_indices[5:], wrist_q2, self.limits[5:]):
            self.conf.data.qpos[idx] = float(np.clip(val, lo, hi))
        self.wrist_task.set_target_from_configuration(self.conf)

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
              elbow_hint: np.ndarray = None, soft_orientation: bool = False, wrist_return_step: float = 0.0, **kwargs):
        target_pos = np.asarray(target_pos, dtype=np.float64)
        target_rot = np.asarray(target_rot, dtype=np.float64)
        q_start = np.clip(np.asarray(init_qpos if init_qpos is not None else self.last_qpos, dtype=np.float64),
                          self.limits[:, 0], self.limits[:, 1])
        # Soft orientation with the wrist frozen: the frozen wrist angles are moved toward straight (the nominal
        # wrist) by at most wrist_return_step rad before the solve (ik_dispatcher WRIST_RETURN: step ~ hand travel)
        if wrist_return_step > 0.0 and soft_orientation and WRIST_FREEZE_SOFT:
            q_start = q_start.copy()
            q_start[5:] += np.clip(self.nominal_qpos[5:] - q_start[5:], -wrist_return_step, wrist_return_step)
        if mink is None:
            self.last_ori_relaxed = False
            return q_start, False, 0.0, 0.0, 0

        r_scipy = R.from_matrix(target_rot)
        tq = r_scipy.as_quat()
        target_se3 = mink.SE3.from_rotation_and_translation(
            mink.SO3(np.array([tq[3], tq[0], tq[1], tq[2]])), target_pos
        )
        self.frame_task.set_target(target_se3)
        # explicit hint (homing, hand tracking) -> uniform light posture task toward that hint;
        # no hint (mouse gizmo, Quest controllers) -> weighted rest-pose bias toward the natural pose
        posture = self.posture_task if elbow_hint is not None else self.rest_task
        if elbow_hint is not None: self._set_posture_target(elbow_hint)

        # one-sided elbow-out pull, only where the rest-pose bias is used (mouse gizmo, Quest controllers):
        # weight 1 with the elbow at the body side of the shoulder, falling to 0 at ELBOW_OUT_MIN outward
        elbow_task = None
        if elbow_hint is None and ELBOW_OUT_COST > 0.0 and ELBOW_OUT_MIN > 0.0 and getattr(self, "elbow_task", None) is not None:
            for idx, val in zip(self.qpos_indices, q_start): self.conf.data.qpos[idx] = val
            self.conf.update()
            out = self._side * (float(self.conf.data.xpos[self._elbow_bid][1]) - self._shoulder_y)
            w = min(1.0, max(0.0, (ELBOW_OUT_MIN - out) / ELBOW_OUT_MIN))
            if w > 0.0:
                self.elbow_task.set_position_cost(ELBOW_OUT_COST * math.sqrt(w))   # pull ~ cost^2 -> linear in w
                elbow_task = self.elbow_task

        # Orientation relaxation ladder: nominal (3.0), mild (0.9), loose (0.15), zero (0.0)
        soft = bool(soft_orientation) and SOFT_ORI_COST >= 0.0 and getattr(self, "wrist_task", None) is not None
        ori_schedule = [SOFT_ORI_COST] if soft else [3.0, 0.9, 0.15, 0.0]
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

            tasks = [self.frame_task, posture]
            if elbow_task is not None: tasks.append(elbow_task)
            if soft: tasks.append(self.wrist_task)
            constraints = [self.freeze_wrist_constraint if (soft and WRIST_FREEZE_SOFT) else self.freeze_constraint]
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
                # Position first; once position is within tolerance, keep improving orientation
                # (selecting on p_err alone froze orientation mid-way once position had converged).
                if best_p_err > pos_tol:
                    better = p_err < best_p_err
                elif soft:
                    better = p_err <= pos_tol          # orientation is a preference: keep the latest iterate
                else:
                    better = p_err <= pos_tol and r_err_deg < best_rot_err
                if better:
                    best_p_err = p_err
                    best_rot_err = r_err_deg
                    best_q = np.array([float(self.conf.data.qpos[idx]) for idx in self.qpos_indices])
                    best_q = np.clip(best_q, self.limits[:, 0], self.limits[:, 1])
                    ori_relaxed = (ori_w < 3.0) and not soft

                if soft:
                    if SOFT_ORI_FULL_PASS: continue    # one-shot solve: the wrist must settle within this pass
                    # IK is stepped every control tick (drag, gliding step, posture settling): stop as soon as the
                    # hand is on target and let the wrist / elbow settle over the following ticks. Running all 15
                    # iterations per call made every rest-pose / elbow / wrist pull act 15 times per tick: the
                    # joint target leapt 0.1-0.9 rad per step (measured 13/11), i.e. a jerk at every step.
                    if p_err <= pos_tol:
                        success = True
                        break
                    continue
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
