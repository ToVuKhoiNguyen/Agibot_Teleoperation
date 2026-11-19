"""
teleop/app/joint_preset.py
Planned joint-space motions for Home / T-pose / Arms Forward / Arms Up.

Presets used to jump the Cartesian target and let per-tick IK + the joint limiter find a way there: the T-pose and
Forward targets were 9-11 cm outside the reachable sphere (constant "Target out of reach") and the straight joint
line the limiter took could sweep the arm into the torso (armor block halfway). Here every preset is a fixed,
reachable joint goal solved once from HOME, and the motion is a min-jerk joint trajectory whose whole path is
checked against the armor before it starts (falling back to a path through HOME if the direct one is not clear).
"""

import os
import numpy as np
import mujoco

from teleop.kinematics.reach_projection import SHOULDER_ORIGIN_LEFT, SHOULDER_ORIGIN_RIGHT
from teleop.simulation.env_poses import extract_ee_poses

# Wrist targets (left arm; the right arm mirrors y). "shoulder": offset from the shoulder origin,
# "home": offset from the HOME wrist position. All lie well inside the 0.50 m reach.
PRESET_TARGETS = {
    "tpose": ("shoulder", (0.00, 0.44, -0.05)),    # arms out sideways, just below shoulder height
    "forward": ("shoulder", (0.40, 0.00, -0.15)),  # arms straight ahead, chest height
    "up": ("home", (0.05, 0.00, 0.25)),            # hands raised 25 cm from HOME
}
PATH_CLEARANCE_M = 0.005   # required margin above the armor threshold along the whole planned path
PATH_SAMPLES = 40
MIN_SEGMENT_S = 1.0


class JointPresetPlanner:
    def __init__(self, app):
        self.app = app
        self.model = app.env.model
        self.data = mujoco.MjData(self.model)
        self.data.qpos[:] = app.env.data.qpos
        m = self.model
        jl = [m.joint(f"idx{13 + i}_left_arm_joint{i + 1}").id for i in range(7)]
        jr = [m.joint(f"idx{20 + i}_right_arm_joint{i + 1}").id for i in range(7)]
        self.qadr = {"left": [int(m.jnt_qposadr[j]) for j in jl], "right": [int(m.jnt_qposadr[j]) for j in jr]}
        self.dadr = {"left": [int(m.jnt_dofadr[j]) for j in jl], "right": [int(m.jnt_dofadr[j]) for j in jr]}
        self.lims = {"left": m.jnt_range[jl].copy(), "right": m.jnt_range[jr].copy()}
        self.site = {"left": app.env.left_ee_site_id, "right": app.env.right_ee_site_id}
        # The robot's canonical HOME (same as the Orin receiver), NOT the twin's pose at startup: in real-udp mode
        # the twin may already mirror wherever the real arm stood when main.py started.
        from teleop.robot.aimdk_map import HOME
        self.q_home = np.array(HOME, dtype=np.float64)
        # DATA COLLECTION: canonical HOME with shoulder pitch (J1) raised higher by ~50 deg (+0.8727 rad)
        self.q_data_collection = self.q_home.copy()
        self.q_data_collection[0] += np.deg2rad(50.0)
        self.q_data_collection[7] += np.deg2rad(50.0)
        self._goals = {
            "home": self.q_home.copy(),
            "data_collection": self.q_data_collection.copy(),
            "data_col": self.q_data_collection.copy(),
        }

    # ---------------------------------------------------------------- kinematics
    def _set(self, q14):
        self.data.qpos[self.qadr["left"]] = q14[:7]
        self.data.qpos[self.qadr["right"]] = q14[7:]
        mujoco.mj_kinematics(self.model, self.data)

    def ee_poses(self, q14):
        self._set(q14)
        env = self.app.env
        return extract_ee_poses(self.model, self.data, env.left_ee_site_id, env.left_ee_body_id,
                                env.right_ee_site_id, env.right_ee_body_id)

    def _ik_position(self, target, side, q0, iters=600):
        """Damped least-squares position IK on the wrist site (orientation free), joint limits enforced."""
        q, lim, site, qa, va = q0.copy(), self.lims[side], self.site[side], self.qadr[side], self.dadr[side]
        jac = np.zeros((3, self.model.nv))
        err = np.inf
        for _ in range(iters):
            self.data.qpos[qa] = q
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)
            e = target - self.data.site_xpos[site]
            err = float(np.linalg.norm(e))
            if err < 1e-4:
                break
            mujoco.mj_jacSite(self.model, self.data, jac, None, site)
            J = jac[:, va]
            dq = np.linalg.solve(J.T @ J + 2e-3 * np.eye(7), J.T @ e)
            q = np.clip(q + np.clip(dq, -0.1, 0.1), lim[:, 0], lim[:, 1])
        return q, err

    def goal(self, name):
        if name in self._goals:
            return self._goals[name].copy()
        ref, off = PRESET_TARGETS[name]
        home_ee = self.ee_poses(self.q_home)
        self._set(self.q_home)
        q = self.q_home.copy()
        for side, sl, sign, shoulder in (("left", slice(0, 7), 1.0, SHOULDER_ORIGIN_LEFT),
                                         ("right", slice(7, 14), -1.0, SHOULDER_ORIGIN_RIGHT)):
            base = shoulder if ref == "shoulder" else home_ee[side]["pos"]
            target = np.asarray(base, dtype=np.float64) + np.array([off[0], sign * off[1], off[2]])
            q[sl], err = self._ik_position(target, side, self.q_home[sl])
            if err > 0.01:
                print(f"[PRESET] {name}/{side}: goal {err * 100:.1f} cm from target (kept best effort)")
        self._goals[name] = q
        return q.copy()

    # ---------------------------------------------------------------- planning
    def path_clearance(self, qa, qb):
        armor = getattr(self.app, "armor_collision", None)
        if armor is None or not armor.enabled:
            return np.inf
        return min(armor.compute_min_distance(qa + (qb - qa) * s)[0] for s in np.linspace(0.0, 1.0, PATH_SAMPLES))

    def plan(self, q_start, q_goal):
        """Waypoints [q_start, ..., q_goal] with an armor-clear straight joint segment between each pair."""
        armor = getattr(self.app, "armor_collision", None)
        need = (armor.threshold_m + PATH_CLEARANCE_M) if armor is not None else 0.0
        if self.path_clearance(q_start, q_goal) >= need:
            return [q_start, q_goal]
        via = self.q_home
        if not np.allclose(via, q_goal) and not np.allclose(via, q_start) \
                and self.path_clearance(q_start, via) >= need and self.path_clearance(via, q_goal) >= need:
            print("[PRESET] direct joint path touches the armor -> planned via HOME")
            return [q_start, via, q_goal]
        print("[PRESET] no armor-clear path found; moving directly (armor will hold the arm if needed)")
        return [q_start, q_goal]


def _planner(app):
    p = getattr(app, "_preset_planner", None)
    if p is None:
        p = app._preset_planner = JointPresetPlanner(app)
    return p


# Planned moves (Home button, presets) travel no faster than this, whatever --speed is: hand vibration on the real
# robot grows with joint speed, and nobody is waiting on these moves the way a hand-following arm is. 0.8 rad/s =
# speed level 3, the level confirmed smooth on 13/11. HOME_MAX_VEL=1.5 lets Home use the full level-5 speed again.
HOME_MAX_VEL = float(os.environ.get("HOME_MAX_VEL", "0.8"))


def _segment_time(app, qa, qb):
    lim = getattr(app, "mirror_limiter", None)
    vmax = float(getattr(lim, "max_vel", 0.6) or 0.6)
    if HOME_MAX_VEL > 0.0: vmax = min(vmax, HOME_MAX_VEL)
    # min-jerk peak speed = 1.875 x mean: keep the peak at ~90 % of the limiter speed so the limiter never lags
    return max(MIN_SEGMENT_S, 1.875 * float(np.max(np.abs(qb - qa))) / (0.9 * vmax))


def stop_auto_motion(app):
    """Stops any automatic arm motion (planned preset/home, wave/circle/grasp sequence, short IK tracking)."""
    app.homing_active = False
    app.joint_motion = None
    app.active_sequence = None
    app.preset_track_until = 0.0
    app.gizmo_active_l = app.gizmo_active_r = False


def start_joint_motion(app, q_goal14, side="both", label="preset"):
    """Plans and starts a min-jerk joint trajectory to q_goal14 (only `side` arms move)."""
    planner = _planner(app)
    q_start = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right]).astype(np.float64)
    goal = q_start.copy()
    if side in ("left", "both"):
        goal[:7] = q_goal14[:7]
    if side in ("right", "both"):
        goal[7:] = q_goal14[7:]
    wps = planner.plan(q_start, goal)
    segs = [(wps[i], wps[i + 1], _segment_time(app, wps[i], wps[i + 1])) for i in range(len(wps) - 1)]
    app.joint_motion = {"segs": segs, "t": 0.0, "side": side, "label": label}
    app.homing_side = side
    app.homing_duration_current = sum(s[2] for s in segs)
    app.homing_elapsed = 0.0
    app.active_sequence = None
    app.preset_track_until = 0.0
    app.hand_sweep_active = False
    if side in ("left", "both"):
        app.xr_clutch_state_l, app.xr_grip_active_l, app.gizmo_active_l = "LOCKED", False, True
    if side in ("right", "both"):
        app.xr_clutch_state_r, app.xr_grip_active_r, app.gizmo_active_r = "LOCKED", False, True
    app.reach_warning_l = app.reach_warning_r = app.reach_warning = ""
    app.homing_active = True
    msg = f"{label}: planned joint motion ({side}, {len(segs)} segment(s), {app.homing_duration_current:.1f}s)"
    if hasattr(app, "_log_xr_event"):
        app._log_xr_event(msg)
    print(f"[PRESET] {msg}")


def start_preset(app, name, side="both"):
    start_joint_motion(app, _planner(app).goal(name), side=side, label=name)


def _apply_q(app, q14, side):
    planner = _planner(app)
    ee = planner.ee_poses(q14)
    for a, c, sl in (("left", "l", slice(0, 7)), ("right", "r", slice(7, 14))):
        if side not in (a, "both"):
            continue
        q = q14[sl].copy()
        setattr(app, f"last_arm_cmd_{a}", q.copy())
        setattr(app, f"last_ik_sol_{a}", q.copy())
        pos, rot = ee[a]["pos"].copy(), ee[a]["rot"].copy()
        setattr(app, f"cur_target_pos_{c}", pos)
        setattr(app, f"cur_target_rot_{c}", rot)
        setattr(app, f"_last_ee_target_{c}", pos.copy())
        setattr(app, f"_last_ee_rot_{c}", rot.copy())
        for name, val in ((f"gizmo_follower_{c}", pos), (f"gizmo_rot_follower_{c}", rot)):
            f = getattr(app, name, None)
            if f is not None:
                f.reset(val)


def update_joint_motion(app, dt):
    jm = getattr(app, "joint_motion", None)
    if not jm:
        app.homing_active = False
        return
    jm["t"] += dt
    app.homing_elapsed = jm["t"]
    t = jm["t"]
    q = jm["segs"][-1][1]
    done = True
    for qa, qb, dur in jm["segs"]:
        if t < dur:
            tau = t / dur
            s = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
            q = qa + (qb - qa) * s
            done = False
            break
        t -= dur
    if hasattr(app, "target_tracer"):
        app.target_tracer.record_target_source(jm["side"], "home/smooth-home")
    _apply_q(app, q, jm["side"])
    if done:
        app.joint_motion = None
        app.homing_active = False
        if jm["side"] in ("left", "both"):
            app.gizmo_active_l = False
        if jm["side"] in ("right", "both"):
            app.gizmo_active_r = False
        print(f"[PRESET] {jm['label']} reached")
