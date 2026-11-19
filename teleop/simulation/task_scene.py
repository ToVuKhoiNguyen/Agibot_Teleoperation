"""
teleop/simulation/task_scene.py - Physics for the task objects of a practice scene (scene_pick_bin.xml).

The teleop simulation is kinematic: arms and hands are written straight into qpos and only mj_forward runs. A practice
task needs objects that fall, rest on the table and drop into a bin, so this module steps real physics for the free
bodies named task_obj_* while the robot is held exactly where teleop put it (robot qpos restored after every substep,
the robot does not collide with anything).

Grasp = assist, not friction: a hand that closes (four-finger closure >= CLOSE_ON) with an object inside its grip
radius picks it up - the object then follows the hand rigidly. Opening the hand (closure <= OPEN_AT) releases it and
it falls under gravity. Good enough to practise reach / grasp / carry / release over the bin; not a contact model.
"""
import time
import mujoco
import numpy as np

CLOSE_ON, OPEN_AT = 0.45, 0.30          # four-finger closure (0 open .. 1 closed), hysteresis
GRIP_RADIUS = 0.055                     # m, object centre to the grip point (between thumb and fingertips)
FINGER_LIMIT = 1.30                     # rad, finger closure range used by the retargeting (SIM_HAND_COMFORTABLE_LIMITS)


def _mat_to_quat(m):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(m, dtype=np.float64).reshape(9)); return q


class TaskScene:
    """Created by AgibotSimEnv when the model has task_obj_* bodies; None otherwise (normal scene untouched)."""

    @staticmethod
    def detect(model):
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "" for b in range(model.nbody)]
        return TaskScene(model) if any(n.startswith("task_obj_") for n in names) else None

    def __init__(self, model):
        self.model = model
        self.objs = []                                   # (name, body id, qpos adr, dof adr, half height for the grip test)
        for b in range(model.nbody):
            n = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if not n.startswith("task_obj_"): continue
            j = model.body_jntadr[b]
            self.objs.append((n, b, int(model.jnt_qposadr[j]), int(model.jnt_dofadr[j])))
        obj_q = {a + k for _, _, a, _ in self.objs for k in range(7)}
        obj_v = {a + k for _, _, _, a in self.objs for k in range(6)}
        self.robot_q = np.array([i for i in range(model.nq) if i not in obj_q], dtype=int)
        self.robot_v = np.array([i for i in range(model.nv) if i not in obj_v], dtype=int)
        bid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
        self.hands = {}
        for side, p in (("left", "L"), ("right", "R")):
            ts = bid(f"{p}_thumb_swing")                  # the hand (palm) = parent body of the thumb (right_arm_link07)
            hb = int(model.body_parentid[ts]) if ts >= 0 else -1
            tips = [bid(f"{p}_{f}") for f in ("thumb_3", "index_2", "middle_2", "ring_2")]
            fingers = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{p}_{f}_1_joint")]
                       for f in ("index", "middle", "ring", "pinky")]
            if hb >= 0 and min(tips) >= 0: self.hands[side] = (hb, tips, fingers)
        self.held = {"left": None, "right": None}        # side -> (object index, rel pos, rel rot) in the hand frame
        self.grasped = {"left": False, "right": False}
        self.init_q = None
        self.last_t = None

    # ---- helpers -------------------------------------------------------------------------------------------------
    def closure(self, data, side):
        _, _, fingers = self.hands[side]
        return float(np.clip(np.mean([data.qpos[a] for a in fingers]) / FINGER_LIMIT, 0.0, 1.0))

    def grip_point(self, data, side):
        _, tips, _ = self.hands[side]
        return np.mean([data.xpos[t] for t in tips], axis=0)

    def reset(self, data):
        """Objects back to the positions of the scene file (called from AgibotSimEnv.reset)."""
        if self.init_q is None: self.init_q = {a: self.model.qpos0[a:a + 7].copy() for _, _, a, _ in self.objs}
        for _, _, a, v in self.objs:
            data.qpos[a:a + 7] = self.init_q[a]; data.qvel[v:v + 6] = 0.0
        self.held = {"left": None, "right": None}; self.grasped = {"left": False, "right": False}
        self.last_t = None

    def status(self, data):
        """Short text for logs: objects in the bin / held."""
        inside = [n[9:] for n, b, _, _ in self.objs if self.in_bin(data, b)]
        held = {s: self.objs[h[0]][0][9:] for s, h in self.held.items() if h is not None}
        return f"in bin {len(inside)}/{len(self.objs)} {inside} held {held}"

    # ---- web panel / Quest (teleop/web/static/js/task_scene.js) ----------------------------------------------------
    def web_def(self):
        """Shapes of every task_* body (table, bin, objects) for the 3D page: sent once per websocket connection."""
        m, out = self.model, []
        kinds = {mujoco.mjtGeom.mjGEOM_BOX: "box", mujoco.mjtGeom.mjGEOM_CYLINDER: "cylinder",
                 mujoco.mjtGeom.mjGEOM_SPHERE: "sphere"}
        for g in range(m.ngeom):
            b = int(m.geom_bodyid[g]); bn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if not bn.startswith("task_") or int(m.geom_type[g]) not in kinds: continue
            rgba = m.mat_rgba[m.geom_matid[g]] if m.geom_matid[g] >= 0 else m.geom_rgba[g]
            out.append({"body": bn, "type": kinds[int(m.geom_type[g])], "size": [round(float(x), 4) for x in m.geom_size[g]],
                        "pos": [round(float(x), 4) for x in m.geom_pos[g]], "quat": [round(float(x), 5) for x in m.geom_quat[g]],
                        "rgba": [round(float(x), 3) for x in rgba]})
        return {"type": "task_scene_def", "geoms": out}

    def web_poses(self, data):
        """World pose of every task_* body, [x, y, z, qx, qy, qz, qw] (three.js order), + status line."""
        poses = {}
        for b in range(self.model.nbody):
            bn = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
            if not bn.startswith("task_"): continue
            p, q = data.xpos[b], data.xquat[b]
            poses[bn] = [round(float(p[0]), 4), round(float(p[1]), 4), round(float(p[2]), 4),
                         round(float(q[1]), 5), round(float(q[2]), 5), round(float(q[3]), 5), round(float(q[0]), 5)]
        inside = sum(1 for _, b, _, _ in self.objs if self.in_bin(data, b))
        return {"type": "task_poses", "poses": poses, "in_bin": inside, "total": len(self.objs)}

    def in_bin(self, data, body):
        bb = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "task_bin")
        if bb < 0: return False
        d = data.xpos[body] - data.xpos[bb]
        return abs(d[0]) < 0.10 and abs(d[1]) < 0.12 and -0.01 < d[2] < 0.12

    # ---- per control tick ----------------------------------------------------------------------------------------
    def step(self, data):
        """Robot already written into qpos (kinematic). Grasp assist, then real-time physics for the objects only."""
        now = time.monotonic()
        dt = 0.0 if self.last_t is None else min(0.05, max(0.0, now - self.last_t))
        self.last_t = now
        mujoco.mj_kinematics(self.model, data)
        for side in self.hands:
            c = self.closure(data, side)
            hb = self.hands[side][0]
            R_h, p_h = data.xmat[hb].reshape(3, 3), data.xpos[hb]
            if self.held[side] is None and c >= CLOSE_ON and not self.grasped[side]:
                self.grasped[side] = True                # one attempt per closing: closing in empty air picks nothing
                g = self.grip_point(data, side)
                taken = {h[0] for h in self.held.values() if h is not None}
                best, bd = None, GRIP_RADIUS
                for i, (_, b, _, _) in enumerate(self.objs):
                    d = float(np.linalg.norm(data.xpos[b] - g))
                    if i not in taken and d < bd: best, bd = i, d
                if best is not None:
                    b = self.objs[best][1]
                    self.held[side] = (best, R_h.T @ (data.xpos[b] - p_h), R_h.T @ data.xmat[b].reshape(3, 3))
                    print(f"[TASK] {side} hand picked {self.objs[best][0][9:]}", flush=True)
            elif c <= OPEN_AT:
                if self.held[side] is not None:
                    print(f"[TASK] {side} hand released {self.objs[self.held[side][0]][0][9:]}", flush=True)
                self.held[side], self.grasped[side] = None, False
        self._pin_held(data)                             # held objects follow their hand rigidly
        n = int(round(dt / self.model.opt.timestep))
        if n > 0:
            q_robot = data.qpos[self.robot_q].copy()
            for _ in range(min(n, 25)):
                mujoco.mj_step(self.model, data)
                data.qpos[self.robot_q] = q_robot; data.qvel[self.robot_v] = 0.0
                self._pin_held(data)                     # ... during the substeps too
        mujoco.mj_forward(self.model, data)

    def _pin_held(self, data):
        for side, h in self.held.items():
            if h is None: continue
            hb = self.hands[side][0]; _, _, a, v = self.objs[h[0]]
            R_h, p_h = data.xmat[hb].reshape(3, 3), data.xpos[hb]
            data.qpos[a:a + 3] = p_h + R_h @ h[1]
            data.qpos[a + 3:a + 7] = _mat_to_quat(R_h @ h[2])
            data.qvel[v:v + 6] = 0.0
