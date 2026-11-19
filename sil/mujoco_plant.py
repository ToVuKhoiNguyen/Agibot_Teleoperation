"""
sil/mujoco_plant.py - MuJoCo physics body for the simulated AgiBot A2 (used by sil/fake_mc_node.py --physics mujoco).

Runs on ROS 2 Humble system Python 3.10 (mujoco wheel installed for that interpreter).

Model: assets/agibot_a2_ultra/mjcf/scene.xml (fixed base, 'home' keyframe = orin_teleop_core.HOME).

How the simulated MC drives the arms
  command q (what the Orin receiver publishes)
    -> identified MC response (delay + 1st/2nd order, config/mc_plant.json; RealPlantSimulator.true_state)
    -> stiff joint servo in MuJoCo (position actuator, kp/kv, +-100 Nm like the MJCF, velocity feed-forward),
       gravity compensated (the real MC holds pose with ~0.2 mrad static error)
    -> physics: joint limits, inertia, contacts with the body/other arm/legs/head stop the arm and the servo
       pushes up to its torque limit, exactly like a stiff real servo against an obstacle
    -> measured q = MuJoCo qpos + identified sensor effects (static offset, quantization, optional noise)
  Hands: raw 0..2000 -> rad (raw/2000 * SIM_HAND_COMFORTABLE_LIMITS, same map as udp_backend) -> finger
  actuators; mimic joints are the MJCF equality constraints; measured raw from the actuated joint angle.
  Legs + head: held at the standing pose (the real robot balances in RL locomotion; sway not modelled).

Contact pairs that already interpenetrate at HOME (mesh overlaps between mechanically adjacent parts:
wrist link05/link07, shoulder vs base_link, head vs base_link, feet vs floor) are excluded; every other
collidable pair stays active.
"""
import time
import numpy as np
import mujoco

ARM_JOINTS = [f"idx{13+i}_left_arm_joint{i+1}" for i in range(7)] + [f"idx{20+i}_right_arm_joint{i+1}" for i in range(7)]
HAND_ACTIVE = [f"{s}_{f}_joint" for s in ("L", "R") for f in ("thumb_swing", "thumb_1", "index_1", "middle_1", "ring_1", "pinky_1")]
HAND_LIMITS = np.array([1.60, 0.35, 1.30, 1.30, 1.30, 1.30] * 2)  # teleop/robot/aimdk_map.SIM_HAND_COMFORTABLE_LIMITS
STAND_LEGS = {"idx03_left_hip_pitch": -0.30, "idx04_left_tarsus": 0.60, "idx05_left_toe_pitch": -0.30,
              "idx09_right_hip_pitch": -0.30, "idx10_right_tarsus": 0.60, "idx11_right_toe_pitch": -0.30}


class MujocoPlant:
    def __init__(self, xml_path, ref_plant, kp=3000.0, viewer=False):
        self.ref = ref_plant  # RealPlantSimulator (identified MC response)
        spec = mujoco.MjSpec.from_file(xml_path)
        m0 = spec.compile()
        d0 = mujoco.MjData(m0)
        mujoco.mj_resetDataKeyframe(m0, d0, mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_KEY, "home"))
        self._set_legs(m0, d0)
        mujoco.mj_forward(m0, d0)
        bname = lambda m, g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "world"  # noqa: E731
        excl = sorted({tuple(sorted((bname(m0, d0.contact[i].geom1), bname(m0, d0.contact[i].geom2)))) for i in range(d0.ncon)})
        for b1, b2 in excl:
            spec.add_exclude(bodyname1=b1, bodyname2=b2)
        self.excluded_pairs = excl
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        m = self.m

        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)  # noqa: E731
        self.arm_q = np.array([m.jnt_qposadr[jid(n)] for n in ARM_JOINTS])
        self.arm_v = np.array([m.jnt_dofadr[jid(n)] for n in ARM_JOINTS])
        self.hand_q = np.array([m.jnt_qposadr[jid(n)] for n in HAND_ACTIVE])
        act_of = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a, 0]): a for a in range(m.nu)}
        self.arm_act = np.array([act_of[n] for n in ARM_JOINTS])
        self.hand_act = np.array([act_of[n] for n in HAND_ACTIVE])
        self.arm_lo, self.arm_hi = m.jnt_range[[jid(n) for n in ARM_JOINTS]].T

        hold = [n for n in (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt))
                if n and ("hip" in n or "tarsus" in n or "toe" in n or "head" in n)]
        self.hold_q = np.array([m.jnt_qposadr[jid(n)] for n in hold])
        self.hold_v = np.array([m.jnt_dofadr[jid(n)] for n in hold])
        self.hold_val = np.array([STAND_LEGS.get(n, 0.0) for n in hold])
        self.hold_act = [act_of[n] for n in hold if n in act_of]
        self.hold_act_val = [STAND_LEGS.get(n, 0.0) for n in hold if n in act_of]

        # gravity compensation on everything the arm servos carry (arms + hands)
        arm_root = {m.jnt_bodyid[jid(ARM_JOINTS[0])], m.jnt_bodyid[jid(ARM_JOINTS[7])]}
        for b in range(m.nbody):
            p = b
            while p > 0 and p not in arm_root:
                p = m.body_parentid[p]
            if p in arm_root:
                m.body_gravcomp[b] = 1.0

        mujoco.mj_resetDataKeyframe(m, self.d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home"))
        self._set_legs(m, self.d)
        mujoco.mj_forward(m, self.d)
        M = np.zeros((m.nv, m.nv))
        try:
            mujoco.mj_fullM(m, self.d, M)          # mujoco >= 3.4 signature (m, d, dst)
        except TypeError:
            mujoco.mj_fullM(m, M, self.d.qM)       # older (m, dst, qM)
        self.kp = float(kp)
        self.kv = np.array([2.0 * np.sqrt(self.kp * M[v, v]) for v in self.arm_v])  # critically damped per joint
        for i, a in enumerate(self.arm_act):
            m.actuator_gainprm[a, 0] = self.kp
            m.actuator_biasprm[a, 1] = -self.kp
            m.actuator_biasprm[a, 2] = -self.kv[i]
        self.torque_limit = m.actuator_forcerange[self.arm_act, 1].copy()

        self._t_acc = 0.0
        self._push = None  # (dof index, torque, t_end)
        self.viewer = None
        self._last_view = 0.0
        self._want_viewer = bool(viewer)   # the robot stays "on"; its window is re-opened for each new teleop session
        self._reopen = False
        self._open_viewer()
        self.summary_str = (f"MuJoCo physics (kp={self.kp:.0f}, torque<=+-{self.torque_limit.max():.0f} Nm, "
                            f"{len(excl)} home-overlap pairs excluded) + {ref_plant.summary_str}")

    def _open_viewer(self):
        if not self._want_viewer:
            return
        try:
            from mujoco import viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(self.m, self.d, show_left_ui=False, show_right_ui=False)
        except Exception as e:  # noqa: BLE001
            self.viewer = None
            print(f"[mujoco_plant] viewer unavailable ({e}); running headless")

    def request_viewer(self):
        """Thread-safe: ask step_arm to re-open the window if it was closed (called when a teleop session starts)."""
        self._reopen = True

    @staticmethod
    def _set_legs(m, d):
        for n, v in STAND_LEGS.items():
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            if j >= 0:
                d.qpos[m.jnt_qposadr[j]] = v

    # ------------------------------------------------------------- interface used by FakeMC
    def reset(self, q14):
        q14 = np.asarray(q14, dtype=np.float64)
        self.ref.reset(q14)
        self.d.qpos[self.arm_q] = q14
        self.d.qvel[:] = 0.0
        self.d.ctrl[self.arm_act] = q14
        mujoco.mj_forward(self.m, self.d)
        self._t_acc = 0.0

    def reset_hand(self, h12_raw):
        self.ref.reset_hand(np.asarray(h12_raw, dtype=np.float64))
        rad = np.clip(np.asarray(h12_raw) / 2000.0, 0, 1) * HAND_LIMITS
        self.d.qpos[self.hand_q] = rad
        self.d.ctrl[self.hand_act] = rad
        mujoco.mj_forward(self.m, self.d)

    def step(self, cmd_q14, dt):
        """Advances physics by dt with the MC command; returns measured q14."""
        self.ref.step(cmd_q14, dt=dt)
        q_ref = np.clip(self.ref.true_state, self.arm_lo, self.arm_hi)
        v_ref = self.ref.velocity
        # position servo with velocity feed-forward: kp(q_ref - q) + kv(v_ref - qd)
        self.d.ctrl[self.arm_act] = q_ref + self.kv / self.kp * v_ref
        self._t_acc += dt
        ts = self.m.opt.timestep
        now = time.monotonic()
        while self._t_acc >= ts - 1e-9:
            self._t_acc -= ts
            self.d.qfrc_applied[:] = 0.0
            if self._push and now < self._push[2]:
                self.d.qfrc_applied[self._push[0]] = self._push[1]
            self.d.ctrl[self.hold_act] = self.hold_act_val
            self.d.qpos[self.hold_q] = self.hold_val
            self.d.qvel[self.hold_v] = 0.0
            mujoco.mj_step(self.m, self.d)
        self.d.qpos[self.hold_q] = self.hold_val
        self.d.qvel[self.hold_v] = 0.0
        if self._reopen:
            self._reopen = False
            if self._want_viewer and (self.viewer is None or not self.viewer.is_running()):
                self._open_viewer()
        if self.viewer is not None and now - self._last_view > 1 / 30:
            self._last_view = now
            if self.viewer.is_running():
                self.viewer.sync()
        return self.ref.measure(self.d.qpos[self.arm_q])

    def step_hand(self, h12_raw, dt):
        h_ref = self.ref.step_hand(np.asarray(h12_raw, dtype=np.float64), dt=dt)
        self.d.ctrl[self.hand_act] = np.clip(h_ref / 2000.0, 0, 1) * HAND_LIMITS
        return np.clip(self.d.qpos[self.hand_q] / HAND_LIMITS * 2000.0, 0.0, 2000.0)

    @property
    def velocity(self):
        return self.d.qvel[self.arm_v].copy()

    def push(self, joint, delta_rad, duration_s=0.5):
        """External disturbance: torque that would deflect the servo by ~delta_rad, for duration_s."""
        self._push = (int(self.arm_v[joint]), float(self.kp * delta_rad), time.monotonic() + duration_s)

    def contacts(self):
        """Active contacts involving arms/hands: [(bodyA, bodyB, depth_m, normal_force_N)]."""
        out = []
        f6 = np.zeros(6)
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            b1 = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY, self.m.geom_bodyid[c.geom1]) or "world"
            b2 = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY, self.m.geom_bodyid[c.geom2]) or "world"
            if not any(k in b1 + b2 for k in ("arm_link", "L_", "R_", "hand")):
                continue
            mujoco.mj_contactForce(self.m, self.d, i, f6)
            out.append((b1, b2, round(float(-c.dist), 4), round(float(f6[0]), 1)))
        return out

    def actuator_torque(self):
        return self.d.actuator_force[self.arm_act].copy()
