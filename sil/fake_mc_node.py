#!/usr/bin/env python3
"""
sil/fake_mc_node.py - Fake AgiBot Motion Controller for Software-In-the-Loop testing.

Runs on ROS 2 Humble system Python 3.10 (same as the Orin). Stands in for the robot's
MC + motors so the UNMODIFIED scripts/arm_teleop_rx.py can run against it:

  subscribes  /motion/control/arm_joint_command   (sensor_msgs/JointState, BEST_EFFORT)
              /motion/control/hand_joint_command
  publishes   /motion/control/arm_joint_state     (100 Hz, ARM_NAMES_LONG, position+velocity)
              /motion/control/hand_joint_state    (frame_id "AgiHand", raw 0..2000)
  HTTP mock   127.0.0.1:<rpc-port>  aimdk RPC subset (reached through http_proxy from sil/ssh_orin.sh,
              so the vendor SetAction.py / disable_motion_player.sh run unmodified):
                McActionService/GetAction, GetAvailableActions, SetAction (async: PREPARE ~1 s, then switch)
                MotionCommandService/DisableMotionPlayer, EnableMotionPlayer
                McMotionService/PlanningMove (JOINT targets only) + McDataService/GetTaskState - a PLUMBING MOCK:
                  min-jerk joint interpolation at <=0.6 rad/s * velocity_scale. Its smoothness says NOTHING about
                  the real MC planner (PlanningMove was measured on the robot on 16/11: smooth, ~0.92 rad/s).
  node        "motion_player" (ASSUMED): while enabled, publishes /motion/control/hand_joint_command at
              10 Hz, so a receiver started without disable_motion_player.sh fails pre-flight like on the robot.

Arm dynamics: teleop/simulation/real_plant.py with an MC-only plant (config/mc_plant.json,
identified from trace_orin logs: Orin published cmd -> Orin measured state).

ASSUMED behaviours (not yet measured on hardware, see config 'assumptions'):
  * command stream stops -> MC holds the last command
  * any step size is tracked (no rejection) unless mc_step_reject.enabled
  * joint limits enforced by clamping to the AimDK limits (core.VENDOR_MIN/MAX)
  * SetAction(ext_action=X) reports current_action=X (McAction_* name) once the ~1 s transition ends
  * boot state (--boot real): McAction_RL_LOCOMOTION_DEFAULT, motion player enabled

Fault injection / introspection: JSON over UDP on 127.0.0.1:<ctl-port> (reply = JSON):
  {"cmd":"status"}
  {"cmd":"freeze_state","on":true}          stop publishing arm/hand state (stale feedback)
  {"cmd":"dual_publisher","on":true}        a 2nd publisher on arm_joint_command (holds measured pose)
  {"cmd":"set_action","value":"10"}         change GetAction answer (e.g. not 402)
  {"cmd":"state_burst","n":3}               deliver state in bursts of n (DDS batching)
  {"cmd":"push","joint":3,"delta":0.05}     external disturbance on plant state
  {"cmd":"reset","q":[...14]}               teleport plant
"""
import os
import sys
import json
import time
import socket
import argparse
import threading
import importlib.util
import re
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, os.path.join(REPO, rel))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# Single source of truth for names / limits / home: the exact file deployed to the Orin.
core = _load("orin_teleop_core_sil", "scripts/orin_teleop_core.py")
real_plant = _load("real_plant_sil", "teleop/simulation/real_plant.py")

LIMS_MIN, LIMS_MAX = np.array(core.VENDOR_MIN), np.array(core.VENDOR_MAX)   # the real MC enforces the AimDK limits, not ours


def _load_action_enum():
    """McAction name -> number, parsed from the vendor protocol shipped in the repo."""
    path = os.path.join(REPO, "agibot_a2_aimdk-dev2.0/protocol/protobuf/aimdk/protocol/mc/action/mc_action_cmd.proto")
    table = {}
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                m = re.match(r"\s*(McAction_\w+)\s*=\s*(\d+);", line)
                if m:
                    table[m.group(1)] = int(m.group(2))
    except OSError:
        pass
    return table or {"McAction_RL_LOCOMOTION_DEFAULT": 401, "McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO": 402,
                     "McAction_USE_EXT_CMD": 5000}


ACTIONS = _load_action_enum()
ACTION_BY_NUM = {v: k for k, v in ACTIONS.items()}
SERVO_ACTION = "McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO"
BOOT_ACTION = "McAction_RL_LOCOMOTION_DEFAULT"
# GetAvailableActions: every action in vendor proto order (user-confirmed: the real menu shows
# 10 = McAction_STAND_DEFAULT, 12 = McAction_STAND_ARM_EXT_JOINT_SERVO, i.e. the proto list from 01).
AVAILABLE_ACTIONS = [a for a in ACTIONS if a != "McAction_USE_EXT_CMD"]
# ASSUMED: the MC follows /motion/control/arm_joint_command in every *_EXT_JOINT_SERVO action (203/302/402/405)
SERVO_ACTIONS = {a for a in ACTIONS if a.endswith("EXT_JOINT_SERVO")} | {SERVO_ACTION}
ACTION_SWITCH_S = 1.0
PLANNING_ACTIONS = {a for a in ACTIONS if a.endswith("PLANNING_MOVE") or a.endswith("ONLINE_PLANNING")}
PLAN_GROUPS = {"McPlanningGroup_LEFT_ARM": list(range(7)), "McPlanningGroup_RIGHT_ARM": list(range(7, 14)),
               "McPlanningGroup_DUAL_ARM": list(range(14))}
PLAN_VMAX = 0.6  # ASSUMED mock speed [rad/s] at velocity_scale 1


def normalize_action(v):
    """402 / '402' / 'RL_LOCOMOTION_...' / 'McAction_RL_...' -> 'McAction_RL_...' (None if unknown)."""
    if isinstance(v, (int, float)) or (isinstance(v, str) and v.strip().isdigit()):
        return ACTION_BY_NUM.get(int(v))
    if isinstance(v, str) and v.strip():
        name = v.strip() if v.strip().startswith("McAction_") else "McAction_" + v.strip()
        return name if name in ACTIONS else None
    return None


class FakeMC(Node):
    def __init__(self, args):
        super().__init__("fake_mc")
        self.args = args
        self.rate_hz = args.rate
        ref = real_plant.RealPlantSimulator(config_path=args.plant, enabled=not args.ideal)
        if args.physics == "mujoco":
            sys.path.insert(0, os.path.join(REPO, "sil"))
            from mujoco_plant import MujocoPlant
            self.plant = MujocoPlant(os.path.join(REPO, "assets/agibot_a2_ultra/mjcf/scene.xml"), ref,
                                     kp=args.servo_kp, viewer=args.viewer)
        else:
            self.plant = ref
        self.get_logger().info(f"plant: {self.plant.summary_str}")

        q0 = np.array(json.loads(args.init_pose) if args.init_pose else core.HOME, dtype=np.float64)
        self.cmd_q = q0.copy()
        self.meas_q = q0.copy()
        self.cmd_h = np.zeros(12)
        self.meas_h = np.zeros(12)
        self.plant.reset(q0)
        self.plant.reset_hand(self.cmd_h)

        self.lock = threading.Lock()
        boot_real = args.boot == "real"
        self.action = normalize_action(args.action) or (BOOT_ACTION if boot_real else SERVO_ACTION)
        self.ext_action = ""
        self.action_status = "McActionStatus_RUNNING"
        self._pending_action = None  # (name, t_done)
        self.motion_player_on = boot_real if args.motion_player == "auto" else (args.motion_player == "on")
        self.freeze_state = False
        self.dual_pub_on = False
        self.burst_n, self._burst_buf = 1, []
        self.n_arm_cmd = self.n_hand_cmd = self.n_state = 0
        self.last_cmd_t = 0.0
        self.max_cmd_step = 0.0
        self.first_cmd_jump, self.first_cmd_q = None, None
        self.cmd_path, self.stats_q0 = np.zeros(14), None
        self.last_tick = time.monotonic()
        self.plans, self.tasks, self._next_task = [], {}, 1000  # PlanningMove mock: active trajectories, task states

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/motion/control/arm_joint_command", self._on_arm_cmd, qos)
        self.create_subscription(JointState, "/motion/control/hand_joint_command", self._on_hand_cmd, qos)
        self.arm_state_pub = self.create_publisher(JointState, "/motion/control/arm_joint_state", qos)
        self.hand_state_pub = self.create_publisher(JointState, "/motion/control/hand_joint_state", qos)
        self.dual_pub = None
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.log_f = None
        if args.log_dir:
            os.makedirs(args.log_dir, exist_ok=True)
            self.log_f = open(os.path.join(args.log_dir, f"fake_mc_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"), "w", buffering=1)
            self.log_f.write(json.dumps({"event": "START", "plant": args.plant, "plant_summary": self.plant.summary_str,
                                         "rate_hz": self.rate_hz, "init_q": q0.tolist()}) + "\n")

    # ------------------------------------------------------------- ROS callbacks
    def _on_arm_cmd(self, msg: JointState):
        q = core.extract_q14_from_msg(msg)
        if len(q) != 14 or any(not np.isfinite(x) for x in q):
            return
        t_now = time.monotonic()
        if t_now - getattr(self, "_last_arm_msg_t", -1e9) > 3.0 and hasattr(self.plant, "request_viewer"):
            self.plant.request_viewer()  # new teleop session: bring the MuJoCo window back if it was closed
        self._last_arm_msg_t = t_now
        q = np.clip(np.array(q, dtype=np.float64), LIMS_MIN, LIMS_MAX)
        with self.lock:
            if self.action not in SERVO_ACTIONS or self.action_status == "McActionStatus_PREPARE":
                return  # MC ignores external servo commands outside the servo action (ASSUMED)
            self.cmd_path += np.abs(q - self.cmd_q)
            if self.stats_q0 is None:
                self.stats_q0 = self.cmd_q.copy()
            step = float(np.max(np.abs(q - self.cmd_q)))
            if self.first_cmd_jump is None:
                self.first_cmd_jump = step  # vs. setpoint the MC was holding (incl. static offset)
                self.first_cmd_q = q.copy()
            else:
                self.max_cmd_step = max(self.max_cmd_step, step)
            self.cmd_q = q
            self.n_arm_cmd += 1
            self.last_cmd_t = time.monotonic()

    def _on_hand_cmd(self, msg: JointState):
        n2p = dict(zip(msg.name, msg.position))
        if all(k in n2p for k in core.HAND_NAMES):
            h = [float(n2p[k]) for k in core.HAND_NAMES]
        elif len(msg.position) >= 12:
            h = [float(x) for x in msg.position[:12]]
        else:
            return
        with self.lock:
            self.cmd_h = np.clip(np.array(h), 0.0, 2000.0)
            self.n_hand_cmd += 1

    # ------------------------------------------------------------- plant tick
    def _apply_pending_action(self):
        with self.lock:
            if self._pending_action and time.monotonic() >= self._pending_action[1]:
                self.action = self._pending_action[0]
                self.action_status = "McActionStatus_RUNNING"
                self._pending_action = None
                print(f"[fake_mc] action -> {self.action}", flush=True)

    # ------------------------------------------------------------- RPC (aimdk HTTP subset)
    def handle_rpc(self, path: str, body: dict):
        self._apply_pending_action()
        ok = {"code": 0, "msg": "success"}
        svc = path.rsplit("/rpc/", 1)[-1] if "/rpc/" in path else path
        with self.lock:
            if svc.endswith("McActionService/GetAction"):
                return 200, {"header": ok, "info": {"current_action": self.action, "ext_action": self.ext_action,
                                                    "status": self.action_status}}
            if svc.endswith("McActionService/GetAvailableActions"):
                return 200, {"header": ok, "actions": list(AVAILABLE_ACTIONS)}
            if svc.endswith("McActionService/SetAction"):
                cmd = body.get("command", body) if isinstance(body, dict) else {}
                act, ext = cmd.get("action"), cmd.get("ext_action", "")
                target = normalize_action(ext) if normalize_action(act) == "McAction_USE_EXT_CMD" or act in (None, "") else normalize_action(act)
                if target is None and ext:
                    target = normalize_action(ext)
                if target == "McAction_STAND_ARM_EXT_JOINT_SERVO":
                    # User-confirmed on the real robot: SetAction menu 10 (STAND_DEFAULT) then 12 (STAND_ARM_EXT_JOINT_SERVO)
                    # is the procedure, and the MC then reports/serves 402 (RL_LOCOMOTION_ARM_EXT_JOINT_SERVO).
                    target = SERVO_ACTION
                if target not in AVAILABLE_ACTIONS:
                    return 200, {"header": {"code": 1, "msg": f"action not available: {act!r}/{ext!r}"},
                                 "info": {"current_action": self.action, "status": self.action_status}}
                self.ext_action = ext if isinstance(ext, str) else ""
                if target != self.action:
                    self._pending_action = (target, time.monotonic() + ACTION_SWITCH_S)
                    self.action_status = "McActionStatus_PREPARE"
                print(f"[fake_mc] SetAction -> {target} (switch in {ACTION_SWITCH_S:.1f} s)", flush=True)
                return 200, {"header": ok, "info": {"current_action": self.action, "ext_action": self.ext_action,
                                                    "status": self.action_status}}
            if svc.endswith("MotionCommandService/DisableMotionPlayer"):
                self.motion_player_on = False
                print("[fake_mc] motion_player disabled", flush=True)
                return 200, {"header": ok}
            if svc.endswith("MotionCommandService/EnableMotionPlayer"):
                self.motion_player_on = True
                print("[fake_mc] motion_player enabled", flush=True)
                return 200, {"header": ok}
            if svc.endswith("McMotionService/PlanningMove"):
                return self._planning_move(body if isinstance(body, dict) else {})
            if svc.endswith("/GetTaskState"):
                tid = str((body or {}).get("task_id", ""))
                return 200, {"header": ok, "task_id": tid, "state": self.tasks.get(tid, "CommonState_INVALID")}
        return 404, {"header": {"code": 404, "msg": f"SIL mock: unsupported {svc}"}}

    def _planning_move(self, body: dict):
        """PlanningMove mock (lock held): JOINT target -> min-jerk trajectory from the current setpoint."""
        def fail(msg, state="CommonState_FAILURE"):
            return 200, {"header": {"code": 1, "msg": f"SIL mock: {msg}"}, "task_id": "0", "state": state}
        if self.action not in PLANNING_ACTIONS or self.action_status != "McActionStatus_RUNNING":
            return fail(f"PlanningMove not allowed in {self.action}", "CommonState_INVALID")
        idx = PLAN_GROUPS.get(body.get("group"))
        tgt = body.get("target") or {}
        if idx is None:
            return fail(f"group {body.get('group')!r} not mocked")
        if tgt.get("type") != "JOINT":
            return fail(f"target type {tgt.get('type')!r} not mocked (JOINT only)")
        joints = tgt.get("joints") or []
        if len(joints) != len(idx):
            return fail(f"{len(joints)} joints for {body.get('group')} (needs {len(idx)})")
        q1 = np.clip(np.array(joints, dtype=np.float64), LIMS_MIN[idx], LIMS_MAX[idx])
        if any(set(p["idx"]) & set(idx) for p in self.plans):
            if self.action.endswith("ONLINE_PLANNING"):
                # real robot 16/11: in 408 a new target replaces the running one (the arm turns around mid-motion)
                for p in [p for p in self.plans if set(p["idx"]) & set(idx)]:
                    self.tasks[p["task_id"]] = "CommonState_ABORTED"; self.plans.remove(p)
            else:
                return fail("group busy", "CommonState_ABORTED")
        vs = float(((body.get("param") or {}).get("velocity_scale")) or 1.0)
        q0 = self.cmd_q[idx].copy()
        dur = max(0.3, 1.875 * float(np.max(np.abs(q1 - q0))) / (PLAN_VMAX * min(max(vs, 0.05), 1.0)))
        tid = str(self._next_task); self._next_task += 1
        self.plans.append({"idx": idx, "q0": q0, "q1": q1, "t0": time.monotonic(), "T": dur, "task_id": tid})
        self.tasks[tid] = "CommonState_RUNNING"
        print(f"[fake_mc] PlanningMove {body.get('group')} task {tid}: {dur:.2f} s", flush=True)
        return 200, {"header": {"code": 0, "msg": "success"}, "task_id": tid, "state": "CommonState_PENDING"}

    def _advance_plans(self, now: float):
        """Lock held: write the active min-jerk trajectories into the setpoint."""
        for p in list(self.plans):
            u = min(1.0, (now - p["t0"]) / p["T"])
            s = u ** 3 * (10 - 15 * u + 6 * u * u)
            self.cmd_q[p["idx"]] = p["q0"] + s * (p["q1"] - p["q0"])
            if u >= 1.0:
                self.tasks[p["task_id"]] = "CommonState_SUCCESS"
                self.plans.remove(p)

    def _tick(self):
        self._apply_pending_action()
        now = time.monotonic()
        dt = min(0.05, max(1e-4, now - self.last_tick))
        self.last_tick = now
        with self.lock:
            if self.plans:
                self._advance_plans(now)
            cmd_q, cmd_h = self.cmd_q.copy(), self.cmd_h.copy()
        meas = np.clip(self.plant.step(cmd_q, dt=dt), LIMS_MIN, LIMS_MAX)
        meas_h = np.clip(self.plant.step_hand(cmd_h, dt=dt), 0.0, 2000.0)
        vel = self.plant.velocity
        with self.lock:
            self.meas_q, self.meas_h = meas, meas_h
            freeze, burst_n, dual = self.freeze_state, self.burst_n, self.dual_pub_on

        stamp = self.get_clock().now().to_msg()
        if not freeze:
            am = JointState()
            am.header.stamp = stamp
            am.name = list(core.ARM_NAMES_LONG)
            am.position = [float(x) for x in meas]
            am.velocity = [float(x) for x in vel]
            hm = JointState()
            hm.header.stamp = stamp
            hm.header.frame_id = "AgiHand"
            hm.name = list(core.HAND_NAMES)
            hm.position = [float(x) for x in meas_h]
            self._burst_buf.append((am, hm))
            if len(self._burst_buf) >= burst_n:
                for a, h in self._burst_buf:
                    self.arm_state_pub.publish(a)
                    self.hand_state_pub.publish(h)
                    self.n_state += 1
                self._burst_buf = []
        if dual and self.dual_pub is not None:
            dm = JointState()
            dm.header.stamp = stamp
            dm.name = list(core.ARM_NAMES_LONG)
            dm.position = [float(x) for x in meas]
            self.dual_pub.publish(dm)

        if self.log_f:
            self.log_f.write(json.dumps({"t": now, "cmd_q": [round(float(x), 5) for x in cmd_q],
                                         "meas_q": [round(float(x), 5) for x in meas],
                                         "hand_cmd": [round(float(x), 1) for x in cmd_h],
                                         "hand_meas": [round(float(x), 1) for x in meas_h],
                                         "frozen": freeze}) + "\n")

    # ------------------------------------------------------------- control port
    def handle_ctl(self, req: dict) -> dict:
        c = req.get("cmd")
        with self.lock:
            if c == "status":
                return {"ok": True, "action": self.action, "action_status": self.action_status,
                        "motion_player": self.motion_player_on, "freeze_state": self.freeze_state,
                        "dual_publisher": self.dual_pub_on, "burst_n": self.burst_n,
                        "n_arm_cmd": self.n_arm_cmd, "n_hand_cmd": self.n_hand_cmd, "n_state": self.n_state,
                        "cmd_age_s": (time.monotonic() - self.last_cmd_t) if self.last_cmd_t else None,
                        "max_cmd_step_rad": self.max_cmd_step,
                        "first_cmd_jump_rad": self.first_cmd_jump,
                        "cmd_path_rad": self.cmd_path.tolist(),
                        "cmd_net_rad": (self.cmd_q - self.stats_q0).tolist() if self.stats_q0 is not None else [0.0] * 14,
                        "first_cmd_q": self.first_cmd_q.tolist() if self.first_cmd_q is not None else None,
                        "cmd_q": self.cmd_q.tolist(), "meas_q": self.meas_q.tolist(),
                        "cmd_h": self.cmd_h.tolist(), "meas_h": self.meas_h.tolist(),
                        "plant": self.plant.summary_str,
                        "contacts": self.plant.contacts() if hasattr(self.plant, "contacts") else [],
                        "torque_nm": self.plant.actuator_torque().round(2).tolist() if hasattr(self.plant, "actuator_torque") else None}
            if c == "freeze_state":
                self.freeze_state = bool(req.get("on", True)); return {"ok": True}
            if c == "set_action":
                a = normalize_action(req.get("value", SERVO_ACTION))
                if a is None:
                    return {"ok": False, "error": f"unknown action {req.get('value')!r}"}
                self.action, self.action_status, self._pending_action = a, "McActionStatus_RUNNING", None
                return {"ok": True, "action": a}
            if c == "motion_player":
                self.motion_player_on = bool(req.get("on", True)); return {"ok": True}
            if c == "state_burst":
                self.burst_n = max(1, int(req.get("n", 1))); return {"ok": True}
            if c == "reset_stats":
                self.max_cmd_step, self.n_arm_cmd, self.n_hand_cmd, self.n_state = 0.0, 0, 0, 0
                self.first_cmd_jump, self.first_cmd_q = None, None
                self.cmd_path, self.stats_q0 = np.zeros(14), None; return {"ok": True}
            if c == "push":
                j, d = int(req["joint"]), float(req["delta"])
                if hasattr(self.plant, "push"):
                    self.plant.push(j, d)
                elif self.plant._arm is not None:
                    self.plant._arm.y[j] += d
                return {"ok": True}
            if c == "reset":
                q = np.array(req["q"], dtype=np.float64)
                self.cmd_q, self.meas_q = q.copy(), q.copy()
                self.plant.reset(q); return {"ok": True}
        if c == "dual_publisher":
            on = bool(req.get("on", True))
            qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
            if on and self.dual_pub is None:
                self.dual_pub = self.create_publisher(JointState, "/motion/control/arm_joint_command", qos)
            with self.lock:
                self.dual_pub_on = on
            return {"ok": True}
        return {"ok": False, "error": f"unknown cmd {c}"}


def serve_ctl(node: FakeMC, port: int, host: str = "127.0.0.1"):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))
    while rclpy.ok():
        try:
            data, addr = s.recvfrom(65535)
            rep = node.handle_ctl(json.loads(data.decode()))
        except Exception as e:  # noqa: BLE001
            rep = {"ok": False, "error": str(e)}
        try:
            s.sendto(json.dumps(rep).encode(), addr)
        except Exception:
            pass


def serve_rpc(node: FakeMC, port: int, host: str = "127.0.0.1"):
    """aimdk HTTP RPC subset. Requests arrive directly or via http_proxy (absolute-URI request line)."""
    class H(BaseHTTPRequestHandler):
        def do_POST(self):
            n = int(self.headers.get("Content-Length", "0") or 0)
            raw = self.rfile.read(n) if n else b""
            try:
                body = json.loads(raw.decode() or "{}")
            except Exception:
                body = {}
            code, rep = node.handle_rpc(self.path, body)
            data = json.dumps(rep).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def log_message(self, *a):
            pass

    ThreadingHTTPServer((host, port), H).serve_forever()


class MotionPlayerSim(Node):
    """ASSUMED stand-in for the vendor motion_player: publishes hand commands while enabled."""

    def __init__(self, mc: FakeMC):
        super().__init__("motion_player")
        self.mc = mc
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(JointState, "/motion/control/hand_joint_command", qos)
        self.create_timer(0.1, self._tick)

    def _tick(self):
        if not self.mc.motion_player_on:
            return
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "AgiHand"
        m.name = list(core.HAND_NAMES)
        m.position = [0.0] * 12
        self.pub.publish(m)


def a2sim_isolated():
    """True only inside the a2sim namespace: interfaces lo + a2sim-orin, no default route (cannot reach any robot)."""
    ifaces = {name for _, name in socket.if_nameindex()}  # netlink: reflects THIS netns (sysfs may not)
    if not ifaces <= {"lo", "a2sim-orin"} or "a2sim-orin" not in ifaces:
        return False, f"interfaces {sorted(ifaces)}"
    with open("/proc/net/route") as f:
        for line in f.readlines()[1:]:
            if line.split()[1] == "00000000":
                return False, "default route present"
    return True, ""


def main():
    ap = argparse.ArgumentParser(description="Fake AgiBot Motion Controller (SIL)")
    ap.add_argument("--plant", default=os.path.join(REPO, "config", "mc_plant.json"))
    ap.add_argument("--ideal", action="store_true", help="Pass-through plant (measured == command)")
    ap.add_argument("--rate", type=float, default=100.0, help="arm/hand state publish rate (real MC: 100 Hz)")
    ap.add_argument("--init-pose", default=None, help="JSON list of 14 joint angles (default: orin_teleop_core.HOME)")
    ap.add_argument("--boot", choices=["ready", "real"], default="ready",
                    help="ready: already in 402, motion player off (automated tests). "
                         "real: like a freshly booted robot - RL_LOCOMOTION_DEFAULT, motion player on")
    ap.add_argument("--action", default=None, help="override boot action (name or number, e.g. 402)")
    ap.add_argument("--motion-player", choices=["auto", "on", "off"], default="auto")
    ap.add_argument("--ctl-port", type=int, default=int(os.environ.get("SIL_CTL_PORT", "5099")))
    ap.add_argument("--rpc-port", type=int, default=int(os.environ.get("SIL_RPC_PORT", "56322")))
    ap.add_argument("--log-dir", default=None)
    ap.add_argument("--physics", choices=["plant", "mujoco"], default="plant",
                    help="plant: identified per-joint model only; mujoco: MuJoCo physics body (contacts, inertia, limits)")
    ap.add_argument("--servo-kp", type=float, default=3000.0, help="MuJoCo joint servo stiffness (Nm/rad)")
    ap.add_argument("--viewer", action="store_true", help="MuJoCo window showing the simulated robot")
    ap.add_argument("--a2sim", action="store_true",
                    help="run as the robot inside the isolated a2sim network namespace (sil/a2sim): real ROS domain "
                         "232 allowed, RPC on 0.0.0.0:56322 + :56444 (reached as 192.168.100.100), ctl on 0.0.0.0")
    args = ap.parse_args()

    dom = os.environ.get("ROS_DOMAIN_ID", "0")
    isolated_lo = dom != "232" and os.environ.get("ROS_LOCALHOST_ONLY", "0") == "1"
    if args.a2sim:
        ok, why = a2sim_isolated()
        if not ok:
            print(f"FATAL: --a2sim but not inside the isolated a2sim namespace ({why}). Start with sil/a2sim/robot_power_on.sh")
            sys.exit(2)
    elif not isolated_lo:
        print(f"FATAL: refusing to start fake MC with ROS_DOMAIN_ID={dom}, ROS_LOCALHOST_ONLY="
              f"{os.environ.get('ROS_LOCALHOST_ONLY')} (must be isolated from the real robot; use sil/sil_up.sh)")
        sys.exit(2)
    bind_host = "0.0.0.0" if args.a2sim else "127.0.0.1"
    rpc_ports = [args.rpc_port, 56444] if args.a2sim else [args.rpc_port]

    # Refuse to start half-broken when another fake MC still owns the control / RPC ports.
    for port, kind in [(args.ctl_port, socket.SOCK_DGRAM)] + [(p, socket.SOCK_STREAM) for p in rpc_ports]:
        probe = socket.socket(socket.AF_INET, kind)
        if kind == socket.SOCK_STREAM:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # like HTTPServer; still fails if someone LISTENs
        try:
            probe.bind((bind_host, port))
        except OSError:
            print(f"FATAL: 127.0.0.1:{port} busy - another fake MC (or SIL stack) is still running. Stop it first.")
            sys.exit(2)
        finally:
            probe.close()

    rclpy.init()
    node = FakeMC(args)
    mp = MotionPlayerSim(node)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(mp)
    threading.Thread(target=serve_ctl, args=(node, args.ctl_port, bind_host), daemon=True).start()
    for rp in rpc_ports:
        threading.Thread(target=serve_rpc, args=(node, rp, bind_host), daemon=True).start()
    print(f"[fake_mc] ready: state {args.rate:.0f} Hz | ctl udp {bind_host}:{args.ctl_port} | rpc http {bind_host}:{rpc_ports} "
          f"| domain {dom} | {node.plant.summary_str}", flush=True)
    print(f"[fake_mc] action={node.action} motion_player={'ON' if node.motion_player_on else 'off'}", flush=True)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.log_f:
            node.log_f.close()
        mp.destroy_node()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
