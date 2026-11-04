#!/usr/bin/env python3
"""
main.py
Single consolidated entrypoint for AGIBOT A2 Ultra Teleoperation.
Serves HTTPS WebXR & 3D Interactive Web Panel on https://localhost:8012/

Defaults:
  - Starts MuJoCo physics simulation at home keyframe (legs locked, e-stop cleared, hold released)
  - Starts HTTPS server with Web Control Panel, 3D viewport, 25 Hz MuJoCo pose streaming, and /health
  - Auto-generates self-signed SSL certificate if missing
  - Idle XR receiver ready for Meta Quest 3S connection (never blocks or crashes if no headset)
  - Browser is the default viewer (MuJoCo native GUI viewer is OFF by default; --viewer turns it on)
  - Clean shutdown on Ctrl+C (frees port 8012, closes all threads and sockets)
"""

import os
import sys
import time
import signal
import socket
import argparse
from pathlib import Path

# Setup paths
PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.utils.port_check import check_port_available, get_process_holding_port
from teleop.utils.ssl_certs import ensure_ssl_certs
from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.simulation.web_panel import WebPanelServer
from teleop.kinematics.agibot_a2_ik import AgibotA2IK
from teleop.kinematics.mink_ik import AgibotMinkIK
from teleop.utils.collision_checker import CollisionChecker
try:
    from teleop.kinematics.pyroki_ik import AgibotPyrokiIK
except ImportError:
    AgibotPyrokiIK = None
from teleop.kinematics.filters import InputFilterPipeline, ClutchRamp, JointTrajectoryFilter, CriticallyDampedFollower, QuaternionSlerpFollower
from teleop.retargeting.agibot_arm_retargeting import AgibotArmRetargeting
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting
from teleop.tracking.quest_tracking import QuestTrackingReceiver
from teleop.xr.adb_setup import QuestAdbManager
import numpy as np
import mujoco
import yaml
from scipy.spatial.transform import Rotation as R

SCENE_XML = str(PROJECT_ROOT / "assets" / "agibot_a2_ultra" / "mjcf" / "scene.xml")


def get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def print_startup_banner(port: int, host_ip: str, status_dict: dict, quest_dict: dict = None):
    quest_dict = quest_dict or {}
    adb_str = "Active" if quest_dict.get("adb_installed") else "Not found (Install with: sudo apt install adb)"
    dev_str = quest_dict.get("device") or (f"None ({quest_dict.get('device_state')})" if quest_dict.get("device_state") else "None")
    rev_str = "ACTIVE (tcp:8012 -> tcp:8012)" if quest_dict.get("reverse_active") else "Inactive"
    quest_url = quest_dict.get("quest_url", f"https://localhost:{port}/vr")

    print("\n" + "=" * 80)
    print("READY - AGIBOT A2 ULTRA TELEOPERATION SYSTEM ACTIVE")
    print("=" * 80)
    print(f"1. PC BROWSER (3D PANEL) : https://localhost:{port}/")
    print(f"2. QUEST WEBXR (DIRECT)  : {quest_url}")
    print(f"   ADB Status            : {adb_str}")
    print(f"   Connected Headset     : {dev_str}")
    print(f"   ADB Reverse           : {rev_str}")
    print(f"3. QUEST 3S (WI-FI / LAN): https://{host_ip}:{port}/vr")
    print(f"4. HEALTH ENDPOINT       : https://localhost:{port}/health")
    print("-" * 80)
    print("SUBSYSTEM STATUS:")
    print(f"  • Sim Backend : {status_dict.get('sim', 'OK')}")
    print(f"  • Native Viewer: {status_dict.get('viewer', 'OFF')}")
    print(f"  • HTTPS Server: {status_dict.get('server', 'OK')}")
    print(f"  • Web Panel   : {status_dict.get('panel', 'OK')}")
    print(f"  • XR Receiver : {status_dict.get('xr_receiver', 'IDLE (Ready for Quest connection)')}")
    print(f"  • Safety Layer: {status_dict.get('safety', 'OK')}")
    print("=" * 80 + "\n")


class UnifiedTeleopApp:
    def __init__(
        self,
        port: int = 8012,
        render_viewer: bool = False,
        use_controller: bool = False,
        xr_view: bool = False,
        backend: str = "mujoco",
        speed_scale: float = 1.0,
        ik_solver_type: str = "mink",
        self_collision: bool = True,
        no_adb: bool = False,
        no_quest_launch: bool = False,
    ):
        self.port = port
        self.render_viewer = render_viewer
        self.use_controller = use_controller
        self.xr_view = xr_view
        self.backend = backend
        self.speed_scale = speed_scale
        self.ik_solver_type = ik_solver_type.lower()
        self.self_collision = self_collision
        self.no_adb = no_adb
        self.no_quest_launch = no_quest_launch
        self.collision_avoidance = False  # mink CollisionAvoidanceLimit disabled by default (handled by SafetyLayer gate)
        self.collision_blocked = False
        self.min_collision_distance_m = 999.0

        # 1. Port Check
        if not check_port_available(self.port):
            proc_info = get_process_holding_port(self.port)
            print(f"[FATAL] Port {self.port} is already in use!\nProcess: {proc_info}")
            print(f"Please terminate the process holding port {self.port} or specify another port via --port.")
            sys.exit(1)

        # 2. SSL Certs
        certs_dir = str(PROJECT_ROOT / "certs")
        self.cert_path, self.key_path = ensure_ssl_certs(certs_dir)

        # 3. MuJoCo Physics Backend
        print("[INFO] Initializing MuJoCo Simulation Environment (home keyframe)...")
        has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        if self.render_viewer and not has_display:
            print("[WARN] DISPLAY / WAYLAND_DISPLAY not found in environment. MuJoCo native viewer cannot be displayed.")
            self.render_viewer = False

        self.env = AgibotSimEnv(SCENE_XML, render_gui=self.render_viewer)
        self.env.reset()

        home_poses = self.env.get_ee_poses()
        self.robot_home_pos_l = home_poses["left"]["pos"].copy()
        self.robot_home_pos_r = home_poses["right"]["pos"].copy()
        self.robot_home_rot_l = home_poses["left"]["rot"].copy()
        self.robot_home_rot_r = home_poses["right"]["rot"].copy()

        # 4. Retargeting & IK
        print(f"[INFO] Initializing Arm / Hand Retargeters and IK Solvers (Engine: {self.ik_solver_type.upper()})...")
        self.arm_retarget_left = AgibotArmRetargeting(arm="left")
        self.arm_retarget_right = AgibotArmRetargeting(arm="right")
        self.hand_retarget_left = AgibotHandRetargeting(hand="left", max_vel_rad_s=10.0)
        self.hand_retarget_right = AgibotHandRetargeting(hand="right", max_vel_rad_s=10.0)

        if self.ik_solver_type == "mink":
            self.ik_left = AgibotMinkIK(SCENE_XML, arm="left", use_collision_avoidance=self.collision_avoidance)
            self.ik_right = AgibotMinkIK(SCENE_XML, arm="right", use_collision_avoidance=self.collision_avoidance)
        elif self.ik_solver_type == "pyroki":
            if AgibotPyrokiIK is None:
                raise ImportError("Pyroki is not available. Install dependencies or use --ik mink.")
            self.ik_left = AgibotPyrokiIK(arm="left")
            self.ik_right = AgibotPyrokiIK(arm="right")
        else:
            self.ik_left = AgibotA2IK(SCENE_XML, arm="left")
            self.ik_right = AgibotA2IK(SCENE_XML, arm="right")

        # Input filters (One Euro + Slerp + Spike rejection + Deadzone)
        self.input_filter_left = InputFilterPipeline()
        self.input_filter_right = InputFilterPipeline()
        self.clutch_ramp_left = ClutchRamp()
        self.clutch_ramp_right = ClutchRamp()

        # Output trajectory filters (Ruckig / critically damped)
        self.control_dt = 0.005  # 200 Hz fixed-rate control loop
        self.output_filter_left = JointTrajectoryFilter(num_joints=7, max_vel=4.0, max_acc=20.0, max_jerk=100.0, dt=self.control_dt)
        self.output_filter_right = JointTrajectoryFilter(num_joints=7, max_vel=4.0, max_acc=20.0, max_jerk=100.0, dt=self.control_dt)
        self.output_filter_left.reset(self.env.get_left_arm_qpos())
        self.output_filter_right.reset(self.env.get_right_arm_qpos())

        # Gizmo critically damped follower (30 Hz bandwidth) and continuous SLERP rotation follower
        self.gizmo_follower_l = CriticallyDampedFollower(natural_freq_hz=30.0, dt=self.control_dt)
        self.gizmo_follower_r = CriticallyDampedFollower(natural_freq_hz=30.0, dt=self.control_dt)
        self.gizmo_follower_l.reset(self.robot_home_pos_l)
        self.gizmo_follower_r.reset(self.robot_home_pos_r)
        self.gizmo_rot_follower_l = QuaternionSlerpFollower(bandwidth_hz=30.0, dt=self.control_dt)
        self.gizmo_rot_follower_r = QuaternionSlerpFollower(bandwidth_hz=30.0, dt=self.control_dt)
        self.gizmo_rot_follower_l.reset(self.robot_home_rot_l)
        self.gizmo_rot_follower_r.reset(self.robot_home_rot_r)
        self.output_filter_left_gizmo = JointTrajectoryFilter(num_joints=7, max_vel=6.0, max_acc=80.0, max_jerk=250.0, dt=self.control_dt)
        self.output_filter_right_gizmo = JointTrajectoryFilter(num_joints=7, max_vel=6.0, max_acc=80.0, max_jerk=250.0, dt=self.control_dt)
        self.output_filter_left_gizmo.reset(self.env.get_left_arm_qpos())
        self.output_filter_right_gizmo.reset(self.env.get_right_arm_qpos())
        self.last_ik_sol_left = self.env.get_left_arm_qpos().copy()
        self.last_ik_sol_right = self.env.get_right_arm_qpos().copy()
        self.gizmo_active_l = False
        self.gizmo_active_r = False
        self.step_counter = 0

        # Self-Collision Checker & Safety Gate
        self.collision_checker = CollisionChecker(self.env.model, enabled=self.self_collision)
        self.left_arm_qadr = [self.env.model.jnt_qposadr[mujoco.mj_name2id(self.env.model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{13+i}_left_arm_joint{i+1}")] for i in range(7)]
        self.right_arm_qadr = [self.env.model.jnt_qposadr[mujoco.mj_name2id(self.env.model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{20+i}_right_arm_joint{i+1}")] for i in range(7)]

        # 5. Optional XR digital twin
        if self.xr_view:
            try:
                from teleop.xr.xr_robot_view import XRRobotRenderer
                self.xr_renderer = XRRobotRenderer(SCENE_XML)
            except Exception:
                self.xr_renderer = None
        else:
            self.xr_renderer = None

        # 6. Web Panel & 3D Streaming Server
        print(f"[INFO] Starting HTTPS Server on port {self.port}...")
        self.server = WebPanelServer(
            port=self.port,
            cert_file=self.cert_path,
            key_file=self.key_path,
            scene_xml=SCENE_XML
        )

        # 7. WebXR Receiver (Idle until Quest connects)
        from teleop.tracking.webxr_vuer_handler import WebXRReceiver
        self.xr_receiver = WebXRReceiver(use_controller=self.use_controller)
        self.server.xr_receiver = self.xr_receiver
        self.tracker = self.xr_receiver

        # 8. Meta Quest ADB Auto-Setup & Watcher
        self.quest_manager = QuestAdbManager(port=self.port, no_adb=self.no_adb, no_launch=self.no_quest_launch)
        self.server.quest_manager = self.quest_manager
        self.quest_manager.start_watcher()

        self.server.start()

        # State Variables
        self.cur_target_pos_l = self.robot_home_pos_l.copy()
        self.cur_target_pos_r = self.robot_home_pos_r.copy()
        self.cur_target_rot_l = self.robot_home_rot_l.copy()
        self.cur_target_rot_r = self.robot_home_rot_r.copy()

        self.last_arm_cmd_left = self.env.get_left_arm_qpos()
        self.last_arm_cmd_right = self.env.get_right_arm_qpos()
        self.last_hand_cmd_left = np.zeros(6)
        self.last_hand_cmd_right = np.zeros(6)

        self.target_curls_l = np.zeros(5)
        self.target_pinch_l = 0.0
        self.target_curls_r = np.zeros(5)
        self.target_pinch_r = 0.0

        self.active_sequence = None
        self.seq_start_time = 0.0
        self.seq_origin_l = None
        self.seq_origin_r = None
        self.walking_demo_active = False
        self.walk_start_time = 0.0

        self.state_estop = False
        self.manual_held = False
        self.running = True
        self.nan_detected = False
        self.nan_logged_warning = False
        self.last_cmd_name = "None"
        self.last_cmd_time = 0.0
        self.cmd_count = 0
        self.cmd_rate_hz = 0.0
        self.cmd_rate_timer = time.time()
        self.status_warning = ""

        # Quest 3S WebXR Teleoperation Configuration & Clutch State
        self.quest_config_path = PROJECT_ROOT / "config" / "quest3s.yaml"
        self.quest_cfg = {
            "controller_axis_map": [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            "scale": 1.0,
            "rotation_enabled": True,
            "deadband_m": 0.002,
            "grip_threshold": 0.25,
            "tracking_timeout_s": 0.35,
            "estop_combo_hold_s": 0.5
        }
        if self.quest_config_path.exists():
            try:
                with open(self.quest_config_path, "r", encoding="utf-8") as f:
                    loaded_cfg = yaml.safe_load(f)
                    if isinstance(loaded_cfg, dict):
                        self.quest_cfg.update(loaded_cfg)
            except Exception as e:
                print(f"[WARN] Failed to load quest3s.yaml: {e}")

        self.xr_axis_map = np.array(self.quest_cfg.get("controller_axis_map", [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]), dtype=np.float64)
        self.xr_scale = float(self.quest_cfg.get("scale", 1.0))
        self.xr_rotation_enabled = bool(self.quest_cfg.get("rotation_enabled", True))
        self.xr_deadband = float(self.quest_cfg.get("deadband_m", 0.002))
        self.xr_grip_thresh = float(self.quest_cfg.get("grip_threshold", 0.25))
        self.xr_timeout = float(self.quest_cfg.get("tracking_timeout_s", 0.35))
        self.xr_estop_hold_thresh = float(self.quest_cfg.get("estop_combo_hold_s", 0.5))

        self.xr_input_active = False
        self.xr_last_packet_time = 0.0
        self.xr_pkt_count = 0
        self.xr_rate_hz = 0.0
        self.xr_rate_timer = time.time()
        self.xr_grip_active_l = False
        self.xr_grip_active_r = False
        self.xr_anchor_ctrl_pos_l = None
        self.xr_anchor_ctrl_pos_r = None
        self.xr_anchor_ctrl_rot_l = None
        self.xr_anchor_ctrl_rot_r = None
        self.xr_anchor_marker_pos_l = None
        self.xr_anchor_marker_pos_r = None
        self.xr_anchor_marker_rot_l = None
        self.xr_anchor_marker_rot_r = None
        self.xr_prev_btn_b = False
        self.xr_prev_btn_y = False
        self.xr_estop_combo_start_time = None

        # Setup Clean Shutdown Signals
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _handle_signal(self, signum, frame):
        print("\n[INFO] Shutdown signal received. Cleaning up resources...")
        self.running = False

    def _solve_arm_ik(self, arm="both"):
        ee_poses = self.env.get_ee_poses()

        if hasattr(self.ik_left, "conf") and self.ik_left.conf is not None:
            self.ik_left.conf.data.qpos[:] = self.env.data.qpos
        if hasattr(self.ik_right, "conf") and self.ik_right.conf is not None:
            self.ik_right.conf.data.qpos[:] = self.env.data.qpos

        slid_blocked_l = False
        slid_blocked_r = False

        if arm in ("left", "both"):
            if self.gizmo_active_l:
                # Direct critically damped follower (no One Euro, no dead zone, no 0.3s blend)
                follow_pos_l = self.gizmo_follower_l.update(self.cur_target_pos_l, self.control_dt)
                follow_rot_l = self.gizmo_rot_follower_l.update(self.cur_target_rot_l, self.control_dt)
                slid_pos_l, slid_blocked_l = self.collision_checker.project_target_tangent(
                    ee_poses["left"]["pos"], follow_pos_l, self.env.data
                )
                warm_q_l = getattr(self, "last_ik_sol_left", self.env.get_left_arm_qpos())
                q_sol_l, _, _, _, _ = self.ik_left.solve(
                    target_pos=slid_pos_l,
                    target_rot=follow_rot_l,
                    init_qpos=warm_q_l,
                    max_iters=14,
                    pos_tol=0.001, rot_tol=0.035
                )
                self.last_ik_sol_left = q_sol_l.copy()
                cand_left = self.output_filter_left_gizmo.filter(q_sol_l)
            else:
                now_t = time.perf_counter()
                filt_pos_l, filt_rot_l = self.input_filter_left.filter(self.cur_target_pos_l, self.cur_target_rot_l, timestamp=now_t)
                ramped_pos_l = self.clutch_ramp_left.blend(filt_pos_l)

                q_sol_l, _, _, _, _ = self.ik_left.solve(
                    target_pos=ramped_pos_l,
                    target_rot=filt_rot_l,
                    init_qpos=self.last_arm_cmd_left,
                    max_iters=12,
                    pos_tol=0.002, rot_tol=0.035
                )
                cand_left = self.output_filter_left.filter(q_sol_l)
        else:
            cand_left = self.last_arm_cmd_left.copy()

        if arm in ("right", "both"):
            if self.gizmo_active_r:
                follow_pos_r = self.gizmo_follower_r.update(self.cur_target_pos_r, self.control_dt)
                follow_rot_r = self.gizmo_rot_follower_r.update(self.cur_target_rot_r, self.control_dt)
                slid_pos_r, slid_blocked_r = self.collision_checker.project_target_tangent(
                    ee_poses["right"]["pos"], follow_pos_r, self.env.data
                )
                warm_q_r = getattr(self, "last_ik_sol_right", self.env.get_right_arm_qpos())
                q_sol_r, _, _, _, _ = self.ik_right.solve(
                    target_pos=slid_pos_r,
                    target_rot=follow_rot_r,
                    init_qpos=warm_q_r,
                    max_iters=14,
                    pos_tol=0.001, rot_tol=0.035
                )
                self.last_ik_sol_right = q_sol_r.copy()
                cand_right = self.output_filter_right_gizmo.filter(q_sol_r)
            else:
                now_t = time.perf_counter()
                filt_pos_r, filt_rot_r = self.input_filter_right.filter(self.cur_target_pos_r, self.cur_target_rot_r, timestamp=now_t)
                ramped_pos_r = self.clutch_ramp_right.blend(filt_pos_r)

                q_sol_r, _, _, _, _ = self.ik_right.solve(
                    target_pos=ramped_pos_r,
                    target_rot=filt_rot_r,
                    init_qpos=self.last_arm_cmd_right,
                    max_iters=12,
                    pos_tol=0.002, rot_tol=0.035
                )
                cand_right = self.output_filter_right.filter(q_sol_r)
        else:
            cand_right = self.last_arm_cmd_right.copy()

        # Gate 3: SafetyLayer self-collision verification & line-search sub-step interpolation
        if self.self_collision and self.collision_checker.enabled:
            curr_qpos = self.env.data.qpos.copy()
            target_qpos = curr_qpos.copy()
            target_qpos[self.left_arm_qadr] = cand_left
            target_qpos[self.right_arm_qadr] = cand_right

            # Run collision check with hysteresis (<0.5 cm block, >1.0 cm release)
            safe_qpos, blocked, min_dist = self.collision_checker.filter_step(curr_qpos, target_qpos)
            
            margin_block = 0.005
            margin_release = 0.010
            is_currently_blocked = getattr(self, "collision_blocked", False)
            if is_currently_blocked:
                hard_stop = blocked or (min_dist < margin_release)
            else:
                hard_stop = blocked or (min_dist < margin_block)

            collision_stopped = hard_stop or (self.gizmo_active_l and slid_blocked_l) or (self.gizmo_active_r and slid_blocked_r)

            self.collision_blocked = bool(collision_stopped)
            self.min_collision_distance_m = float(min_dist)

            if collision_stopped:
                self.status_warning = "Self-collision blocked"
            
            if hard_stop:
                self.last_arm_cmd_left = safe_qpos[self.left_arm_qadr].copy()
                self.last_arm_cmd_right = safe_qpos[self.right_arm_qadr].copy()
            else:
                self.last_arm_cmd_left = cand_left
                self.last_arm_cmd_right = cand_right
        else:
            self.collision_blocked = False
            self.last_arm_cmd_left = cand_left
            self.last_arm_cmd_right = cand_right

        self.env.set_arm_commands(self.last_arm_cmd_left, self.last_arm_cmd_right)

    def _handle_panel_command(self, cmd: dict):
        name = cmd.get("name") or cmd.get("cmd") or ""
        self.last_cmd_name = name
        self.last_cmd_time = time.time() * 1000.0
        self.cmd_count += 1

        if name == "estop":
            self.state_estop = True
            print("[ALERT] E-STOP ACTIVE")
            return
        elif name == "clear_estop":
            self.state_estop = False
            print("[INFO] E-STOP CLEARED")
            return
        elif name == "hold_toggle":
            self.manual_held = not self.manual_held
            return
        elif name in ("walk_toggle", "walk_demo"):
            self.walking_demo_active = not self.walking_demo_active
            if not self.walking_demo_active:
                # restore legs
                if hasattr(self.env, "non_manip_qpos_indices"):
                    self.env.data.qpos[self.env.non_manip_qpos_indices] = 0.0
            else:
                self.walk_start_time = time.time()
            return

        if self.state_estop:
            self.status_warning = "Command ignored: E-STOP is active."
            return

        if name in ("arm_jog", "jog_l", "jog_r"):
            arm = cmd.get("arm", "left" if name == "jog_l" else "right")
            axis_val = cmd.get("axis", 0)
            if isinstance(axis_val, str):
                axis_val = axis_val.lower()
                if axis_val in ("x", "0"): axis = 0
                elif axis_val in ("y", "1"): axis = 1
                elif axis_val in ("z", "2"): axis = 2
                elif axis_val in ("roll", "pitch", "yaw"):
                    r_axis = 0 if axis_val == "roll" else (1 if axis_val == "pitch" else 2)
                    direction = float(cmd.get("dir", 1.0))
                    step_deg = float(cmd.get("step_deg", 5.0))
                    rot_vec = np.zeros(3)
                    rot_vec[r_axis] = np.radians(step_deg) * direction
                    # Local import removed in favor of top-level import
                    dR = R.from_rotvec(rot_vec).as_matrix()
                    if arm == "left":
                        self.cur_target_rot_l = dR @ self.cur_target_rot_l
                        self._solve_arm_ik("left")
                    else:
                        self.cur_target_rot_r = dR @ self.cur_target_rot_r
                        self._solve_arm_ik("right")
                    return
                else: axis = 0
            else:
                axis = int(axis_val)

            direction = float(cmd.get("dir", 1.0))
            if "step_cm" in cmd:
                step = float(cmd.get("step_cm", 2.0)) / 100.0 * direction
            else:
                step = float(cmd.get("step", 0.02)) * (direction if "dir" in cmd else 1.0)

            if arm == "left":
                new_t = self.cur_target_pos_l.copy()
                new_t[axis] += step
                new_t[0] = np.clip(new_t[0], -0.2, 0.9)
                new_t[1] = np.clip(new_t[1], -0.9, 0.9)
                new_t[2] = np.clip(new_t[2], 0.5, 1.8)
                self.cur_target_pos_l = new_t
                self.gizmo_active_l = True
                self._solve_arm_ik("left")
            else:
                new_t = self.cur_target_pos_r.copy()
                new_t[axis] += step
                new_t[0] = np.clip(new_t[0], -0.2, 0.9)
                new_t[1] = np.clip(new_t[1], -0.9, 0.9)
                new_t[2] = np.clip(new_t[2], 0.5, 1.8)
                self.cur_target_pos_r = new_t
                self.gizmo_active_r = True
                self._solve_arm_ik("right")

        elif name == "xr_controllers":
            now_t = time.time()
            self.xr_last_packet_time = now_t
            self.xr_input_active = True
            self.xr_pkt_count += 1

            left_pkt = cmd.get("left")
            right_pkt = cmd.get("right")
            rot_enabled = cmd.get("rotation_enabled", self.xr_rotation_enabled)

            # Button states extraction
            btn_x = False
            btn_y = False
            btn_a = False
            btn_b = False
            if left_pkt:
                btn_x = bool(left_pkt.get("btn_x", left_pkt.get("buttons", {}).get("primary", False)))
                btn_y = bool(left_pkt.get("btn_y", left_pkt.get("buttons", {}).get("secondary", False)))
            if right_pkt:
                btn_a = bool(right_pkt.get("btn_a", right_pkt.get("buttons", {}).get("primary", False)))
                btn_b = bool(right_pkt.get("btn_b", right_pkt.get("buttons", {}).get("secondary", False)))

            # E-STOP Combo: Hold B + Y simultaneously > 0.5s
            if btn_b and btn_y:
                if self.xr_estop_combo_start_time is None:
                    self.xr_estop_combo_start_time = now_t
                elif (now_t - self.xr_estop_combo_start_time) >= self.xr_estop_hold_thresh:
                    if not self.state_estop:
                        self.state_estop = True
                        print("[ALERT] E-STOP ACTIVE (Quest Combo B+Y held > 0.5s)")
            else:
                self.xr_estop_combo_start_time = None

            # Toggle Hold: single press B or Y (rising edge, if not both held)
            if not (btn_b and btn_y):
                if btn_b and not self.xr_prev_btn_b:
                    self.manual_held = not self.manual_held
                    print(f"[INFO] Hold toggled via Quest Button B: {'HELD' if self.manual_held else 'TRACKING'}")
                if btn_y and not self.xr_prev_btn_y:
                    self.manual_held = not self.manual_held
                    print(f"[INFO] Hold toggled via Quest Button Y: {'HELD' if self.manual_held else 'TRACKING'}")

            self.xr_prev_btn_b = btn_b
            self.xr_prev_btn_y = btn_y

            # Reset Target: X resets Left marker to EE; A resets Right marker to EE
            ee_poses = self.env.get_ee_poses()
            if btn_x:
                self.cur_target_pos_l = ee_poses["left"]["pos"].copy()
                self.cur_target_rot_l = ee_poses["left"]["rot"].copy()
                self.gizmo_follower_l.reset(self.cur_target_pos_l)
                self.gizmo_rot_follower_l.reset(self.cur_target_rot_l)
                self.xr_grip_active_l = False
            if btn_a:
                self.cur_target_pos_r = ee_poses["right"]["pos"].copy()
                self.cur_target_rot_r = ee_poses["right"]["rot"].copy()
                self.gizmo_follower_r.reset(self.cur_target_pos_r)
                self.gizmo_rot_follower_r.reset(self.cur_target_rot_r)
                self.xr_grip_active_r = False

            # Analog triggers -> hand retargeting
            if left_pkt and "trigger" in left_pkt:
                trig_l = float(left_pkt["trigger"])
                self.target_curls_l = np.full(5, np.clip(trig_l, 0.0, 1.0))
            if right_pkt and "trigger" in right_pkt:
                trig_r = float(right_pkt["trigger"])
                self.target_curls_r = np.full(5, np.clip(trig_r, 0.0, 1.0))

            # Smart Clutch State Machine per arm
            # Left Arm Clutch
            if left_pkt and left_pkt.get("pos") is not None:
                p_ctrl_l = np.array(left_pkt["pos"], dtype=np.float64)
                q_ctrl_l = left_pkt.get("quat")  # [w, x, y, z]
                grip_l = bool(left_pkt.get("grip", False) or left_pkt.get("grip_val", 0.0) >= self.xr_grip_thresh)

                if grip_l:
                    if not self.xr_grip_active_l:
                        # Rising edge: capture anchors
                        self.xr_grip_active_l = True
                        self.xr_anchor_ctrl_pos_l = p_ctrl_l.copy()
                        self.xr_anchor_marker_pos_l = self.cur_target_pos_l.copy()
                        if q_ctrl_l is not None and len(q_ctrl_l) == 4:
                            self.xr_anchor_ctrl_rot_l = R.from_quat([q_ctrl_l[1], q_ctrl_l[2], q_ctrl_l[3], q_ctrl_l[0]]).as_matrix()
                            self.xr_anchor_marker_rot_l = self.cur_target_rot_l.copy()
                    else:
                        # While gripped: delta tracking
                        delta_ctrl_xr = p_ctrl_l - self.xr_anchor_ctrl_pos_l
                        if np.linalg.norm(delta_ctrl_xr) >= self.xr_deadband:
                            delta_rob = self.xr_axis_map @ delta_ctrl_xr
                            new_pos = self.xr_anchor_marker_pos_l + self.xr_scale * delta_rob
                            # Workspace clamp
                            new_pos[0] = np.clip(new_pos[0], -0.2, 0.9)
                            new_pos[1] = np.clip(new_pos[1], -0.9, 0.9)
                            new_pos[2] = np.clip(new_pos[2], 0.5, 1.8)
                            self.cur_target_pos_l = new_pos
                            self.gizmo_active_l = True

                        if rot_enabled and q_ctrl_l is not None and len(q_ctrl_l) == 4 and self.xr_anchor_ctrl_rot_l is not None:
                            r_curr_ctrl = R.from_quat([q_ctrl_l[1], q_ctrl_l[2], q_ctrl_l[3], q_ctrl_l[0]]).as_matrix()
                            r_rel_xr = r_curr_ctrl @ self.xr_anchor_ctrl_rot_l.T
                            r_rel_rob = self.xr_axis_map @ r_rel_xr @ self.xr_axis_map.T
                            self.cur_target_rot_l = r_rel_rob @ self.xr_anchor_marker_rot_l
                else:
                    if self.xr_grip_active_l:
                        self.xr_grip_active_l = False

            # Right Arm Clutch
            if right_pkt and right_pkt.get("pos") is not None:
                p_ctrl_r = np.array(right_pkt["pos"], dtype=np.float64)
                q_ctrl_r = right_pkt.get("quat")  # [w, x, y, z]
                grip_r = bool(right_pkt.get("grip", False) or right_pkt.get("grip_val", 0.0) >= self.xr_grip_thresh)

                if grip_r:
                    if not self.xr_grip_active_r:
                        # Rising edge: capture anchors
                        self.xr_grip_active_r = True
                        self.xr_anchor_ctrl_pos_r = p_ctrl_r.copy()
                        self.xr_anchor_marker_pos_r = self.cur_target_pos_r.copy()
                        if q_ctrl_r is not None and len(q_ctrl_r) == 4:
                            self.xr_anchor_ctrl_rot_r = R.from_quat([q_ctrl_r[1], q_ctrl_r[2], q_ctrl_r[3], q_ctrl_r[0]]).as_matrix()
                            self.xr_anchor_marker_rot_r = self.cur_target_rot_r.copy()
                    else:
                        # While gripped: delta tracking
                        delta_ctrl_xr = p_ctrl_r - self.xr_anchor_ctrl_pos_r
                        if np.linalg.norm(delta_ctrl_xr) >= self.xr_deadband:
                            delta_rob = self.xr_axis_map @ delta_ctrl_xr
                            new_pos = self.xr_anchor_marker_pos_r + self.xr_scale * delta_rob
                            # Workspace clamp
                            new_pos[0] = np.clip(new_pos[0], -0.2, 0.9)
                            new_pos[1] = np.clip(new_pos[1], -0.9, 0.9)
                            new_pos[2] = np.clip(new_pos[2], 0.5, 1.8)
                            self.cur_target_pos_r = new_pos
                            self.gizmo_active_r = True

                        if rot_enabled and q_ctrl_r is not None and len(q_ctrl_r) == 4 and self.xr_anchor_ctrl_rot_r is not None:
                            r_curr_ctrl = R.from_quat([q_ctrl_r[1], q_ctrl_r[2], q_ctrl_r[3], q_ctrl_r[0]]).as_matrix()
                            r_rel_xr = r_curr_ctrl @ self.xr_anchor_ctrl_rot_r.T
                            r_rel_rob = self.xr_axis_map @ r_rel_xr @ self.xr_axis_map.T
                            self.cur_target_rot_r = r_rel_rob @ self.xr_anchor_marker_rot_r
                else:
                    if self.xr_grip_active_r:
                        self.xr_grip_active_r = False

            if self.xr_grip_active_l and self.xr_grip_active_r:
                self._solve_arm_ik("both")
            elif self.xr_grip_active_l:
                self._solve_arm_ik("left")
            elif self.xr_grip_active_r:
                self._solve_arm_ik("right")

        elif name in ("set_wrist_target", "wrist_target"):
            arm = cmd.get("arm", cmd.get("side", "left"))
            pos = cmd.get("pos")
            rot_quat = cmd.get("rot_quat")
            if pos is not None and len(pos) == 3:
                p = np.array(pos, dtype=np.float64)
                p_orig = p.copy()
                p[0] = np.clip(p[0], -0.2, 0.9)
                p[1] = np.clip(p[1], -0.9, 0.9)
                p[2] = np.clip(p[2], 0.5, 1.8)
                if arm == "left":
                    if np.linalg.norm(p - p_orig) > 1e-4:
                        self.status_warning = "Left wrist target clamped to workspace"
                    self.cur_target_pos_l = p
                    self.gizmo_active_l = True
                    if hasattr(self, "input_filter_left"):
                        self.input_filter_left.reset()
                else:
                    if np.linalg.norm(p - p_orig) > 1e-4:
                        self.status_warning = "Right wrist target clamped to workspace"
                    self.cur_target_pos_r = p
                    self.gizmo_active_r = True
                    if hasattr(self, "input_filter_right"):
                        self.input_filter_right.reset()

            if rot_quat is not None and len(rot_quat) == 4:
                scipy_quat = [rot_quat[1], rot_quat[2], rot_quat[3], rot_quat[0]]
                rot_mat = R.from_quat(scipy_quat).as_matrix()
                if arm == "left": self.cur_target_rot_l = rot_mat
                else: self.cur_target_rot_r = rot_mat

            self._solve_arm_ik(arm)

        elif name in ("hand", "hand_l", "hand_r"):
            arm = cmd.get("arm", "left" if name == "hand_l" else "right")
            action = cmd.get("action", "")
            if action == "open": curls, pinch = np.zeros(5), 0.0
            elif action == "close": curls, pinch = np.ones(5), 0.0
            elif action == "pinch": curls, pinch = np.array([0.9, 0.9, 0.1, 0.1, 0.1]), 1.0
            else:
                val = float(cmd.get("value", cmd.get("closure", 0.0)))
                curls, pinch = np.full(5, val), 0.0

            if arm == "left": self.target_curls_l, self.target_pinch_l = curls, pinch
            else: self.target_curls_r, self.target_pinch_r = curls, pinch

        elif name in ("preset", "reset"):
            pose = cmd.get("pose", "home") if name == "preset" else "reset"
            if pose in ("home", "reset"):
                self.active_sequence = None
                self.walking_demo_active = False
                self.env.reset()
                self.last_arm_cmd_left = self.env.get_left_arm_qpos()
                self.last_arm_cmd_right = self.env.get_right_arm_qpos()
                self.last_hand_cmd_left = np.zeros(6)
                self.last_hand_cmd_right = np.zeros(6)
                home_p = self.env.get_ee_poses()
                self.cur_target_pos_l = home_p["left"]["pos"].copy()
                self.cur_target_pos_r = home_p["right"]["pos"].copy()
                self.cur_target_rot_l = home_p["left"]["rot"].copy()
                self.cur_target_rot_r = home_p["right"]["rot"].copy()
                if hasattr(self, "input_filter_left"): self.input_filter_left.reset()
                if hasattr(self, "input_filter_right"): self.input_filter_right.reset()
                if hasattr(self, "output_filter_left"): self.output_filter_left.reset(self.last_arm_cmd_left)
                if hasattr(self, "output_filter_right"): self.output_filter_right.reset(self.last_arm_cmd_right)
                if hasattr(self, "gizmo_follower_l"): self.gizmo_follower_l.reset(self.cur_target_pos_l)
                if hasattr(self, "gizmo_follower_r"): self.gizmo_follower_r.reset(self.cur_target_pos_r)
                if hasattr(self, "gizmo_rot_follower_l"): self.gizmo_rot_follower_l.reset(self.cur_target_rot_l)
                if hasattr(self, "gizmo_rot_follower_r"): self.gizmo_rot_follower_r.reset(self.cur_target_rot_r)
                if hasattr(self, "output_filter_left_gizmo"): self.output_filter_left_gizmo.reset(self.last_arm_cmd_left)
                if hasattr(self, "output_filter_right_gizmo"): self.output_filter_right_gizmo.reset(self.last_arm_cmd_right)
                self.last_ik_sol_left = self.last_arm_cmd_left.copy()
                self.last_ik_sol_right = self.last_arm_cmd_right.copy()
                self.gizmo_active_l = False
                self.gizmo_active_r = False
                self.xr_grip_active_l = False
                self.xr_grip_active_r = False
            elif pose == "forward":
                self.active_sequence = None
                self.cur_target_pos_l = self.robot_home_pos_l + np.array([0.25, 0.0, 0.0])
                self.cur_target_pos_r = self.robot_home_pos_r + np.array([0.25, 0.0, 0.0])
                self._solve_arm_ik("both")
            elif pose == "up":
                self.active_sequence = None
                self.cur_target_pos_l = self.robot_home_pos_l + np.array([0.05, 0.0, 0.25])
                self.cur_target_pos_r = self.robot_home_pos_r + np.array([0.05, 0.0, 0.25])
                self._solve_arm_ik("both")
            elif pose == "tpose":
                self.active_sequence = None
                self.cur_target_pos_l = self.robot_home_pos_l + np.array([0.0, 0.20, 0.0])
                self.cur_target_pos_r = self.robot_home_pos_r + np.array([0.0, -0.20, 0.0])
                self._solve_arm_ik("both")
            elif pose == "wave":
                self.active_sequence = "wave"
                self.seq_start_time = time.time()
                self.seq_origin_r = self.cur_target_pos_r.copy()
                self.gizmo_active_r = True
            elif pose == "circle":
                self.active_sequence = "circle"
                self.seq_start_time = time.time()
                self.seq_origin_l = self.cur_target_pos_l.copy()
                self.seq_origin_r = self.cur_target_pos_r.copy()
            elif pose == "grasp":
                self.active_sequence = "grasp"
                self.seq_start_time = time.time()
            elif pose == "stop":
                self.active_sequence = None

    def _update_sequence(self, dt: float):
        t_seq = time.time() - self.seq_start_time
        if self.active_sequence == "wave":
            if t_seq > 3.0:
                self.active_sequence = None
                return
            omega = 2.0 * np.pi * 1.5
            dy = 0.08 * np.sin(omega * t_seq)
            dz = 0.04 * np.cos(omega * t_seq)
            self.cur_target_pos_r = self.seq_origin_r + np.array([0.0, dy, dz])
            self._solve_arm_ik("right")
        elif self.active_sequence == "circle":
            omega = 2.0 * np.pi * 0.5
            dy = 0.06 * np.cos(omega * t_seq)
            dz = 0.06 * np.sin(omega * t_seq)
            self.cur_target_pos_l = self.seq_origin_l + np.array([0.0, dy, dz])
            self.cur_target_pos_r = self.seq_origin_r + np.array([0.0, -dy, dz])
            self._solve_arm_ik("both")
        elif self.active_sequence == "grasp":
            if t_seq < 1.0:
                frac = t_seq / 1.0
                self.cur_target_pos_l = self.robot_home_pos_l + frac * np.array([0.25, 0.0, 0.0])
                self.cur_target_pos_r = self.robot_home_pos_r + frac * np.array([0.25, 0.0, 0.0])
                self._solve_arm_ik("both")
            elif t_seq < 2.0:
                self.target_curls_l = np.ones(5)
                self.target_curls_r = np.ones(5)
            elif t_seq < 3.0:
                self.target_curls_l = np.zeros(5)
                self.target_curls_r = np.zeros(5)
            else:
                self.active_sequence = None

    def _update_walk_demo(self):
        t = (time.time() - self.walk_start_time)
        omega = 2.0 * np.pi * 0.8
        swing = 0.25 * np.sin(omega * t)
        if hasattr(self.env, "qpos_idx_map"):
            if "idx03_left_hip_pitch" in self.env.qpos_idx_map:
                self.env.data.qpos[self.env.qpos_idx_map["idx03_left_hip_pitch"]] = swing
            if "idx09_right_hip_pitch" in self.env.qpos_idx_map:
                self.env.data.qpos[self.env.qpos_idx_map["idx09_right_hip_pitch"]] = -swing

    def run(self):
        dt = getattr(self, "control_dt", 0.01)
        step = 0
        local_ip = get_local_ip()

        has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
        viewer_label = "Active (Passive GUI Window)" if (self.env.viewer is not None) else ("Disabled (No display)" if not has_display else "OFF")
        initial_status = {
            "sim": "MuJoCo (home keyframe, legs locked, arms active)",
            "viewer": viewer_label,
            "server": f"HTTPS/WSS Listening on 0.0.0.0:{self.port}",
            "panel": "Ready (3D Viewport + Gizmos + Buttons)",
            "xr_receiver": "Active & Listening (Meta Quest 3S WebXR)" if self.tracker else "Idle",
            "safety": "Active (Joint limits, velocity clamps, E-Stop cleared)",
        }
        quest_status = self.quest_manager.get_status() if hasattr(self, "quest_manager") else {}
        print_startup_banner(self.port, local_ip, initial_status, quest_status)

        try:
            target_step_dt = dt / self.speed_scale
            next_step_time = time.perf_counter()

            while self.running:
                t_step_start = time.perf_counter()

                # Process Web Panel commands
                cmd = self.server.get_command()
                while cmd is not None:
                    self._handle_panel_command(cmd)
                    cmd = self.server.get_command()

                # Quest 3S Watchdog: if no packet for timeout, release clutches
                now_t = time.time()
                if self.xr_input_active and (now_t - self.xr_last_packet_time) > self.xr_timeout:
                    if self.xr_grip_active_l or self.xr_grip_active_r:
                        self.xr_grip_active_l = False
                        self.xr_grip_active_r = False
                        self.status_warning = "Quest XR tracking lost (watchdog 350ms): clutches released"
                    self.xr_input_active = False

                if self.active_sequence:
                    self._update_sequence(dt)
                elif not self.state_estop:
                    # Continuous tracking convergence for current targets
                    self._solve_arm_ik("both")
                if self.walking_demo_active:
                    self._update_walk_demo()

                # Step simulation
                if not self.state_estop:
                    self.last_hand_cmd_left = self.hand_retarget_left.retarget_curls(self.target_curls_l, pinch_strength=self.target_pinch_l, dt=dt)
                    self.last_hand_cmd_right = self.hand_retarget_right.retarget_curls(self.target_curls_r, pinch_strength=self.target_pinch_r, dt=dt)
                    self.env.set_arm_commands(self.last_arm_cmd_left, self.last_arm_cmd_right)
                    self.env.set_hand_commands(self.last_hand_cmd_left, self.last_hand_cmd_right)
                    self.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=self.last_arm_cmd_left, hold_right_qpos=self.last_arm_cmd_right)
                else:
                    self.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=self.last_arm_cmd_left, hold_right_qpos=self.last_arm_cmd_right)

                # Performance tracking telemetry (lag and steady error)
                if step % 20 == 0 and (self.gizmo_active_l or self.gizmo_active_r):
                    ee_now = self.env.get_ee_poses()
                    if self.gizmo_active_l and self.gizmo_follower_l.v is not None:
                        v_mag = float(np.linalg.norm(self.gizmo_follower_l.v))
                        err_m = float(np.linalg.norm(np.array(ee_now["left"]["pos"]) - self.cur_target_pos_l))
                        if v_mag > 0.03:
                            lag_ms = (err_m / v_mag) * 1000.0
                            print(f"[GIZMO TRACK] L lag: {lag_ms:.1f} ms | steady err: {err_m*100.0:.2f} cm (vel: {v_mag:.2f} m/s)")
                    if self.gizmo_active_r and self.gizmo_follower_r.v is not None:
                        v_mag = float(np.linalg.norm(self.gizmo_follower_r.v))
                        err_m = float(np.linalg.norm(np.array(ee_now["right"]["pos"]) - self.cur_target_pos_r))
                        if v_mag > 0.03:
                            lag_ms = (err_m / v_mag) * 1000.0
                            print(f"[GIZMO TRACK] R lag: {lag_ms:.1f} ms | steady err: {err_m*100.0:.2f} cm (vel: {v_mag:.2f} m/s)")

                # Stream ~30 Hz visual body poses to 3D Digital Twin (every 7th step at 200 Hz)
                if step % 7 == 0:
                    poses = self.env.get_visual_body_poses()
                    # NaN / Inf guard
                    has_nan = False
                    for k, v in poses.items():
                        if any(np.isnan(x) or np.isinf(x) for x in v):
                            has_nan = True
                            self.nan_detected = True
                            if not self.nan_logged_warning:
                                print(f"[WARN] NaN/Inf detected in streamed pose for '{k}'. Skipping frame.")
                                self.nan_logged_warning = True
                            break
                    if not has_nan:
                        self.server.broadcast_poses(poses)

                # Telemetry updates (20 Hz at 200 Hz loop = every 10 steps)
                if step % 10 == 0:
                    ee = self.env.get_ee_poses()
                    now_t = time.time()
                    elapsed_rate = now_t - self.cmd_rate_timer
                    if elapsed_rate >= 1.0:
                        self.cmd_rate_hz = self.cmd_count / elapsed_rate
                        self.cmd_count = 0
                        self.cmd_rate_timer = now_t

                    elapsed_xr = now_t - self.xr_rate_timer
                    if elapsed_xr >= 1.0:
                        self.xr_rate_hz = self.xr_pkt_count / elapsed_xr
                        self.xr_pkt_count = 0
                        self.xr_rate_timer = now_t

                    closure_l = float(np.mean(self.last_hand_cmd_left[2:6])) / 1.700
                    closure_r = float(np.mean(self.last_hand_cmd_right[2:6])) / 1.700

                    xr_active = False
                    if self.tracker is not None and getattr(self.tracker, "tv_wrapper", None) is not None:
                        tvuer = getattr(self.tracker.tv_wrapper, "tvuer", None)
                        if tvuer is not None and hasattr(tvuer, "motion_data_ready_shared"):
                            with tvuer.motion_data_ready_shared.get_lock():
                                xr_active = bool(tvuer.motion_data_ready_shared.value)

                    err_l = float(np.linalg.norm(np.array(ee["left"]["pos"]) - self.cur_target_pos_l)) * 1000.0
                    err_r = float(np.linalg.norm(np.array(ee["right"]["pos"]) - self.cur_target_pos_r)) * 1000.0

                    status_packet = {
                        "sim_running": True,
                        "nan_detected": bool(self.nan_detected),
                        "viewer_enabled": bool(self.render_viewer),
                        "viewer_running": bool(self.env.viewer is not None and getattr(self.env.viewer, "is_running", lambda: False)()),
                        "xr_active": xr_active or self.xr_input_active,
                        "input_source": "QUEST VR" if self.xr_input_active else "MOUSE",
                        "xr_controllers_active": bool(self.xr_input_active),
                        "xr_rate_hz": float(self.xr_rate_hz),
                        "xr_grip_l": bool(self.xr_grip_active_l),
                        "xr_grip_r": bool(self.xr_grip_active_r),
                        "ee_l": [float(x) for x in ee["left"]["pos"]],
                        "ee_r": [float(x) for x in ee["right"]["pos"]],
                        "rot_l": [[float(c) for c in row] for row in ee["left"]["rot"]],
                        "rot_r": [[float(c) for c in row] for row in ee["right"]["rot"]],
                        "tgt_l": [float(x) for x in self.cur_target_pos_l],
                        "tgt_r": [float(x) for x in self.cur_target_pos_r],
                        "ik_err_l": err_l,
                        "ik_err_r": err_r,
                        "hand_closure_l": float(np.clip(closure_l, 0.0, 1.0)),
                        "hand_closure_r": float(np.clip(closure_r, 0.0, 1.0)),
                        "held": bool(self.manual_held),
                        "estop": bool(self.state_estop),
                        "collision_blocked": bool(self.collision_blocked),
                        "min_distance_mm": float(self.min_collision_distance_m * 1000.0),
                        "qpos_l": [float(x) for x in self.last_arm_cmd_left],
                        "qpos_r": [float(x) for x in self.last_arm_cmd_right],
                        "walking": bool(self.walking_demo_active),
                        "cmd_rate_hz": float(self.cmd_rate_hz),
                        "last_cmd_name": str(self.last_cmd_name),
                        "last_cmd_time": float(self.last_cmd_time),
                        "warning": str(self.status_warning)
                    }
                    self.server.set_status(status_packet)
                    self.status_warning = ""

                step += 1
                next_step_time += target_step_dt
                now_perf = time.perf_counter()
                sleep_t = next_step_time - now_perf
                if sleep_t > 0.0005:
                    time.sleep(sleep_t)
                elif sleep_t < -0.05:
                    next_step_time = now_perf

        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self):
        print("[INFO] Shutting down teleoperation application...")
        if hasattr(self, "quest_manager") and self.quest_manager:
            self.quest_manager.stop()
        if hasattr(self, "server") and self.server:
            self.server.stop()
        if hasattr(self, "env") and self.env:
            self.env.close()
        if self.tracker and getattr(self.tracker, "tv_wrapper", None):
            try:
                self.tracker.tv_wrapper.close()
            except Exception:
                pass
        print("[INFO] Cleanup complete. Exiting.")


def main():
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    parser = argparse.ArgumentParser(description="AGIBOT A2 Ultra Teleoperation Unified Launcher")
    parser.add_argument("--port", type=int, default=8012, help="HTTPS server port (default: 8012)")
    parser.add_argument("--viewer", action="store_true", default=None, help="Launch native MuJoCo passive viewer window (accepted alias; enabled by default if display exists)")
    parser.add_argument("--no-viewer", action="store_true", help="Disable native MuJoCo passive viewer window")
    parser.add_argument("--headless", action="store_true", help="Explicitly run headless (forces native viewer OFF)")
    parser.add_argument("--controller", action="store_true", help="Use controller clutch teleop mode")
    parser.add_argument("--xr-view", action="store_true", help="Enable XR digital twin in Quest headset")
    parser.add_argument("--backend", type=str, default="mujoco", choices=["mujoco", "replay"], help="Physics backend")
    parser.add_argument("--speed-scale", type=float, default=1.0, help="Simulation execution speed multiplier")
    parser.add_argument("--ik", type=str, default="mink", choices=["mink", "dls", "pyroki"], help="Inverse kinematics engine (default: mink)")
    parser.add_argument("--self-collision", type=str, default="on", choices=["on", "off"], help="Enable/disable self-collision safety gate (default: on)")
    parser.add_argument("--collision-avoidance", action="store_true", help="Enable mink self-collision avoidance limit")
    parser.add_argument("--no-adb", action="store_true", help="Disable automatic ADB detection and port reversal")
    parser.add_argument("--no-quest-launch", action="store_true", help="Do not automatically launch Quest browser on headset")
    args = parser.parse_args()

    # MuJoCo viewer is ON by default when display exists; --no-viewer or --headless forces it off
    if args.headless or args.no_viewer:
        render_viewer = False
    elif args.viewer is True:
        render_viewer = True
    else:
        render_viewer = has_display

    self_col_enabled = (args.self_collision.lower() == "on")

    app = UnifiedTeleopApp(
        port=args.port,
        render_viewer=render_viewer,
        use_controller=args.controller,
        xr_view=args.xr_view,
        backend=args.backend,
        speed_scale=args.speed_scale,
        ik_solver_type=args.ik,
        self_collision=self_col_enabled,
        no_adb=args.no_adb,
        no_quest_launch=args.no_quest_launch
    )
    app.run()


if __name__ == "__main__":
    main()
