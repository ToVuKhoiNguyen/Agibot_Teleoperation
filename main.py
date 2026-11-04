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
from teleop.kinematics.filters import InputFilterPipeline, ClutchRamp, JointTrajectoryFilter
from teleop.retargeting.agibot_arm_retargeting import AgibotArmRetargeting
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting
from teleop.tracking.quest_tracking import QuestTrackingReceiver
import numpy as np

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


def print_startup_banner(port: int, host_ip: str, status_dict: dict):
    print("\n" + "=" * 80)
    print("READY - AGIBOT A2 ULTRA TELEOPERATION SYSTEM ACTIVE")
    print("=" * 80)
    print(f"1. PC BROWSER (3D PANEL) : https://localhost:{port}/")
    print(f"2. QUEST 3S (USB / ADB)  : https://localhost:{port}/?ws=wss://localhost:{port}")
    print(f"   ADB Reverse Command   : adb reverse tcp:{port} tcp:{port}")
    print(f"   ADB Launch Command    : adb shell am start -a android.intent.action.VIEW -d \"https://localhost:{port}/?ws=wss://localhost:{port}\"")
    print(f"3. QUEST 3S (WI-FI / LAN): https://{host_ip}:{port}/?ws=wss://{host_ip}:{port}")
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
        collision_avoidance: bool = False,
    ):
        self.port = port
        self.render_viewer = render_viewer
        self.use_controller = use_controller
        self.xr_view = xr_view
        self.backend = backend
        self.speed_scale = speed_scale
        self.ik_solver_type = ik_solver_type.lower()
        self.collision_avoidance = collision_avoidance

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
        self.hand_retarget_left = AgibotHandRetargeting(hand="left")
        self.hand_retarget_right = AgibotHandRetargeting(hand="right")

        if self.ik_solver_type == "mink":
            self.ik_left = AgibotMinkIK(SCENE_XML, arm="left", use_collision_avoidance=self.collision_avoidance)
            self.ik_right = AgibotMinkIK(SCENE_XML, arm="right", use_collision_avoidance=self.collision_avoidance)
        else:
            self.ik_left = AgibotA2IK(SCENE_XML, arm="left")
            self.ik_right = AgibotA2IK(SCENE_XML, arm="right")

        # Input filters (One Euro + Slerp + Spike rejection + Deadzone)
        self.input_filter_left = InputFilterPipeline()
        self.input_filter_right = InputFilterPipeline()
        self.clutch_ramp_left = ClutchRamp()
        self.clutch_ramp_right = ClutchRamp()

        # Output trajectory filters (Ruckig / critically damped)
        self.output_filter_left = JointTrajectoryFilter(num_joints=7, max_vel=3.0, max_acc=6.28)
        self.output_filter_right = JointTrajectoryFilter(num_joints=7, max_vel=3.0, max_acc=6.28)
        self.output_filter_left.reset(self.env.get_left_arm_qpos())
        self.output_filter_right.reset(self.env.get_right_arm_qpos())

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

        # Setup Clean Shutdown Signals
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _handle_signal(self, signum, frame):
        print("\n[INFO] Shutdown signal received. Cleaning up resources...")
        self.running = False

    def _solve_arm_ik(self, arm="both"):
        if arm in ("left", "both"):
            # Apply input filters
            filt_pos_l, filt_rot_l = self.input_filter_left.filter(self.cur_target_pos_l, self.cur_target_rot_l)
            ramped_pos_l = self.clutch_ramp_left.blend(filt_pos_l)

            q_sol_l, _, _, _, _ = self.ik_left.solve(
                target_pos=ramped_pos_l,
                target_rot=filt_rot_l,
                init_qpos=self.last_arm_cmd_left,
                max_iters=25,
                pos_tol=0.005, rot_tol=0.035
            )
            # Output trajectory smoothing
            self.last_arm_cmd_left = self.output_filter_left.filter(q_sol_l)

        if arm in ("right", "both"):
            # Apply input filters
            filt_pos_r, filt_rot_r = self.input_filter_right.filter(self.cur_target_pos_r, self.cur_target_rot_r)
            ramped_pos_r = self.clutch_ramp_right.blend(filt_pos_r)

            q_sol_r, _, _, _, _ = self.ik_right.solve(
                target_pos=ramped_pos_r,
                target_rot=filt_rot_r,
                init_qpos=self.last_arm_cmd_right,
                max_iters=25,
                pos_tol=0.005, rot_tol=0.035
            )
            # Output trajectory smoothing
            self.last_arm_cmd_right = self.output_filter_right.filter(q_sol_r)

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
                    from scipy.spatial.transform import Rotation as R
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
                new_t[1] = np.clip(new_t[1], 0.1, 0.9)
                new_t[2] = np.clip(new_t[2], 0.5, 1.8)
                self.cur_target_pos_l = new_t
                self._solve_arm_ik("left")
            else:
                new_t = self.cur_target_pos_r.copy()
                new_t[axis] += step
                new_t[0] = np.clip(new_t[0], -0.2, 0.9)
                new_t[1] = np.clip(new_t[1], -0.9, -0.1)
                new_t[2] = np.clip(new_t[2], 0.5, 1.8)
                self.cur_target_pos_r = new_t
                self._solve_arm_ik("right")

        elif name == "set_wrist_target":
            arm = cmd.get("arm", "left")
            pos = cmd.get("pos")
            rot_quat = cmd.get("rot_quat")
            if pos is not None and len(pos) == 3:
                p = np.array(pos, dtype=np.float64)
                p[0] = np.clip(p[0], -0.2, 0.9)
                p[2] = np.clip(p[2], 0.5, 1.8)
                if arm == "left":
                    p[1] = np.clip(p[1], 0.1, 0.9)
                    self.cur_target_pos_l = p
                else:
                    p[1] = np.clip(p[1], -0.9, -0.1)
                    self.cur_target_pos_r = p

            if rot_quat is not None and len(rot_quat) == 4:
                from scipy.spatial.transform import Rotation as R
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

        elif name == "preset":
            pose = cmd.get("pose", "home")
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
        dt = 0.02
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
        print_startup_banner(self.port, local_ip, initial_status)

        try:
            while self.running:
                t_start = time.time()

                # Process Web Panel commands
                cmd = self.server.get_command()
                while cmd is not None:
                    self._handle_panel_command(cmd)
                    cmd = self.server.get_command()

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
                    self.env.step(num_substeps=5, hold_arms=True, hold_left_qpos=self.last_arm_cmd_left, hold_right_qpos=self.last_arm_cmd_right)
                else:
                    self.env.step(num_substeps=5, hold_arms=True, hold_left_qpos=self.last_arm_cmd_left, hold_right_qpos=self.last_arm_cmd_right)

                # Stream 25 Hz visual body poses to 3D Digital Twin (every 2nd step)
                if step % 2 == 0:
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

                # Telemetry updates (5 Hz)
                if step % 10 == 0:
                    ee = self.env.get_ee_poses()
                    now_t = time.time()
                    elapsed_rate = now_t - self.cmd_rate_timer
                    if elapsed_rate >= 1.0:
                        self.cmd_rate_hz = self.cmd_count / elapsed_rate
                        self.cmd_count = 0
                        self.cmd_rate_timer = now_t

                    closure_l = float(np.mean(self.last_hand_cmd_left[2:6])) / 1.700
                    closure_r = float(np.mean(self.last_hand_cmd_right[2:6])) / 1.700

                    xr_active = False
                    if self.tracker is not None and getattr(self.tracker, "tv_wrapper", None) is not None:
                        tvuer = getattr(self.tracker.tv_wrapper, "tvuer", None)
                        if tvuer is not None and hasattr(tvuer, "motion_data_ready_shared"):
                            with tvuer.motion_data_ready_shared.get_lock():
                                xr_active = bool(tvuer.motion_data_ready_shared.value)

                    status_packet = {
                        "sim_running": True,
                        "nan_detected": bool(self.nan_detected),
                        "viewer_enabled": bool(self.render_viewer),
                        "viewer_running": bool(self.env.viewer is not None and getattr(self.env.viewer, "is_running", lambda: False)()),
                        "xr_active": xr_active,
                        "ee_l": [float(x) for x in ee["left"]["pos"]],
                        "ee_r": [float(x) for x in ee["right"]["pos"]],
                        "tgt_l": [float(x) for x in self.cur_target_pos_l],
                        "tgt_r": [float(x) for x in self.cur_target_pos_r],
                        "ik_err_l": 0.0,
                        "ik_err_r": 0.0,
                        "hand_closure_l": float(np.clip(closure_l, 0.0, 1.0)),
                        "hand_closure_r": float(np.clip(closure_r, 0.0, 1.0)),
                        "held": bool(self.manual_held),
                        "estop": bool(self.state_estop),
                        "walking": bool(self.walking_demo_active),
                        "cmd_rate_hz": float(self.cmd_rate_hz),
                        "last_cmd_name": str(self.last_cmd_name),
                        "last_cmd_time": float(self.last_cmd_time),
                        "warning": str(self.status_warning)
                    }
                    self.server.set_status(status_packet)
                    self.status_warning = ""

                step += 1
                elapsed = time.time() - t_start
                sleep_t = (dt / self.speed_scale) - elapsed
                if sleep_t > 0.001:
                    time.sleep(sleep_t)

        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self):
        print("[INFO] Shutting down teleoperation application...")
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
    parser.add_argument("--ik", type=str, default="mink", choices=["mink", "dls"], help="Inverse kinematics engine (default: mink)")
    parser.add_argument("--collision-avoidance", action="store_true", help="Enable mink self-collision avoidance limit")
    args = parser.parse_args()

    # MuJoCo viewer is ON by default when display exists; --no-viewer or --headless forces it off
    if args.headless or args.no_viewer:
        render_viewer = False
    elif args.viewer is True:
        render_viewer = True
    else:
        render_viewer = has_display

    app = UnifiedTeleopApp(
        port=args.port,
        render_viewer=render_viewer,
        use_controller=args.controller,
        xr_view=args.xr_view,
        backend=args.backend,
        speed_scale=args.speed_scale,
        ik_solver_type=args.ik,
        collision_avoidance=args.collision_avoidance
    )
    app.run()


if __name__ == "__main__":
    main()
