#!/usr/bin/env python3
"""
scripts/run_teleoperation.py
End-to-End Bilateral Arm and Dexterous Hand Teleoperation of AGIBOT A2 Ultra in MuJoCo.
Pipeline: Meta Quest 3S (WebXR) -> Tracking -> Retargeting -> IK -> MuJoCo Sim.
"""

import os
import sys
import time
import argparse
import socket
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.kinematics.agibot_a2_ik import AgibotA2IK
from teleop.retargeting.agibot_arm_retargeting import AgibotArmRetargeting
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting
from teleop.tracking.quest_tracking import QuestTrackingReceiver, MockQuestStreamer

SCENE_XML = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "mjcf", "scene.xml")


def get_local_ip():
    """Gets local LAN IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def print_connection_block(host_ip: str, port: int, panel: bool = False):
    """Prints the exact URL and ADB commands in a single clear block."""
    pc_url = f"https://localhost:{port}/?ws=wss://localhost:{port}"
    panel_url = f"https://localhost:{port}/panel"
    quest_usb_url = f"https://localhost:{port}/?ws=wss://localhost:{port}"
    quest_wifi_url = f"https://{host_ip}:{port}/?ws=wss://{host_ip}:{port}"

    print("\n" + "=" * 80)
    print("AGIBOT A2 ULTRA WEBXR TELEOPERATION SERVER (PROTOCOL: HTTPS)")
    print("=" * 80)
    print(f"1. WEB CONTROL PANEL URL : {panel_url}")
    print(f"2. PC BROWSER XR URL     : {pc_url}")
    print(f"3. QUEST 3S (USB / ADB)  : {quest_usb_url}")
    print(f"   ADB Reverse Command   : adb reverse tcp:{port} tcp:{port}")
    print(f"   ADB Launch Command    : adb shell am start -a android.intent.action.VIEW -d \"{quest_usb_url}\"")
    print(f"4. QUEST 3S (WI-FI / LAN): {quest_wifi_url}")
    print("=" * 80 + "\n")


HELP_EPILOG = """
========================================================================================
QUEST 3S CONNECTION GUIDE (USB & WI-FI):
========================================================================================
1. USB TETHERED CONNECTION:
   Run ADB reverse port-forwarding on this PC:
     $ adb reverse tcp:8012 tcp:8012
   Open in Quest Browser or launch directly:
     $ adb shell am start -a android.intent.action.VIEW -d "https://localhost:8012/?ws=wss://localhost:8012"

2. WI-FI / LAN CONNECTION:
   Open in Quest Browser:
     https://<HOST_IP>:8012/?ws=wss://<HOST_IP>:8012

3. CONTROLS:
   - [ENTER]: Complete initial wrist pose calibration.
   - [r]: Resume / Follow teleoperation motion.
   - [q]: Emergency Stop (E-Stop).
========================================================================================
"""


class TeleopController:
    """
    Main controller orchestrating the full teleoperation loop with calibration.
    Supports both hand-tracking and clutch-based controller teleoperation modes.
    """

    def __init__(
        self,
        scene_xml: str,
        mock: bool = False,
        gui: bool = False,
        timeout_s: float = 0.5,
        use_controller: bool = False,
        scale: float = 1.0,
        no_rotation: bool = False,
        use_panel: bool = False,
        xr_view: bool = False,
        port: int = 8012
    ):
        self.scene_xml = scene_xml
        self.mock = mock
        self.gui = gui
        self.timeout_s = timeout_s
        self.use_controller = use_controller
        self.scale = scale
        self.no_rotation = no_rotation
        self.use_panel = use_panel
        self.xr_view = xr_view
        self.port = port

        # Load Quest 3S controller axis map from config/quest3s.yaml
        config_path = os.path.join(PROJECT_ROOT, "config", "quest3s.yaml")
        self.axis_map = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
        if os.path.exists(config_path):
            try:
                import yaml
                with open(config_path, "r") as f:
                    cfg = yaml.safe_load(f)
                if "controller_axis_map" in cfg:
                    self.axis_map = np.array(cfg["controller_axis_map"], dtype=np.float64)
            except Exception as e:
                print(f"[WARN] Failed to load config/quest3s.yaml: {e}")

        # 1. Simulation Environment
        print("[INFO] Initializing MuJoCo Simulation Environment (home keyframe)...")
        self.env = AgibotSimEnv(scene_xml, render_gui=gui)
        self.env.reset()

        # Record Robot Home Wrists
        home_poses = self.env.get_ee_poses()
        self.robot_home_pos_l = home_poses["left"]["pos"].copy()
        self.robot_home_pos_r = home_poses["right"]["pos"].copy()
        self.robot_home_rot_l = home_poses["left"]["rot"].copy()
        self.robot_home_rot_r = home_poses["right"]["rot"].copy()

        # Calibration Offsets (for hand tracking mode)
        self.calibrated = False
        self.calib_offset_l = np.zeros(3, dtype=np.float64)
        self.calib_offset_r = np.zeros(3, dtype=np.float64)

        # Clutch State (for controller mode)
        self.grip_active_l = False
        self.grip_active_r = False
        self.ctrl_anchor_pos_l = None
        self.ctrl_anchor_pos_r = None
        self.ctrl_anchor_rot_l = None
        self.ctrl_anchor_rot_r = None
        self.robot_anchor_pos_l = self.robot_home_pos_l.copy()
        self.robot_anchor_pos_r = self.robot_home_pos_r.copy()
        self.robot_anchor_rot_l = self.robot_home_rot_l.copy()
        self.robot_anchor_rot_r = self.robot_home_rot_r.copy()

        # Thumbstick pinch toggle state
        self.pinch_mode_l = False
        self.pinch_mode_r = False
        self.last_thumbstick_l = False
        self.last_thumbstick_r = False

        # Current target poses
        self.cur_target_pos_l = self.robot_home_pos_l.copy()
        self.cur_target_pos_r = self.robot_home_pos_r.copy()
        self.cur_target_rot_l = self.robot_home_rot_l.copy()
        self.cur_target_rot_r = self.robot_home_rot_r.copy()

        # 2. Retargeting Modules
        print("[INFO] Initializing Arm and Hand Retargeters...")
        self.arm_retarget_left = AgibotArmRetargeting(arm="left", scale=self.scale)
        self.arm_retarget_right = AgibotArmRetargeting(arm="right", scale=self.scale)
        self.hand_retarget_left = AgibotHandRetargeting(hand="left")
        self.hand_retarget_right = AgibotHandRetargeting(hand="right")

        # 3. IK Solvers
        print("[INFO] Initializing Bilateral DLS IK Solvers...")
        self.ik_left = AgibotA2IK(scene_xml, arm="left")
        self.ik_right = AgibotA2IK(scene_xml, arm="right")

        # 4. Web Control Panel Server & Tracking Stream
        self.panel_server = None
        cert_path = os.path.join(PROJECT_ROOT, "certs", "cert.pem")
        key_path = os.path.join(PROJECT_ROOT, "certs", "key.pem")

        if self.use_panel:
            from teleop.simulation.web_panel import WebPanelServer
            print(f"[INFO] Launching Web Control Panel on https://localhost:{self.port}/panel ...")
            self.panel_server = WebPanelServer(port=self.port, cert_file=cert_path, key_file=key_path, scene_xml=scene_xml)
            self.panel_server.start()

        if self.xr_view:
            print("[INFO] XR Robot Digital Twin Renderer enabled (--xr-view).")
            try:
                from teleop.xr.xr_robot_view import XRRobotRenderer
                self.xr_renderer = XRRobotRenderer(scene_xml)
            except Exception as e:
                print(f"[WARN] Could not initialize XR renderer: {e}")
                self.xr_renderer = None
        else:
            self.xr_renderer = None

        if self.mock:
            print("[INFO] Using Mock Quest 3S Streamer for automated validation.")
            self.mock_streamer = MockQuestStreamer(frequency_hz=60.0)
            self.tracker = None
        elif not self.use_panel:
            mode_name = "Controllers" if self.use_controller else "Hands"
            print(f"[INFO] Starting WebXR TeleVuer HTTPS Server on 0.0.0.0:{self.port} ({mode_name} mode)...")
            try:
                from televuer import TeleVuerWrapper
                tv = TeleVuerWrapper(
                    use_hand_tracking=not self.use_controller,
                    cert_file=cert_path,
                    key_file=key_path,
                    display_mode="pass-through"
                )
                print("[INFO] WebXR TeleVuer server process running.")
            except Exception as e:
                print(f"[ERROR] WebXR TeleVuer server failed to start: {e}")
                tv = None
            self.tracker = QuestTrackingReceiver(tv_wrapper=tv, timeout_s=timeout_s)
            self.mock_streamer = None
        else:
            self.mock_streamer = None
            self.tracker = None

        # Sequence / Preset State
        self.active_sequence = None
        self.seq_start_time = 0.0
        self.seq_origin_l = None
        self.seq_origin_r = None
        self.walking_demo_active = False
        self.walk_start_time = 0.0
        self.target_curls_l = np.zeros(5)
        self.target_pinch_l = 0.0
        self.target_curls_r = np.zeros(5)
        self.target_pinch_r = 0.0
        self.last_cmd_name = "None"
        self.last_cmd_time = 0.0
        self.cmd_count = 0
        self.cmd_rate_hz = 0.0
        self.cmd_rate_timer = time.time()
        self.status_warning = ""

        # State
        self.state_running = True
        self.state_estop = False
        self.manual_held = False
        self.last_arm_cmd_left = self.env.get_left_arm_qpos()
        self.last_arm_cmd_right = self.env.get_right_arm_qpos()
        self.last_hand_cmd_left = np.zeros(6)
        self.last_hand_cmd_right = np.zeros(6)

        self.tracking_loss_events = 0
        self.estop_triggered = False

    def calibrate(self, auto: bool = False):
        """Calibrates initial wrist offset once valid Quest tracking has been received."""
        if self.use_panel:
            print("[INFO] Web Control Panel mode active. Teleop calibration skipped.")
            self.calibrated = True
            return

        if self.use_controller:
            print("[INFO] Controller clutch teleop mode enabled. Enter-calibration skipped.")
            # Wait for valid controller tracking
            if not self.mock and self.tracker is not None:
                print("[INFO] Waiting for Quest 3S controllers...")
                valid_start = None
                last_msg_time = 0.0
                while True:
                    now = time.time()
                    frame = self.tracker.update()
                    l_track = frame.get("left", {})
                    r_track = frame.get("right", {})
                    l_valid = l_track.get("valid", False) and np.linalg.norm(l_track.get("wrist_pos", np.zeros(3))) > 1e-4
                    r_valid = r_track.get("valid", False) and np.linalg.norm(r_track.get("wrist_pos", np.zeros(3))) > 1e-4
                    if l_valid or r_valid:
                        if valid_start is None:
                            valid_start = now
                        elif (now - valid_start) >= 1.0:
                            print(f"[INFO] Controllers verified active (L={l_valid}, R={r_valid}). Clutch teleop ready.")
                            break
                    else:
                        valid_start = None
                        if (now - last_msg_time) >= 2.0:
                            print("WAITING FOR CONTROLLERS")
                            last_msg_time = now
                    time.sleep(0.05)
            self.calibrated = True
            return

        print("=" * 75)
        print("INITIAL WRIST POSE CALIBRATION")
        print("=" * 75)
        print(f"Robot Home Left Wrist : [{self.robot_home_pos_l[0]:+.3f}, {self.robot_home_pos_l[1]:+.3f}, {self.robot_home_pos_l[2]:+.3f}] m")
        print(f"Robot Home Right Wrist: [{self.robot_home_pos_r[0]:+.3f}, {self.robot_home_pos_r[1]:+.3f}, {self.robot_home_pos_r[2]:+.3f}] m")

        # 1. Wait for valid tracking for >= 1s if real XR
        if not self.mock and self.tracker is not None:
            print("[INFO] Checking for live Quest 3S XR session tracking stream...")
            valid_start = None
            last_msg_time = 0.0
            prev_l_pos = None
            prev_r_pos = None

            while True:
                now = time.time()
                frame = self.tracker.update()
                l_track = frame.get("left", {})
                r_track = frame.get("right", {})

                l_pos = l_track.get("wrist_pos", None)
                r_pos = r_track.get("wrist_pos", None)

                l_valid = l_track.get("valid", False) and (l_pos is not None) and np.all(np.isfinite(l_pos)) and (np.linalg.norm(l_pos) > 1e-4)
                r_valid = r_track.get("valid", False) and (r_pos is not None) and np.all(np.isfinite(r_pos)) and (np.linalg.norm(r_pos) > 1e-4)

                # Accept if both hands valid OR at least one hand valid (with warning for missing)
                is_valid = l_valid or r_valid

                if is_valid:
                    if valid_start is None:
                        valid_start = now
                        prev_l_pos = l_pos.copy() if l_valid else None
                        prev_r_pos = r_pos.copy() if r_valid else None
                    elif (now - valid_start) >= 1.0:
                        if l_valid and not r_valid:
                            print("[WARN] Right hand tracking missing during calibration; mirroring left or holding right.")
                        elif r_valid and not l_valid:
                            print("[WARN] Left hand tracking missing during calibration; mirroring right or holding left.")
                        print(f"[INFO] Stable live Quest tracking stream verified (duration: {now - valid_start:.1f} s).")
                        break
                else:
                    valid_start = None
                    if (now - last_msg_time) >= 2.0:
                        print("WAITING FOR QUEST XR SESSION")
                        last_msg_time = now

                time.sleep(0.05)

        if not auto and not self.mock and sys.stdin.isatty():
            print("\n>> Please hold both hands in front of your chest matching the robot pose.")
            try:
                input(">> Press [ENTER] to calibrate...")
            except (EOFError, KeyboardInterrupt):
                pass

        if self.mock:
            frame = self.mock_streamer.get_frame()
        else:
            frame = self.tracker.update()

        op_wrist_l = frame["left"]["wrist_pos"].copy()
        op_wrist_r = frame["right"]["wrist_pos"].copy()
        print(f"Operator Left Wrist (raw XR) : [{op_wrist_l[0]:+.3f}, {op_wrist_l[1]:+.3f}, {op_wrist_l[2]:+.3f}] m")
        print(f"Operator Right Wrist (raw XR): [{op_wrist_r[0]:+.3f}, {op_wrist_r[1]:+.3f}, {op_wrist_r[2]:+.3f}] m")

        raw_pos_l, _ = self.arm_retarget_left.retarget_pose(
            xr_wrist_pos=op_wrist_l,
            xr_wrist_rot_mat=frame["left"]["wrist_rot"]
        )
        raw_pos_r, _ = self.arm_retarget_right.retarget_pose(
            xr_wrist_pos=op_wrist_r,
            xr_wrist_rot_mat=frame["right"]["wrist_rot"]
        )

        offset_l = self.robot_home_pos_l - raw_pos_l
        offset_r = self.robot_home_pos_r - raw_pos_r

        # Sanity check: plausible offsets (|offset| < 1 m)
        norm_l = float(np.linalg.norm(offset_l))
        norm_r = float(np.linalg.norm(offset_r))
        if norm_l > 1.0 or norm_r > 1.0:
            print(f"[WARN] Calibration offset norm exceeds 1.0 m (L={norm_l:.3f} m, R={norm_r:.3f} m). Clamping to 1.0 m.")
            if norm_l > 1.0:
                offset_l = offset_l * (1.0 / norm_l)
            if norm_r > 1.0:
                offset_r = offset_r * (1.0 / norm_r)

        self.calib_offset_l = offset_l
        self.calib_offset_r = offset_r
        self.calibrated = True

        print(f"[PASS] Calibration Complete!")
        print(f"  - Left Wrist Offset : [{self.calib_offset_l[0]:+.3f}, {self.calib_offset_l[1]:+.3f}, {self.calib_offset_l[2]:+.3f}] m")
        print(f"  - Right Wrist Offset: [{self.calib_offset_r[0]:+.3f}, {self.calib_offset_r[1]:+.3f}, {self.calib_offset_r[2]:+.3f}] m")
        print("=" * 75 + "\n")

    def on_key_press(self, key):
        if key == 'q':
            print("\n[ALERT] EMERGENCY STOP (E-STOP) TRIGGERED ('q')!")
            self.state_estop = True
            self.state_running = False
            self.estop_triggered = True
        elif key == 'r':
            print("\n[INFO] RESUMING TELEOPERATION ('r').")
            self.state_estop = False
            self.state_running = True

    def _handle_panel_command(self, cmd: dict):
        name = cmd.get("name") or cmd.get("cmd") or ""
        self.last_cmd_name = name
        self.last_cmd_time = time.time() * 1000.0
        self.cmd_count += 1
        print(f"[CMD] Received panel command: {cmd}")

        if name == "estop":
            self.on_key_press('q')
            return
        elif name == "clear_estop":
            self.on_key_press('r')
            return
        elif name == "hold_toggle":
            self.manual_held = not self.manual_held
            print(f"[PANEL] Hold toggle: {self.manual_held}")
            return
        elif name in ("walk_toggle", "walk_demo"):
            self.walking_demo_active = not self.walking_demo_active
            if self.walking_demo_active:
                self.walk_start_time = time.time()
                print("[PANEL] Walk (visual demo) STARTED (visual only, not physics).")
            else:
                self._stop_walk_demo()
                print("[PANEL] Walk (visual demo) STOPPED. Legs restored to home.")
            return

        if self.state_estop:
            self.status_warning = "Command ignored: E-STOP is active."
            return

        ee = self.env.get_ee_poses()

        if name in ("arm_jog", "jog_l", "jog_r"):
            arm = cmd.get("arm", "left" if name == "jog_l" else "right")
            axis_val = cmd.get("axis", 0)
            if isinstance(axis_val, str):
                axis_val = axis_val.lower()
                if axis_val in ("x", "0"):
                    axis = 0
                elif axis_val in ("y", "1"):
                    axis = 1
                elif axis_val in ("z", "2"):
                    axis = 2
                elif axis_val in ("roll", "pitch", "yaw"):
                    # Rotation jog
                    r_axis = 0 if axis_val == "roll" else (1 if axis_val == "pitch" else 2)
                    direction = float(cmd.get("dir", 1.0))
                    step_deg = float(cmd.get("step_deg", 5.0))
                    step_rad = np.radians(step_deg) * direction
                    rot_vec = np.zeros(3)
                    rot_vec[r_axis] = step_rad
                    from scipy.spatial.transform import Rotation as R
                    dR = R.from_rotvec(rot_vec).as_matrix()
                    if arm == "left":
                        self.cur_target_rot_l = dR @ self.cur_target_rot_l
                        self._solve_arm_ik("left")
                    else:
                        self.cur_target_rot_r = dR @ self.cur_target_rot_r
                        self._solve_arm_ik("right")
                    return
                else:
                    axis = 0
            else:
                axis = int(axis_val)

            direction = float(cmd.get("dir", 1.0))
            if "step_cm" in cmd:
                step = float(cmd.get("step_cm", 2.0)) / 100.0 * direction
            else:
                step = float(cmd.get("step", 0.02)) * (direction if "dir" in cmd else 1.0)

            if arm == "left":
                new_target = self.cur_target_pos_l.copy()
                new_target[axis] += step
                new_target[0] = np.clip(new_target[0], -0.2, 0.9)
                new_target[1] = np.clip(new_target[1], 0.1, 0.9)
                new_target[2] = np.clip(new_target[2], 0.5, 1.8)
                self.cur_target_pos_l = new_target
                self._solve_arm_ik("left")
            else:
                new_target = self.cur_target_pos_r.copy()
                new_target[axis] += step
                new_target[0] = np.clip(new_target[0], -0.2, 0.9)
                new_target[1] = np.clip(new_target[1], -0.9, -0.1)
                new_target[2] = np.clip(new_target[2], 0.5, 1.8)
                self.cur_target_pos_r = new_target
                self._solve_arm_ik("right")

        elif name == "set_wrist_target":
            arm = cmd.get("arm", "left")
            pos = cmd.get("pos")
            rot_quat = cmd.get("rot_quat") # [w, x, y, z]
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
                # rot_quat is [w, x, y, z] -> scipy expects [x, y, z, w]
                scipy_quat = [rot_quat[1], rot_quat[2], rot_quat[3], rot_quat[0]]
                rot_mat = R.from_quat(scipy_quat).as_matrix()
                if arm == "left":
                    self.cur_target_rot_l = rot_mat
                else:
                    self.cur_target_rot_r = rot_mat

            self._solve_arm_ik(arm)

        elif name in ("hand", "hand_l", "hand_r"):
            arm = cmd.get("arm", "left" if name == "hand_l" else "right")
            action = cmd.get("action", "")
            if action == "open":
                curls = np.zeros(5)
                pinch = 0.0
            elif action == "close":
                curls = np.ones(5)
                pinch = 0.0
            elif action == "pinch":
                curls = np.array([0.9, 0.9, 0.1, 0.1, 0.1])
                pinch = 1.0
            else:
                val = float(cmd.get("value", cmd.get("closure", 0.0)))
                curls = np.full(5, val)
                pinch = 0.0

            if arm == "left":
                self.target_curls_l = curls
                self.target_pinch_l = pinch
            else:
                self.target_curls_r = curls
                self.target_pinch_r = pinch

        elif name == "preset":
            pose = cmd.get("pose", "home")
            self._trigger_preset(pose)

    def _trigger_preset(self, pose: str):
        if pose == "reset" or pose == "home":
            self.active_sequence = None
            self.walking_demo_active = False
            self.env.reset()
            self.last_arm_cmd_left = self.env.get_left_arm_qpos()
            self.last_arm_cmd_right = self.env.get_right_arm_qpos()
            self.last_hand_cmd_left = np.zeros(6)
            self.last_hand_cmd_right = np.zeros(6)
            home_poses = self.env.get_ee_poses()
            self.cur_target_pos_l = home_poses["left"]["pos"].copy()
            self.cur_target_pos_r = home_poses["right"]["pos"].copy()
            self.cur_target_rot_l = home_poses["left"]["rot"].copy()
            self.cur_target_rot_r = home_poses["right"]["rot"].copy()
            print("[PANEL] Reset to Home keyframe executed.")

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
            print("[PANEL] Active sequence stopped.")

    def _solve_arm_ik(self, arm="both"):
        if arm in ("left", "both"):
            q_sol_l, _, pos_err_l, _, _ = self.ik_left.solve(
                target_pos=self.cur_target_pos_l,
                target_rot=self.cur_target_rot_l,
                init_qpos=self.last_arm_cmd_left,
                max_iters=35, pos_tol=0.005, rot_tol=0.035
            )
            self.last_arm_cmd_left = q_sol_l
        if arm in ("right", "both"):
            q_sol_r, _, pos_err_r, _, _ = self.ik_right.solve(
                target_pos=self.cur_target_pos_r,
                target_rot=self.cur_target_rot_r,
                init_qpos=self.last_arm_cmd_right,
                max_iters=35, pos_tol=0.005, rot_tol=0.035
            )
            self.last_arm_cmd_right = q_sol_r
        self.env.set_arm_commands(self.last_arm_cmd_left, self.last_arm_cmd_right)

    def _update_sequence(self, dt: float):
        t_seq = time.time() - self.seq_start_time
        if self.active_sequence == "wave":
            if t_seq > 3.0:
                self.active_sequence = None
                return
            # 3 s sine wave on right wrist
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
            # 0-1s reach fwd, 1-2s close hands, 2-3s open hands, 3s done
            if t_seq < 1.0:
                frac = t_seq / 1.0
                self.cur_target_pos_l = self.robot_home_pos_l + frac * np.array([0.25, 0.0, 0.0])
                self.cur_target_pos_r = self.robot_home_pos_r + frac * np.array([0.25, 0.0, 0.0])
                self._solve_arm_ik("both")
            elif t_seq < 2.0:
                frac = (t_seq - 1.0) / 1.0
                curls = np.full(5, frac)
                self.last_hand_cmd_left = self.hand_retarget_left.retarget_curls(curls, dt=dt)
                self.last_hand_cmd_right = self.hand_retarget_right.retarget_curls(curls, dt=dt)
                self.env.set_hand_commands(self.last_hand_cmd_left, self.last_hand_cmd_right)
            elif t_seq < 3.0:
                frac = 1.0 - (t_seq - 2.0) / 1.0
                curls = np.full(5, frac)
                self.last_hand_cmd_left = self.hand_retarget_left.retarget_curls(curls, dt=dt)
                self.last_hand_cmd_right = self.hand_retarget_right.retarget_curls(curls, dt=dt)
                self.env.set_hand_commands(self.last_hand_cmd_left, self.last_hand_cmd_right)
            else:
                self.active_sequence = None

    def _update_walk_demo(self):
        t_walk = time.time() - self.walk_start_time
        omega = 2.0 * np.pi * 1.0
        pitch = 0.25 * np.sin(omega * t_walk)
        # Apply to hip pitch joints (idx03, idx09) within safe range [-0.5, 0.5] rad
        for jname, sgn in [("idx03_left_hip_pitch", 1.0), ("idx09_right_hip_pitch", -1.0)]:
            if jname in self.env.qpos_idx_map:
                adr = self.env.qpos_idx_map[jname]
                self.env.data.qpos[adr] = sgn * pitch
                self.env.data.qvel[self.env.qvel_idx_map[jname]] = 0.0

    def _stop_walk_demo(self):
        self.walking_demo_active = False
        if hasattr(self.env, "non_manip_qpos_indices"):
            self.env.data.qpos[self.env.non_manip_qpos_indices] = 0.0
            self.env.data.qvel[self.env.non_manip_qvel_indices] = 0.0

    def step_teleop(self, frame: dict, dt: float = 0.02) -> dict:
        metrics = {
            "tracking_valid": True,
            "held_pose": False,
            "estop_active": self.state_estop,
            "ik_left_err_mm": 0.0,
            "ik_right_err_mm": 0.0,
        }

        # ---------------- WEB CONTROL PANEL COMMAND DISPATCH ----------------
        if self.panel_server is not None:
            cmd = self.panel_server.get_command()
            while cmd is not None:
                self._handle_panel_command(cmd)
                cmd = self.panel_server.get_command()

        if self.state_estop:
            self.env.set_arm_commands(self.last_arm_cmd_left, self.last_arm_cmd_right)
            self.env.set_hand_commands(self.last_hand_cmd_left, self.last_hand_cmd_right)
            self.env.step(num_substeps=5, hold_arms=True, hold_left_qpos=self.last_arm_cmd_left, hold_right_qpos=self.last_arm_cmd_right)
            metrics["held_pose"] = True
            return metrics

        # Update active preset sequences (wave, circle, grasp)
        if self.active_sequence is not None:
            self._update_sequence(dt)

        # Update visual-only walk demo
        if self.walking_demo_active:
            self._update_walk_demo()

        # If manual held is enabled, freeze arm
        if self.manual_held:
            self.env.set_arm_commands(self.last_arm_cmd_left, self.last_arm_cmd_right)
            self.env.set_hand_commands(self.last_hand_cmd_left, self.last_hand_cmd_right)
            self.env.step(num_substeps=5, hold_arms=True, hold_left_qpos=self.last_arm_cmd_left, hold_right_qpos=self.last_arm_cmd_right)
            metrics["held_pose"] = True
            return metrics

        # If in Web Control Panel mode, update hands smoothly, step simulation and return
        if self.use_panel:
            self.last_hand_cmd_left = self.hand_retarget_left.retarget_curls(self.target_curls_l, pinch_strength=self.target_pinch_l, dt=dt)
            self.last_hand_cmd_right = self.hand_retarget_right.retarget_curls(self.target_curls_r, pinch_strength=self.target_pinch_r, dt=dt)
            self.env.set_arm_commands(self.last_arm_cmd_left, self.last_arm_cmd_right)
            self.env.set_hand_commands(self.last_hand_cmd_left, self.last_hand_cmd_right)
            self.env.step(num_substeps=5, hold_arms=True, hold_left_qpos=self.last_arm_cmd_left, hold_right_qpos=self.last_arm_cmd_right)
            metrics["held_pose"] = self.manual_held
            return metrics

        left_track = frame["left"] if frame else {"valid": True, "wrist_pos": self.robot_home_pos_l, "wrist_rot": self.robot_home_rot_l}
        right_track = frame["right"] if frame else {"valid": True, "wrist_pos": self.robot_home_pos_r, "wrist_rot": self.robot_home_rot_r}

        # ---------------- CONTROLLER CLUTCH TELEOP MODE ----------------
        if self.use_controller:
            l_ctrl = left_track.get("ctrl", {})
            r_ctrl = right_track.get("ctrl", {})
            grip_l = bool(l_ctrl.get("grip", False))
            grip_r = bool(r_ctrl.get("grip", False))
            trig_l = float(l_ctrl.get("trigger", 0.0))
            trig_r = float(r_ctrl.get("trigger", 0.0))
            thumb_l = bool(l_ctrl.get("thumbstick", False))
            thumb_r = bool(r_ctrl.get("thumbstick", False))

            # Thumbstick click toggles pinch mode
            if thumb_l and not self.last_thumbstick_l:
                self.pinch_mode_l = not self.pinch_mode_l
            self.last_thumbstick_l = thumb_l

            if thumb_r and not self.last_thumbstick_r:
                self.pinch_mode_r = not self.pinch_mode_r
            self.last_thumbstick_r = thumb_r

            # Left Arm Clutch
            ee_poses = self.env.get_ee_poses()
            if grip_l and left_track["valid"]:
                if not self.grip_active_l:
                    # Clutch engaged: anchor positions
                    self.grip_active_l = True
                    self.ctrl_anchor_pos_l = left_track["wrist_pos"].copy()
                    self.ctrl_anchor_rot_l = left_track["wrist_rot"].copy()
                    self.robot_anchor_pos_l = ee_poses["left"]["pos"].copy()
                    self.robot_anchor_rot_l = ee_poses["left"]["rot"].copy()

                # Delta in XR frame mapped to robot frame via axis_map
                delta_xr = left_track["wrist_pos"] - self.ctrl_anchor_pos_l
                delta_robot = self.axis_map @ delta_xr
                self.cur_target_pos_l = self.robot_anchor_pos_l + self.scale * delta_robot

                if self.no_rotation:
                    self.cur_target_rot_l = self.robot_anchor_rot_l.copy()
                else:
                    R_delta = left_track["wrist_rot"] @ self.ctrl_anchor_rot_l.T
                    self.cur_target_rot_l = R_delta @ self.robot_anchor_rot_l
            else:
                self.grip_active_l = False
                self.ctrl_anchor_pos_l = None

            # Right Arm Clutch
            if grip_r and right_track["valid"]:
                if not self.grip_active_r:
                    self.grip_active_r = True
                    self.ctrl_anchor_pos_r = right_track["wrist_pos"].copy()
                    self.ctrl_anchor_rot_r = right_track["wrist_rot"].copy()
                    self.robot_anchor_pos_r = ee_poses["right"]["pos"].copy()
                    self.robot_anchor_rot_r = ee_poses["right"]["rot"].copy()

                delta_xr = right_track["wrist_pos"] - self.ctrl_anchor_pos_r
                delta_robot = self.axis_map @ delta_xr
                self.cur_target_pos_r = self.robot_anchor_pos_r + self.scale * delta_robot

                if self.no_rotation:
                    self.cur_target_rot_r = self.robot_anchor_rot_r.copy()
                else:
                    R_delta = right_track["wrist_rot"] @ self.ctrl_anchor_rot_r.T
                    self.cur_target_rot_r = R_delta @ self.robot_anchor_rot_r
            else:
                self.grip_active_r = False
                self.ctrl_anchor_pos_r = None

            # Solve IK for arms that are gripped
            if self.grip_active_l:
                q_sol_l, _, pos_err_l, _, _ = self.ik_left.solve(
                    target_pos=self.cur_target_pos_l,
                    target_rot=self.cur_target_rot_l,
                    init_qpos=self.last_arm_cmd_left,
                    max_iters=25, pos_tol=0.005, rot_tol=0.035
                )
                diff_l = np.clip(q_sol_l - self.last_arm_cmd_left, -3.0 * dt, 3.0 * dt)
                self.last_arm_cmd_left += diff_l
                metrics["ik_left_err_mm"] = pos_err_l * 1000.0
            else:
                metrics["ik_left_err_mm"] = 0.0

            if self.grip_active_r:
                q_sol_r, _, pos_err_r, _, _ = self.ik_right.solve(
                    target_pos=self.cur_target_pos_r,
                    target_rot=self.cur_target_rot_r,
                    init_qpos=self.last_arm_cmd_right,
                    max_iters=25, pos_tol=0.005, rot_tol=0.035
                )
                diff_r = np.clip(q_sol_r - self.last_arm_cmd_right, -3.0 * dt, 3.0 * dt)
                self.last_arm_cmd_right += diff_r
                metrics["ik_right_err_mm"] = pos_err_r * 1000.0
            else:
                metrics["ik_right_err_mm"] = 0.0

            # Trigger drives dexterous hand open/close (0=open, 1=closed)
            curls_l = np.full(5, trig_l, dtype=np.float64)
            pinch_l = 1.0 if self.pinch_mode_l else 0.0
            hand_cmd_l = self.hand_retarget_left.retarget_curls(curls_l, pinch_strength=pinch_l, dt=dt)

            curls_r = np.full(5, trig_r, dtype=np.float64)
            pinch_r = 1.0 if self.pinch_mode_r else 0.0
            hand_cmd_r = self.hand_retarget_right.retarget_curls(curls_r, pinch_strength=pinch_r, dt=dt)

            self.last_hand_cmd_left = hand_cmd_l.copy()
            self.last_hand_cmd_right = hand_cmd_r.copy()

            self.env.set_arm_commands(self.last_arm_cmd_left, self.last_arm_cmd_right)
            self.env.set_hand_commands(hand_cmd_l, hand_cmd_r)

            # If neither gripped, hold arms frozen to prevent sagging
            neither_gripped = (not self.grip_active_l) and (not self.grip_active_r)
            metrics["held_pose"] = neither_gripped
            self.env.step(
                num_substeps=5,
                hold_arms=neither_gripped,
                hold_left_qpos=self.last_arm_cmd_left,
                hold_right_qpos=self.last_arm_cmd_right
            )
            return metrics

        # ---------------- HAND TRACKING MODE ----------------
        if not left_track["valid"] or not right_track["valid"]:
            self.tracking_loss_events += 1
            metrics["tracking_valid"] = False
            metrics["held_pose"] = True
            self.env.set_arm_commands(self.last_arm_cmd_left, self.last_arm_cmd_right)
            self.env.set_hand_commands(self.last_hand_cmd_left, self.last_hand_cmd_right)
            self.env.step(num_substeps=5, hold_arms=True, hold_left_qpos=self.last_arm_cmd_left, hold_right_qpos=self.last_arm_cmd_right)
            return metrics

        t_pos_l_raw, t_rot_l = self.arm_retarget_left.retarget_pose(
            xr_wrist_pos=left_track["wrist_pos"],
            xr_wrist_rot_mat=left_track["wrist_rot"]
        )
        t_pos_r_raw, t_rot_r = self.arm_retarget_right.retarget_pose(
            xr_wrist_pos=right_track["wrist_pos"],
            xr_wrist_rot_mat=right_track["wrist_rot"]
        )

        t_pos_l = t_pos_l_raw + self.calib_offset_l
        t_pos_r = t_pos_r_raw + self.calib_offset_r

        q_sol_l, succ_l, pos_err_l, rot_err_l, _ = self.ik_left.solve(
            target_pos=t_pos_l,
            target_rot=t_rot_l,
            init_qpos=self.last_arm_cmd_left,
            max_iters=25,
            pos_tol=0.005,
            rot_tol=0.035
        )
        q_sol_r, succ_r, pos_err_r, rot_err_r, _ = self.ik_right.solve(
            target_pos=t_pos_r,
            target_rot=t_rot_r,
            init_qpos=self.last_arm_cmd_right,
            max_iters=25,
            pos_tol=0.005,
            rot_tol=0.035
        )

        metrics["ik_left_err_mm"] = pos_err_l * 1000.0
        metrics["ik_right_err_mm"] = pos_err_r * 1000.0

        max_arm_step = 3.0 * dt
        diff_l = np.clip(q_sol_l - self.last_arm_cmd_left, -max_arm_step, max_arm_step)
        safe_arm_cmd_l = self.last_arm_cmd_left + diff_l

        diff_r = np.clip(q_sol_r - self.last_arm_cmd_right, -max_arm_step, max_arm_step)
        safe_arm_cmd_r = self.last_arm_cmd_right + diff_r

        self.last_arm_cmd_left = safe_arm_cmd_l.copy()
        self.last_arm_cmd_right = safe_arm_cmd_r.copy()

        curls_l = left_track.get("curls", np.zeros(5))
        pinch_l = left_track.get("pinch", 0.0)
        hand_cmd_l = self.hand_retarget_left.retarget_curls(curls_l, pinch_strength=pinch_l, dt=dt)

        curls_r = right_track.get("curls", np.zeros(5))
        pinch_r = right_track.get("pinch", 0.0)
        hand_cmd_r = self.hand_retarget_right.retarget_curls(curls_r, pinch_strength=pinch_r, dt=dt)

        self.last_hand_cmd_left = hand_cmd_l.copy()
        self.last_hand_cmd_right = hand_cmd_r.copy()

        self.env.set_arm_commands(safe_arm_cmd_l, safe_arm_cmd_r)
        self.env.set_hand_commands(hand_cmd_l, hand_cmd_r)
        self.env.step(num_substeps=5)

        return metrics

    def run_loop(self, duration_s: float = 0.0, test_safety: bool = False, auto_calib: bool = False):
        self.calibrate(auto=auto_calib or self.mock or (not sys.stdin.isatty()))

        mode_str = f"{duration_s} s" if duration_s > 0 else "Continuous (infinite)"
        print(f"[INFO] Teleoperation Loop Active ({mode_str})...")
        print("       Controls: 'r' to Follow | 'q' for Emergency Stop | Ctrl+C to Exit")

        dt = 0.02
        infinite = (duration_s <= 0.0)
        total_steps = int(duration_s / dt) if not infinite else 0
        step = 0
        start_time = time.time()

        try:
            while infinite or (step < total_steps):
                t_start = time.time()
                t = step * dt

                if self.mock:
                    if test_safety and (total_steps > 0 and total_steps // 3 <= step < total_steps // 3 + 20):
                        self.mock_streamer.simulate_tracking_loss = True
                    else:
                        self.mock_streamer.simulate_tracking_loss = False
                    frame = self.mock_streamer.get_frame()
                elif self.tracker is not None:
                    frame = self.tracker.update()
                else:
                    frame = None

                if test_safety and total_steps > 0 and step == int(total_steps * 0.85):
                    self.on_key_press('q')

                metrics = self.step_teleop(frame, dt=dt)
                elapsed_step = time.time() - t_start

                # Stream 25 Hz visual body poses to 3D Digital Twin (every 2nd step)
                if self.panel_server is not None and (step % 2 == 0):
                    poses = self.env.get_visual_body_poses()
                    self.panel_server.broadcast_poses(poses)

                # Update 5 Hz telemetry status for Web Control Panel
                if self.panel_server is not None and (step % 10 == 0):
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
                        "xr_active": xr_active,
                        "ee_l": [float(x) for x in ee["left"]["pos"]],
                        "ee_r": [float(x) for x in ee["right"]["pos"]],
                        "tgt_l": [float(x) for x in self.cur_target_pos_l],
                        "tgt_r": [float(x) for x in self.cur_target_pos_r],
                        "ik_err_l": float(metrics["ik_left_err_mm"]),
                        "ik_err_r": float(metrics["ik_right_err_mm"]),
                        "hand_closure_l": float(np.clip(closure_l, 0.0, 1.0)),
                        "hand_closure_r": float(np.clip(closure_r, 0.0, 1.0)),
                        "held": bool(metrics.get("held_pose", False) or self.manual_held),
                        "estop": bool(self.state_estop),
                        "walking": bool(self.walking_demo_active),
                        "cmd_rate_hz": float(self.cmd_rate_hz),
                        "last_cmd_name": str(self.last_cmd_name),
                        "last_cmd_time": float(self.last_cmd_time),
                        "warning": str(self.status_warning)
                    }
                    self.panel_server.set_status(status_packet)
                    self.status_warning = ""

                if step % 50 == 0:
                    ee = self.env.get_ee_poses()
                    if self.use_controller:
                        l_ctrl = frame["left"].get("ctrl", {})
                        r_ctrl = frame["right"].get("ctrl", {})
                        hz = 1.0 / max(0.001, elapsed_step)
                        print(f"  [{t:.2f}s] {hz:.0f}Hz | Grip: L={self.grip_active_l} R={self.grip_active_r} | "
                              f"Trig: L={l_ctrl.get('trigger', 0.0):.2f} R={r_ctrl.get('trigger', 0.0):.2f} | "
                              f"Tgt: L=[{self.cur_target_pos_l[0]:+.2f},{self.cur_target_pos_l[1]:+.2f},{self.cur_target_pos_l[2]:+.2f}] "
                              f"R=[{self.cur_target_pos_r[0]:+.2f},{self.cur_target_pos_r[1]:+.2f},{self.cur_target_pos_r[2]:+.2f}] | "
                              f"IK Err: L={metrics['ik_left_err_mm']:.1f}mm R={metrics['ik_right_err_mm']:.1f}mm")
                    else:
                        print(f"  [{t:.2f}s] Step {step:5d} | "
                              f"Left EE: [{ee['left']['pos'][0]:+.3f}, {ee['left']['pos'][1]:+.3f}, {ee['left']['pos'][2]:+.3f}] | "
                              f"IK Err: L={metrics['ik_left_err_mm']:.1f}mm, R={metrics['ik_right_err_mm']:.1f}mm | "
                              f"Held: {metrics['held_pose']} | E-Stop: {metrics['estop_active']}")

                step += 1
                sleep_time = dt - elapsed_step
                if sleep_time > 0.001:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n[INFO] Interrupted by user.")
        finally:
            elapsed = time.time() - start_time
            print(f"[INFO] Teleoperation Loop Terminated after {elapsed:.2f} s ({step} steps).")
            self.close()

        return True

    def close(self):
        self.env.close()
        if self.panel_server is not None:
            try:
                self.panel_server.stop()
            except Exception:
                pass
        if self.tracker and self.tracker.tv_wrapper:
            try:
                self.tracker.tv_wrapper.close()
            except Exception:
                pass


def run_xr_debug(port: int = 8012, use_controller: bool = False):
    """
    XR Diagnostics Mode: runs Vuer server only (no MuJoCo sim).
    Prints diagnostics once per second: client connection, XR session, wrist data,
    hand/controller arrivals, and exact validity failure reasons.
    """
    mode_str = "CONTROLLER MODE" if use_controller else "HAND TRACKING MODE"
    print("\n" + "=" * 80)
    print(f"STARTING XR DEBUG MODE (VUER SERVER ONLY, NO SIMULATION) - [{mode_str}]")
    print("=" * 80)
    cert_path = os.path.join(PROJECT_ROOT, "certs", "cert.pem")
    key_path = os.path.join(PROJECT_ROOT, "certs", "key.pem")

    try:
        from televuer import TeleVuerWrapper
        tv = TeleVuerWrapper(
            use_hand_tracking=not use_controller,
            cert_file=cert_path,
            key_file=key_path,
            display_mode="pass-through"
        )
        print("[INFO] WebXR TeleVuer diagnostic server process launched.")
    except Exception as e:
        print(f"[ERROR] Failed to start WebXR TeleVuer: {e}")
        return

    tracker = QuestTrackingReceiver(tv_wrapper=tv, timeout_s=0.5)

    print("[INFO] Diagnostics active. Press Ctrl+C to terminate.")
    sec = 0
    try:
        while True:
            time.sleep(1.0)
            sec += 1
            frame = tracker.update()
            tele_data = tracker.last_tele_data

            # 1. Websocket connected check
            ws_connected = False
            if hasattr(tv, "tvuer") and tv.tvuer is not None and hasattr(tv.tvuer, "vuer"):
                ws_pool = getattr(tv.tvuer.vuer, "ws", {})
                ws_connected = bool(ws_pool and len(ws_pool) > 0)

            # 2. XR session active / motion ready
            is_ready = bool(getattr(tele_data, "motion_data_ready", False)) if tele_data is not None else False
            head_mat = getattr(tele_data, "head_pose", None) if tele_data is not None else None
            xr_active = is_ready or (head_mat is not None and np.linalg.norm(head_mat[:3, 3]) > 1e-4)

            # 3. Left / Right wrist raw type / shape
            l_mat = getattr(tele_data, "left_wrist_pose", None) if tele_data is not None else None
            r_mat = getattr(tele_data, "right_wrist_pose", None) if tele_data is not None else None

            l_type_str = f"{type(l_mat).__name__}{l_mat.shape if isinstance(l_mat, np.ndarray) else ''}"
            r_type_str = f"{type(r_mat).__name__}{r_mat.shape if isinstance(r_mat, np.ndarray) else ''}"

            l_pos = l_mat[:3, 3] if (isinstance(l_mat, np.ndarray) and l_mat.shape == (4, 4)) else None
            r_pos = r_mat[:3, 3] if (isinstance(r_mat, np.ndarray) and r_mat.shape == (4, 4)) else None

            # 4. Hand vs Controller
            hand_pts = getattr(tele_data, "left_hand_pos", None) if tele_data is not None else None
            has_fingers = bool(hand_pts is not None and np.any(np.abs(hand_pts) > 1e-4))
            has_ctrl = tracker.has_controller_data or bool(getattr(tele_data, "left_ctrl_trigger", False))

            l_ctrl = frame["left"].get("ctrl", {})
            r_ctrl = frame["right"].get("ctrl", {})

            if use_controller:
                if not ws_connected and not is_ready:
                    print(f"[{sec:03d}s] CONTROLLER: NO DATA (waiting for connection)")
                else:
                    l_ctrl_str = f"pos=[{l_pos[0]:+.2f},{l_pos[1]:+.2f},{l_pos[2]:+.2f}] trig={l_ctrl.get('trigger',0.0):.2f} grip={l_ctrl.get('grip',False)} a={l_ctrl.get('a',False)} b={l_ctrl.get('b',False)}" if l_pos is not None else "NO DATA"
                    r_ctrl_str = f"pos=[{r_pos[0]:+.2f},{r_pos[1]:+.2f},{r_pos[2]:+.2f}] trig={r_ctrl.get('trigger',0.0):.2f} grip={r_ctrl.get('grip',False)} a={r_ctrl.get('a',False)} b={r_ctrl.get('b',False)}" if r_pos is not None else "NO DATA"
                    print(f"[{sec:03d}s] WS: {'YES' if ws_connected else 'NO'} | XR: {'YES' if xr_active else 'NO'} | L_Ctrl: {l_ctrl_str} | R_Ctrl: {r_ctrl_str}")
                continue

            # 5. Exact validity failure reason for hand tracking mode
            reasons = []
            if not ws_connected:
                reasons.append("no websocket client connected")
            if not xr_active:
                reasons.append("Enter VR / XR session not active (motion_data_ready=False)")
            if l_pos is None:
                reasons.append(f"left wrist is {l_type_str}")
            elif np.linalg.norm(l_pos) < 1e-4:
                reasons.append("left wrist all zeros")
            elif not np.all(np.isfinite(l_pos)):
                reasons.append("left wrist has non-finite values")

            if r_pos is None:
                reasons.append(f"right wrist is {r_type_str}")
            elif np.linalg.norm(r_pos) < 1e-4:
                reasons.append("right wrist all zeros")
            elif not np.all(np.isfinite(r_pos)):
                reasons.append("right wrist has non-finite values")

            fail_reason = ", ".join(reasons) if reasons else "VALID (Live Tracking Streaming)"

            l_str = f"[{l_pos[0]:+.3f}, {l_pos[1]:+.3f}, {l_pos[2]:+.3f}]" if l_pos is not None else "None"
            r_str = f"[{r_pos[0]:+.3f}, {r_pos[1]:+.3f}, {r_pos[2]:+.3f}]" if r_pos is not None else "None"

            print(f"[{sec:03d}s] WS: {'YES' if ws_connected else 'NO'} | "
                  f"XR Active: {'YES' if xr_active else 'NO'} | "
                  f"HandData: {'YES' if has_fingers else 'NO'} | "
                  f"CtrlData: {'YES' if has_ctrl else 'NO'} | "
                  f"L_Wrist: {l_str} ({l_type_str}) | "
                  f"R_Wrist: {r_str} ({r_type_str}) | "
                  f"Status: {fail_reason}")

    except KeyboardInterrupt:
        print("\n[INFO] Exiting XR Debug Mode.")
    finally:
        tv.close()


def main():
    local_ip = get_local_ip()

    parser = argparse.ArgumentParser(
        description="AGIBOT A2 Ultra Teleoperation Controller (Meta Quest 3S WebXR)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=HELP_EPILOG
    )
    parser.add_argument("--mock", action="store_true", default=False, help="Use mock Quest 3S stream for CI/testing")
    parser.add_argument("--real-xr", action="store_true", help="Connect to live Meta Quest 3S WebXR session")
    parser.add_argument("--panel", action="store_true", help="Serve mouse-driven Web Control Panel on https://localhost:PORT/panel")
    parser.add_argument("--controller", action="store_true", help="Enable clutch-style controller teleop mode (hand tracking off)")
    parser.add_argument("--xr-view", action="store_true", help="Render low-poly robot digital twin inside Quest XR scene")
    parser.add_argument("--no-rotation", action="store_true", help="Position-only teleop (wrist orientation fixed)")
    parser.add_argument("--scale", type=float, default=1.0, help="Arm motion scale factor (default: 1.0)")
    parser.add_argument("--xr-debug", action="store_true", help="Run Vuer server only in XR diagnostics mode (no MuJoCo sim)")
    parser.add_argument("--headless", action="store_true", help="Disable GUI viewer and run headless")
    parser.add_argument("--duration", type=float, default=0.0, help="Execution duration in seconds (0 = infinite continuous)")
    parser.add_argument("--auto-calibrate", action="store_true", help="Auto-calibrate wrist offset without pressing Enter")
    parser.add_argument("--no-safety-test", action="store_true", help="Disable synthetic tracking loss / e-stop injection")
    parser.add_argument("--port", type=int, default=8012, help="WebXR port (default: 8012)")
    args = parser.parse_args()

    use_mock = args.mock
    use_gui = not args.headless

    print_connection_block(local_ip, args.port)

    if args.xr_debug:
        run_xr_debug(port=args.port, use_controller=args.controller)
        return

    controller = TeleopController(
        scene_xml=SCENE_XML,
        mock=use_mock,
        gui=use_gui,
        use_controller=args.controller,
        scale=args.scale,
        no_rotation=args.no_rotation,
        use_panel=args.panel,
        xr_view=args.xr_view,
        port=args.port
    )

    success = controller.run_loop(
        duration_s=args.duration,
        test_safety=not args.no_safety_test and args.mock,
        auto_calib=args.auto_calibrate or use_mock or (not sys.stdin.isatty())
    )
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
