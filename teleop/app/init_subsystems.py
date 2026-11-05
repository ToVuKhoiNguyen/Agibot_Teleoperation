"""
teleop/app/init_subsystems.py
Initializes simulation environment, IK engines, servers, retargeters, and filters.
"""

import os
import sys
from pathlib import Path
import numpy as np
import mujoco

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
from teleop.kinematics.filters import (
    InputFilterPipeline,
    ClutchRamp,
    JointTrajectoryFilter,
    CriticallyDampedFollower,
    QuaternionSlerpFollower,
)
from teleop.retargeting.agibot_arm_retargeting import AgibotArmRetargeting
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting
from teleop.xr.adb_setup import QuestAdbManager
from teleop.robot import SimBackend, DryRunBackend, Ros2DryBackend, ReplayBackend, RealBackend, SafetyLayer


def init_subsystems(
    app,
    port: int = 8012,
    render_viewer: bool = False,
    use_controller: bool = False,
    xr_view: bool = False,
    backend: str = "sim",
    speed_scale: float = 1.0,
    ik_solver_type: str = "mink",
    self_collision: bool = True,
    no_adb: bool = False,
    no_quest_launch: bool = False,
    project_root: Path = None,
    scene_xml: str = None,
    allow_hardware_risk: bool = False,
    arms: str = "both",
    no_hand: bool = False,
    real_max_vel: float = 0.3,
    i_accept_higher_speed: bool = False,
    hand_raw_max: int = 800,
    enable_thumb_swing: bool = False,
):
    app.port = port
    app.render_viewer = render_viewer
    app.use_controller = use_controller
    app.xr_view = xr_view
    app.backend = backend
    app.speed_scale = speed_scale
    app.ik_solver_type = ik_solver_type.lower()
    app.self_collision = self_collision
    app.no_adb = no_adb
    app.no_quest_launch = no_quest_launch
    app.collision_avoidance = False
    app.collision_blocked = False
    app.min_collision_distance_m = 999.0

    # 1. Port Check
    if not check_port_available(app.port):
        proc_info = get_process_holding_port(app.port)
        print(f"[FATAL] Port {app.port} is already in use!\nProcess: {proc_info}")
        print(f"Please terminate the process holding port {app.port} or specify another port via --port.")
        sys.exit(1)

    # 2. SSL Certs
    certs_dir = str(project_root / "certs")
    app.cert_path, app.key_path = ensure_ssl_certs(certs_dir)

    # 3. MuJoCo Physics Backend
    print("[INFO] Initializing MuJoCo Simulation Environment (home keyframe)...")
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if app.render_viewer and not has_display:
        print("[WARN] DISPLAY / WAYLAND_DISPLAY not found in environment. MuJoCo native viewer cannot be displayed.")
        app.render_viewer = False

    app.env = AgibotSimEnv(scene_xml, render_gui=app.render_viewer)
    app.env.reset()

    home_poses = app.env.get_ee_poses()
    app.robot_home_pos_l = home_poses["left"]["pos"].copy()
    app.robot_home_pos_r = home_poses["right"]["pos"].copy()
    app.robot_home_rot_l = home_poses["left"]["rot"].copy()
    app.robot_home_rot_r = home_poses["right"]["rot"].copy()

    # 3b. Real/Sim Robot Backend and SafetyLayer
    app.allow_hardware_risk = bool(allow_hardware_risk)
    backend_key = (app.backend or "sim").lower()
    if backend_key in ("sim", "mujoco"):
        app.robot_backend = SimBackend(app.env)
    elif backend_key == "dryrun":
        app.robot_backend = DryRunBackend()
    elif backend_key == "ros2dry":
        app.robot_backend = Ros2DryBackend()
    elif backend_key == "replay":
        app.robot_backend = ReplayBackend()
    elif backend_key in ("real", "ros2"):
        from teleop.robot import Ros2Backend
        app.robot_backend = Ros2Backend(
            arms=arms,
            no_hand=no_hand,
            real_max_vel=real_max_vel,
            i_accept_higher_speed=i_accept_higher_speed,
            hand_raw_max=hand_raw_max,
            enable_thumb_swing=enable_thumb_swing,
            allow_hardware_risk=app.allow_hardware_risk,
        )
    else:
        app.robot_backend = SimBackend(app.env)

    max_vel = real_max_vel if backend_key in ("real", "ros2") else 1.0
    app.robot_safety = SafetyLayer(app.robot_backend, max_velocity_rad_s=max_vel)
    try:
        app.robot_safety.connect()
    except Exception as e:
        if backend_key == "real":
            print(f"[FATAL] RealBackend connection failed: {e}")
            raise
        else:
            print(f"[WARN] Robot backend connect: {e}")

    # 4. Retargeting & IK
    print(f"[INFO] Initializing Arm / Hand Retargeters and IK Solvers (Engine: {app.ik_solver_type.upper()})...")
    app.arm_retarget_left = AgibotArmRetargeting(arm="left")
    app.arm_retarget_right = AgibotArmRetargeting(arm="right")
    app.hand_retarget_left = AgibotHandRetargeting(hand="left", max_vel_rad_s=10.0)
    app.hand_retarget_right = AgibotHandRetargeting(hand="right", max_vel_rad_s=10.0)

    if app.ik_solver_type == "mink":
        app.ik_left = AgibotMinkIK(scene_xml, arm="left", use_collision_avoidance=app.collision_avoidance)
        app.ik_right = AgibotMinkIK(scene_xml, arm="right", use_collision_avoidance=app.collision_avoidance)
    elif app.ik_solver_type == "pyroki":
        if AgibotPyrokiIK is None:
            raise ImportError("Pyroki is not available. Install dependencies or use --ik mink.")
        app.ik_left = AgibotPyrokiIK(arm="left")
        app.ik_right = AgibotPyrokiIK(arm="right")
    else:
        app.ik_left = AgibotA2IK(scene_xml, arm="left")
        app.ik_right = AgibotA2IK(scene_xml, arm="right")

    # Input filters
    app.input_filter_left = InputFilterPipeline()
    app.input_filter_right = InputFilterPipeline()
    app.clutch_ramp_left = ClutchRamp()
    app.clutch_ramp_right = ClutchRamp()

    # Output trajectory filters
    app.control_dt = 0.005  # 200 Hz fixed-rate control loop
    app.output_filter_left = JointTrajectoryFilter(num_joints=7, max_vel=4.0, max_acc=20.0, max_jerk=100.0, dt=app.control_dt)
    app.output_filter_right = JointTrajectoryFilter(num_joints=7, max_vel=4.0, max_acc=20.0, max_jerk=100.0, dt=app.control_dt)
    app.output_filter_left.reset(app.env.get_left_arm_qpos())
    app.output_filter_right.reset(app.env.get_right_arm_qpos())

    # Gizmo followers
    app.gizmo_follower_l = CriticallyDampedFollower(natural_freq_hz=30.0, dt=app.control_dt)
    app.gizmo_follower_r = CriticallyDampedFollower(natural_freq_hz=30.0, dt=app.control_dt)
    app.gizmo_follower_l.reset(app.robot_home_pos_l)
    app.gizmo_follower_r.reset(app.robot_home_pos_r)
    app.gizmo_rot_follower_l = QuaternionSlerpFollower(bandwidth_hz=30.0, dt=app.control_dt)
    app.gizmo_rot_follower_r = QuaternionSlerpFollower(bandwidth_hz=30.0, dt=app.control_dt)
    app.gizmo_rot_follower_l.reset(app.robot_home_rot_l)
    app.gizmo_rot_follower_r.reset(app.robot_home_rot_r)
    app.output_filter_left_gizmo = JointTrajectoryFilter(num_joints=7, max_vel=6.0, max_acc=80.0, max_jerk=250.0, dt=app.control_dt)
    app.output_filter_right_gizmo = JointTrajectoryFilter(num_joints=7, max_vel=6.0, max_acc=80.0, max_jerk=250.0, dt=app.control_dt)
    app.output_filter_left_gizmo.reset(app.env.get_left_arm_qpos())
    app.output_filter_right_gizmo.reset(app.env.get_right_arm_qpos())
    app.last_ik_sol_left = app.env.get_left_arm_qpos().copy()
    app.last_ik_sol_right = app.env.get_right_arm_qpos().copy()
    app.gizmo_active_l = False
    app.gizmo_active_r = False
    app.step_counter = 0

    # Self-Collision Checker
    app.collision_checker = CollisionChecker(app.env.model, enabled=app.self_collision)
    app.left_arm_qadr = [app.env.model.jnt_qposadr[mujoco.mj_name2id(app.env.model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{13+i}_left_arm_joint{i+1}")] for i in range(7)]
    app.right_arm_qadr = [app.env.model.jnt_qposadr[mujoco.mj_name2id(app.env.model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{20+i}_right_arm_joint{i+1}")] for i in range(7)]

    # XR Digital Twin (optional)
    if app.xr_view:
        try:
            from teleop.xr.xr_robot_view import XRRobotRenderer
            app.xr_renderer = XRRobotRenderer(scene_xml)
        except Exception:
            app.xr_renderer = None
    else:
        app.xr_renderer = None

    # Web Panel & 3D Streaming Server
    print(f"[INFO] Starting HTTPS Server on port {app.port}...")
    app.server = WebPanelServer(
        port=app.port,
        cert_file=app.cert_path,
        key_file=app.key_path,
        scene_xml=scene_xml
    )

    # WebXR Receiver
    from teleop.tracking.webxr_vuer_handler import WebXRReceiver
    app.xr_receiver = WebXRReceiver(use_controller=app.use_controller)
    app.server.xr_receiver = app.xr_receiver
    app.tracker = app.xr_receiver

    # Meta Quest ADB Auto-Setup & Watcher
    app.quest_manager = QuestAdbManager(port=app.port, no_adb=app.no_adb, no_launch=app.no_quest_launch)
    app.server.quest_manager = app.quest_manager
    app.quest_manager.start_watcher()

    app.server.start()

