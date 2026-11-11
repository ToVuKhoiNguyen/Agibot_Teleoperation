"""
teleop/app/init_state.py
Initializes application runtime session state, telemetry counters, and Quest VR configuration.
"""

import time
import signal
import collections
from pathlib import Path
import numpy as np
import yaml
from teleop.kinematics.mirror_limiter import MirrorLimiter


def init_state(app, project_root: Path):
    app.cur_target_pos_l = app.robot_home_pos_l.copy()
    app.cur_target_pos_r = app.robot_home_pos_r.copy()
    app.cur_target_rot_l = app.robot_home_rot_l.copy()
    app.cur_target_rot_r = app.robot_home_rot_r.copy()

    app.last_arm_cmd_left = app.env.get_left_arm_qpos()
    app.last_arm_cmd_right = app.env.get_right_arm_qpos()
    app.last_hand_cmd_left = np.zeros(6)
    app.last_hand_cmd_right = np.zeros(6)

    prof = getattr(app, "speed_profile", "normal")
    init_q14 = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
    tau_cfg = None
    cfg_p = project_root / "config" / "tau_id.json"
    if cfg_p.exists():
        try:
            import json
            with open(cfg_p, "r", encoding="utf-8") as f:
                tau_cfg = json.load(f).get("tau_arm")
        except Exception: pass
    gate_m = getattr(app, "gate_mode", "legacy")
    if gate_m == "lagged" and tau_cfg is None:
        print("[WARNING] --gate-mode lagged requested but config/tau_id.json is missing! Falling back to slow/legacy mode (never guessing tau).")
        gate_m = "legacy"
        app.gate_mode = "legacy"
        if hasattr(app, "robot_safety") and app.robot_safety is not None:
            app.robot_safety.gate_mode = "legacy"

    app.mirror_limiter = MirrorLimiter(
        speed_profile=prof, initial_q14=init_q14,
        accept_higher_speed=getattr(app, "i_accept_higher_speed", False),
        gate_mode=gate_m,
        tau_id=tau_cfg
    )
    print("[CommandChain] Chain: Input(Gizmo/XR) -> CartesianFollower(critically-damped) -> IK -> MonotoneLimiter(per-joint, open-loop) -> UDP/Sim")
    is_real_backend = getattr(app, "backend", "sim").lower() in ("real", "ros2", "real-udp", "udp")
    app.sync_state = "SYNC" if is_real_backend else "ENGAGED"
    app.arm_engaged_l = False if is_real_backend else True
    app.arm_engaged_r = False if is_real_backend else True
    app.max_sim_real_gap_rad = 0.0
    app.last_state_feedback_age_s = 999.0
    app.real_q14_measured = None

    app.target_curls_l = np.zeros(5)
    app.target_pinch_l = 0.0
    app.target_curls_r = np.zeros(5)
    app.target_pinch_r = 0.0

    app.active_sequence = None
    app.seq_start_time = 0.0
    app.seq_origin_l = None
    app.seq_origin_r = None
    app.walking_demo_active = False
    app.walk_start_time = 0.0

    app.state_estop = False
    app.manual_held = False
    app.running = True
    app.nan_detected = False
    app.nan_logged_warning = False
    app.last_cmd_name = "None"
    app.last_cmd_time = 0.0
    app.cmd_count = 0
    app.cmd_rate_hz = 0.0
    app.cmd_rate_timer = time.time()
    app.status_warning = ""
    app.session_start_time = time.time()
    app.tracking_loss_events = 0
    app.clamp_counter_arm = 0
    app.clamp_counter_hand = 0
    app.fault_events = 0
    app.frame_gaps = 0
    app.msg_ages_ms = []
    app.rejected_other_mode = 0

    # Quest 3S WebXR Teleoperation Configuration & Clutch State
    app.quest_config_path = project_root / "config" / "quest3s.yaml"
    app.quest_cfg = {
        "controller_axis_map": [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        "scale": 1.0,
        "rotation_enabled": True,
        "deadband_m": 0.002,
        "grip_threshold": 0.25,
        "trigger_threshold": 0.25,
        "tracking_timeout_s": 0.35,
        "estop_combo_hold_s": 0.5,
        "home_hold_s": 1.5,
        "home_duration": 1.5,
        "precision_scale_factor": 0.3,
        "precision_follower_factor": 0.5,
        "finger_closure_rate": 1.0,
        "thumbstick_deadband": 0.15,
        "thumbstick_x_deadband": 0.40
    }
    if app.quest_config_path.exists():
        try:
            with open(app.quest_config_path, "r", encoding="utf-8") as f:
                loaded_cfg = yaml.safe_load(f)
                if isinstance(loaded_cfg, dict):
                    app.quest_cfg.update(loaded_cfg)
        except Exception as e:
            print(f"[WARN] Failed to load quest3s.yaml: {e}")

    app.xr_axis_map = np.array(app.quest_cfg.get("controller_axis_map", [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]), dtype=np.float64)
    app.xr_rot_axis_map = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
    app.xr_scale = float(app.quest_cfg.get("scale", 1.0))
    app.xr_rotation_enabled = bool(app.quest_cfg.get("rotation_enabled", True))
    app.xr_deadband = float(app.quest_cfg.get("deadband_m", 0.002))
    app.xr_grip_thresh = float(app.quest_cfg.get("grip_threshold", 0.25))
    app.xr_trigger_thresh = float(app.quest_cfg.get("trigger_threshold", 0.25))
    app.xr_timeout = float(app.quest_cfg.get("tracking_timeout_s", 0.30))
    app.xr_estop_hold_thresh = float(app.quest_cfg.get("estop_combo_hold_s", 0.5))
    app.xr_home_hold_s = float(app.quest_cfg.get("home_hold_s", 1.5))
    app.xr_precision_scale_factor = float(app.quest_cfg.get("precision_scale_factor", 0.3))
    app.xr_precision_follower_factor = float(app.quest_cfg.get("precision_follower_factor", 0.5))
    app.xr_closure_rate = float(app.quest_cfg.get("finger_closure_rate", 1.0))
    app.xr_stick_deadband = float(app.quest_cfg.get("thumbstick_deadband", 0.15))
    app.xr_stick_x_deadband = float(app.quest_cfg.get("thumbstick_x_deadband", 0.40))
    app.xr_rig_yaw, app.xr_tracking_lost_l, app.xr_tracking_lost_r = 0.0, False, False
    app.xr_reengage_required_l, app.xr_reengage_required_r = False, False
    app.input_mode = getattr(app, "input_mode", "controllers")
    app.hand_follow_mode, app.hand_elbow_hint = "absolute", False
    app.cal_head_pos = np.array([0.0, 1.5, 0.0], dtype=np.float64)
    app.cal_head_yaw, app.handtrack_clutch_l, app.handtrack_clutch_r = 0.0, False, False
    app.xr_handtrack_status = None
    app.handtrack_diag_status = {"left": {}, "right": {}}
    app.fingers_follow_always = True
    app.handtrack_live_bars = {"left": None, "right": None}
    app.limit_indicator = None
    from teleop.hand.recorder import HandRecorder
    app.hand_recorder = HandRecorder()

    app.xr_input_active = False
    app.xr_last_packet_time = 0.0
    app.xr_pkt_count = 0
    app.xr_total_pkts = 0
    app.xr_rate_hz = 0.0
    app.xr_rate_timer = time.time()
    app.xr_grip_active_l = False
    app.xr_grip_active_r = False
    app.xr_anchor_ctrl_pos_l = None
    app.xr_anchor_ctrl_pos_r = None
    app.xr_anchor_ctrl_rot_l = None
    app.xr_anchor_ctrl_rot_r = None
    app.xr_anchor_marker_pos_l = None
    app.xr_anchor_marker_pos_r = None
    app.xr_anchor_marker_rot_l = None
    app.xr_anchor_marker_rot_r = None
    app.xr_prev_btn_b = False
    app.xr_prev_btn_y = False
    app.xr_estop_combo_start_time = None

    # Precision mode per side
    app.precision_l = False
    app.precision_r = False

    # Clutch state per side: "LOCKED" | "TRANSLATE" | "ROTATE" | "6D"
    app.xr_clutch_state_l = "LOCKED"
    app.xr_clutch_state_r = "LOCKED"

    # Grasp types and finger closure state
    app.grasp_type_l = "cylinder"
    app.grasp_type_r = "cylinder"
    app.hand_closure_l = 0.0
    app.hand_closure_r = 0.0
    app.hand_log_ring = collections.deque(maxlen=300)
    app.stick_x_state_l = 0
    app.stick_x_state_r = 0
    app.xr_thumb_click_prev_l = False
    app.xr_thumb_click_prev_r = False

    # Home hold progress
    app.x_hold_start = None
    app.x_home_triggered = False
    app.xr_home_progress_l = 0.0
    app.a_hold_start = None
    app.a_home_triggered = False
    app.xr_home_progress_r = 0.0

    # Raw signals for UI telemetry
    app.xr_left_connected = False
    app.xr_right_connected = False
    app.xr_btn_x = False
    app.xr_btn_y = False
    app.xr_btn_a = False
    app.xr_btn_b = app.xr_btn_menu_l = app.xr_btn_menu_r = app.xr_thumb_click_l = app.xr_thumb_click_r = False
    app.xr_grip_val_l = app.xr_grip_val_r = app.xr_trig_val_l = app.xr_trig_val_r = 0.0
    app.xr_stick_x_l = app.xr_stick_y_l = app.xr_stick_x_r = app.xr_stick_y_r = 0.0

    # Event log (last 8 events)
    app.xr_event_log = []

    # Direct joint command overrides & sweep test state
    app.direct_hand_override_l = False
    app.direct_hand_override_r = False
    app.direct_hand_target_l = np.zeros(6, dtype=np.float64)
    app.direct_hand_target_r = np.zeros(6, dtype=np.float64)
    app.hand_sweep_active = False
    app.hand_sweep_mode = "single_joints"
    app.hand_sweep_side = "both"
    app.hand_sweep_time = 0.0
    app.hand_sweep_joint_idx = 0
    app.hand_sweep_info = "Ready"
    app.hand_sweep_saved_joints_l = np.zeros(6, dtype=np.float64)
    app.hand_sweep_saved_joints_r = np.zeros(6, dtype=np.float64)

    # Smooth Homing State
    app.home_duration = float(app.quest_cfg.get("home_duration", 1.5))
    app.homing_active = False
    app.homing_side = "both"
    app.homing_elapsed = 0.0
    app.homing_duration_current = app.home_duration
    app.robot_home_q_l = app.env.get_left_arm_qpos().copy()
    app.robot_home_q_r = app.env.get_right_arm_qpos().copy()
    app.homing_start_pos_l = app.robot_home_pos_l.copy()
    app.homing_start_pos_r = app.robot_home_pos_r.copy()
    app.homing_start_rot_l = app.robot_home_rot_l.copy()
    app.homing_start_rot_r = app.robot_home_rot_r.copy()
    app.homing_start_q_l = app.robot_home_q_l.copy()
    app.homing_start_q_r = app.robot_home_q_r.copy()
    app.last_gizmo_cmd_time = 0.0

    # Hand Safety Gate State
    app.current_safe_hand_l = np.zeros(6, dtype=np.float64)
    app.current_safe_hand_r = np.zeros(6, dtype=np.float64)
    app.hand_collision_blocked = False
    app.min_hand_distance_m = 999.0

    # Setup Clean Shutdown Signals
    signal.signal(signal.SIGINT, app._handle_signal)
    signal.signal(signal.SIGTERM, app._handle_signal)

