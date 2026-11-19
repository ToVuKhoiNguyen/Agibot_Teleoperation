#!/usr/bin/env python3
"""
main.py
Single consolidated entrypoint for AGIBOT A2 Ultra Teleoperation.
Coordinates submodules in teleop.app.* and hosts 200 Hz simulation control loop.
"""

import os
import sys
import time
import argparse
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.app import (
    init_subsystems,
    init_state,
    handle_signal,
    log_xr_event,
    update_precision,
    start_smooth_homing,
    start_data_collection,
    reset_arm_to_home,
    update_homing,
    update_hand_sweep,
    solve_arm_ik,
    handle_system_cmd,
    handle_xr_controllers_input,
    handle_hand_cmd,
    handle_target_cmd,
    update_sequence,
    run_control_loop,
)

SCENE_XML = str(PROJECT_ROOT / "assets" / "agibot_a2_ultra" / "mjcf" / "scene.xml")


class UnifiedTeleopApp:
    """Main application orchestrating MuJoCo physics, WebXR teleoperation, and safety layers."""

    def __init__(
        self,
        port: int = 8012,
        render_viewer: bool = False,
        use_controller: bool = False,
        xr_view: bool = False,
        backend: str = "real-udp",
        speed_scale: float = 1.0,
        ik_solver_type: str = "mink",
        self_collision: bool = False,
        no_adb: bool = False,
        no_quest_launch: bool = False,
        allow_hardware_risk: bool = True,
        arms: str = "both",
        no_hand: bool = False,
        real_max_vel: float = 0.8,
        i_accept_higher_speed: bool = False,
        hand_raw_max: int = 2000,
        enable_thumb_swing: bool = False,
        allow_handtracking_real: bool = False,
        input_mode: str = "controllers",
        real_follower: bool = False,
        speed: int = 3,
        speed_profile: str = None,
        gate_mode: str = "legacy",
        neck_pitch: float = 0.0,
        neck_yaw: float = 0.0,
    ):
        self.init_neck_pitch = float(neck_pitch)
        self.init_neck_yaw = float(neck_yaw)
        self.speed = int(speed) if speed in (1, 2, 3, 4, 5, 6) else 3
        self.speed_profile = str(self.speed)
        self.gate_mode = gate_mode
        vel_caps = {1: (0.40, 1.80), 2: (0.60, 2.40), 3: (0.80, 3.00), 4: (1.00, 3.00), 5: (1.50, 4.00), 6: (2.00, 6.28)}
        v_prof, a_prof = vel_caps[self.speed]
        print(f"[SPEED] Active Speed Level: {self.speed} (Cap: {v_prof:.2f} rad/s, Accel: {a_prof:.2f} rad/s^2, Step: {v_prof*0.02:.4f} rad/tick)")
        init_subsystems(
            self,
            port=port,
            render_viewer=render_viewer,
            use_controller=use_controller,
            xr_view=xr_view,
            backend=backend,
            speed_scale=speed_scale,
            ik_solver_type=ik_solver_type,
            self_collision=self_collision,
            no_adb=no_adb,
            no_quest_launch=no_quest_launch,
            project_root=PROJECT_ROOT,
            scene_xml=SCENE_XML,
            allow_hardware_risk=allow_hardware_risk,
            arms=arms,
            no_hand=no_hand,
            real_max_vel=v_prof,
            i_accept_higher_speed=i_accept_higher_speed,
            hand_raw_max=hand_raw_max,
            enable_thumb_swing=enable_thumb_swing,
            allow_handtracking_real=allow_handtracking_real,
            input_mode=input_mode,
            real_follower=real_follower,
        )
        init_state(self, project_root=PROJECT_ROOT)

    def _handle_signal(self, signum, frame):
        handle_signal(self, signum, frame)

    def _log_xr_event(self, msg: str):
        log_xr_event(self, msg)

    def _update_precision(self, side: str, is_precision: bool):
        update_precision(self, side, is_precision)

    def _start_smooth_homing(self, side: str = "both", duration: float = None):
        start_smooth_homing(self, side, duration)

    def _start_data_collection(self, side: str = "both"):
        start_data_collection(self, side)

    def _reset_arm_to_home(self, side: str):
        reset_arm_to_home(self, side)

    def _update_homing(self, dt: float):
        update_homing(self, dt)

    def _update_hand_sweep(self, dt: float):
        update_hand_sweep(self, dt)

    def _solve_arm_ik(self, arm="both", posture_hint_l: np.ndarray = None, posture_hint_r: np.ndarray = None):
        solve_arm_ik(self, arm, posture_hint_l, posture_hint_r)

    def _handle_panel_command(self, cmd: dict):
        name = cmd.get("name") or cmd.get("cmd") or ""
        self.last_cmd_name = name
        self.last_cmd_time = time.time() * 1000.0
        self.cmd_count += 1

        if handle_system_cmd(self, name, cmd):
            return

        if name == "xr_controllers":
            if getattr(self, "input_mode", "controllers") != "controllers":
                self.rejected_other_mode = getattr(self, "rejected_other_mode", 0) + 1
                return  # STRICT ISOLATION: hands mode drops controller packets
            handle_xr_controllers_input(self, cmd)
            return

        if name in ("xr_handtrack", "xr_hands"):
            if getattr(self, "input_mode", "controllers") != "hands":
                self.rejected_other_mode = getattr(self, "rejected_other_mode", 0) + 1
                return  # STRICT ISOLATION: controllers mode drops hand tracking packets
            from teleop.app.handtrack_cmd import handle_xr_handtrack_input
            handle_xr_handtrack_input(self, cmd)
            return

        if handle_hand_cmd(self, name, cmd):
            return

        if handle_target_cmd(self, name, cmd):
            return

    def _update_sequence(self, dt: float):
        update_sequence(self, dt)

    def run(self):
        run_control_loop(self)

    def close(self):
        print("[INFO] Shutting down teleoperation application...")
        if hasattr(self, "quest_manager") and self.quest_manager:
            self.quest_manager.stop()
        if hasattr(self, "server") and self.server:
            self.server.stop()
        if hasattr(self, "robot_safety") and self.robot_safety:
            try:
                self.robot_safety.disconnect()
            except Exception:
                pass
        if hasattr(self, "env") and self.env:
            self.env.close()
        if self.tracker and getattr(self.tracker, "tv_wrapper", None):
            try:
                self.tracker.tv_wrapper.close()
            except Exception:
                pass
        try:
            from teleop.utils.session_logger import save_session_log
            save_session_log(self)
        except Exception as e:
            print(f"[WARN] Failed to write session log: {e}")
        print("[INFO] Cleanup complete. Exiting.")


def main():
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    parser = argparse.ArgumentParser(description="AGIBOT A2 Ultra Teleoperation Unified Launcher")
    parser.add_argument("--sim", action="store_true", help="Run simulation mode (default: real-udp)")
    parser.add_argument("--speed", type=int, default=3, choices=[1, 2, 3, 4, 5, 6],
                        help="Speed level: 1=0.4, 2=0.6, 3=0.8, 4=1.0 rad/s, 5=1.5 rad/s / 4.0 rad/s^2 (follows ~90 %% of measured 1:1 hand motion), "
                             "6=2.0 rad/s / 6.28 rad/s^2 (the firmware acceleration limit). Default 3 (0.8 rad/s, smoothest; level 6 = 2.0 rad/s). Must match receiver --speed.")
    parser.add_argument("--port", type=int, default=8012, help="HTTPS server port (default: 8012)")
    parser.add_argument("--axis-lock", action=argparse.BooleanOptionalAction, default=True,
                        help="Quest controllers, soft axis lock: a hand move that is almost straight along one robot axis "
                             "(within 20 deg) is kept exactly on that axis; a diagonal move is followed in 3D. "
                             "Same for rotation. Default ON; --no-axis-lock for always-free 3D motion")
    parser.add_argument("--xr-scale", type=float, default=1.0,
                        help="Quest controllers: robot motion / hand motion at start-up (default 1.0 = 1:1); it can be "
                             "changed live from the web panel (Controller -> Robot Scale)")
    parser.add_argument("--smooth", type=int, default=1, choices=[0, 1, 2, 3],
                        help="direct limiter smoothing (acceleration cap, proportional acceleration, output moving "
                             "average): 0 = off (instant stops), 1 = light, 2 = medium, 3 = smoothest acceleration (default 16/11, default is 1 since 17/11; "
                             "output averages 6 + 4 ticks since 17/11 = ~40 ms lag, 60 ms less than before)")
    parser.add_argument("--limiter", type=str, default="direct", choices=["brake", "direct"],
                        help="Joint limiter law: direct (default: low-latency, straight-line, no overshoot) or brake "
                             "(former law: also limits deceleration, trails by the braking distance)")
    parser.add_argument("--viewer", action="store_true", default=None, help="Launch native MuJoCo passive viewer window")
    parser.add_argument("--no-viewer", action="store_true", help="Disable native MuJoCo passive viewer window")
    parser.add_argument("--headless", action="store_true", help="Explicitly run headless (forces native viewer OFF)")
    parser.add_argument("--no-controller", action="store_true", help="Disable controller mode (default: controller on)")
    parser.add_argument("--xr-view", action="store_true", help="Enable XR digital twin in Quest headset")
    parser.add_argument("--backend", type=str, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--arms", type=str, default="both", choices=["left", "right", "both"], help="Active manipulation arms (default: both)")
    parser.add_argument("--no-hand", action="store_true", help="Disable hand publishing")
    parser.add_argument("--enable-thumb-swing", action="store_true", help="Enable active thumb swing (default: held neutral)")
    parser.add_argument("--hand-raw-max", type=int, default=2000, help="Cap maximum raw hand output in range 0..2000 (default: 2000 = full travel)")
    parser.add_argument("--elbow-out", type=float, default=15.0, help="Elbow-out preference: pull the elbow outward while it is closer than this many cm to the body side of the shoulder (default 15; 9 = leave HOME-like postures alone; 0 = off)")
    parser.add_argument("--thumb-rest", type=int, default=500, help="Default thumb pose: raw 0..2000 that both thumb joints keep when the hand is open (default: 500 = 25%%, 0 = straight)")
    parser.add_argument("--thumb-pose", type=str, default="800,0,1750,1760", help="Thumb 'swing,flex' raw 0..2000 with the hand open, then closed (default 800,0,1750,1760: open swing 800 and wide aperture, folds to 1750,1760 to meet index tip); 2 numbers = open pose only; 'off' = thumb follows the hand from fully open (then --thumb-rest applies). Use the same open pose on arm_teleop_rx.py")
    parser.add_argument("--scene", type=str, default="default", choices=["default", "pick_bin"], help="Practice scene: pick_bin = table + 4 objects + bin (pick objects into the bin), shown on the web page / Quest (and the MuJoCo window with --viewer). With --sim the robot is simulated; with the real robot the scene is VIRTUAL around the digital twin (objects follow the commanded hand, nothing physical)")
    parser.add_argument("--speed-scale", type=float, default=1.0, help="Simulation execution speed multiplier")
    parser.add_argument("--ik", type=str, default="mink", choices=["mink", "dls", "pyroki"], help="Inverse kinematics engine (default: mink)")
    parser.add_argument("--self-collision", type=str, default="off", choices=["on", "off"], help=argparse.SUPPRESS)
    parser.add_argument("--no-adb", action="store_true", help="Disable automatic ADB detection and port reversal")
    parser.add_argument("--no-quest-launch", action="store_true", help="Do not automatically launch Quest browser on headset")
    parser.add_argument("--input", type=str, default=None, choices=["hands", "controllers"], help=argparse.SUPPRESS)
    parser.add_argument("--sim-follower", type=str, default="ideal", choices=["ideal", "real"], help=argparse.SUPPRESS)
    parser.add_argument("--neck-pitch", type=float, default=0.0, help="Initial neck pitch angle in degrees (default: 0.0, positive = backwards)")
    parser.add_argument("--neck-yaw", type=float, default=0.0, help="Initial neck yaw angle in degrees (default: 0.0)")
    args = parser.parse_args()
    if args.limiter:
        os.environ["LIMITER_LAW"] = args.limiter
    # --smooth -> (acceleration cap rad/s^2, velocity response time s, output moving average in 10 ms ticks)
    # level 3 output averages 6 + 4 ticks since 17/11 (was 12 + 10): ~60 ms less lag, command ~10% rougher on 16/11 logs
    _acc, _tv, _fir = {0: (0.0, 0.0, 1), 1: (6.0, 0.04, 4), 2: (4.0, 0.06, 8), 3: (3.0, 0.10, 6)}[args.smooth]
    os.environ.setdefault("DIRECT_SMOOTH_ACC", str(_acc))
    os.environ.setdefault("DIRECT_SMOOTH_TV", str(_tv))
    os.environ.setdefault("DIRECT_OUT_FIR", str(_fir))
    # second, shorter average = braking that builds up and fades out gradually (same top speed and peak
    # deceleration; a stop ends about 25 ms later). DIRECT_OUT_FIR2=1 switches it off.
    os.environ.setdefault("DIRECT_OUT_FIR2", {0: "1", 1: "1", 3: "4"}.get(args.smooth, "10"))   # level 3: 4, level 1 (default): off, both since 17/11

    if args.scene != "default":   # also with the real robot: virtual table / bin / objects around the digital twin
        os.environ["SIM_SCENE_XML"] = str(PROJECT_ROOT / "assets" / "agibot_a2_ultra" / "mjcf" / f"scene_{args.scene}.xml")
    backend = "sim" if args.sim else (args.backend if args.backend else "real-udp")
    is_real = backend in ("real", "ros2", "real-udp", "udp")
    if not is_real and not has_display and not (args.headless or args.no_viewer) and os.path.exists("/tmp/.X11-unix/X0"):
        os.environ["DISPLAY"] = ":0"   # terminal without DISPLAY (ssh / old tmux) on the laptop: still open the MuJoCo window
        has_display = True
    # --sim: MuJoCo window ON by default (--no-viewer / --headless = off); real robot: off unless --viewer
    render_viewer = True if args.viewer is True else (False if (args.headless or args.no_viewer or is_real) else has_display)
    use_controller = not args.no_controller

    app = UnifiedTeleopApp(
        port=args.port, render_viewer=render_viewer, use_controller=use_controller, xr_view=args.xr_view,
        backend=backend, speed_scale=args.speed_scale, ik_solver_type=args.ik, self_collision=False,
        no_adb=args.no_adb, no_quest_launch=args.no_quest_launch, allow_hardware_risk=True,
        arms=args.arms, no_hand=args.no_hand, hand_raw_max=args.hand_raw_max,
        enable_thumb_swing=args.enable_thumb_swing, input_mode="controllers" if use_controller else "mouse",
        speed=args.speed,
        neck_pitch=args.neck_pitch, neck_yaw=args.neck_yaw,
    )
    app.thumb_rest_raw = int(max(0, min(2000, args.thumb_rest)))
    from teleop.robot.aimdk_map import parse_thumb_pose
    app.thumb_pose_raw = parse_thumb_pose(args.thumb_pose)
    import teleop.kinematics.mink_ik as _mink_ik
    _mink_ik.ELBOW_OUT_MIN = max(0.0, min(0.20, args.elbow_out / 100.0))   # read by the IK at every solve
    if hasattr(getattr(app, "robot_backend", None), "_send_packet_now"): app.robot_backend.hand_raw_max = int(max(0, min(2000, args.hand_raw_max)))
    app.xr_axis_lock = bool(args.axis_lock)
    if args.xr_scale is not None:
        app.xr_scale = max(0.1, min(2.0, float(args.xr_scale)))
    print(f"[XR] limiter: {args.limiter} | axis lock: {'ON' if app.xr_axis_lock else 'off'} | scale: {app.xr_scale:.2f} | speed: {args.speed} | smooth: {args.smooth}")
    app.run()


if __name__ == "__main__":
    main()
