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
    slerp_quaternion,
    start_smooth_homing,
    reset_arm_to_home,
    update_homing,
    update_hand_sweep,
    solve_arm_ik,
    handle_system_cmd,
    handle_xr_controllers_input,
    handle_hand_cmd,
    handle_target_cmd,
    update_sequence,
    update_walk_demo,
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
        backend: str = "sim",
        speed_scale: float = 1.0,
        ik_solver_type: str = "mink",
        self_collision: bool = True,
        no_adb: bool = False,
        no_quest_launch: bool = False,
        allow_hardware_risk: bool = False,
        arms: str = "both",
        no_hand: bool = False,
        real_max_vel: float = 0.3,
        i_accept_higher_speed: bool = False,
        hand_raw_max: int = 800,
        enable_thumb_swing: bool = False,
        allow_handtracking_real: bool = False,
        input_mode: str = "controllers",
        real_follower: bool = False,
        speed_profile: str = "normal",
    ):
        self.speed_profile = speed_profile
        v_prof = 0.20 if speed_profile == "slow" else (0.48 if speed_profile == "fast" else 0.30)
        print(f"[PROFILE] Active Profile: {speed_profile.upper()} ({v_prof:.2f} rad/s) | Steady error v*tau: {v_prof*0.1:.3f} rad (tau assumed 0.10s)")
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
            real_max_vel=real_max_vel,
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

    def _slerp_quaternion(self, rot_start: np.ndarray, rot_target: np.ndarray, alpha: float) -> np.ndarray:
        return slerp_quaternion(rot_start, rot_target, alpha)

    def _start_smooth_homing(self, side: str = "both", duration: float = None):
        start_smooth_homing(self, side, duration)

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

    def _update_walk_demo(self):
        update_walk_demo(self)

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
    parser.add_argument("--port", type=int, default=8012, help="HTTPS server port (default: 8012)")
    parser.add_argument("--viewer", action="store_true", default=None, help="Launch native MuJoCo passive viewer window (accepted alias; enabled by default if display exists)")
    parser.add_argument("--no-viewer", action="store_true", help="Disable native MuJoCo passive viewer window")
    parser.add_argument("--headless", action="store_true", help="Explicitly run headless (forces native viewer OFF)")
    parser.add_argument("--controller", action="store_true", help="Use controller clutch teleop mode")
    parser.add_argument("--xr-view", action="store_true", help="Enable XR digital twin in Quest headset")
    parser.add_argument("--backend", type=str, default="sim", choices=["sim", "mujoco", "dryrun", "replay", "real", "ros2", "ros2dry", "real-udp"], help="Robot execution backend (default: sim)")
    parser.add_argument("--arms", type=str, default="both", choices=["left", "right", "both"], help="Active manipulation arms (default: both)")
    parser.add_argument("--no-hand", action="store_true", help="Disable hand publishing")
    parser.add_argument("--real-max-vel", type=float, default=0.3, help="Max velocity cap for real robot (default: 0.3 rad/s)")
    parser.add_argument("--i-accept-higher-speed", action="store_true", help="Permit real robot velocity up to 3.0 rad/s (default ceiling: 1.0 rad/s)")
    parser.add_argument("--enable-thumb-swing", action="store_true", help="Enable active thumb swing (default: held neutral)")
    parser.add_argument("--hand-raw-max", type=int, default=800, help="Cap maximum raw hand output in range 0..2000 (default: 800)")
    parser.add_argument("--i-understand-hardware-risk", action="store_true", help="Explicit acknowledgement required to unlock real robot hardware")
    parser.add_argument("--speed-scale", type=float, default=1.0, help="Simulation execution speed multiplier")
    parser.add_argument("--ik", type=str, default="mink", choices=["mink", "dls", "pyroki"], help="Inverse kinematics engine (default: mink)")
    parser.add_argument("--self-collision", type=str, default="on", choices=["on", "off"], help="Enable/disable self-collision safety gate (default: on)")
    parser.add_argument("--collision-avoidance", action="store_true", help="Enable mink self-collision avoidance limit")
    parser.add_argument("--no-adb", action="store_true", help="Disable automatic ADB detection and port reversal")
    parser.add_argument("--no-quest-launch", action="store_true", help="Do not automatically launch Quest browser on headset")
    parser.add_argument("--input", type=str, default=None, choices=["hands", "controllers"], help="Exclusive input mode: hands or controllers")
    parser.add_argument("--sim-follower", type=str, default="ideal", choices=["ideal", "real"])
    parser.add_argument("--speed-profile", type=str, default="normal", choices=["slow", "normal", "fast"], help="Speed profile: slow (0.2), normal (0.3 DEFAULT), fast (0.5)")
    parser.add_argument("--real-follower", action="store_true", help="Enable sim real-like follower mode (alias for --sim-follower real)")
    parser.add_argument("--allow-handtracking-real", action="store_true", default=False, help=argparse.SUPPRESS)
    args = parser.parse_args()

    is_real = args.backend in ("real", "ros2", "real-udp", "udp")
    render_viewer = True if args.viewer is True else (False if (args.headless or args.no_viewer or is_real) else has_display)
    self_col_enabled = (args.self_collision.lower() == "on")
    effective_input_mode = args.input if args.input is not None else ("controllers" if args.controller else "controllers")
    is_real_follower = (args.sim_follower == "real") or getattr(args, "real_follower", False)

    app = UnifiedTeleopApp(
        port=args.port, render_viewer=render_viewer, use_controller=args.controller, xr_view=args.xr_view,
        backend=args.backend, speed_scale=args.speed_scale, ik_solver_type=args.ik, self_collision=self_col_enabled,
        no_adb=args.no_adb, no_quest_launch=args.no_quest_launch, allow_hardware_risk=getattr(args, "i_understand_hardware_risk", False),
        arms=args.arms, no_hand=args.no_hand, real_max_vel=args.real_max_vel, i_accept_higher_speed=args.i_accept_higher_speed,
        hand_raw_max=args.hand_raw_max, enable_thumb_swing=args.enable_thumb_swing,
        allow_handtracking_real=getattr(args, "allow_handtracking_real", False), input_mode=effective_input_mode,
        real_follower=is_real_follower, speed_profile=args.speed_profile,
    )
    app.run()


if __name__ == "__main__":
    main()
