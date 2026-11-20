"""
teleop/app
Unified teleoperation application modules.
"""

from .init_subsystems import init_subsystems
from .init_state import init_state
from .homing_controller import (
    handle_signal,
    log_xr_event,
    update_precision,
    start_smooth_homing,
    reset_arm_to_home,
    update_homing,
)
from .hand_sweep import update_hand_sweep
from .ik_dispatcher import solve_arm_ik
from .cmd_system import handle_system_cmd
from .xr_clutch_mapper import update_xr_clutch
from .xr_input_handler import handle_xr_controllers_input
from .cmd_hands import handle_hand_cmd
from .cmd_targets import handle_target_cmd
from .motion_demos import update_sequence
from .telemetry_builder import build_status_packet
from .control_loop import run_control_loop

__all__ = [
    "init_subsystems",
    "init_state",
    "handle_signal",
    "log_xr_event",
    "update_precision",
    "start_smooth_homing",
    "reset_arm_to_home",
    "update_homing",
    "update_hand_sweep",
    "solve_arm_ik",
    "handle_system_cmd",
    "update_xr_clutch",
    "handle_xr_controllers_input",
    "handle_hand_cmd",
    "handle_target_cmd",
    "update_sequence",
    "build_status_packet",
    "run_control_loop",
]
