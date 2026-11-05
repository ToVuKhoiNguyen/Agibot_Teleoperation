"""
teleop/safety
Safety, self-collision prevention, and kinematic limits for AGIBOT A2 Ultra teleop.
"""

from .pair_builder import discover_collision_geoms, build_collision_pairs, build_hand_collision_pairs
from .arm_safety import compute_min_distance_impl, check_configuration_impl, filter_step_impl, project_target_tangent_impl
from .hand_safety import check_hand_collision_impl, filter_hand_step_impl

__all__ = [
    "discover_collision_geoms",
    "build_collision_pairs",
    "build_hand_collision_pairs",
    "compute_min_distance_impl",
    "check_configuration_impl",
    "filter_step_impl",
    "project_target_tangent_impl",
    "check_hand_collision_impl",
    "filter_hand_step_impl",
]
