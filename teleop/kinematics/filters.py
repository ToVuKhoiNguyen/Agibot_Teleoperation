"""
teleop/kinematics/filters.py
Backward-compatible facade re-exporting signal processing and trajectory followers.
"""

from .signal_filters import (
    LowPassFilter,
    OneEuroFilter,
    QuaternionFilter,
    InputFilterPipeline,
)
from .followers import (
    ClutchRamp,
    JointTrajectoryFilter,
    CriticallyDampedFollower,
    QuaternionSlerpFollower,
)

__all__ = [
    "LowPassFilter",
    "OneEuroFilter",
    "QuaternionFilter",
    "InputFilterPipeline",
    "ClutchRamp",
    "JointTrajectoryFilter",
    "CriticallyDampedFollower",
    "QuaternionSlerpFollower",
]
