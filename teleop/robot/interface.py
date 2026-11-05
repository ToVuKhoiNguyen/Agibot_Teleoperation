"""
teleop/robot/interface.py
Abstract RobotInterface defining contract for physical and simulated robot backends.
"""

from abc import ABC, abstractmethod
import numpy as np


class RobotInterface(ABC):
    """Abstract base class for robot hardware and simulation backends."""

    @abstractmethod
    def connect(self) -> None:
        """Establishes connection to robot hardware or initializes simulation."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Safely disconnects from robot backend."""
        pass

    @abstractmethod
    def send_arm(self, q14: np.ndarray) -> None:
        """Commands 14-DoF arm joint angles in radians: [L0..L6, R0..R6]."""
        pass

    @abstractmethod
    def send_hand(self, h12: np.ndarray) -> None:
        """Commands 12-DoF dexterous hand normalized closures [0.0 (open) .. 1.0 (closed)]."""
        pass

    @abstractmethod
    def read_state(self) -> tuple[np.ndarray, np.ndarray, float]:
        """
        Reads latest state from robot backend.
        Returns:
            (q14, h12, timestamp)
            q14: 14-DoF arm positions in radians [L0..L6, R0..R6]
            h12: 12-DoF hand positions normalized [0.0..1.0]
            timestamp: UNIX epoch or monotonic seconds
        """
        pass

    @abstractmethod
    def estop(self) -> None:
        """Triggers emergency stop; freezes actuators and activates safe damping."""
        pass

    @abstractmethod
    def hold(self) -> None:
        """Commands robot to hold current posture."""
        pass

    @abstractmethod
    def is_alive(self) -> bool:
        """Returns True if backend connection and robot telemetry are healthy."""
        pass

