"""
teleop/robot/real_backend_stub.py
Locked RealBackend stub requiring explicit dual hardware-risk confirmations.
"""

import os
import numpy as np
from teleop.robot.interface import RobotInterface


class RealBackend(RobotInterface):
    """
    Locked real-robot hardware backend.
    Enforces dual-lock gate:
    1. Environment variable AGIBOT_ALLOW_REAL=1
    2. CLI / constructor flag allow_hardware_risk=True (--i-understand-hardware-risk)
    """

    def __init__(self, allow_hardware_risk: bool = False):
        self.allow_hardware_risk = bool(allow_hardware_risk)
        self.connected = False
        self._q14 = np.zeros(14, dtype=np.float64)
        self._h12 = np.zeros(12, dtype=np.float64)

    def connect(self) -> None:
        env_allowed = (os.environ.get("AGIBOT_ALLOW_REAL") == "1")
        if not (env_allowed and self.allow_hardware_risk):
            raise RuntimeError(
                "Hardware execution locked: set AGIBOT_ALLOW_REAL=1 and pass --i-understand-hardware-risk"
            )

        # Even if both unlock flags are present, driver implementation is not yet installed.
        raise NotImplementedError(
            "Real robot ROS 2 driver is not implemented yet. Do not bypass."
        )

    def disconnect(self) -> None:
        self.connected = False

    def send_arm(self, q14: np.ndarray) -> None:
        raise NotImplementedError("Real robot driver not loaded.")

    def send_hand(self, h12: np.ndarray) -> None:
        raise NotImplementedError("Real robot driver not loaded.")

    def read_state(self) -> tuple[np.ndarray, np.ndarray, float]:
        raise NotImplementedError("Real robot driver not loaded.")

    def estop(self) -> None:
        pass

    def hold(self) -> None:
        pass

    def is_alive(self) -> bool:
        return False

