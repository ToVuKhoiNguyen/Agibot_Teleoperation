"""
teleop/robot/backends.py
RobotInterface implementations for Simulation, DryRun, and Replay.
"""

import time
import numpy as np
from teleop.robot.interface import RobotInterface


class SimBackend(RobotInterface):
    """Adapter to AgibotSimEnv (MuJoCo simulation)."""

    def __init__(self, env):
        self.env = env
        self.connected = False
        self._last_h12 = np.zeros(12, dtype=np.float64)

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def send_arm(self, q14: np.ndarray) -> None:
        if self.env is not None and self.connected:
            q = np.asarray(q14, dtype=np.float64)
            self.env.set_arm_commands(q[:7], q[7:])

    def send_hand(self, h12: np.ndarray) -> None:
        if self.env is not None and self.connected:
            h = np.asarray(h12, dtype=np.float64)
            self._last_h12 = h.copy()
            self.env.set_hand_commands(h[:6], h[6:])

    def read_state(self) -> tuple[np.ndarray, np.ndarray, float]:
        if self.env is None:
            return np.zeros(14, dtype=np.float64), np.zeros(12, dtype=np.float64), time.time()
        ql = self.env.get_left_arm_qpos()
        qr = self.env.get_right_arm_qpos()
        q14 = np.concatenate([ql, qr])
        t = getattr(self.env.data, "time", time.time())
        return q14, self._last_h12.copy(), t

    def estop(self) -> None:
        self.hold()

    def hold(self) -> None:
        if self.env is not None:
            ql = self.env.get_left_arm_qpos()
            qr = self.env.get_right_arm_qpos()
            self.env.set_arm_commands(ql, qr)

    def is_alive(self) -> bool:
        return self.connected and (self.env is not None)


class DryRunBackend(RobotInterface):
    """Dry-run backend that logs command stream and rates without robot or socket."""

    def __init__(self, log_interval_s: float = 1.0):
        self.log_interval_s = float(log_interval_s)
        self.connected = False
        self._q14 = np.zeros(14, dtype=np.float64)
        self._h12 = np.zeros(12, dtype=np.float64)
        self._cmd_count = 0
        self._last_log_t = time.time()
        self._interval_cmd_count = 0

    def connect(self) -> None:
        self.connected = True
        self._last_log_t = time.time()
        self._cmd_count = 0
        self._interval_cmd_count = 0
        print("[DryRunBackend] Connected (virtual dry-run mode, no hardware/sockets).")

    def disconnect(self) -> None:
        self.connected = False
        print("[DryRunBackend] Disconnected.")

    def _maybe_log(self, t_now: float) -> None:
        elapsed = t_now - self._last_log_t
        if elapsed >= self.log_interval_s:
            rate = self._interval_cmd_count / elapsed if elapsed > 0 else 0.0
            print(
                f"[DRYRUN] rate: {rate:5.1f} Hz | cmds: {self._cmd_count} | "
                f"arm L: {self._q14[:7].round(3)}, R: {self._q14[7:].round(3)} | "
                f"hand L: {self._h12[:6].round(2)}, R: {self._h12[6:].round(2)}"
            )
            self._last_log_t = t_now
            self._interval_cmd_count = 0

    def send_arm(self, q14: np.ndarray) -> None:
        self._q14 = np.asarray(q14, dtype=np.float64).copy()
        self._cmd_count += 1
        self._interval_cmd_count += 1
        self._maybe_log(time.time())

    def send_hand(self, h12: np.ndarray) -> None:
        self._h12 = np.asarray(h12, dtype=np.float64).copy()
        self._maybe_log(time.time())

    def read_state(self) -> tuple[np.ndarray, np.ndarray, float]:
        return self._q14.copy(), self._h12.copy(), time.time()

    def estop(self) -> None:
        print("[DryRunBackend] ESTOP commanded.")

    def hold(self) -> None:
        print("[DryRunBackend] HOLD commanded.")

    def is_alive(self) -> bool:
        return self.connected


class ReplayBackend(RobotInterface):
    """Feeds a pre-recorded joint/hand trajectory."""

    def __init__(self, trajectory: list = None):
        self.trajectory = trajectory or []
        self.idx = 0
        self.connected = False
        self._q14 = np.zeros(14, dtype=np.float64)
        self._h12 = np.zeros(12, dtype=np.float64)

    def load_trajectory(self, trajectory: list) -> None:
        self.trajectory = trajectory
        self.idx = 0

    def connect(self) -> None:
        self.connected = True
        self.idx = 0
        if self.trajectory:
            item = self.trajectory[0]
            self._q14 = np.asarray(item[0], dtype=np.float64).copy()
            self._h12 = np.asarray(item[1], dtype=np.float64).copy()

    def disconnect(self) -> None:
        self.connected = False

    def send_arm(self, q14: np.ndarray) -> None:
        pass

    def send_hand(self, h12: np.ndarray) -> None:
        pass

    def read_state(self) -> tuple[np.ndarray, np.ndarray, float]:
        if self.trajectory and self.connected:
            item = self.trajectory[self.idx]
            self._q14 = np.asarray(item[0], dtype=np.float64).copy()
            self._h12 = np.asarray(item[1], dtype=np.float64).copy()
            self.idx = (self.idx + 1) % len(self.trajectory)
        return self._q14.copy(), self._h12.copy(), time.time()

    def estop(self) -> None:
        self.connected = False

    def hold(self) -> None:
        pass

    def is_alive(self) -> bool:
        return self.connected

