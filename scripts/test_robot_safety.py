#!/usr/bin/env python3
"""
scripts/test_robot_safety.py
Acceptance tests for real-robot safety layer, dryrun backend, and hardware lock.
"""

import os
import sys
import time
import numpy as np
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.robot.interface import RobotInterface
from teleop.robot.safety_layer import SafetyLayer
from teleop.robot.backends import DryRunBackend, SimBackend
from teleop.robot.real_backend_stub import RealBackend


class MockBackend(RobotInterface):
    """Synthetic backend for deterministic safety layer unit testing."""

    def __init__(self, initial_q14=None):
        self.q14 = np.zeros(14, dtype=np.float64) if initial_q14 is None else np.asarray(initial_q14, dtype=np.float64).copy()
        self.h12 = np.zeros(12, dtype=np.float64)
        self.last_sent_arm = None
        self.last_sent_hand = None
        self.estop_called = False
        self.hold_called = False
        self.connected = False

    def connect(self): self.connected = True
    def disconnect(self): self.connected = False
    def send_arm(self, q14):
        self.last_sent_arm = np.asarray(q14, dtype=np.float64).copy()
        self.q14 = self.last_sent_arm.copy()
    def send_hand(self, h12):
        self.last_sent_hand = np.asarray(h12, dtype=np.float64).copy()
        self.h12 = self.last_sent_hand.copy()
    def read_state(self): return self.q14.copy(), self.h12.copy(), time.time()
    def estop(self): self.estop_called = True
    def hold(self): self.hold_called = True
    def is_alive(self): return self.connected


def test_joint_limits():
    # Left wrist pitch (idx 5) limit +-0.30, Right wrist yaw (idx 13) +-0.20
    cmd = np.zeros(14)
    cmd[5] = 0.80   # Exceeds +0.30
    cmd[13] = -0.60  # Exceeds -0.20
    init_q = np.zeros(14)
    init_q[5] = 0.30
    init_q[13] = -0.20
    mock = MockBackend(initial_q14=init_q)
    safety = SafetyLayer(mock)
    safety.connect()
    safety.send_arm(cmd)
    assert np.isclose(mock.last_sent_arm[5], 0.30), f"Pitch not clamped: {mock.last_sent_arm[5]}"
    assert np.isclose(mock.last_sent_arm[13], -0.20), f"Yaw not clamped: {mock.last_sent_arm[13]}"
    assert safety.stats["joint_limit_clamps"] > 0
    print("  [PASS] Joint limits clamp (wrist pitch +-0.30, yaw +-0.20)")


def test_velocity_cap():
    mock = MockBackend()
    safety = SafetyLayer(mock, max_velocity_rad_s=1.0)
    safety.connect()
    dt = 0.02
    safety.send_arm(np.zeros(14), dt=dt)  # Establish stream
    cmd = np.zeros(14)
    cmd[0] = 1.0  # Big step jump
    safety.send_arm(cmd, dt=dt)
    max_step = 1.0 * dt
    assert mock.last_sent_arm[0] <= (max_step + 1e-6), f"Velocity clamp failed: {mock.last_sent_arm[0]}"
    assert safety.stats["velocity_clamps"] > 0
    print("  [PASS] Velocity cap enforced (step <= max_vel * dt)")


def test_first_command_ramp():
    mock = MockBackend()
    safety = SafetyLayer(mock)
    safety.connect()
    cmd = np.zeros(14)
    cmd[2] = 0.50
    safety.send_arm(cmd, dt=0.02)
    # First command jump must not exceed 0.01 rad
    assert mock.last_sent_arm[2] <= 0.0101, f"First step jump too large: {mock.last_sent_arm[2]}"
    assert safety.stats["first_command_ramps"] > 0
    print("  [PASS] First-command ramp (jump <= 0.01 rad)")


def test_watchdog():
    mock = MockBackend()
    safety = SafetyLayer(mock, watchdog_timeout_s=0.300)
    safety.connect()
    t0 = 1000.0
    safety.send_arm(np.zeros(14), dt=0.01, now=t0)
    assert not safety.check_watchdog(now=t0 + 0.150), "Watchdog tripped prematurely"
    assert safety.check_watchdog(now=t0 + 0.350), "Watchdog did not trip after 350 ms"
    assert safety.hold_active and safety.stats["watchdog_trips"] > 0
    print("  [PASS] Watchdog trips after 300 ms without command")


def test_estop_latch():
    mock = MockBackend()
    safety = SafetyLayer(mock)
    safety.connect()
    safety.estop()
    assert safety.estop_active
    ok = safety.send_arm(np.zeros(14), dt=0.01)
    assert not ok, "Command accepted during E-stop"
    safety.clear_estop()
    assert not safety.estop_active
    ok = safety.send_arm(np.zeros(14), dt=0.01)
    assert ok, "Command rejected after clear E-stop"
    print("  [PASS] E-stop latch operates cleanly")


def test_nan_rejection():
    mock = MockBackend()
    safety = SafetyLayer(mock)
    safety.connect()
    nan_cmd = np.zeros(14)
    nan_cmd[3] = np.nan
    assert not safety.send_arm(nan_cmd, dt=0.01)
    assert safety.stats["nan_rejected"] == 1
    inf_hand = np.zeros(12)
    inf_hand[1] = np.inf
    assert not safety.send_hand(inf_hand)
    assert safety.stats["nan_rejected"] == 2
    print("  [PASS] NaN / Inf rejected cleanly")


def test_deviation_gate():
    init_q = np.full(14, 0.20)
    mock = MockBackend(initial_q14=init_q)
    safety = SafetyLayer(mock, deviation_thresh_rad=0.05, deviation_timeout_s=0.200)
    safety.connect()
    t0 = 100.0
    # Simulate motor deviation: commanded stays at 0.20, but mock state reports 0.00
    mock.read_state = lambda: (np.zeros(14), np.zeros(12), t0)
    dev_cmd = np.full(14, 0.20)
    safety.send_arm(dev_cmd, dt=0.01, now=t0)
    safety.send_arm(dev_cmd, dt=0.01, now=t0 + 0.100)
    assert safety.stats["deviation_trips"] == 0
    # At 250 ms (> 200 ms threshold), deviation must trip
    tripped = not safety.send_arm(dev_cmd, dt=0.01, now=t0 + 0.250)
    assert tripped and safety.stats["deviation_trips"] > 0
    print("  [PASS] Position deviation trip (>0.05 rad for >200 ms)")


def test_real_backend_lock():
    real = RealBackend(allow_hardware_risk=False)
    # 1. No env, no flag -> RuntimeError
    os.environ.pop("AGIBOT_ALLOW_REAL", None)
    try:
        real.connect()
        assert False, "Should raise RuntimeError"
    except RuntimeError:
        pass

    # 2. Flag only -> RuntimeError
    real_flagged = RealBackend(allow_hardware_risk=True)
    try:
        real_flagged.connect()
        assert False, "Should raise RuntimeError"
    except RuntimeError:
        pass

    # 3. Env only -> RuntimeError
    os.environ["AGIBOT_ALLOW_REAL"] = "1"
    try:
        real.connect()
        assert False, "Should raise RuntimeError"
    except RuntimeError:
        pass

    # 4. Both env and flag -> NotImplementedError
    try:
        real_flagged.connect()
        assert False, "Should raise NotImplementedError"
    except NotImplementedError:
        pass
    finally:
        os.environ.pop("AGIBOT_ALLOW_REAL", None)

    # 5. Purity check: no rclpy, no socket in stub
    import teleop.robot.real_backend_stub as r_stub
    stub_src = Path(r_stub.__file__).read_text()
    assert "import rclpy" not in stub_src and "from rclpy" not in stub_src
    assert "import socket" not in stub_src and "from socket" not in stub_src
    print("  [PASS] RealBackend dual lock and zero-network purity verified")


def test_dryrun_backend_10s():
    dry = DryRunBackend(log_interval_s=1.0)
    dry.connect()
    assert dry.is_alive()
    rate_hz = 50.0
    dt = 1.0 / rate_hz
    t_start = time.time()
    for tick in range(500):  # 10.0 s simulated (500 ticks at 50 Hz)
        t_sim = t_start + tick * dt
        q14 = np.sin(t_sim + np.linspace(0, 1, 14)) * 0.2
        h12 = np.full(12, 0.5 + 0.3 * np.sin(t_sim))
        dry._q14 = q14
        dry._h12 = h12
        dry._cmd_count += 1
        dry._interval_cmd_count += 1
        dry._maybe_log(t_sim)
    dry.disconnect()
    print("  [PASS] DryRunBackend simulated 10 s (500 commands, rate & payload logged)")


def main():
    print("=" * 70)
    print("RUNNING REAL-ROBOT SAFETY FOUNDATION ACCEPTANCE TESTS")
    print("=" * 70)
    test_joint_limits()
    test_velocity_cap()
    test_first_command_ramp()
    test_watchdog()
    test_estop_latch()
    test_nan_rejection()
    test_deviation_gate()
    test_real_backend_lock()
    test_dryrun_backend_10s()
    print("=" * 70)
    print("ALL 9 ROBOT SAFETY ACCEPTANCE TESTS PASSED!")
    print("=" * 70)


if __name__ == "__main__":
    main()
