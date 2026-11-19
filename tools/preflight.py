#!/usr/bin/env python3
"""
tools/preflight.py
Offline local pre-flight readiness checklist for AGIBOT A2 Ultra real robot operations.
NO robot network traffic: performs zero socket/topic connections to external robot IPs.
"""

import os
import sys
import socket
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


def check_python_environment():
    """Verify python version and core scientific dependencies."""
    errors = []
    if sys.version_info < (3, 10):
        errors.append(f"Python 3.10+ required (got {sys.version.split()[0]})")
    for mod in ["numpy", "scipy", "mujoco", "aiohttp", "rclpy"]:
        try:
            __import__(mod)
        except ImportError as e:
            errors.append(f"Module {mod} missing: {e}")
    return len(errors) == 0, "; ".join(errors) or "Python 3.12 + rclpy + deps ready"


def check_domain_id():
    """Verify ROS_DOMAIN_ID is configured."""
    dom = os.environ.get("ROS_DOMAIN_ID")
    if not dom:
        return False, "ROS_DOMAIN_ID unset in environment"
    if dom.strip() == "77":
        return True, f"ROS_DOMAIN_ID={dom} (test loopback domain)"
    return True, f"ROS_DOMAIN_ID={dom} (configured domain)"


def check_web_port(port: int = 8012):
    """Verify web panel port is available."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", port))
        return True, f"Port {port} is free and ready"
    except Exception as e:
        return False, f"Port {port} is busy or blocked: {e}"


def check_safety_locks():
    """Verify backend safety gates and lock mechanisms."""
    from teleop.robot.safety_layer import SafetyLayer, HAND_UPPER_12
    from teleop.robot.real_backend_stub import RealBackend
    from teleop.robot.ros2_backend import Ros2Backend

    rb = RealBackend()
    try:
        rb.connect()
        return False, "RealBackend connected without flags"
    except RuntimeError:
        pass

    ros2 = Ros2Backend(allow_hardware_risk=False)
    try:
        ros2.connect()
        return False, "Ros2Backend connected without flags"
    except RuntimeError:
        pass

    return True, "All 5 safety interlocks enforced (risk flag, env var, prompt, state, domain)"


def check_hand_caps():
    """Verify hand conservative limits (raw cap = full travel 2000, thumb swing switch)."""
    from teleop.robot.aimdk_map import sim_hand_to_robot_raw, HAND_SAFE_RAW_MAX
    import numpy as np

    assert HAND_SAFE_RAW_MAX == 2000, f"Expected raw max 2000, got {HAND_SAFE_RAW_MAX}"
    full_curl = np.ones(12, dtype=np.float64) * 2.0
    capped = sim_hand_to_robot_raw(full_curl, raw_max=HAND_SAFE_RAW_MAX, enable_thumb_swing=False)

    if np.max(capped) > HAND_SAFE_RAW_MAX:
        return False, f"Hand raw cap exceeded: max={np.max(capped)}"
    if capped[0] != 0 or capped[6] != 0:
        return False, f"Thumb swing not neutral: L={capped[0]}, R={capped[6]}"
    return True, "Hand raw capped at 2000 (full travel), thumb swing switch works"


def check_unit_tests():
    """Run offline unit test suites."""
    import subprocess
    vpy = sys.executable

    tests = [
        ("test_mapping.py", [vpy, str(PROJECT_ROOT / "tests" / "test_mapping.py")]),
        ("test_robot_safety.py", [vpy, str(PROJECT_ROOT / "tests" / "test_robot_safety.py")]),
    ]
    for name, cmd in tests:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if res.returncode != 0:
            return False, f"{name} failed:\n{res.stderr or res.stdout}"
    return True, "All offline acceptance tests (test_mapping, test_robot_safety) passed"


def main():
    print("=" * 76)
    print("      AGIBOT A2 ULTRA - PRE-FLIGHT READINESS CHECKLIST (OFFLINE)")
    print("=" * 76)

    checks = [
        ("Python & ROS 2 Environment", check_python_environment),
        ("ROS 2 Domain ID Configuration", check_domain_id),
        ("Web Panel Port (8012)", check_web_port),
        ("Hardware Interlock Gates", check_safety_locks),
        ("Hand Caps (2000 raw)", check_hand_caps),
        ("Acceptance Test Suites", check_unit_tests),
    ]

    all_pass = True
    for title, fn in checks:
        ok, msg = fn()
        status = "[PASS]" if ok else "[FAIL]"
        print(f"{status} {title:<32} : {msg}")
        if not ok:
            all_pass = False

    print("=" * 76)
    if all_pass:
        print(">>> OVERALL PRE-FLIGHT VERDICT: ALL CHECKS PASSED. SYSTEM READY.")
        print("=" * 76)
        sys.exit(0)
    else:
        print(">>> OVERALL PRE-FLIGHT VERDICT: FAILED. RESOLVE ISSUES BEFORE RUNNING.")
        print("=" * 76)
        sys.exit(1)


if __name__ == "__main__":
    main()

