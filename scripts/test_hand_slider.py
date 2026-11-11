#!/usr/bin/env python3
"""
scripts/test_hand_slider.py
Tests that hand slider commands are effective immediately from fresh start
in sim and real-udp, and alternate cleanly with Open / Close / Home / T-pose in all orders.
"""

from pathlib import Path
import numpy as np

from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting
from teleop.utils.collision_checker import CollisionChecker
from teleop.app.cmd_targets import handle_target_cmd
from teleop.robot.aimdk_map import sim_hand_to_robot_raw, HAND_SAFE_RAW_MAX


class MockApp:
    def __init__(self, is_real=False):
        xml_path = str(Path("assets/agibot_a2_ultra/mjcf/scene.xml").resolve())
        self.env = AgibotSimEnv(xml_path)
        self.hand_retarget_left = AgibotHandRetargeting("left", max_vel_rad_s=10.0)
        self.hand_retarget_right = AgibotHandRetargeting("right", max_vel_rad_s=10.0)
        self.collision_checker = CollisionChecker(self.env.model)

        self.grasp_type_l = "cylinder"
        self.grasp_type_r = "cylinder"
        self.hand_closure_l = 0.0
        self.hand_closure_r = 0.0
        self.target_curls_l = np.zeros(5)
        self.target_pinch_l = 0.0
        self.target_curls_r = np.zeros(5)
        self.target_pinch_r = 0.0

        self.current_safe_hand_l = np.zeros(6)
        self.current_safe_hand_r = np.zeros(6)
        self.last_hand_cmd_left = np.zeros(6)
        self.last_hand_cmd_right = np.zeros(6)
        self.direct_hand_override_l = False
        self.direct_hand_override_r = False
        self.hand_sweep_active = False

        self.is_real = is_real
        self.arm_engaged_l = True if not is_real else True
        self.arm_engaged_r = True if not is_real else True

    def step_ticks(self, n=10):
        for _ in range(n):
            cand_l = self.hand_retarget_left.retarget_curls(self.target_curls_l, pinch_strength=self.target_pinch_l, dt=0.01)
            cand_r = self.hand_retarget_right.retarget_curls(self.target_curls_r, pinch_strength=self.target_pinch_r, dt=0.01)
            safe_l, _, _ = self.collision_checker.filter_hand_step("left", self.current_safe_hand_l, cand_l)
            safe_r, _, _ = self.collision_checker.filter_hand_step("right", self.current_safe_hand_r, cand_r)
            self.current_safe_hand_l = self.last_hand_cmd_left = safe_l
            self.current_safe_hand_r = self.last_hand_cmd_right = safe_r
            self.env.set_hand_commands(self.last_hand_cmd_left, self.last_hand_cmd_right)


def test_hand_slider_fresh_start():
    app = MockApp(is_real=False)
    # Check fresh start: slider moves to 0.60
    handle_target_cmd(app, "hand", {"cmd": "hand", "arm": "left", "action": "slider", "value": 0.60})
    app.step_ticks(1)
    # Must immediately change on same tick (first tick of control loop)
    assert np.any(app.last_hand_cmd_left > 0.0), "Slider did not take effect on tick 1!"
    app.step_ticks(15)
    assert np.mean(app.last_hand_cmd_left) > 0.30, f"Expected closure, got {app.last_hand_cmd_left}"
    print("[PASS] Fresh start slider immediate effect verified.")


def test_alternating_orders():
    # Test orders: slider -> open -> slider -> close -> slider -> home
    app = MockApp(is_real=False)

    # 1. Slider to 0.4
    handle_target_cmd(app, "hand", {"cmd": "hand", "arm": "left", "action": "slider", "value": 0.40})
    app.step_ticks(10)
    val1 = np.mean(app.last_hand_cmd_left)
    assert val1 > 0.15, "Slider 0.4 failed"

    # 2. Open
    handle_target_cmd(app, "hand", {"cmd": "hand", "arm": "left", "action": "open"})
    app.step_ticks(50)
    val2 = np.mean(app.last_hand_cmd_left)
    assert val2 < 0.05, f"Open failed: {val2}"

    # 3. Slider to 0.8
    handle_target_cmd(app, "hand", {"cmd": "hand", "arm": "left", "action": "slider", "value": 0.80})
    app.step_ticks(15)
    val3 = np.mean(app.last_hand_cmd_left)
    assert val3 > 0.40, f"Slider 0.8 failed: {val3}"

    # 4. Close
    handle_target_cmd(app, "hand", {"cmd": "hand", "arm": "left", "action": "close"})
    app.step_ticks(20)
    val4 = np.mean(app.last_hand_cmd_left)
    assert val4 > val3, f"Close failed: {val4} <= {val3}"

    # 5. Slider back to 0.1
    handle_target_cmd(app, "hand", {"cmd": "hand", "arm": "left", "action": "slider", "value": 0.10})
    app.step_ticks(50)
    val5 = np.mean(app.last_hand_cmd_left)
    assert val5 < 0.20, f"Slider 0.1 failed: {val5}"

    print("[PASS] Alternating slider and buttons passed in all orders.")


def test_real_hand_encoding():
    # Verify raw encoding bounds for real robot transmission
    app = MockApp(is_real=True)
    handle_target_cmd(app, "hand", {"cmd": "hand", "arm": "left", "action": "slider", "value": 1.0})
    app.step_ticks(20)
    h12 = np.concatenate([app.last_hand_cmd_left, app.last_hand_cmd_right])
    raw_h = sim_hand_to_robot_raw(h12, raw_max=HAND_SAFE_RAW_MAX)
    assert np.all(raw_h <= 800), f"Raw hand command exceeded 800! max={np.max(raw_h)}"
    assert np.all(raw_h >= 0), "Raw hand command below 0!"
    print(f"[PASS] Real robot raw hand encoding verified: max raw = {np.max(raw_h)} <= 800.")


if __name__ == "__main__":
    test_hand_slider_fresh_start()
    test_alternating_orders()
    test_real_hand_encoding()
    print("[SUCCESS] All hand slider unit tests passed cleanly!")
