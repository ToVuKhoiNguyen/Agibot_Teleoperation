"""
tests/test_reach_recall.py
Unit and regression tests for Reach Recall and VR Tethering mechanism.
"""

import time
import numpy as np
from teleop.kinematics.reach_projection import (
    project_to_reachable_workspace,
    SHOULDER_ORIGIN_LEFT,
    SHOULDER_ORIGIN_RIGHT,
    DEFAULT_MAX_REACH_M
)
from main import UnifiedTeleopApp


def test_reach_projection():
    print("Testing reach projection...")
    # Point inside reach
    p_in = SHOULDER_ORIGIN_LEFT + np.array([0.35, 0.0, 0.0])
    p_clamped, is_c, d_b = project_to_reachable_workspace(p_in, side="left")
    assert not is_c
    assert d_b == 0.0
    assert np.allclose(p_in, p_clamped)

    # Point outside reach in +X
    p_out = SHOULDER_ORIGIN_LEFT + np.array([0.8, 0.0, 0.0])
    p_clamped, is_c, d_b = project_to_reachable_workspace(p_out, side="left")
    assert is_c
    assert np.isclose(d_b, 0.3, atol=1e-4)
    dist_shldr = np.linalg.norm(p_clamped - SHOULDER_ORIGIN_LEFT)
    assert np.isclose(dist_shldr, DEFAULT_MAX_REACH_M, atol=1e-4)

    # Point outside in -Y (torso / cross)
    p_cross = SHOULDER_ORIGIN_LEFT + np.array([0.0, -0.7, 0.0])
    p_clamped, is_c, d_b = project_to_reachable_workspace(p_cross, side="left")
    assert is_c
    assert np.isclose(d_b, 0.2, atol=1e-4)
    assert np.isclose(np.linalg.norm(p_clamped - SHOULDER_ORIGIN_LEFT), DEFAULT_MAX_REACH_M, atol=1e-4)
    print("Reach projection tests passed.")


def test_all():
    test_reach_projection()

    captured_events = []

    class MockServer:
        def broadcast_event(self, event):
            captured_events.append(event)

    app = UnifiedTeleopApp(backend="sim", render_viewer=False, no_adb=True, no_quest_launch=True, port=8099)
    app.server = MockServer()

    print("Testing gizmo recall events & commands...")
    # 1. Release inside reach (e.g. 0.3m from shoulder)
    captured_events.clear()
    inside_p = SHOULDER_ORIGIN_LEFT + np.array([0.3, 0.1, 0.0])
    app.cur_target_pos_l = inside_p.copy()
    from teleop.app.cmd_targets import handle_target_cmd
    handle_target_cmd(app, "gizmo_end", {"cmd": "gizmo_end", "arm": "left"})
    assert len(captured_events) == 0, f"Expected 0 recall events for inside reach, got {len(captured_events)}"

    # 2. Release 0.1m, 0.3m, 0.6m beyond reach
    for beyond_m in (0.1, 0.3, 0.6):
        captured_events.clear()
        out_p = SHOULDER_ORIGIN_LEFT + np.array([DEFAULT_MAX_REACH_M + beyond_m, 0.0, 0.0])
        app.cur_target_pos_l = out_p.copy()
        handle_target_cmd(app, "gizmo_end", {"cmd": "gizmo_end", "arm": "left"})
        assert len(captured_events) == 1, f"Expected 1 recall event, got {len(captured_events)}"
        ev = captured_events[0]
        assert ev["type"] == "gizmo_recall"
        assert ev["arm"] == "left"
        assert ev["duration_ms"] == 250
        rec_pos = np.array(ev["pos"])
        expected_p = SHOULDER_ORIGIN_LEFT + np.array([DEFAULT_MAX_REACH_M, 0.0, 0.0])
        err = np.linalg.norm(rec_pos - expected_p)
        assert err < 0.001, f"Recall position error {err:.5f} m exceeds 1 mm"
        assert np.linalg.norm(app.cur_target_pos_l - expected_p) < 0.001
    print("Gizmo recall events passed.")

    print("Testing VR mapping: 1:1 inside, progressive scaling near the reach limit, exact return...")
    from teleop.app.xr_clutch_mapper import update_xr_clutch, SOFT_REACH_START
    # Engage 0.30 m in front of the shoulder (radial direction = robot +X)
    p_home = SHOULDER_ORIGIN_LEFT + np.array([0.30, 0.0, 0.0])
    app.cur_target_pos_l = p_home.copy()

    def hand(x_fwd):   # hand displacement forward (-Z XR) -> robot +X
        update_xr_clutch(app, {"pos": [0.0, 0.0, -x_fwd], "trigger": 1.0, "grip": 0.0, "tracking": True}, None, False)

    hand(0.0)
    assert app.xr_clutch_state_l == "TRANSLATE"

    # Inside SOFT_REACH_START x reach: exactly 1:1
    hand(0.08)
    assert np.allclose(app.cur_target_pos_l, p_home + np.array([0.08, 0.0, 0.0]), atol=1e-9), "Not 1:1 inside reach"

    # Hand far past the robot's reach (linear target would be 0.80 m): the target approaches the reach sphere
    # progressively and never leaves it
    hand(0.50)
    dist_after_out = np.linalg.norm(app.cur_target_pos_l - SHOULDER_ORIGIN_LEFT)
    print(f"Dist after out: {dist_after_out:.4f} m (Max reach: {DEFAULT_MAX_REACH_M:.4f} m)")
    assert SOFT_REACH_START * DEFAULT_MAX_REACH_M < dist_after_out <= DEFAULT_MAX_REACH_M + 1e-9

    # Coming back moves the target immediately (no dead zone at the boundary)
    hand(0.40)
    assert np.linalg.norm(app.cur_target_pos_l - SHOULDER_ORIGIN_LEFT) < dist_after_out - 0.01, "Dead zone on return"

    # Hand back at +0.08 m and at the start: the target is back at the same spots (no drift)
    hand(0.08)
    err_ret = np.linalg.norm(app.cur_target_pos_l - (p_home + np.array([0.08, 0.0, 0.0])))
    print(f"1:1 error after overreach and return: {err_ret*1000:.3f} mm")
    assert err_ret < 1e-6, f"Mapping drifted after overreach ({err_ret*1000:.3f} mm)"
    hand(0.0)
    assert np.linalg.norm(app.cur_target_pos_l - p_home) < 1e-6, "Target did not return to the start"

    # Release
    update_xr_clutch(app, {"pos": [0.0, 0.0, 0.0], "trigger": 0.0, "grip": 0.0, "tracking": True}, None, False)
    assert app.xr_clutch_state_l == "LOCKED"

    assert app.xr_anchor_ctrl_pos_l is None

    # Re-grip: no jump
    pos_before_regrip = app.cur_target_pos_l.copy()
    update_xr_clutch(app, {"pos": [0.0, 0.0, 0.0], "trigger": 1.0, "grip": 0.0, "tracking": True}, None, False)
    assert np.allclose(app.cur_target_pos_l, pos_before_regrip)

    print("VR 1:1 mapping tests passed.")


if __name__ == "__main__":
    test_all()
    print("ALL TESTS PASSED SUCCESSFULLY!")
