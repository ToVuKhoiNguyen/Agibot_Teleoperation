#!/usr/bin/env python3
"""
scripts/test_panel_api.py
Automated acceptance test for the Web Control Panel REST API:
  - Left arm jog +X 10 cm
  - Right arm jog +Z 10 cm
  - Left hand close
  - Preset wave motion (3 s)
  - Reset to home keyframe
  - Emergency stop (E-STOP)
Auto-spawns main.py if port 8012 is closed.
"""

import os
import sys
import time
import ssl
import json
import urllib.request
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from teleop.utils.server_process import ensure_server_running, stop_spawned_server

PANEL_API_URL = "https://localhost:8012/api/command"
STATUS_API_URL = "https://localhost:8012/api/status"


def get_ssl_context():
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


def post_cmd(cmd_dict: dict):
    ctx = get_ssl_context()
    data = json.dumps(cmd_dict).encode("utf-8")
    req = urllib.request.Request(
        PANEL_API_URL,
        data=data,
        headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, context=ctx, timeout=3.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def get_status():
    ctx = get_ssl_context()
    req = urllib.request.Request(STATUS_API_URL)
    with urllib.request.urlopen(req, context=ctx, timeout=3.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def run_tests():
    print("=" * 80)
    print("RUNNING AUTOMATED WEB CONTROL PANEL API TEST")
    print("=" * 80)

    spawned_proc = None
    try:
        spawned_proc, did_spawn = ensure_server_running(timeout_s=20.0)

        # 1. Wait for server status ready
        print("[INFO] Connecting to https://localhost:8012/api/status ...")
        st = None
        for _ in range(25):
            try:
                st = get_status()
                if st and "ee_l" in st and len(st["ee_l"]) == 3:
                    break
            except Exception:
                time.sleep(0.5)

        if not st or "ee_l" not in st:
            print("[FAIL] Web Control Panel server not responding on https://localhost:8012.")
            return False

        print("[PASS] Connected to Web Control Panel server.")

        # Reset to known home first
        post_cmd({"name": "preset", "pose": "reset"})
        time.sleep(0.8)
        st_home = get_status()
        home_l = np.array(st_home["ee_l"])
        home_r = np.array(st_home["ee_r"])
        print(f"Home Left EE : [{home_l[0]:+.3f}, {home_l[1]:+.3f}, {home_l[2]:+.3f}] m")
        print(f"Home Right EE: [{home_r[0]:+.3f}, {home_r[1]:+.3f}, {home_r[2]:+.3f}] m")

        passed_all = True

        # TEST 1: Left +X 10 cm (0.10 m)
        print("\n--- TEST 1: Left Arm Jog +X 10 cm ---")
        post_cmd({"name": "jog_l", "axis": 0, "step": 0.10})
        time.sleep(1.0)
        st = get_status()
        actual_disp_l = np.array(st["ee_l"]) - home_l
        err_pct_x = abs(actual_disp_l[0] - 0.10) / 0.10 * 100.0
        sign_ok = (actual_disp_l[0] > 0)
        t1_pass = (err_pct_x <= 15.0) and sign_ok
        print(f"Result: disp=[{actual_disp_l[0]:+.3f}, {actual_disp_l[1]:+.3f}, {actual_disp_l[2]:+.3f}] m | "
              f"X Error: {err_pct_x:.1f}% | Pass: {t1_pass}")
        if not t1_pass: passed_all = False

        # TEST 2: Right +Z 10 cm (0.10 m)
        print("\n--- TEST 2: Right Arm Jog +Z 10 cm ---")
        post_cmd({"name": "jog_r", "axis": 2, "step": 0.10})
        time.sleep(0.8)
        st = get_status()
        actual_disp_r = np.array(st["ee_r"]) - home_r
        err_pct_z = abs(actual_disp_r[2] - 0.10) / 0.10 * 100.0
        sign_ok = (actual_disp_r[2] > 0)
        t2_pass = (err_pct_z <= 15.0) and sign_ok
        print(f"Result: disp=[{actual_disp_r[0]:+.3f}, {actual_disp_r[1]:+.3f}, {actual_disp_r[2]:+.3f}] m | "
              f"Z Error: {err_pct_z:.1f}% | Pass: {t2_pass}")
        if not t2_pass: passed_all = False

        # TEST 3: Left Close Hand
        print("\n--- TEST 3: Left Hand Close ---")
        post_cmd({"name": "hand_l", "action": "close"})
        time.sleep(0.6)
        st = get_status()
        closure = st.get("hand_closure_l", 0.0)
        t3_pass = (closure >= 0.90)
        print(f"Result: closure={closure*100:.1f}% (expected >= 90%) | Pass: {t3_pass}")
        if not t3_pass: passed_all = False

        # TEST 4: Wave Right Hand 3 s
        print("\n--- TEST 4: Wave Right Hand (3 s sine motion) ---")
        post_cmd({"name": "preset", "pose": "wave"})
        positions = []
        for _ in range(6):
            time.sleep(0.5)
            st = get_status()
            positions.append(np.array(st["ee_r"]))
        pos_matrix = np.array(positions)
        y_variation = float(np.ptp(pos_matrix[:, 1]))
        t4_pass = (y_variation > 0.04)
        print(f"Result: Right EE Y-axis peak-to-peak motion={y_variation*1000.0:.1f} mm | Pass: {t4_pass}")
        if not t4_pass: passed_all = False

        # TEST 5: Reset to Home Keyframe
        print("\n--- TEST 5: Reset to Home Keyframe ---")
        post_cmd({"name": "preset", "pose": "reset"})
        time.sleep(1.0)
        st = get_status()
        dev_l = np.linalg.norm(np.array(st["ee_l"]) - home_l) * 1000.0
        dev_r = np.linalg.norm(np.array(st["ee_r"]) - home_r) * 1000.0
        t5_pass = (dev_l < 1.0) and (dev_r < 1.0)
        print(f"Result: Home Deviation Left: {dev_l:.4f} mm, Right: {dev_r:.4f} mm (< 1.0 mm) | Pass: {t5_pass}")
        if not t5_pass: passed_all = False

        # TEST 6: Emergency Stop (E-STOP)
        print("\n--- TEST 6: Emergency Stop (E-STOP) ---")
        post_cmd({"name": "estop"})
        time.sleep(0.5)
        st = get_status()
        estop_active = st.get("estop", False)
        # Attempt to jog while in e-stop
        p_before = np.array(st["ee_l"])
        post_cmd({"name": "jog_l", "axis": 0, "step": 0.10})
        time.sleep(0.5)
        st2 = get_status()
        p_after = np.array(st2["ee_l"])
        moved = float(np.linalg.norm(p_after - p_before))
        t6_pass = estop_active and (moved < 1e-4)
        print(f"Result: E-STOP Active: {estop_active} | Motion while stopped: {moved*1000.0:.4f} mm | Pass: {t6_pass}")
        if not t6_pass: passed_all = False

        # Clear E-STOP to leave server clean
        post_cmd({"name": "clear_estop"})

        # TEST 7: 3D Gizmo Endpoint - Right Wrist Translation (+10 cm Z)
        print("\n--- TEST 7: 3D Gizmo Endpoint - Right Wrist Translate (+10 cm Z) ---")
        post_cmd({"name": "preset", "pose": "reset"})
        time.sleep(0.8)
        st_reset = get_status()
        cur_l = np.array(st_reset["ee_l"])
        cur_r = np.array(st_reset["ee_r"])

        # Send target marker displacement to right wrist via set_wrist_target
        target_pos_r = [float(cur_r[0]), float(cur_r[1]), float(cur_r[2] + 0.10)]
        post_cmd({
            "cmd": "set_wrist_target",
            "side": "right",
            "arm": "right",
            "pos": target_pos_r
        })
        time.sleep(1.0)
        st_r_moved = get_status()
        disp_r = np.array(st_r_moved["ee_r"]) - cur_r
        disp_l_iso = np.array(st_r_moved["ee_l"]) - cur_l
        err_pct_z_gizmo = abs(disp_r[2] - 0.10) / 0.10 * 100.0
        isolation_l_mm = float(np.linalg.norm(disp_l_iso)) * 1000.0
        t7_pass = (err_pct_z_gizmo <= 15.0) and (isolation_l_mm < 2.0)
        print(f"Result: Right disp Z={disp_r[2]*1000.0:+.1f} mm (Err: {err_pct_z_gizmo:.1f}%) | "
              f"Left unchanged: {isolation_l_mm:.2f} mm (< 2.0 mm) | Pass: {t7_pass}")
        if not t7_pass: passed_all = False

        # TEST 8: 3D Gizmo Endpoint - Left & Right Wrist Rotation (+30 deg Yaw)
        print("\n--- TEST 8: 3D Gizmo Endpoint - Left/Right Wrist Rotation (+30 deg Yaw) ---")
        from scipy.spatial.transform import Rotation as R
        yaw30 = R.from_euler("z", 30, degrees=True)

        # 8a: Left Wrist +30 deg Yaw
        post_cmd({"name": "preset", "pose": "reset"})
        time.sleep(0.8)
        st_pre = get_status()
        rot_l_orig = np.array(st_pre["rot_l"])
        rot_r_orig = np.array(st_pre["rot_r"])
        rot_l_target = (yaw30 * R.from_matrix(rot_l_orig)).as_matrix()
        ql_scipy = R.from_matrix(rot_l_target).as_quat() # [x, y, z, w]
        ql_param = [float(ql_scipy[3]), float(ql_scipy[0]), float(ql_scipy[1]), float(ql_scipy[2])] # [w, x, y, z]

        post_cmd({
            "cmd": "set_wrist_target",
            "side": "left",
            "arm": "left",
            "pos": st_pre["ee_l"],
            "rot_quat": ql_param
        })
        time.sleep(1.0)
        st_rot_l = get_status()
        rot_l_achieved = np.array(st_rot_l["rot_l"])
        rot_l_diff = yaw30.inv() * (R.from_matrix(rot_l_achieved) * R.from_matrix(rot_l_orig).inv())
        angle_l_err_deg = abs(float(np.rad2deg(rot_l_diff.magnitude())))
        right_rot_diff = R.from_matrix(st_rot_l["rot_r"]) * R.from_matrix(rot_r_orig).inv()
        right_rot_iso_deg = float(np.rad2deg(right_rot_diff.magnitude()))
        t8a_pass = (angle_l_err_deg <= 3.0) and (right_rot_iso_deg <= 1.0)
        print(f"Result (Left Yaw): err={angle_l_err_deg:.2f}° (<= 3.0°) | Right unchanged: {right_rot_iso_deg:.2f}° | Pass: {t8a_pass}")
        if not t8a_pass: passed_all = False

        # 8b: Right Wrist +30 deg Yaw
        post_cmd({"name": "preset", "pose": "reset"})
        time.sleep(0.8)
        st_pre_r = get_status()
        rot_l_orig2 = np.array(st_pre_r["rot_l"])
        rot_r_orig2 = np.array(st_pre_r["rot_r"])
        rot_r_target = (yaw30 * R.from_matrix(rot_r_orig2)).as_matrix()
        qr_scipy = R.from_matrix(rot_r_target).as_quat() # [x, y, z, w]
        qr_param = [float(qr_scipy[3]), float(qr_scipy[0]), float(qr_scipy[1]), float(qr_scipy[2])]

        post_cmd({
            "cmd": "set_wrist_target",
            "side": "right",
            "arm": "right",
            "pos": st_pre_r["ee_r"],
            "rot_quat": qr_param
        })
        time.sleep(1.0)
        st_rot_r = get_status()
        rot_r_achieved = np.array(st_rot_r["rot_r"])
        rot_r_diff = yaw30.inv() * (R.from_matrix(rot_r_achieved) * R.from_matrix(rot_r_orig2).inv())
        angle_r_err_deg = abs(float(np.rad2deg(rot_r_diff.magnitude())))
        left_rot_diff = R.from_matrix(st_rot_r["rot_l"]) * R.from_matrix(rot_l_orig2).inv()
        left_rot_iso_deg = float(np.rad2deg(left_rot_diff.magnitude()))
        t8b_pass = (angle_r_err_deg <= 3.0) and (left_rot_iso_deg <= 1.0)
        print(f"Result (Right Yaw): err={angle_r_err_deg:.2f}° (<= 3.0°) | Left unchanged: {left_rot_iso_deg:.2f}° | Pass: {t8b_pass}")
        if not t8b_pass: passed_all = False

        # Final reset to home
        post_cmd({"name": "preset", "pose": "reset"})

        print("=" * 80)
        if passed_all:
            print("[SUCCESS] ALL PANEL API ACCEPTANCE TESTS PASSED!")
        else:
            print("[FAILURE] One or more panel API tests failed.")

        return passed_all
    finally:
        if spawned_proc:
            stop_spawned_server(spawned_proc)


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)
