"""
scripts/test_xr_controllers_p1.py
First half of Quest 3S WebXR Controller test suite (Tests 1 to 5).
"""

import time, json, asyncio
import numpy as np
from scipy.spatial.transform import Rotation as R


async def run_xr_tests_part1(session, ws, base_ctrl_pos, quat_identity, home_l, fetch_status):
    # ------------------------------------------------------------------
    # TEST 1: Trigger-only moves position only (rotation locked)
    # ------------------------------------------------------------------
    print("\n--- TEST 1: Trigger-Only Moves Position Only (Rotation Locked) ---")
    pkt_anchor = {
        "cmd": "xr_controllers",
        "t": time.time(),
        "left": {
            "pos": base_ctrl_pos.tolist(),
            "quat": quat_identity,
            "grip": False,
            "grip_val": 0.0,
            "trigger": 1.0
        }
    }
    await ws.send_str(json.dumps(pkt_anchor))
    await asyncio.sleep(0.05)

    st_init = await fetch_status(session)
    init_tgt_rot_l = np.array(st_init["tgt_rot_l"])

    rot_yaw_30 = R.from_euler('z', 30.0, degrees=True).as_quat()
    quat_yaw_30 = [rot_yaw_30[3], rot_yaw_30[0], rot_yaw_30[1], rot_yaw_30[2]]

    for i in range(1, 20):
        frac = min(1.0, i / 10.0)
        cur_p = base_ctrl_pos + np.array([0.0, 0.0, -0.10 * frac])
        pkt = {
            "cmd": "xr_controllers",
            "t": time.time(),
            "left": {
                "pos": cur_p.tolist(),
                "quat": quat_yaw_30,
                "grip": False,
                "grip_val": 0.0,
                "trigger": 1.0
            }
        }
        await ws.send_str(json.dumps(pkt))
        await asyncio.sleep(0.02)

    await asyncio.sleep(0.1)
    st1 = await fetch_status(session)
    tgt_disp_x = st1["tgt_l"][0] - home_l[0]
    tgt_rot_change = np.linalg.norm(np.array(st1["tgt_rot_l"]) - init_tgt_rot_l)
    print(f"  Target Disp X: {tgt_disp_x*100.0:+.2f} cm (expected ~+10.0 cm)")
    print(f"  Target Rotation change: {tgt_rot_change:.4f} (expected 0.0)")
    assert abs(tgt_disp_x - 0.10) < 0.02, f"Target disp {tgt_disp_x} not near 0.10m"
    assert tgt_rot_change < 0.001, f"Target rotation changed while trigger-only: {tgt_rot_change}"
    print("  [PASS] Trigger-only moved position only; rotation remained locked.")

    # ------------------------------------------------------------------
    # TEST 2: Grip-only rotates only (position locked)
    # ------------------------------------------------------------------
    print("\n--- TEST 2: Grip-Only Rotates Only (Position Locked) ---")
    tgt_before_trig = np.array(st1["tgt_l"])
    tgt_rot_before_trig = np.array(st1["tgt_rot_l"])
    cur_ctrl_pos = base_ctrl_pos + np.array([0.0, 0.0, -0.10])
    pkt_trig_anchor = {
        "cmd": "xr_controllers",
        "t": time.time(),
        "left": {
            "pos": cur_ctrl_pos.tolist(),
            "quat": quat_identity,
            "grip": True,
            "grip_val": 1.0,
            "trigger": 0.0
        }
    }
    await ws.send_str(json.dumps(pkt_trig_anchor))
    await asyncio.sleep(0.05)

    for i in range(1, 20):
        p_offset = cur_ctrl_pos + np.array([0.05 * (i / 15.0), 0.0, 0.0])
        rot_step = R.from_euler('y', 30.0 * (i / 15.0), degrees=True).as_quat()
        q_step = [rot_step[3], rot_step[0], rot_step[1], rot_step[2]]
        pkt = {
            "cmd": "xr_controllers",
            "t": time.time(),
            "left": {
                "pos": p_offset.tolist(),
                "quat": q_step,
                "grip": True,
                "grip_val": 1.0,
                "trigger": 0.0
            }
        }
        await ws.send_str(json.dumps(pkt))
        await asyncio.sleep(0.02)

    await asyncio.sleep(0.1)
    st2 = await fetch_status(session)
    tgt_disp_pos = np.linalg.norm(np.array(st2["tgt_l"]) - tgt_before_trig)
    rot_change2 = np.linalg.norm(np.array(st2["tgt_rot_l"]) - tgt_rot_before_trig)
    print(f"  Target Pos Drift: {tgt_disp_pos*1000.0:.2f} mm (expected < 2.0 mm)")
    print(f"  Target Rotation change: {rot_change2:.4f} (expected > 0.05)")
    assert tgt_disp_pos < 0.002, f"Target pos moved during grip-only: {tgt_disp_pos*1000.0} mm"
    assert rot_change2 > 0.05, "Target rotation did not update during grip-only"
    print("  [PASS] Grip-only rotated only; position remained locked.")

    # ------------------------------------------------------------------
    # TEST 3: Both Held -> Full 6D Teleoperation
    # ------------------------------------------------------------------
    print("\n--- TEST 3: Both Held -> Full 6D Teleoperation ---")
    tgt_before_6d = np.array(st2["tgt_l"])
    rot_before_6d = np.array(st2["tgt_rot_l"])
    pkt_both = {
        "cmd": "xr_controllers",
        "t": time.time(),
        "left": {
            "pos": cur_ctrl_pos.tolist(),
            "quat": quat_identity,
            "grip": True,
            "grip_val": 1.0,
            "trigger": 1.0
        }
    }
    await ws.send_str(json.dumps(pkt_both))
    await asyncio.sleep(0.05)

    for i in range(1, 15):
        cur_p = cur_ctrl_pos + np.array([-0.05 * (i / 10.0), 0.05 * (i / 10.0), 0.0])
        rot_step = R.from_euler('z', 20.0 * (i / 10.0), degrees=True).as_quat()
        q_step = [rot_step[3], rot_step[0], rot_step[1], rot_step[2]]
        pkt = {
            "cmd": "xr_controllers",
            "t": time.time(),
            "left": {
                "pos": cur_p.tolist(),
                "quat": q_step,
                "grip": True,
                "grip_val": 1.0,
                "trigger": 1.0
            }
        }
        await ws.send_str(json.dumps(pkt))
        await asyncio.sleep(0.02)

    await asyncio.sleep(0.1)
    st3 = await fetch_status(session)
    pos_6d_diff = np.linalg.norm(np.array(st3["tgt_l"]) - tgt_before_6d)
    rot_6d_diff = np.linalg.norm(np.array(st3["tgt_rot_l"]) - rot_before_6d)
    print(f"  6D Pos change: {pos_6d_diff*100.0:.2f} cm")
    print(f"  6D Rot change: {rot_6d_diff:.4f}")
    assert pos_6d_diff > 0.03, "Position did not update in 6D mode"
    assert rot_6d_diff > 0.05, "Rotation did not update in 6D mode"
    print("  [PASS] 6D mode teleoperates both position and rotation simultaneously.")

    # ------------------------------------------------------------------
    # TEST 4: No Jump on State Change (< 2 mm / 0.5 deg)
    # ------------------------------------------------------------------
    print("\n--- TEST 4: No Jump on State Transitions (< 2 mm / 0.5 deg) ---")
    transitions = [
        ("LOCKED", 0.0, 0.0),
        ("TRANSLATE", 0.0, 1.0),
        ("6D", 1.0, 1.0),
        ("ROTATE", 1.0, 0.0),
        ("LOCKED", 0.0, 0.0)
    ]
    cur_test_p = base_ctrl_pos + np.array([0.1, 0.05, -0.05])
    for name, g_val, t_val in transitions:
        st_pre = await fetch_status(session)
        p_pre = np.array(st_pre["tgt_l"])
        r_pre = np.array(st_pre["tgt_rot_l"])

        pkt_trans = {
            "cmd": "xr_controllers",
            "t": time.time(),
            "left": {
                "pos": cur_test_p.tolist(),
                "quat": quat_identity,
                "grip": g_val > 0.5,
                "grip_val": g_val,
                "trigger": t_val
            }
        }
        await ws.send_str(json.dumps(pkt_trans))
        await asyncio.sleep(0.05)

        st_post = await fetch_status(session)
        p_post = np.array(st_post["tgt_l"])
        r_post = np.array(st_post["tgt_rot_l"])

        jump_mm = np.linalg.norm(p_post - p_pre) * 1000.0
        rot_jump_deg = np.rad2deg(np.arccos(np.clip((np.trace(r_post @ r_pre.T) - 1.0) / 2.0, -1.0, 1.0)))
        print(f"  Transition -> {name:9s} : Jump = {jump_mm:.3f} mm, Rot Jump = {rot_jump_deg:.3f} deg")
        assert jump_mm < 2.0, f"Position jump {jump_mm} mm exceeds 2.0 mm limit on {name}"
        assert rot_jump_deg < 0.5, f"Rotation jump {rot_jump_deg} deg exceeds 0.5 deg limit on {name}"
    print("  [PASS] Zero jump on all clutch state transitions verified.")

    # ------------------------------------------------------------------
    # TEST 5: Joystick Y Velocity Closes Then Holds
    # ------------------------------------------------------------------
    print("\n--- TEST 5: Thumbstick Y Velocity Control of Closure & Hold ---")
    for _ in range(15):
        pkt_stick = {
            "cmd": "xr_controllers",
            "t": time.time(),
            "left": {
                "pos": base_ctrl_pos.tolist(),
                "quat": quat_identity,
                "grip": False,
                "trigger": 0.0,
                "thumbstick": [0.0, -0.8]
            }
        }
        await ws.send_str(json.dumps(pkt_stick))
        await asyncio.sleep(0.03)

    await asyncio.sleep(0.18)
    st5 = await fetch_status(session)
    closure_after_push = st5.get("xr_status", {}).get("left", {}).get("closure", st5["hand_closure_l"])
    print(f"  Hand closure after push: {closure_after_push*100.0:.1f} %")
    assert closure_after_push > 0.15, f"Closure {closure_after_push} did not increase on stick push"

    for _ in range(10):
        pkt_hold = {
            "cmd": "xr_controllers",
            "t": time.time(),
            "left": {
                "pos": base_ctrl_pos.tolist(),
                "quat": quat_identity,
                "grip": False,
                "trigger": 0.0,
                "thumbstick": [0.0, 0.0]
            }
        }
        await ws.send_str(json.dumps(pkt_hold))
        await asyncio.sleep(0.03)

    await asyncio.sleep(0.08)
    st5_hold = await fetch_status(session)
    closure_held = st5_hold.get("xr_status", {}).get("left", {}).get("closure", st5_hold["hand_closure_l"])
    print(f"  Hand closure after release: {closure_held*100.0:.1f} %")
    assert abs(closure_held - closure_after_push) < 0.001, "Closure did not hold when thumbstick released"
    print("  [PASS] Thumbstick Y closes fingers with velocity and holds upon release.")

