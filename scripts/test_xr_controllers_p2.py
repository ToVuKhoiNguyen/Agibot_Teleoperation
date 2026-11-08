"""
scripts/test_xr_controllers_p2.py
Second half of Quest 3S WebXR Controller test suite (Tests 6 to 10).
"""

import time, json, asyncio
import numpy as np


async def run_xr_tests_part2(session, ws, base_ctrl_pos, quat_identity, home_r, fetch_status, send_rest_cmd):
    # TEST 6: Thumbstick Click Panic-Open
    print("\n--- TEST 6: Thumbstick Click -> Panic Open (Resets Closure to 0) ---")
    pkt_click = {
        "cmd": "xr_controllers",
        "t": time.time(),
        "left": {
            "pos": base_ctrl_pos.tolist(),
            "quat": quat_identity,
            "thumbstick": [0.0, 0.0],
            "thumbstick_click": True
        }
    }
    await ws.send_str(json.dumps(pkt_click))
    await asyncio.sleep(0.15)
    st6 = await fetch_status(session)
    closure_panic = st6.get("xr_status", {}).get("left", {}).get("closure", st6["hand_closure_l"])
    print(f"  Closure after panic click: {closure_panic*100.0:.1f} %")
    assert closure_panic < 0.05, f"Hand did not panic open: {closure_panic}"
    print("  [PASS] Panic open reset closure immediately to 0.")

    # TEST 7: Button A Hold 1.5s Goes Home
    print("\n--- TEST 7: Button A Hold 1.5s Goes Home (Progress Ring/Bar) ---")
    await send_rest_cmd(session, {"cmd": "arm_jog", "arm": "right", "axis": "x", "step_cm": 15, "dir": 1})
    await asyncio.sleep(0.3)
    st7_away = await fetch_status(session)
    dist_away = np.linalg.norm(np.array(st7_away["tgt_r"]) - home_r)
    assert dist_away > 0.05, "Failed to jog right arm away from home"

    for i in range(25):
        pkt_a = {
            "cmd": "xr_controllers",
            "t": time.time(),
            "right": {
                "pos": base_ctrl_pos.tolist(),
                "quat": quat_identity,
                "btn_a": True,
                "buttons": {"primary": True}
            }
        }
        await ws.send_str(json.dumps(pkt_a))
        await asyncio.sleep(0.07)

    dist_home = 999.0
    for _ in range(12):
        await asyncio.sleep(0.25)
        st7_home = await fetch_status(session)
        dist_home = np.linalg.norm(np.array(st7_home["tgt_r"]) - home_r)
        if dist_home < 0.01:
            break
    print(f"  Right target distance from home: {dist_home*1000.0:.2f} mm")
    assert dist_home < 0.01, f"Right arm did not return home on 1.5s hold: {dist_home*1000.0} mm"
    print("  [PASS] A-hold 1.5s returned arm to home marker pose.")

    # TEST 8: Button B Toggles Precision Mode
    print("\n--- TEST 8: Button B Toggles Precision Mode (Scale x0.3) ---")
    pkt_b_press = {
        "cmd": "xr_controllers",
        "t": time.time(),
        "right": {"pos": base_ctrl_pos.tolist(), "btn_b": True, "buttons": {"secondary": True}}
    }
    await ws.send_str(json.dumps(pkt_b_press))
    await asyncio.sleep(0.1)
    st8_on = await fetch_status(session)
    prec_on = st8_on.get("xr_status", {}).get("right", {}).get("precision", False)
    print(f"  Precision mode status: {prec_on}")
    assert prec_on, "Precision mode was not enabled by Button B"

    pkt_b_rel = {"cmd": "xr_controllers", "t": time.time(), "right": {"pos": base_ctrl_pos.tolist(), "btn_b": False, "buttons": {"secondary": False}}}
    await ws.send_str(json.dumps(pkt_b_rel))
    await asyncio.sleep(0.05)
    pkt_b_press2 = {"cmd": "xr_controllers", "t": time.time(), "right": {"pos": base_ctrl_pos.tolist(), "btn_b": True, "buttons": {"secondary": True}}}
    await ws.send_str(json.dumps(pkt_b_press2))
    await asyncio.sleep(0.1)
    st8_off = await fetch_status(session)
    prec_off = st8_off.get("xr_status", {}).get("right", {}).get("precision", False)
    print(f"  Precision mode after 2nd press: {prec_off}")
    assert not prec_off, "Precision mode was not toggled off"
    print("  [PASS] Button B cleanly toggles precision mode.")

    # TEST 9: B + Y Held 0.5s Triggers E-STOP
    print("\n--- TEST 9: Quest E-Stop Combo (B + Y held > 0.5s) ---")
    for _ in range(15):
        pkt_estop = {
            "cmd": "xr_controllers",
            "t": time.time(),
            "left": {"pos": base_ctrl_pos.tolist(), "btn_y": True, "buttons": {"secondary": True}},
            "right": {"pos": base_ctrl_pos.tolist(), "btn_b": True, "buttons": {"secondary": True}}
        }
        await ws.send_str(json.dumps(pkt_estop))
        await asyncio.sleep(0.05)

    await asyncio.sleep(0.2)
    st9 = await fetch_status(session)
    print(f"  E-STOP Status: {st9.get('estop')}")
    assert st9.get("estop", False), "E-STOP was not triggered by B+Y hold!"
    print("  [PASS] E-STOP triggered reliably by B+Y combo.")

    await send_rest_cmd(session, {"name": "clear_estop"})
    await send_rest_cmd(session, {"name": "preset", "pose": "reset"})
    await asyncio.sleep(0.5)

    # TEST 10: 350 ms Tracking Watchdog
    print("\n--- TEST 10: 350 ms Tracking Watchdog ---")
    for _ in range(5):
        pkt_engage = {
            "cmd": "xr_controllers",
            "t": time.time(),
            "left": {
                "pos": base_ctrl_pos.tolist(),
                "quat": quat_identity,
                "grip": True,
                "grip_val": 1.0,
                "trigger": 1.0
            }
        }
        await ws.send_str(json.dumps(pkt_engage))
        await asyncio.sleep(0.02)
    await asyncio.sleep(0.05)
    st10_pre = await fetch_status(session)
    assert st10_pre.get("xr_grip_l", False), "Clutch should be active prior to silence"

    print("  Ceasing packets for 0.45 s to trigger watchdog...")
    await asyncio.sleep(0.45)
    st10_post = await fetch_status(session)
    grip_after = st10_post.get("xr_grip_l", False)
    clutch_after = st10_post.get("xr_status", {}).get("left", {}).get("clutch_state", "LOCKED")
    print(f"  Clutch state after timeout: {clutch_after} | Grip L: {grip_after}")
    assert not grip_after and clutch_after == "LOCKED", "Watchdog failed to release clutch"
    print("  [PASS] Watchdog released clutch after 350 ms silence.")

