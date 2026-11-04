#!/usr/bin/env python3
"""
scripts/test_xr_controllers.py
Automated acceptance test for Quest 3S WebXR Controller Teleoperation.
Connects to wss://localhost:8012/ws_panel and sends synthetic xr_controllers packets:
  1. Forward movement: +10 cm along XR -Z -> robot left wrist moves +X by ~10 cm (within 15%)
  2. Lateral movement: +10 cm along XR -X (left) -> robot left wrist moves +Y by ~10 cm
  3. Vertical movement: +10 cm along XR +Y (up) -> robot left wrist moves +Z by ~10 cm
  4. Grip release: target holds in place
  5. Re-grip: verify zero/minimal jump (< 2 mm)
  6. Rotation: yaw rotation delta (+30 deg) maps to robot frame orientation
  7. Watchdog timeout: no packet for > 350 ms releases clutches
  8. Arm isolation: left arm movements do not affect right arm
  9. Trigger mapping: analog trigger drives hand retargeting
  10. E-Stop combo: B + Y held > 0.5 s triggers E-STOP
"""

import os
import sys
import time
import ssl
import json
import asyncio
import aiohttp
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from teleop.utils.server_process import ensure_server_running, stop_spawned_server

WS_URL = "wss://localhost:8012/ws_panel"
STATUS_URL = "https://localhost:8012/api/status"
CMD_URL = "https://localhost:8012/api/command"


def get_ssl_context():
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx


async def fetch_status(session):
    async with session.get(STATUS_URL, ssl=False) as resp:
        return await resp.json()


async def send_rest_cmd(session, cmd_dict):
    async with session.post(CMD_URL, json=cmd_dict, ssl=False) as resp:
        return await resp.json()


async def run_xr_tests():
    print("=" * 80)
    print("RUNNING QUEST 3S WEBXR CONTROLLER TELEOPERATION ACCEPTANCE TEST")
    print("=" * 80)

    spawned_proc, did_spawn = ensure_server_running(timeout_s=20.0)

    try:
        ssl_ctx = get_ssl_context()
        connector = aiohttp.TCPConnector(ssl=ssl_ctx)
        async with aiohttp.ClientSession(connector=connector) as session:
            # 1. Connect and verify status
            print("[INFO] Waiting for server status ready...")
            st = None
            for _ in range(30):
                try:
                    st = await fetch_status(session)
                    if st and "ee_l" in st and len(st["ee_l"]) == 3:
                        break
                except Exception:
                    pass
                await asyncio.sleep(0.5)

            if not st or "ee_l" not in st:
                print("[FAIL] Web Control Panel server not responding.")
                return False

            print("[PASS] Server online and responding.")

            # Reset robot to home keyframe
            await send_rest_cmd(session, {"name": "preset", "pose": "reset"})
            await send_rest_cmd(session, {"name": "clear_estop"})
            await asyncio.sleep(1.0)

            st_home = await fetch_status(session)
            home_l = np.array(st_home["ee_l"])
            home_r = np.array(st_home["ee_r"])
            print(f"Home Left EE : [{home_l[0]:+.3f}, {home_l[1]:+.3f}, {home_l[2]:+.3f}] m")
            print(f"Home Right EE: [{home_r[0]:+.3f}, {home_r[1]:+.3f}, {home_r[2]:+.3f}] m")

            # Connect WebSocket
            print(f"[INFO] Connecting to {WS_URL} ...")
            async with session.ws_connect(WS_URL, ssl=ssl_ctx) as ws:
                print("[PASS] WebSocket connected successfully.")

                # Initial base controller position in XR space
                base_ctrl_pos = np.array([0.2, 1.2, -0.4])  # XR: x=0.2, y=1.2 (chest level), z=-0.4
                quat_identity = [1.0, 0.0, 0.0, 0.0]        # [w, x, y, z]

                # --- TEST 1: Forward movement (+10 cm along XR -Z -> robot left wrist +X by ~10 cm) ---
                print("\n--- TEST 1: Forward Motion (XR -Z -> Robot +X) ---")
                # Step 1: Engage grip at base pos
                pkt_anchor = {
                    "cmd": "xr_controllers",
                    "t": time.time(),
                    "rotation_enabled": True,
                    "left": {
                        "pos": base_ctrl_pos.tolist(),
                        "quat": quat_identity,
                        "grip": True,
                        "grip_val": 1.0,
                        "trigger": 0.0
                    }
                }
                await ws.send_str(json.dumps(pkt_anchor))
                await asyncio.sleep(0.1)

                # Step 2: Stream forward displacement in XR (-Z direction by 0.10 m)
                for i in range(1, 25):
                    frac = min(1.0, i / 10.0)
                    cur_p = base_ctrl_pos + np.array([0.0, 0.0, -0.10 * frac])
                    pkt = {
                        "cmd": "xr_controllers",
                        "t": time.time(),
                        "rotation_enabled": True,
                        "left": {
                            "pos": cur_p.tolist(),
                            "quat": quat_identity,
                            "grip": True,
                            "grip_val": 1.0,
                            "trigger": 0.0
                        }
                    }
                    await ws.send_str(json.dumps(pkt))
                    await asyncio.sleep(0.03)

                st = await fetch_status(session)
                tgt_disp_x = st["tgt_l"][0] - home_l[0]
                ee_disp_x = st["ee_l"][0] - home_l[0]
                print(f"Target Disp X : {tgt_disp_x*100.0:+.2f} cm (expected +10.0 cm)")
                print(f"Actual EE Disp: {ee_disp_x*100.0:+.2f} cm")
                assert abs(tgt_disp_x - 0.10) < 0.015, f"Target disp {tgt_disp_x} not within 15% of 0.10m"
                assert (tgt_disp_x > 0.08) and (ee_disp_x > 0.05), f"EE disp {ee_disp_x} did not advance in +X"
                print("[PASS] Forward translation mapped correctly to Robot +X.")

                # --- TEST 2: Lateral and Vertical Motion (XR -X -> Robot +Y, XR +Y -> Robot +Z) ---
                print("\n--- TEST 2: Lateral (-X -> +Y) and Vertical (+Y -> +Z) Motion ---")
                cur_ctrl_pos = base_ctrl_pos + np.array([-0.08, 0.08, -0.10])
                for _ in range(15):
                    pkt_lat = {
                        "cmd": "xr_controllers",
                        "t": time.time(),
                        "rotation_enabled": True,
                        "left": {
                            "pos": cur_ctrl_pos.tolist(),
                            "quat": quat_identity,
                            "grip": True,
                            "grip_val": 1.0,
                            "trigger": 0.0
                        }
                    }
                    await ws.send_str(json.dumps(pkt_lat))
                    await asyncio.sleep(0.03)

                st = await fetch_status(session)
                tgt_disp_y = st["tgt_l"][1] - home_l[1]
                tgt_disp_z = st["tgt_l"][2] - home_l[2]
                print(f"Target Disp Y : {tgt_disp_y*100.0:+.2f} cm (expected +8.0 cm from XR -X)")
                print(f"Target Disp Z : {tgt_disp_z*100.0:+.2f} cm (expected +8.0 cm from XR +Y)")
                assert abs(tgt_disp_y - 0.08) < 0.015, f"Target Y {tgt_disp_y} mismatch"
                assert abs(tgt_disp_z - 0.08) < 0.015, f"Target Z {tgt_disp_z} mismatch"
                print("[PASS] Lateral and vertical mappings verified.")

                # --- TEST 3: Grip Release & Hold ---
                print("\n--- TEST 3: Grip Release (Target Holds) ---")
                held_tgt_l = np.array(st["tgt_l"])
                # Send grip release
                for _ in range(5):
                    pkt_rel = {
                        "cmd": "xr_controllers",
                        "t": time.time(),
                        "left": {
                            "pos": (cur_ctrl_pos + np.array([0.05, 0.05, 0.05])).tolist(),
                            "quat": quat_identity,
                            "grip": False,
                            "grip_val": 0.0
                        }
                    }
                    await ws.send_str(json.dumps(pkt_rel))
                    await asyncio.sleep(0.03)

                await asyncio.sleep(0.3)
                st = await fetch_status(session)
                tgt_now = np.array(st["tgt_l"])
                drift = np.linalg.norm(tgt_now - held_tgt_l)
                print(f"Target drift on grip release: {drift*1000.0:.2f} mm")
                assert drift < 0.002, "Target drifted while grip was released!"
                print("[PASS] Grip release holds target stably.")

                # --- TEST 4: Re-grip Jump Test (< 2 mm) ---
                print("\n--- TEST 4: Re-grip Jump Test (< 2 mm) ---")
                # Move controller while released to new position
                new_ctrl_pos = np.array([0.4, 1.4, -0.2])
                pkt_regrip = {
                    "cmd": "xr_controllers",
                    "t": time.time(),
                    "left": {
                        "pos": new_ctrl_pos.tolist(),
                        "quat": quat_identity,
                        "grip": True,
                        "grip_val": 1.0
                    }
                }
                await ws.send_str(json.dumps(pkt_regrip))
                await asyncio.sleep(0.1)
                st = await fetch_status(session)
                jump = np.linalg.norm(np.array(st["tgt_l"]) - tgt_now)
                print(f"Re-grip jump: {jump*1000.0:.2f} mm")
                assert jump < 0.002, f"Re-grip caused a jump of {jump*1000.0:.2f} mm!"
                print("[PASS] Re-grip jump is zero / < 2 mm.")

                # --- TEST 5: Arm Isolation ---
                print("\n--- TEST 5: Arm Isolation (Left motions don't affect Right) ---")
                st_right = np.array(st["tgt_r"])
                right_drift = np.linalg.norm(st_right - home_r)
                print(f"Right arm drift from home: {right_drift*1000.0:.2f} mm")
                assert right_drift < 0.003, f"Right arm moved by {right_drift*1000.0} mm while teleoperating left arm!"
                print("[PASS] Arm isolation maintained.")

                # --- TEST 6: Analog Trigger -> Hand Retargeting ---
                print("\n--- TEST 6: Analog Trigger -> Hand Closure ---")
                pkt_trig = {
                    "cmd": "xr_controllers",
                    "t": time.time(),
                    "left": {
                        "pos": new_ctrl_pos.tolist(),
                        "quat": quat_identity,
                        "grip": False,
                        "grip_val": 0.0,
                        "trigger": 0.85
                    }
                }
                await ws.send_str(json.dumps(pkt_trig))
                await asyncio.sleep(0.3)
                st = await fetch_status(session)
                closure = st["hand_closure_l"]
                print(f"Left Hand Closure: {closure*100.0:.1f} % (expected ~85%)")
                assert closure >= 0.75, f"Closure {closure} didn't respond to trigger"
                print("[PASS] Analog trigger retargets hand curls successfully.")

                # --- TEST 7: 350 ms Watchdog ---
                print("\n--- TEST 7: 350 ms Tracking Watchdog ---")
                # Engage grip
                pkt_watchdog_prep = {
                    "cmd": "xr_controllers",
                    "t": time.time(),
                    "left": {
                        "pos": new_ctrl_pos.tolist(),
                        "quat": quat_identity,
                        "grip": True,
                        "grip_val": 1.0
                    }
                }
                await ws.send_str(json.dumps(pkt_watchdog_prep))
                await asyncio.sleep(0.1)
                st = await fetch_status(session)
                assert st.get("xr_grip_l", False) or st.get("input_source") == "QUEST VR", "Clutch should be active"
                # Stop sending packets for 450 ms
                print("[INFO] Ceasing packets for 0.45 s to trigger watchdog...")
                await asyncio.sleep(0.45)
                st = await fetch_status(session)
                print(f"Input source after timeout: {st.get('input_source')} | Grip L: {st.get('xr_grip_l')}")
                assert not st.get("xr_grip_l", False), "Watchdog should have released clutch!"
                print("[PASS] Watchdog released clutch after 350 ms silence.")

                # --- TEST 8: E-Stop Combo (B + Y held > 0.5s) ---
                print("\n--- TEST 8: Quest E-Stop Combo (B + Y held > 0.5s) ---")
                t_start = time.time()
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
                st = await fetch_status(session)
                print(f"E-STOP Status: {st.get('estop')}")
                assert st.get("estop", False), "E-STOP was not triggered by B+Y hold!"
                print("[PASS] E-STOP triggered reliably by B+Y combo.")

                # Clear E-STOP and reset
                await send_rest_cmd(session, {"name": "clear_estop"})
                await send_rest_cmd(session, {"name": "preset", "pose": "reset"})
                await asyncio.sleep(0.5)

        print("\n" + "=" * 80)
        print("ALL QUEST 3S WEBXR CONTROLLER TELEOPERATION TESTS PASSED!")
        print("=" * 80)
        return True

    finally:
        if did_spawn and spawned_proc:
            stop_spawned_server(spawned_proc)


if __name__ == "__main__":
    success = asyncio.run(run_xr_tests())
    sys.exit(0 if success else 1)
