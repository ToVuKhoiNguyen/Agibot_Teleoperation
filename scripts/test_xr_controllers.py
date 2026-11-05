#!/usr/bin/env python3
"""
scripts/test_xr_controllers.py
Automated acceptance test for Quest 3S WebXR Controller Teleoperation:
  1. Grip-only moves position only (rotation locked)
  2. Trigger-only rotates only (position locked)
  3. Both held -> full 6D teleoperation
  4. No jump on state change (< 2 mm / 0.5 deg)
  5. Joystick Y velocity closes then holds
  6. Thumbstick click panic-open
  7. Button A hold 1.5s goes home (progress increments to 1.0)
  8. Button B toggles precision mode (x0.3 scale)
  9. Button B+Y 0.5s combo triggers E-STOP
  10. 350 ms watchdog releases clutches
"""

import os
import sys
import time
import ssl
import json
import asyncio
import aiohttp
import numpy as np
from scipy.spatial.transform import Rotation as R

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

                base_ctrl_pos = np.array([0.2, 1.2, -0.4])  # XR: x=0.2, y=1.2, z=-0.4
                quat_identity = [1.0, 0.0, 0.0, 0.0]        # [w, x, y, z]

                # ------------------------------------------------------------------
                # TEST 1: Grip-only moves position only (rotation locked)
                # ------------------------------------------------------------------
                print("\n--- TEST 1: Grip-Only Moves Position Only (Rotation Locked) ---")
                # Anchor in grip-only
                pkt_anchor = {
                    "cmd": "xr_controllers",
                    "t": time.time(),
                    "left": {
                        "pos": base_ctrl_pos.tolist(),
                        "quat": quat_identity,
                        "grip": True,
                        "grip_val": 1.0,
                        "trigger": 0.0
                    }
                }
                await ws.send_str(json.dumps(pkt_anchor))
                await asyncio.sleep(0.05)

                st_init = await fetch_status(session)
                init_tgt_rot_l = np.array(st_init["tgt_rot_l"])

                # Move position by +10 cm in XR (-Z) AND apply 30-deg yaw rotation in controller
                rot_yaw_30 = R.from_euler('z', 30.0, degrees=True).as_quat()  # [x, y, z, w]
                quat_yaw_30 = [rot_yaw_30[3], rot_yaw_30[0], rot_yaw_30[1], rot_yaw_30[2]]  # [w, x, y, z]

                for i in range(1, 20):
                    frac = min(1.0, i / 10.0)
                    cur_p = base_ctrl_pos + np.array([0.0, 0.0, -0.10 * frac])
                    pkt = {
                        "cmd": "xr_controllers",
                        "t": time.time(),
                        "left": {
                            "pos": cur_p.tolist(),
                            "quat": quat_yaw_30,
                            "grip": True,
                            "grip_val": 1.0,
                            "trigger": 0.0
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
                assert tgt_rot_change < 0.001, f"Target rotation changed while grip-only: {tgt_rot_change}"
                print("  [PASS] Grip-only moved position only; rotation remained locked.")

                # ------------------------------------------------------------------
                # TEST 2: Trigger-only rotates only (position locked)
                # ------------------------------------------------------------------
                print("\n--- TEST 2: Trigger-Only Rotates Only (Position Locked) ---")
                tgt_before_trig = np.array(st1["tgt_l"])
                tgt_rot_before_trig = np.array(st1["tgt_rot_l"])
                # Transition to Trigger-only (grip=0, trigger=1.0)
                cur_ctrl_pos = base_ctrl_pos + np.array([0.0, 0.0, -0.10])
                pkt_trig_anchor = {
                    "cmd": "xr_controllers",
                    "t": time.time(),
                    "left": {
                        "pos": cur_ctrl_pos.tolist(),
                        "quat": quat_identity,
                        "grip": False,
                        "grip_val": 0.0,
                        "trigger": 1.0
                    }
                }
                await ws.send_str(json.dumps(pkt_trig_anchor))
                await asyncio.sleep(0.05)

                # Rotate controller yaw +30 deg while also moving controller pos (which should be ignored!)
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
                            "grip": False,
                            "grip_val": 0.0,
                            "trigger": 1.0
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
                assert tgt_disp_pos < 0.002, f"Target pos moved during trigger-only: {tgt_disp_pos*1000.0} mm"
                assert rot_change2 > 0.05, "Target rotation did not update during trigger-only"
                print("  [PASS] Trigger-only rotated only; position remained locked.")

                # ------------------------------------------------------------------
                # TEST 3: Both Held -> Full 6D Teleoperation
                # ------------------------------------------------------------------
                print("\n--- TEST 3: Both Held -> Full 6D Teleoperation ---")
                tgt_before_6d = np.array(st2["tgt_l"])
                rot_before_6d = np.array(st2["tgt_rot_l"])
                # Engage both
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
                    ("TRANSLATE", 1.0, 0.0),
                    ("6D", 1.0, 1.0),
                    ("ROTATE", 0.0, 1.0),
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
                # Push thumbstick forward (y = -0.8) for 0.4 s to close fingers
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

                # Release thumbstick (y = 0.0) for 0.3 s: closure must HOLD!
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

                # ------------------------------------------------------------------
                # TEST 6: Thumbstick Click Panic-Open
                # ------------------------------------------------------------------
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

                # ------------------------------------------------------------------
                # TEST 7: Button A Hold 1.5s Goes Home
                # ------------------------------------------------------------------
                print("\n--- TEST 7: Button A Hold 1.5s Goes Home (Progress Ring/Bar) ---")
                # Move Right arm away from home first
                await send_rest_cmd(session, {"cmd": "arm_jog", "arm": "right", "axis": "x", "step_cm": 15, "dir": 1})
                await asyncio.sleep(0.3)
                st7_away = await fetch_status(session)
                dist_away = np.linalg.norm(np.array(st7_away["tgt_r"]) - home_r)
                assert dist_away > 0.05, "Failed to jog right arm away from home"

                # Send Button A held for 1.6s
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

                # Wait for smooth homing interpolation to complete (home_duration = 1.5s)
                await asyncio.sleep(1.6)
                st7_home = await fetch_status(session)
                dist_home = np.linalg.norm(np.array(st7_home["tgt_r"]) - home_r)
                print(f"  Right target distance from home: {dist_home*1000.0:.2f} mm")
                assert dist_home < 0.01, f"Right arm did not return home on 1.5s hold: {dist_home*1000.0} mm"
                print("  [PASS] A-hold 1.5s returned arm to home marker pose.")

                # ------------------------------------------------------------------
                # TEST 8: Button B Toggles Precision Mode
                # ------------------------------------------------------------------
                print("\n--- TEST 8: Button B Toggles Precision Mode (Scale x0.3) ---")
                # Rising edge B
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

                # Release B then press B again -> toggle OFF
                pkt_b_rel = {"cmd": "xr_controllers", "t": time.time(), "right": {"pos": base_ctrl_pos.tolist(), "btn_b": False}}
                await ws.send_str(json.dumps(pkt_b_rel))
                await asyncio.sleep(0.05)
                await ws.send_str(json.dumps(pkt_b_press))
                await asyncio.sleep(0.1)
                st8_off = await fetch_status(session)
                prec_off = st8_off.get("xr_status", {}).get("right", {}).get("precision", False)
                print(f"  Precision mode after 2nd press: {prec_off}")
                assert not prec_off, "Precision mode was not toggled off"
                print("  [PASS] Button B cleanly toggles precision mode.")

                # ------------------------------------------------------------------
                # TEST 9: B + Y Held 0.5s Triggers E-STOP
                # ------------------------------------------------------------------
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

                # Clear E-STOP
                await send_rest_cmd(session, {"name": "clear_estop"})
                await send_rest_cmd(session, {"name": "preset", "pose": "reset"})
                await asyncio.sleep(0.5)

                # ------------------------------------------------------------------
                # TEST 10: 350 ms Tracking Watchdog
                # ------------------------------------------------------------------
                print("\n--- TEST 10: 350 ms Tracking Watchdog ---")
                pkt_engage = {
                    "cmd": "xr_controllers",
                    "t": time.time(),
                    "left": {
                        "pos": base_ctrl_pos.tolist(),
                        "quat": quat_identity,
                        "grip": True,
                        "grip_val": 1.0
                    }
                }
                await ws.send_str(json.dumps(pkt_engage))
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

        print("\n" + "=" * 80)
        print("ALL QUEST 3S CONTROLLER & MAPPING TESTS PASSED SUCCESSFULLY!")
        print("=" * 80)
        return True

    finally:
        if did_spawn and spawned_proc:
            stop_spawned_server(spawned_proc)


if __name__ == "__main__":
    success = asyncio.run(run_xr_tests())
    sys.exit(0 if success else 1)
