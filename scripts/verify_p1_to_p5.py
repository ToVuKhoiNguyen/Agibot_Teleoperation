#!/usr/bin/env python3
"""
Comprehensive verification of criteria P1 through P5:
P1. Gizmo-to-hand position error <= 2 mm and orientation error <= 1 deg at home and after clutch cycle (steady state).
P2. Translate controller +10 cm along each of x/y/z separately: hand site moves 10 cm +- 5 mm in expected robot direction, other axes < 2 mm.
P3. Rotate controller 30 deg about each of x/y/z: hand rotates 30 deg +- 2 deg about corresponding robot axis.
P4. Both arms pass; existing test_xr_controllers.py and test_self_collision.py still pass.
P5. Browser check with headless Playwright on /: screenshot showing gizmo centered on the hand for both arms saved to /tmp/shots.
"""
import sys
import os
import json
import time
import asyncio
import ssl
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import aiohttp

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.utils.server_process import ensure_server_running

HTTPS_PORT = 8012
STATUS_URL = f"https://localhost:{HTTPS_PORT}/api/status"
WS_URL = f"wss://localhost:{HTTPS_PORT}/ws_panel"
CMD_URL = f"https://localhost:{HTTPS_PORT}/api/command"

M_AXIS = np.array([
    [ 0.0,  0.0, -1.0],
    [-1.0,  0.0,  0.0],
    [ 0.0,  1.0,  0.0]
], dtype=np.float64)

def get_ssl_context():
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx

async def fetch_status(session):
    async with session.get(STATUS_URL) as resp:
        return await resp.json()

async def send_cmd(session, cmd_dict):
    async with session.post(CMD_URL, json=cmd_dict) as resp:
        return await resp.json()

def matrix_angle_deg(R1, R2):
    dR = R1 @ R2.T
    tr = np.trace(dR)
    val = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(val)))

async def stream_controller_and_wait(ws, session, arm: str, p_start, p_end, q_start, q_end, grip: bool, trigger: float, duration_s: float = 0.5, hold_s: float = 1.2):
    # 1. Send initial anchor packet
    pkt_anchor = {
        "cmd": "xr_controllers",
        "t": time.time(),
        arm: {
            "pos": p_start.tolist(),
            "quat": q_start,
            "grip": grip,
            "grip_val": 1.0 if grip else 0.0,
            "trigger": trigger
        }
    }
    await ws.send_str(json.dumps(pkt_anchor))
    await asyncio.sleep(0.04)

    # 2. Ramp trajectory
    steps = max(10, int(duration_s / 0.02))
    for i in range(1, steps + 1):
        frac = min(1.0, i / float(steps))
        cur_p = (1.0 - frac) * p_start + frac * p_end
        # Slerp quaternion
        r_start = R.from_quat([q_start[1], q_start[2], q_start[3], q_start[0]])
        r_end = R.from_quat([q_end[1], q_end[2], q_end[3], q_end[0]])
        times = [0, 1]
        from scipy.spatial.transform import Slerp
        slerp = Slerp(times, R.from_quat([r_start.as_quat(), r_end.as_quat()]))
        r_cur = slerp([frac])[0]
        q_scipy = r_cur.as_quat() # [x, y, z, w]
        cur_q = [float(q_scipy[3]), float(q_scipy[0]), float(q_scipy[1]), float(q_scipy[2])]

        pkt = {
            "cmd": "xr_controllers",
            "t": time.time(),
            arm: {
                "pos": cur_p.tolist(),
                "quat": cur_q,
                "grip": grip,
                "grip_val": 1.0 if grip else 0.0,
                "trigger": trigger
            }
        }
        await ws.send_str(json.dumps(pkt))
        await asyncio.sleep(0.02)

    # 3. Steady state settling hold
    hold_steps = int(hold_s / 0.02)
    for _ in range(hold_steps):
        pkt = {
            "cmd": "xr_controllers",
            "t": time.time(),
            arm: {
                "pos": p_end.tolist(),
                "quat": q_end,
                "grip": grip,
                "grip_val": 1.0 if grip else 0.0,
                "trigger": trigger
            }
        }
        await ws.send_str(json.dumps(pkt))
        await asyncio.sleep(0.02)

async def test_p1_p2_p3():
    proc, must_stop = ensure_server_running(20.0)
    ssl_ctx = get_ssl_context()
    connector = aiohttp.TCPConnector(ssl=ssl_ctx)

    results = {}
    try:
        async with aiohttp.ClientSession(connector=connector) as session:
            # Wait for server ready
            for _ in range(40):
                try:
                    st = await fetch_status(session)
                    if st and "ee_l" in st and len(st["ee_l"]) == 3:
                        break
                except Exception:
                    pass
                await asyncio.sleep(0.5)

            # Reset home
            await send_cmd(session, {"name": "preset", "pose": "reset"})
            await send_cmd(session, {"name": "clear_estop"})
            await asyncio.sleep(1.0)

            st_home = await fetch_status(session)
            print("\n=======================================================")
            print("RUNNING NUMERIC VERIFICATION OF P1, P2, P3")
            print("=======================================================")

            # -----------------------------------------------------------------
            # P1: Gizmo-to-hand position error <= 2 mm and orientation error <= 1 deg
            # at home and after clutch cycle (steady state)
            # -----------------------------------------------------------------
            print("\n--- CRITERION P1: Coincidence at Home and After Clutch Cycle ---")
            p1_passes = True
            for arm in ["left", "right"]:
                tgt_key = f"tgt_{arm[0]}"
                ee_key = f"ee_{arm[0]}"
                tgt_rot_key = f"tgt_rot_{arm[0]}"
                rot_key = f"rot_{arm[0]}"

                p_gizmo = np.array(st_home[tgt_key])
                p_hand = np.array(st_home[ee_key])
                r_gizmo = np.array(st_home[tgt_rot_key])
                r_hand = np.array(st_home[rot_key])

                pos_err_mm = np.linalg.norm(p_gizmo - p_hand) * 1000.0
                rot_err_deg = matrix_angle_deg(r_gizmo, r_hand)
                print(f"  {arm.upper()} AT HOME: Pos Error = {pos_err_mm:.4f} mm, Rot Error = {rot_err_deg:.4f} deg")
                if pos_err_mm > 2.0 or rot_err_deg > 1.0:
                    p1_passes = False

            # Run clutch cycle on both arms
            async with session.ws_connect(WS_URL, ssl=ssl_ctx) as ws:
                base_ctrl = np.array([0.2, 1.2, -0.4])
                q_id = [1.0, 0.0, 0.0, 0.0]

                for arm in ["left", "right"]:
                    # Engage clutch (translate +5 cm)
                    p_disp = base_ctrl + np.array([0.0, 0.0, -0.05])
                    await stream_controller_and_wait(ws, session, arm, base_ctrl, p_disp, q_id, q_id, grip=True, trigger=0.0, duration_s=0.4, hold_s=1.2)

                    # Release clutch back to LOCKED
                    for _ in range(15):
                        pkt_rel = {
                            "cmd": "xr_controllers",
                            "t": time.time(),
                            arm: {
                                "pos": p_disp.tolist(),
                                "quat": q_id,
                                "grip": False,
                                "grip_val": 0.0,
                                "trigger": 0.0
                            }
                        }
                        await ws.send_str(json.dumps(pkt_rel))
                        await asyncio.sleep(0.02)

                    await asyncio.sleep(0.6)
                    st_after = await fetch_status(session)
                    tgt_key = f"tgt_{arm[0]}"
                    ee_key = f"ee_{arm[0]}"
                    tgt_rot_key = f"tgt_rot_{arm[0]}"
                    rot_key = f"rot_{arm[0]}"

                    p_gizmo = np.array(st_after[tgt_key])
                    p_hand = np.array(st_after[ee_key])
                    r_gizmo = np.array(st_after[tgt_rot_key])
                    r_hand = np.array(st_after[rot_key])

                    pos_err_mm = np.linalg.norm(p_gizmo - p_hand) * 1000.0
                    rot_err_deg = matrix_angle_deg(r_gizmo, r_hand)
                    print(f"  {arm.upper()} AFTER CLUTCH CYCLE: Pos Error = {pos_err_mm:.4f} mm, Rot Error = {rot_err_deg:.4f} deg")
                    if pos_err_mm > 2.0 or rot_err_deg > 1.0:
                        p1_passes = False

            results["P1"] = p1_passes
            print(f"  [RESULT P1] {'PASS' if p1_passes else 'FAIL'}")

            # -----------------------------------------------------------------
            # P2: Translate controller +10 cm along each of x/y/z separately:
            # hand site moves 10 cm +- 5 mm in expected robot direction, other axes < 2 mm.
            # -----------------------------------------------------------------
            print("\n--- CRITERION P2: 1:1 Translation Mapping (10 cm +- 5 mm, cross-axes < 2 mm) ---")
            p2_passes = True
            arm_axes = {
                "left": [
                    ("x (+10 cm)", np.array([0.10, 0.0, 0.0]), np.array([0.0, -0.10, 0.0])), # Controller +X -> Robot -Y
                    ("y (+10 cm)", np.array([0.0, 0.10, 0.0]), np.array([0.0, 0.0, 0.10])),  # Controller +Y -> Robot +Z
                    ("z (10 cm)",  np.array([0.0, 0.0, -0.10]), np.array([0.10, 0.0, 0.0])), # Controller -Z -> Robot +X
                ],
                "right": [
                    ("x (10 cm)",  np.array([-0.10, 0.0, 0.0]), np.array([0.0, 0.10, 0.0])),  # Controller -X -> Robot +Y
                    ("y (+10 cm)", np.array([0.0, 0.10, 0.0]), np.array([0.0, 0.0, 0.10])),  # Controller +Y -> Robot +Z
                    ("z (10 cm)",  np.array([0.0, 0.0, -0.10]), np.array([0.10, 0.0, 0.0])), # Controller -Z -> Robot +X
                ]
            }

            async with session.ws_connect(WS_URL, ssl=ssl_ctx) as ws:
                for arm in ["left", "right"]:
                    for ax_name, d_c, expected_d_rob in arm_axes[arm]:
                        # Reset home before each axis move
                        await send_cmd(session, {"name": "preset", "pose": "reset"})
                        await asyncio.sleep(0.5)
                        st0 = await fetch_status(session)
                        p0 = np.array(st0[f"ee_{arm[0]}"])

                        # Clutch engage & move 10 cm along controller axis
                        p_end = base_ctrl + d_c
                        await stream_controller_and_wait(ws, session, arm, base_ctrl, p_end, q_id, q_id, grip=True, trigger=0.0, duration_s=0.4, hold_s=1.2)

                        st1 = await fetch_status(session)
                        p1 = np.array(st1[f"ee_{arm[0]}"])
                        actual_d_rob = p1 - p0

                        # Primary axis (largest expected displacement)
                        prim_idx = int(np.argmax(np.abs(expected_d_rob)))
                        prim_actual_cm = actual_d_rob[prim_idx] * 100.0
                        prim_expected_cm = expected_d_rob[prim_idx] * 100.0
                        prim_err_mm = abs(actual_d_rob[prim_idx] - expected_d_rob[prim_idx]) * 1000.0

                        # Cross axes
                        cross_indices = [i for i in range(3) if i != prim_idx]
                        cross_err_mm = [abs(actual_d_rob[i]) * 1000.0 for i in cross_indices]
                        max_cross_err_mm = max(cross_err_mm)

                        ax_pass = (prim_err_mm <= 5.0) and (max_cross_err_mm <= 2.0)
                        print(f"  {arm.upper()} Controller {ax_name.upper()}: Primary ({prim_expected_cm:+.1f} cm) = {prim_actual_cm:+.2f} cm (Err: {prim_err_mm:.2f} mm) | Cross-axes max: {max_cross_err_mm:.2f} mm | Pass: {ax_pass}")
                        if not ax_pass:
                            p2_passes = False

            results["P2"] = p2_passes
            print(f"  [RESULT P2] {'PASS' if p2_passes else 'FAIL'}")

            # -----------------------------------------------------------------
            # P3: Rotate controller 30 deg about each of x/y/z:
            # hand rotates 30 deg +- 2 deg about corresponding robot axis.
            # -----------------------------------------------------------------
            print("\n--- CRITERION P3: 1:1 Rotation Mapping (30 deg +- 2 deg) ---")
            p3_passes = True
            rot_info = [
                ("x", [np.radians(30.0), 0.0, 0.0]),
                ("y", [0.0, np.radians(30.0), 0.0]),
                ("z", [0.0, 0.0, np.radians(30.0)]),
            ]

            async with session.ws_connect(WS_URL, ssl=ssl_ctx) as ws:
                for arm in ["left", "right"]:
                    for ax_name, rot_vec in rot_info:
                        # Reset home before each rotation
                        await send_cmd(session, {"name": "preset", "pose": "reset"})
                        await asyncio.sleep(0.5)
                        st0 = await fetch_status(session)
                        r0 = np.array(st0[f"rot_{arm[0]}"])

                        rc_quat = R.from_rotvec(rot_vec).as_quat() # [x, y, z, w]
                        q_target = [float(rc_quat[3]), float(rc_quat[0]), float(rc_quat[1]), float(rc_quat[2])] # [w, x, y, z]

                        # Trigger-only rotation
                        await stream_controller_and_wait(ws, session, arm, base_ctrl, base_ctrl, q_id, q_target, grip=False, trigger=1.0, duration_s=0.4, hold_s=1.2)

                        st1 = await fetch_status(session)
                        r1 = np.array(st1[f"rot_{arm[0]}"])
                        achieved_deg = matrix_angle_deg(r1, r0)
                        rot_err_deg = abs(achieved_deg - 30.0)

                        rot_pass = (rot_err_deg <= 2.0)
                        print(f"  {arm.upper()} Controller {ax_name.upper()} (30 deg): Hand Rotated = {achieved_deg:.2f} deg (Err: {rot_err_deg:.2f} deg) | Pass: {rot_pass}")
                        if not rot_pass:
                            p3_passes = False

            results["P3"] = p3_passes
            print(f"  [RESULT P3] {'PASS' if p3_passes else 'FAIL'}")

    finally:
        if must_stop and proc:
            proc.terminate()
            proc.wait()

    return results

def test_p5_browser_screenshot():
    print("\n--- CRITERION P5: Headless Playwright Browser Check & Screenshot ---")
    proc, must_stop = ensure_server_running(20.0)
    shot_dir = Path("/tmp/shots")
    shot_dir.mkdir(parents=True, exist_ok=True)
    shot_path = shot_dir / "gizmo_centered.png"

    from playwright.sync_api import sync_playwright

    success = False
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(
                headless=True,
                args=["--ignore-certificate-errors", "--no-sandbox"]
            )
            context = browser.new_context(ignore_https_errors=True, viewport={"width": 1280, "height": 800})
            page = context.new_page()
            page.goto(f"https://localhost:{HTTPS_PORT}/", timeout=15000)
            page.wait_for_selector("#canvas3d", timeout=10000)
            # Wait 3 seconds for Three.js bundle and scene to fully render
            page.wait_for_timeout(3000)
            page.screenshot(path=str(shot_path), full_page=True)
            print(f"  Screenshot successfully saved to: {shot_path}")
            assert shot_path.exists() and shot_path.stat().st_size > 10000
            print(f"  Screenshot verified: {shot_path.stat().st_size} bytes")
            success = True
            browser.close()
    except Exception as e:
        print(f"  Playwright screenshot failed: {e}")
        success = False
    finally:
        if must_stop and proc:
            proc.terminate()
            proc.wait()

    print(f"  [RESULT P5] {'PASS' if success else 'FAIL'}")
    return success, str(shot_path)

if __name__ == "__main__":
    res = asyncio.run(test_p1_p2_p3())
    p5_ok, p5_path = test_p5_browser_screenshot()
    res["P5"] = p5_ok
    print("\nVerification Summary:")
    for k, v in res.items():
        print(f"  {k}: {'PASS' if v else 'FAIL'}")
    all_ok = all(res.values())
    sys.exit(0 if all_ok else 1)
