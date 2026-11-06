#!/usr/bin/env python3
"""
scripts/test_xr_controllers.py
Automated acceptance test for Quest 3S WebXR Controller Teleoperation:
  Tests 1-5: p1 (trigger, grip, 6D, transitions, thumbstick velocity closure)
  Tests 6-10: p2 (panic open, A-home, precision mode, E-STOP, watchdog)
"""

import os
import sys
import ssl
import asyncio
import aiohttp
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from teleop.utils.server_process import ensure_server_running, stop_spawned_server
from scripts.test_xr_controllers_p1 import run_xr_tests_part1
from scripts.test_xr_controllers_p2 import run_xr_tests_part2

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
            home_l = np.array(st_home.get("tgt_l", st_home["ee_l"]))
            home_r = np.array(st_home.get("tgt_r", st_home["ee_r"]))
            print(f"Home Left Target : [{home_l[0]:+.3f}, {home_l[1]:+.3f}, {home_l[2]:+.3f}] m")
            print(f"Home Right Target: [{home_r[0]:+.3f}, {home_r[1]:+.3f}, {home_r[2]:+.3f}] m")

            print(f"[INFO] Connecting to {WS_URL} ...")
            async with session.ws_connect(WS_URL, ssl=ssl_ctx) as ws:
                print("[PASS] WebSocket connected successfully.")

                base_ctrl_pos = np.array([0.2, 1.2, -0.4])
                quat_identity = [1.0, 0.0, 0.0, 0.0]

                # Run Tests 1 to 5
                await run_xr_tests_part1(
                    session, ws, base_ctrl_pos, quat_identity, home_l, fetch_status
                )

                # Run Tests 6 to 10
                await run_xr_tests_part2(
                    session, ws, base_ctrl_pos, quat_identity, home_r, fetch_status, send_rest_cmd
                )

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
