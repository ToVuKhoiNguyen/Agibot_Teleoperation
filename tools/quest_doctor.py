#!/usr/bin/env python3
"""
tools/quest_doctor.py
Diagnostics and health verification tool for Meta Quest WebXR connection:
1. Checks adb binary
2. Checks connected headset state via adb devices
3. Checks adb reverse port forwarding status
4. Checks local HTTPS port listening
5. Checks curl -k https://localhost:<port>/vr from PC
6. Checks headset connectivity to reversed port via adb shell
7. Prints actionable troubleshooting guide for any failures
"""

import os
import sys
import shutil
import socket
import ssl
import subprocess
import urllib.request
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.xr.adb_setup import parse_adb_devices_output, select_target_device

PORT = 8012


def check_port_listening(port: int) -> bool:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(0.5)
    try:
        s.connect(("127.0.0.1", port))
        s.close()
        return True
    except Exception:
        return False


def test_https_get(url: str) -> tuple[bool, str]:
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "QuestDoctor/1.0"})
        with urllib.request.urlopen(req, context=ctx, timeout=3.0) as resp:
            content = resp.read().decode("utf-8", errors="ignore")
            return (resp.status == 200), f"HTTP {resp.status} ({len(content)} bytes)"
    except Exception as e:
        return False, str(e)


def main():
    print("=" * 80)
    print("QUEST WEBXR TELEOPERATION DIAGNOSTICS (QUEST DOCTOR)")
    print("=" * 80)

    all_ok = True
    adb_bin = shutil.which("adb")
    target_serial = None

    # 1. ADB Binary Check
    print("[1/6] Checking ADB installation...")
    if adb_bin:
        print(f"  [PASS] ADB found at: {adb_bin}")
    else:
        print("  [FAIL] ADB is not installed or not in PATH.")
        print("         -> Action: Run 'sudo apt update && sudo apt install adb -y'")
        all_ok = False

    # 2. ADB Server & Devices Check
    print("\n[2/6] Checking Meta Quest device connection...")
    if adb_bin:
        try:
            subprocess.run([adb_bin, "start-server"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
            res = subprocess.run([adb_bin, "devices"], capture_output=True, text=True, timeout=5)
            devices = parse_adb_devices_output(res.stdout)
            serial, state = select_target_device(devices)
            target_serial = serial

            if state == "device":
                print(f"  [PASS] Authorized Quest device detected: {serial}")
            elif state == "unauthorized":
                print(f"  [FAIL] Quest device detected ({serial}) but UNAUTHORIZED!")
                print("         -> Action: Put on the headset and click 'Allow USB debugging'")
                all_ok = False
            elif state == "none":
                print("  [WARN] No Quest headset detected via USB.")
                print("         -> Action: Plug USB-C data cable into PC and ensure headset is awake")
            else:
                print(f"  [WARN] Device in state: '{state}'")
        except Exception as e:
            print(f"  [FAIL] Error running adb devices: {e}")
            all_ok = False
    else:
        print("  [SKIP] ADB not available.")

    # 3. ADB Reverse Check
    print(f"\n[3/6] Checking ADB reverse port forwarding (tcp:{PORT} -> tcp:{PORT})...")
    if adb_bin and target_serial:
        try:
            res = subprocess.run([adb_bin, "-s", target_serial, "reverse", "--list"], capture_output=True, text=True, timeout=5)
            if f"tcp:{PORT}" in res.stdout:
                print(f"  [PASS] Reverse port active on {target_serial}: {res.stdout.strip()}")
            else:
                print(f"  [WARN] Reverse port not currently set on {target_serial}.")
                print(f"         -> Action: Run 'adb -s {target_serial} reverse tcp:{PORT} tcp:{PORT}' or run main.py")
        except Exception as e:
            print(f"  [FAIL] Could not query reverse list: {e}")
    else:
        print("  [SKIP] No authorized device available to verify adb reverse.")

    # 4. Port Listening Check
    print(f"\n[4/6] Checking local port {PORT} listener...")
    port_open = check_port_listening(PORT)
    if port_open:
        print(f"  [PASS] Server is listening on port {PORT}")
    else:
        print(f"  [WARN] Port {PORT} is closed. (main.py is not currently running)")
        print("         -> Action: Start system via 'python main.py'")

    # 5. HTTPS PC Localhost Check
    print(f"\n[5/6] Checking PC access to https://localhost:{PORT}/vr ...")
    if port_open:
        ok, msg = test_https_get(f"https://localhost:{PORT}/vr")
        if ok:
            print(f"  [PASS] Successfully fetched /vr: {msg}")
        else:
            print(f"  [FAIL] Failed to fetch /vr: {msg}")
            all_ok = False
    else:
        print(f"  [SKIP] Port {PORT} is closed.")

    # 6. Headset TCP probe Check
    print(f"\n[6/6] Checking headset connectivity to reversed port tcp:{PORT}...")
    if adb_bin and target_serial and port_open:
        try:
            nc_check = subprocess.run(
                [adb_bin, "-s", target_serial, "shell", f"toybox nc -w 1 127.0.0.1 {PORT} </dev/null"],
                capture_output=True, text=True, timeout=3
            )
            if nc_check.returncode == 0:
                print(f"  [PASS] Headset successfully connected to localhost:{PORT} through reverse tunnel!")
            else:
                # If reverse port is active, connection succeeds; if not, connection refused
                print(f"  [WARN] Headset probe to localhost:{PORT} returned code {nc_check.returncode}")
                if "refused" in nc_check.stdout.lower() or "refused" in nc_check.stderr.lower():
                    print("         -> Reverse port is not forwarding or port is refused by headset.")
                else:
                    print(f"         Output: {nc_check.stdout} {nc_check.stderr}")
        except Exception as e:
            print(f"  [WARN] Headset probe skipped / error: {e}")
    else:
        print("  [SKIP] Requires authorized headset and running server.")

    print("\n" + "=" * 80)
    if all_ok:
        print("DIAGNOSTICS RESULT: ALL SYSTEMS READY FOR QUEST TELEOPERATION")
    else:
        print("DIAGNOSTICS RESULT: ISSUES DETECTED - PLEASE SEE ACTIONS ABOVE")
    print("=" * 80)


if __name__ == "__main__":
    main()
