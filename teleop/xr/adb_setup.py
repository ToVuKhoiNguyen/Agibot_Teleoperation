"""
teleop/xr/adb_setup.py
Automatic ADB management, port reverse, and Quest Browser launcher for Meta Quest.
Handles auto-detection, unauthorized state, reverse verification, background watching,
and /vr page launch.
"""

import os
import shutil
import subprocess
import threading
import time
from typing import Dict, List, Optional, Tuple


def parse_adb_devices_output(output: str) -> List[Tuple[str, str]]:
    """
    Parses `adb devices` text output into a list of (serial, state) tuples.
    Possible states: 'device', 'unauthorized', 'offline', etc.
    """
    devices = []
    lines = output.strip().splitlines()
    for line in lines:
        line = line.strip()
        if not line or line.startswith("List of devices") or line.startswith("*"):
            continue
        parts = line.split()
        if len(parts) >= 2:
            devices.append((parts[0], parts[1]))
    return devices


def select_target_device(devices: List[Tuple[str, str]]) -> Tuple[Optional[str], str]:
    """
    Selects the active device from parsed devices.
    Returns (device_id, status_message).
    """
    if not devices:
        return None, "none"

    # Prioritize the first connected authorized device
    for serial, state in devices:
        if state == "device":
            return serial, "device"

    # If no authorized device, check for unauthorized
    for serial, state in devices:
        if state == "unauthorized":
            return serial, "unauthorized"

    return devices[0][0], devices[0][1]


class QuestAdbManager:
    """
    Manages ADB connection, reverse port forwarding, and background monitoring for Meta Quest.
    """

    def __init__(self, port: int = 8012, no_adb: bool = False, no_launch: bool = False):
        self.port = port
        self.no_adb = no_adb
        self.no_launch = no_launch

        self.adb_bin = shutil.which("adb") if not self.no_adb else None
        self.device_id: Optional[str] = None
        self.device_state: str = "none" if not self.adb_bin else "disconnected"
        self.reverse_active: bool = False
        self.has_launched_once: bool = False
        self.last_status_str: str = ""

        self.running: bool = False
        self.thread: Optional[threading.Thread] = None

    def start_server_if_needed(self):
        if not self.adb_bin:
            return
        try:
            subprocess.run([self.adb_bin, "start-server"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
        except Exception:
            pass

    def check_devices(self) -> Tuple[Optional[str], str]:
        if not self.adb_bin:
            return None, "no_adb"
        try:
            res = subprocess.run([self.adb_bin, "devices"], capture_output=True, text=True, timeout=5)
            devices = parse_adb_devices_output(res.stdout)
            serial, state = select_target_device(devices)
            return serial, state
        except Exception as e:
            return None, f"error: {e}"

    def check_reverse_list(self, serial: str) -> bool:
        if not self.adb_bin or not serial:
            return False
        try:
            res = subprocess.run([self.adb_bin, "-s", serial, "reverse", "--list"], capture_output=True, text=True, timeout=5)
            # Example output: (reverse) tcp:8012 tcp:8012
            expected = f"tcp:{self.port}"
            return expected in res.stdout
        except Exception:
            return False

    def setup_reverse(self, serial: str) -> bool:
        if not self.adb_bin or not serial:
            return False
        try:
            cmd = [self.adb_bin, "-s", serial, "reverse", f"tcp:{self.port}", f"tcp:{self.port}"]
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
            return self.check_reverse_list(serial)
        except Exception as e:
            print(f"[WARN] adb reverse failed for {serial}: {e}")
            return False

    def launch_quest_browser(self, serial: Optional[str] = None, force: bool = False) -> bool:
        """
        Launches the /vr page on Quest Browser using com.oculus.browser.
        """
        if not self.adb_bin:
            return False
        target_serial = serial or self.device_id
        if not target_serial or self.device_state != "device":
            return False
        if not force and self.no_launch:
            return False

        vr_url = f"https://localhost:{self.port}/vr"
        cmd = [
            self.adb_bin, "-s", target_serial, "shell", "am", "start",
            "-a", "android.intent.action.VIEW",
            "-d", vr_url,
            "com.oculus.browser"
        ]
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
            self.has_launched_once = True
            print(f"[INFO] Launched Quest Browser ({vr_url}) on {target_serial}")
            return True
        except Exception as e:
            print(f"[WARN] Failed to launch Quest Browser on {target_serial}: {e}")
            return False

    def update_once(self):
        """
        Checks adb devices, handles states, re-applies reverse if needed, and launches on first connect.
        """
        if not self.adb_bin:
            self.device_state = "adb_not_found"
            return

        serial, state = self.check_devices()
        status_changed = (serial != self.device_id) or (state != self.device_state)

        if state == "unauthorized":
            if status_changed:
                print(f"[WARN] Quest ({serial}) is UNAUTHORIZED: Put on the headset and tap Allow USB debugging")
            self.device_id = serial
            self.device_state = "unauthorized"
            self.reverse_active = False
            return

        if state != "device":
            if status_changed and self.device_id:
                print(f"[INFO] Quest disconnected / state changed: {state}")
            self.device_id = None
            self.device_state = state
            self.reverse_active = False
            return

        # Device is authorized and present
        self.device_id = serial
        self.device_state = "device"

        # Check if reverse is currently active
        is_reversed = self.check_reverse_list(serial)
        if not is_reversed:
            is_reversed = self.setup_reverse(serial)
            if is_reversed:
                print(f"[INFO] ADB reverse applied: tcp:{self.port} -> tcp:{self.port} on {serial}")
            else:
                print(f"[WARN] Failed to establish ADB reverse on {serial}")

        self.reverse_active = is_reversed

        if status_changed:
            print(f"[INFO] Quest connected: {serial} (reverse: {'active' if self.reverse_active else 'failed'})")
            if not self.has_launched_once and not self.no_launch:
                self.launch_quest_browser(serial)

    def start_watcher(self):
        if self.no_adb or not self.adb_bin:
            return
        self.start_server_if_needed()
        self.running = True
        self.update_once()

        def _loop():
            while self.running:
                time.sleep(3.0)
                try:
                    self.update_once()
                except Exception:
                    pass

        self.thread = threading.Thread(target=_loop, daemon=True, name="QuestAdbWatcher")
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def get_status(self) -> Dict:
        return {
            "adb_installed": bool(self.adb_bin),
            "device": self.device_id,
            "device_state": self.device_state,
            "reverse_active": bool(self.reverse_active),
            "quest_url": f"https://localhost:{self.port}/vr",
            "has_launched": bool(self.has_launched_once)
        }

