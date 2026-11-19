"""
teleop/utils/startup_banner.py
Network IP discovery and startup console banner printing.
"""

import socket


def get_local_ip() -> str:
    """Detect the host primary local IP address."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def print_startup_banner(port: int, host_ip: str, status_dict: dict, quest_dict: dict = None):
    """Prints the formatted startup readiness banner with access URLs and subsystem statuses."""
    quest_dict = quest_dict or {}
    adb_str = "Active" if quest_dict.get("adb_installed") else "Not found (Install with: sudo apt install adb)"
    dev_str = quest_dict.get("device") or (f"None ({quest_dict.get('device_state')})" if quest_dict.get("device_state") else "None")
    rev_str = f"ACTIVE (tcp:{port}, tcp:{port + 1})" if quest_dict.get("reverse_active") else "Inactive"
    quest_url = quest_dict.get("quest_url", f"http://localhost:{port + 1}/")

    print("\n" + "=" * 80)
    print("READY - AGIBOT A2 ULTRA TELEOPERATION SYSTEM ACTIVE")
    print("=" * 80)
    print(f"1. PC BROWSER (3D PANEL) : https://localhost:{port}/")
    print(f"2. QUEST WEBXR (USB/ADB) : {quest_url}   (no certificate needed)")
    print(f"   ADB Status            : {adb_str}")
    print(f"   Connected Headset     : {dev_str}")
    print(f"   ADB Reverse           : {rev_str}")
    print(f"3. QUEST 3S (WI-FI / LAN): https://{host_ip}:{port}/")
    print(f"4. HEALTH ENDPOINT       : https://localhost:{port}/health")
    print("-" * 80)
    print("SUBSYSTEM STATUS:")
    print(f"  • Sim Backend : {status_dict.get('sim', 'OK')}")
    print(f"  • Native Viewer: {status_dict.get('viewer', 'OFF')}")
    print(f"  • HTTPS Server: {status_dict.get('server', 'OK')}")
    print(f"  • Web Panel   : {status_dict.get('panel', 'OK')}")
    print(f"  • XR Receiver : {status_dict.get('xr_receiver', 'IDLE (Ready for Quest connection)')}")
    print(f"  • Safety Layer: {status_dict.get('safety', 'OK')}")
    print("=" * 80 + "\n")

