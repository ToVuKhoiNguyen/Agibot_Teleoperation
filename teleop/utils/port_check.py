"""
teleop/utils/port_check.py
Utility to check port availability, detect holding PID/process, and report clearly.
"""

import socket
import subprocess

def check_port_available(port: int, host: str = "0.0.0.0") -> bool:
    """Checks if a TCP port is available to bind."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((host, port))
            return True
        except OSError:
            return False

def get_process_holding_port(port: int) -> str:
    """Uses ss or lsof to identify which PID/process is holding the port."""
    try:
        out = subprocess.check_output(["ss", "-ltnp"], text=True)
        for line in out.splitlines():
            if f":{port} " in line:
                return line.strip()
    except Exception:
        pass
    try:
        out = subprocess.check_output(["lsof", "-i", f":{port}"], text=True)
        return out.strip()
    except Exception:
        pass
    return f"Port {port} in use by another process."
