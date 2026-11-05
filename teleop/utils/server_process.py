"""
teleop/utils/server_process.py
Helper to auto-launch and gracefully stop main.py for test suites.
"""

import os
import sys
import time
import ssl
import subprocess
import urllib.request
from pathlib import Path

HEALTH_URL = "https://localhost:8012/health"

def get_ssl_context():
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    return ctx

def is_server_healthy() -> bool:
    try:
        req = urllib.request.Request(HEALTH_URL)
        with urllib.request.urlopen(req, context=get_ssl_context(), timeout=1.0) as resp:
            return resp.status == 200
    except Exception:
        return False

def ensure_server_running(timeout_s: float = 20.0):
    """
    Checks if main.py is running. If not, spawns it as a subprocess and waits for /health.
    Returns (process_handle_or_None, did_spawn_bool).
    """
    if is_server_healthy():
        return None, False

    print(f"[INFO] Port 8012 is closed. Spawning main.py server (waiting up to {timeout_s}s for /health)...")
    project_root = Path(__file__).resolve().parent.parent.parent
    main_script = project_root / "main.py"
    python_bin = sys.executable

    log_file = open("/tmp/main_server.log", "w", buffering=1)
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    proc = subprocess.Popen(
        [python_bin, "-u", str(main_script), "--headless", "--no-adb"],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        cwd=str(project_root)
    )

    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if is_server_healthy():
            print("[INFO] Server is healthy and ready.")
            return proc, True
        time.sleep(0.5)

    proc.terminate()
    raise TimeoutError(f"Server did not become healthy within {timeout_s} seconds.")

def stop_spawned_server(proc):
    if proc is not None:
        print("[INFO] Terminating auto-spawned test server...")
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except Exception:
            proc.kill()
        print("[INFO] Server stopped.")
