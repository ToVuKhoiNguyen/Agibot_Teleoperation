#!/usr/bin/env python3
"""
scripts/capture_screenshots.py
Captures final verification screenshots for release:
  - /tmp/shots/final_full.png  (full web panel with 3D viewport, stats overlay, cards)
  - /tmp/shots/final_gizmo.png (focused on 3D gizmos and robot end-effectors)
  - /tmp/shots/final_fault.png (latched hardware fault safety banner)
"""

import os
import sys
import time
import subprocess
from pathlib import Path
from playwright.sync_api import sync_playwright

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def main():
    port = 8012
    vpy = str(PROJECT_ROOT / "venv" / "bin" / "python")
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "77"
    env["ROS_LOCALHOST_ONLY"] = "1"

    shots_dir = Path("/tmp/shots")
    shots_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Launching server on port {port}...")
    proc = subprocess.Popen([vpy, "main.py", "--headless", "--port", str(port)], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    try:
        # Wait for health
        time.sleep(2.5)

        # Check curl / and /health
        curl_root = subprocess.run(["curl", "-k", "-I", f"https://localhost:{port}/"], capture_output=True, text=True)
        assert "200 OK" in curl_root.stdout or "HTTP/1.1 200" in curl_root.stdout, f"Root returned: {curl_root.stdout}"
        print("  [PASS] curl -k -I https://localhost:8012/ -> 200 OK")

        curl_health = subprocess.run(["curl", "-k", "-I", f"https://localhost:{port}/health"], capture_output=True, text=True)
        assert "200 OK" in curl_health.stdout or "HTTP/1.1 200" in curl_health.stdout, f"Health returned: {curl_health.stdout}"
        print("  [PASS] curl -k -I https://localhost:8012/health -> 200 OK")

        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True)
            ctx = browser.new_context(ignore_https_errors=True)
            page = ctx.new_page()
            page.set_viewport_size({"width": 1440, "height": 900})

            page.goto(f"https://127.0.0.1:{port}/", timeout=12000)
            page.wait_for_selector("#viewport-container", timeout=8000)
            time.sleep(1.5)

            # 1. Final Full View
            full_path = shots_dir / "final_full.png"
            page.screenshot(path=str(full_path))
            print(f"  [PASS] Saved final full view to {full_path} ({full_path.stat().st_size} bytes)")

            # 2. Final Gizmo View (clip or focus on 3D viewport)
            gizmo_path = shots_dir / "final_gizmo.png"
            viewport_el = page.locator("#viewport-container")
            viewport_el.screenshot(path=str(gizmo_path))
            print(f"  [PASS] Saved final gizmo view to {gizmo_path} ({gizmo_path.stat().st_size} bytes)")

            # 3. Final Fault View (with Hardware Fault Banner)
            fault_path = shots_dir / "final_fault.png"
            page.evaluate("""() => {
                window.forceFaultBanner = true;
                window.forceFaultText = '⚠️ HARDWARE FAULT: Position deviation >0.05 rad (>200ms) on joint 3';
            }""")
            time.sleep(0.5)
            page.screenshot(path=str(fault_path))
            print(f"  [PASS] Saved final fault banner view to {fault_path} ({fault_path.stat().st_size} bytes)")

            browser.close()

        print("[INFO] All 3 screenshots captured successfully.")
    finally:
        proc.terminate()
        proc.communicate(timeout=3)


if __name__ == "__main__":
    main()

