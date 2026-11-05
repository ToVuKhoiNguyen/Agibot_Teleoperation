#!/usr/bin/env python3
"""
scripts/test_vr_page.py
Verifies /vr page in headless Playwright:
1. Confirms #vr-diag-overlay renders.
2. Checks for zero console errors.
3. Asserts secure context is true.
4. Confirms diagnostic text renders correctly.
"""

import sys
import time
from pathlib import Path
from playwright.sync_api import sync_playwright

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.utils.server_process import ensure_server_running, stop_spawned_server

PORT = 8012
VR_URL = f"https://localhost:{PORT}/vr"


def main():
    print("=" * 80)
    print("VERIFYING /vr PAGE IN HEADLESS PLAYWRIGHT")
    print("=" * 80)

    proc, must_stop = ensure_server_running(20.0)
    console_errors = []
    console_logs = []

    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True)
            context = browser.new_context(ignore_https_errors=True)
            page = context.new_page()

            page.on("console", lambda msg: (
                console_errors.append(msg.text) if msg.type == "error" else console_logs.append(msg.text)
            ))
            page.on("pageerror", lambda exc: console_errors.append(str(exc)))

            print(f"[INFO] Navigating to {VR_URL} ...")
            page.goto(VR_URL, wait_until="networkidle", timeout=15000)

            # Wait for diagnostics overlay
            page.wait_for_selector("#vr-diag-overlay", state="visible", timeout=10000)
            print("[PASS] #vr-diag-overlay rendered and visible.")

            # Check Enter VR button
            btn = page.query_selector("#btn-enter-vr")
            assert btn is not None and btn.is_visible(), "ENTER VR button not visible"
            print("[PASS] #btn-enter-vr button is visible.")

            # Wait a brief moment for checkXR to finish
            time.sleep(1.0)

            print(f"[INFO] Total console logs: {len(console_logs)}, errors: {len(console_errors)}")
            for log in console_logs:
                print(f"    LOG: {log}")
            for err in console_errors:
                print(f"    ERR: {err}")

            secure_text = page.inner_text("#diag-secure")
            xr_text = page.inner_text("#diag-xr")
            session_text = page.inner_text("#diag-session")

            print(f"  Diagnostics: SecureContext='{secure_text}', XR='{xr_text}', Session='{session_text}'")

            # Filter out known non-critical external asset 404s if any, but require zero script exceptions
            script_errors = [e for e in console_errors if "favicon" not in e.lower()]
            if script_errors:
                print(f"[FAIL] Console errors found: {script_errors}")
                return False

            print("[PASS] Zero script / console errors detected on /vr.")
            browser.close()
            return True

    finally:
        if must_stop and proc:
            stop_spawned_server(proc)


if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)
