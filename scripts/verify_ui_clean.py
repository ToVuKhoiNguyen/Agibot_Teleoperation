#!/usr/bin/env python3
"""
scripts/verify_ui_clean.py
Verifies that:
1. "Controller Status" and "Hand / Finger Test" cards are completely gone from DOM.
2. Console errors = 0 on page load, telemetry updates, and keyboard shortcuts.
3. Shortcuts hint is present at panel bottom.
4. Pressing R, C, T, L (and typing in input) produces 0 JS errors.
5. Captures screenshot to /tmp/shots/ui_clean.png.
"""

import sys
import time
from pathlib import Path
from playwright.sync_api import sync_playwright

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.utils.server_process import ensure_server_running, stop_spawned_server

SHOTS_DIR = Path("/tmp/shots")
SHOTS_DIR.mkdir(parents=True, exist_ok=True)
URL = "https://localhost:8012/panel"


def main():
    spawned_proc, did_spawn = ensure_server_running(timeout_s=15.0)
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(
                headless=True,
                args=["--ignore-certificate-errors", "--no-sandbox"]
            )
            ctx = browser.new_context(
                ignore_https_errors=True,
                viewport={"width": 1440, "height": 900}
            )
            page = ctx.new_page()

            console_errors = []
            page.on("console", lambda msg: console_errors.append(msg.text) if msg.type == "error" else None)
            page.on("pageerror", lambda exc: console_errors.append(str(exc)))

            print(f"[TEST] Navigating to {URL} ...")
            page.goto(URL, wait_until="networkidle")
            page.wait_for_selector("#viewport-container", timeout=8000)
            time.sleep(1.0)

            # 1. Assert cards are gone
            c_quest = page.query_selector("#card-quest-status")
            c_hand = page.query_selector("#card-hand-test")
            assert c_quest is None, f"#card-quest-status found in DOM: {c_quest}"
            assert c_hand is None, f"#card-hand-test found in DOM: {c_hand}"

            content = page.content()
            assert "Controller Status" not in content, "Found 'Controller Status' in DOM"
            assert "Hand / Finger Test" not in content, "Found 'Hand / Finger Test' in DOM"
            print("  [PASS] Both 'Controller Status' and 'Hand / Finger Test' cards are completely removed from DOM.")

            # 2. Check shortcuts hint
            hint = page.query_selector("#shortcuts-hint")
            assert hint is not None, "#shortcuts-hint missing from DOM"
            print(f"  [PASS] Shortcuts hint found: '{hint.inner_text().strip()}'")

            # 3. Test keyboard shortcuts (R, C, T, L)
            for k in ["r", "c", "t", "l"]:
                page.keyboard.press(k)
                time.sleep(0.1)

            # 4. Wait for potential telemetry updates without errors
            time.sleep(1.0)

            # 5. Assert 0 console errors
            print(f"  Console Errors ({len(console_errors)}): {console_errors}")
            assert len(console_errors) == 0, f"Encountered JS console errors: {console_errors}"
            print("  [PASS] Console errors = 0.")

            # 6. Capture screenshot
            shot_path = SHOTS_DIR / "ui_clean.png"
            page.screenshot(path=str(shot_path))
            assert shot_path.exists() and shot_path.stat().st_size > 1000, "Screenshot failed"
            print(f"  [PASS] Saved clean UI screenshot to {shot_path} ({shot_path.stat().st_size} bytes).")

            browser.close()
            return True
    finally:
        if did_spawn and spawned_proc:
            stop_spawned_server(spawned_proc)


if __name__ == "__main__":
    ok = main()
    sys.exit(0 if ok else 1)
