"""
tests/test_nudge_playwright.py
Playwright browser DOM tests verifying +-x/y/z nudge buttons move gizmo,
unfreeze TARGET_FROZEN, chain from client pose, and settle with std 0.
Scenarios:
1. Fresh start: 5x +X moves gizmo 5*step, target == gizmo, arm follows, std 0.
2. After drag+release (TARGET_FROZEN): 5x +X unfreezes and moves gizmo.
3. After Home: 5x +X moves gizmo from home pose.
"""

import os
import sys
import ssl
import time
import subprocess
import numpy as np
from playwright.sync_api import sync_playwright

PORT = 8016
BASE_URL = f"https://127.0.0.1:{PORT}"


def wait_for_server(url: str, timeout: float = 15.0) -> bool:
    import urllib.request
    ctx = ssl._create_unverified_context()
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with urllib.request.urlopen(f"{url}/health", context=ctx, timeout=1.0) as resp:
                if resp.status == 200:
                    return True
        except Exception:
            time.sleep(0.3)
    return False


def run_playwright_nudge_tests():
    env = os.environ.copy()
    env["PYTHONPATH"] = "."
    env["ROS_DOMAIN_ID"] = "77"
    env["ROS_LOCALHOST_ONLY"] = "1"

    cmd = [
        sys.executable, "main.py", "--sim", "--headless",
        "--no-adb", "--no-quest-launch", "--port", str(PORT), "--speed", "3"
    ]
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    results = []
    try:
        assert wait_for_server(BASE_URL, timeout=20.0), f"Server failed to start on port {PORT}"
        print(f"[OK] Server listening at {BASE_URL}")

        with sync_playwright() as p:
            browser = p.chromium.launch(headless=True, args=["--ignore-certificate-errors"])
            ctx = browser.new_context(ignore_https_errors=True)
            page = ctx.new_page()
            page.goto(BASE_URL)
            page.wait_for_load_state("networkidle")
            time.sleep(2.0)

            # Wait for 3D scene & gizmo to initialize
            page.wait_for_function("() => typeof targetSphereL !== 'undefined' && targetSphereL !== null")
            btn_selector = 'button[data-cmd*=\'"axis":"x"\'][data-cmd*=\'"arm":"left"\'][data-cmd*=\'"dir":1\']'
            page.wait_for_selector(btn_selector)

            # -------------------------------------------------------------
            # SCENARIO 1: Fresh start -> click +X 5x
            # -------------------------------------------------------------
            pos0 = page.evaluate("() => [targetSphereL.position.x, targetSphereL.position.y, targetSphereL.position.z]")
            plan0 = page.evaluate("() => window.lastTelemetryState ? window.lastTelemetryState.q_plan_l : null")

            step_cm = page.evaluate("() => parseFloat(document.getElementById('step_xyz_l').value)")
            expected_step_m = (step_cm / 100.0)

            for click_idx in range(5):
                page.click(btn_selector)
                time.sleep(0.15)

            time.sleep(0.5)
            pos1 = page.evaluate("() => [targetSphereL.position.x, targetSphereL.position.y, targetSphereL.position.z]")
            delta_x = pos1[0] - pos0[0]
            expected_total_m = 5 * expected_step_m

            assert np.isclose(delta_x, expected_total_m, atol=0.005), (
                f"Fresh start: gizmo moved {delta_x:.4f}m, expected {expected_total_m:.4f}m"
            )

            # Check server target equals client gizmo pos
            tgt_l = page.evaluate("() => window.lastTelemetryState ? window.lastTelemetryState.tgt_l : null")
            assert tgt_l is not None and np.isclose(tgt_l[0], pos1[0], atol=0.005), (
                f"Server target ({tgt_l}) != client gizmo ({pos1})"
            )

            # Check arm followed (PLAN / state changed)
            time.sleep(0.8)
            plan1 = page.evaluate("() => window.lastTelemetryState ? window.lastTelemetryState.q_plan_l : null")
            assert plan1 is not None and not np.allclose(plan0, plan1, atol=1e-3), "Arm did not follow nudge"

            # Wait for arm to complete arrival and settle (~2s for 0.10m travel)
            time.sleep(2.0)

            # Check no stray motion (std 0 over 3 s)
            samples = []
            for _ in range(15):
                q = page.evaluate("() => window.lastTelemetryState ? window.lastTelemetryState.q_plan_l : null")
                if q: samples.append(q)
                time.sleep(0.2)
            std_tail = float(np.max(np.std(np.array(samples), axis=0)))
            assert std_tail < 0.0005, f"Stray motion after settling: std={std_tail:.6f}"
            results.append(("1. Fresh start 5x +X", f"dx={delta_x*100:.1f}cm, std={std_tail:.1e}", "PASS"))

            # -------------------------------------------------------------
            # SCENARIO 2: After drag+release (TARGET_FROZEN) -> click +X 5x
            # -------------------------------------------------------------
            # Freeze target
            page.evaluate("() => sendCmd({cmd: 'gizmo_end', arm: 'left'})")
            time.sleep(0.5)
            warn = page.evaluate("() => window.lastTelemetryState ? window.lastTelemetryState.warning : ''")
            assert warn == "TARGET_FROZEN", f"Expected TARGET_FROZEN, got {warn}"

            pos_before_freeze_nudge = page.evaluate("() => targetSphereL.position.x")
            for _ in range(5):
                page.click(btn_selector)
                time.sleep(0.15)

            time.sleep(0.5)
            warn_after = page.evaluate("() => window.lastTelemetryState ? window.lastTelemetryState.warning : ''")
            assert warn_after != "TARGET_FROZEN", f"TARGET_FROZEN was not unfrozen by nudge: {warn_after}"

            pos_after_freeze_nudge = page.evaluate("() => targetSphereL.position.x")
            delta_freeze = pos_after_freeze_nudge - pos_before_freeze_nudge
            assert np.isclose(delta_freeze, expected_total_m, atol=0.005), (
                f"Frozen nudge: gizmo moved {delta_freeze:.4f}m, expected {expected_total_m:.4f}m"
            )
            results.append(("2. Frozen release 5x +X", f"Unfroze ({warn_after or 'OK'}), dx={delta_freeze*100:.1f}cm", "PASS"))

            # -------------------------------------------------------------
            # SCENARIO 3: After Home -> click +X 5x
            # -------------------------------------------------------------
            page.evaluate("() => sendCmd({cmd: 'preset', pose: 'home'})")
            time.sleep(2.5)  # Wait for homing

            pos_home = page.evaluate("() => targetSphereL.position.x")
            for _ in range(5):
                page.click(btn_selector)
                time.sleep(0.15)

            time.sleep(0.5)
            pos_after_home_nudge = page.evaluate("() => targetSphereL.position.x")
            delta_home = pos_after_home_nudge - pos_home
            assert np.isclose(delta_home, expected_total_m, atol=0.005), (
                f"Home nudge: gizmo moved {delta_home:.4f}m, expected {expected_total_m:.4f}m"
            )
            results.append(("3. After Home 5x +X", f"dx={delta_home*100:.1f}cm from home", "PASS"))

            ctx.close()
            browser.close()

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except Exception:
            proc.kill()

    print("\n" + "=" * 70)
    print(f"{'Scenario':<28} | {'Measurement':<28} | {'Status'}")
    print("-" * 70)
    for name, metric, st in results:
        print(f"{name:<28} | {metric:<28} | {st}")
    print("=" * 70)
    return True


if __name__ == "__main__":
    ok = run_playwright_nudge_tests()
    sys.exit(0 if ok else 1)
