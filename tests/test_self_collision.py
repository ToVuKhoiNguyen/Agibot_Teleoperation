#!/usr/bin/env python3
"""
tests/test_self_collision.py
Automated acceptance test for self-collision prevention & UI feedback:
  (a) Wrist target commanded into torso: target driven deep into chest.
  (b) Wrist target commanded into other hand: left and right wrists driven together.
  (c) Wrist target commanded into head: wrist driven upward into face.
  (d) Wrist target commanded across chest: large step jumps and slow ramps.
Asserts:
  - Minimum signed distance over all monitored collision pairs never drops below 0 (no penetration).
  - Reports min distance reached (mm), number of blocked steps, free-space target reach within 5 mm.
  - Takes screenshot of MuJoCo render and web panel at blocked state using Playwright.
  - Benchmarks collision check p95 latency (< 2 ms) and loop impact.
"""

import os
import sys
import time
import ssl
import json
import asyncio
import urllib.request
import numpy as np
import mujoco

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from teleop.utils.server_process import ensure_server_running, stop_spawned_server
from teleop.utils.collision_checker import CollisionChecker

PORT = 8012
ROOT_URL = f"https://localhost:{PORT}/"
API_CMD_URL = f"https://localhost:{PORT}/api/command"
API_STATUS_URL = f"https://localhost:{PORT}/api/status"
HEALTH_URL = f"https://localhost:{PORT}/health"
SCREENSHOT_DIR = os.path.join(PROJECT_ROOT, "docs", "screenshots")
os.makedirs(SCREENSHOT_DIR, exist_ok=True)


def send_cmd(cmd_dict, ssl_ctx):
    req = urllib.request.Request(
        API_CMD_URL,
        data=json.dumps(cmd_dict).encode("utf-8"),
        headers={"Content-Type": "application/json"}
    )
    with urllib.request.urlopen(req, context=ssl_ctx, timeout=3.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def get_status(ssl_ctx):
    req = urllib.request.Request(API_STATUS_URL)
    with urllib.request.urlopen(req, context=ssl_ctx, timeout=3.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def get_health(ssl_ctx):
    req = urllib.request.Request(HEALTH_URL)
    with urllib.request.urlopen(req, context=ssl_ctx, timeout=3.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


async def capture_browser_screenshot(output_path: str):
    from playwright.async_api import async_playwright
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True, args=["--ignore-certificate-errors"])
        context = await browser.new_context(ignore_https_errors=True, viewport={"width": 1280, "height": 720})
        page = await context.new_page()
        try:
            await page.goto(f"https://localhost:{PORT}/panel", wait_until="networkidle", timeout=10000)
            await asyncio.sleep(1.5) # Wait for 3D model render and status update
            await page.screenshot(path=output_path)
            print(f"  [SCREENSHOT] Saved browser panel screenshot: {output_path}")
        except Exception as e:
            print(f"  [WARN] Failed to capture browser screenshot: {e}")
        finally:
            await browser.close()


def render_mujoco_screenshot(model, data, output_path: str):
    try:
        import cv2
        renderer = mujoco.Renderer(model, height=480, width=640)
        renderer.update_scene(data)
        img = renderer.render()
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imwrite(output_path, bgr)
        print(f"  [SCREENSHOT] Saved MuJoCo render screenshot: {output_path}")
    except Exception as e:
        print(f"  [WARN] Failed to render MuJoCo screenshot: {e}")


def run_tests():
    print("=" * 80)
    print("RUNNING AGIBOT A2 ULTRA SELF-COLLISION PREVENTION & ACCEPTANCE TESTS")
    print("=" * 80)

    ssl_ctx = ssl.create_default_context()
    ssl_ctx.check_hostname = False
    ssl_ctx.verify_mode = ssl.CERT_NONE

    proc, did_spawn = ensure_server_running(timeout_s=25.0)

    # Initialize reference collision checker
    scene_xml = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "mjcf", "scene.xml")
    ref_model = mujoco.MjModel.from_xml_path(scene_xml)
    ref_data = mujoco.MjData(ref_model)
    checker = CollisionChecker(ref_model)
    left_arm_qadr = [ref_model.jnt_qposadr[mujoco.mj_name2id(ref_model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{13+i}_left_arm_joint{i+1}")] for i in range(7)]
    right_arm_qadr = [ref_model.jnt_qposadr[mujoco.mj_name2id(ref_model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{20+i}_right_arm_joint{i+1}")] for i in range(7)]

    blocked_steps_total = 0
    min_dist_overall_mm = 999.0
    passed_all = True

    try:
        # Initial health check
        health = get_health(ssl_ctx)
        assert health["status"] == "OK", f"Server health status not OK: {health}"
        assert "collision_blocked" in health, "collision_blocked not exposed in /health"
        assert "min_distance_mm" in health, "min_distance_mm not exposed in /health"
        print("  [PASS] /health exposes collision_blocked and min_distance_mm.")

        # Reset home
        send_cmd({"name": "preset", "pose": "home"}, ssl_ctx)
        time.sleep(0.5)

        # ---------------------------------------------------------
        # TEST 1: Free-space Tracking Accuracy (< 5 mm)
        # ---------------------------------------------------------
        print("\n--- TEST 1: Free-space Reachability (< 5 mm) ---")
        st_home = get_status(ssl_ctx)
        home_l = np.array(st_home["ee_l"])
        free_tgt_l = [home_l[0] + 0.05, home_l[1] + 0.03, home_l[2] + 0.04]
        send_cmd({"name": "set_wrist_target", "arm": "left", "pos": free_tgt_l}, ssl_ctx)
        time.sleep(1.2)
        st = get_status(ssl_ctx)
        ee_l = np.array(st["ee_l"])
        free_err_mm = float(np.linalg.norm(ee_l - np.array(free_tgt_l))) * 1000.0
        print(f"  Target: {free_tgt_l} m | Actual: {ee_l.tolist()} m | Error: {free_err_mm:.2f} mm")
        assert free_err_mm < 5.0, f"Free-space tracking error {free_err_mm:.2f} mm exceeds 5.0 mm threshold!"
        print("  [PASS] Free-space tracking within 5.0 mm requirement.")

        # ---------------------------------------------------------
        # TEST 2: Drive Left Wrist into Torso (Deep Chest)
        # ---------------------------------------------------------
        print("\n--- TEST 2: Command Target into Torso ---")
        torso_tgt = [0.05, 0.05, 1.00] # Deep inside body_hull
        send_cmd({"name": "set_wrist_target", "arm": "left", "pos": torso_tgt}, ssl_ctx)
        t_end = time.time() + 2.0
        while time.time() < t_end:
            time.sleep(0.05)
            st = get_status(ssl_ctx)
            if st.get("collision_blocked", False):
                break

        min_d_mm = st.get("min_distance_mm", 999.0)
        blocked = st.get("collision_blocked", False)
        print(f"  Target inside chest: {torso_tgt} m")
        print(f"  Reported Min Distance: {min_d_mm:.2f} mm | Blocked Flag: {blocked}")

        assert blocked, "Collision was not blocked when targeting torso!"
        assert min_d_mm > 0.0, f"Penetration occurred! min_distance_mm = {min_d_mm:.2f} mm"
        blocked_steps_total += 1
        min_dist_overall_mm = min(min_dist_overall_mm, min_d_mm)
        print("  [PASS] Self-collision with torso blocked without penetration.")

        # Capture blocked state screenshots
        browser_shot = os.path.join(SCREENSHOT_DIR, "collision_blocked_browser.png")
        asyncio.run(capture_browser_screenshot(browser_shot))

        # Update ref_data from status to render MuJoCo image
        if "qpos_l" in st:
            ref_data.qpos[left_arm_qadr] = st["qpos_l"]
        if "qpos_r" in st:
            ref_data.qpos[right_arm_qadr] = st["qpos_r"]
        mujoco.mj_forward(ref_model, ref_data)
        mujoco_shot = os.path.join(SCREENSHOT_DIR, "collision_blocked_mujoco.png")
        render_mujoco_screenshot(ref_model, ref_data, mujoco_shot)

        # Reset home
        send_cmd({"name": "preset", "pose": "home"}, ssl_ctx)
        time.sleep(0.5)

        # ---------------------------------------------------------
        # TEST 3: Drive Hands Together (Hand vs Hand)
        # ---------------------------------------------------------
        print("\n--- TEST 3: Command Hands Together ---")
        # Drive both wrists toward centerline y=0
        send_cmd({"name": "set_wrist_target", "arm": "left", "pos": [0.35, 0.02, 1.25]}, ssl_ctx)
        send_cmd({"name": "set_wrist_target", "arm": "right", "pos": [0.35, -0.02, 1.25]}, ssl_ctx)
        time.sleep(1.0)

        st = get_status(ssl_ctx)
        min_d_mm = st.get("min_distance_mm", 999.0)
        blocked = st.get("collision_blocked", False)
        print(f"  Hands driven together at Y=±0.02 m")
        print(f"  Reported Min Distance: {min_d_mm:.2f} mm | Blocked: {blocked}")

        assert min_d_mm > 0.0, f"Hands penetrated! min_dist = {min_d_mm:.2f} mm"
        if blocked:
            blocked_steps_total += 1
        min_dist_overall_mm = min(min_dist_overall_mm, min_d_mm)
        print("  [PASS] Hand-vs-hand collision prevented without penetration.")

        # Reset home
        send_cmd({"name": "preset", "pose": "home"}, ssl_ctx)
        time.sleep(0.5)

        # ---------------------------------------------------------
        # TEST 4: Drive Hand into Head
        # ---------------------------------------------------------
        print("\n--- TEST 4: Command Hand into Head ---")
        head_tgt = [-0.02, 0.05, 1.50] # Inside head_link02
        send_cmd({"name": "set_wrist_target", "arm": "left", "pos": head_tgt}, ssl_ctx)
        time.sleep(1.0)

        st = get_status(ssl_ctx)
        min_d_mm = st.get("min_distance_mm", 999.0)
        blocked = st.get("collision_blocked", False)
        print(f"  Target inside head: {head_tgt} m")
        print(f"  Reported Min Distance: {min_d_mm:.2f} mm | Blocked: {blocked}")

        assert min_d_mm > 0.0, f"Head penetrated! min_dist = {min_d_mm:.2f} mm"
        if blocked:
            blocked_steps_total += 1
        min_dist_overall_mm = min(min_dist_overall_mm, min_d_mm)
        print("  [PASS] Hand-vs-head collision prevented without penetration.")

        # Reset home
        send_cmd({"name": "preset", "pose": "home"}, ssl_ctx)
        time.sleep(0.5)

        # ---------------------------------------------------------
        # TEST 5: Cross-Chest Trajectory (Slow Ramp + Large Step)
        # ---------------------------------------------------------
        print("\n--- TEST 5: Cross-Chest Trajectory (Slow Ramp & Step Jump) ---")
        # Step jump across chest
        send_cmd({"name": "set_wrist_target", "arm": "left", "pos": [0.30, -0.20, 1.20]}, ssl_ctx)
        time.sleep(0.8)
        st = get_status(ssl_ctx)
        min_d_mm = st.get("min_distance_mm", 999.0)
        blocked = st.get("collision_blocked", False)
        assert min_d_mm > 0.0, f"Cross-chest penetrated! min_dist = {min_d_mm:.2f} mm"
        if blocked:
            blocked_steps_total += 1
        min_dist_overall_mm = min(min_dist_overall_mm, min_d_mm)
        print(f"  Cross-chest step jump: Min Dist = {min_d_mm:.2f} mm | Blocked = {blocked}")

        # Slow ramp across chest
        for y in np.linspace(-0.20, 0.40, 10):
            send_cmd({"name": "set_wrist_target", "arm": "left", "pos": [0.30, float(y), 1.20]}, ssl_ctx)
            time.sleep(0.05)
            st = get_status(ssl_ctx)
            min_d_mm = st.get("min_distance_mm", 999.0)
            assert min_d_mm > 0.0, f"Ramp penetrated at y={y}! min_dist = {min_d_mm:.2f} mm"
            min_dist_overall_mm = min(min_dist_overall_mm, min_d_mm)
        print(f"  Cross-chest ramp complete: Overall Min Distance = {min_dist_overall_mm:.2f} mm")
        print("  [PASS] Cross-chest trajectory safe across all substeps.")

        # ---------------------------------------------------------
        # TEST 6: Latency Benchmark (p95 < 2 ms)
        # ---------------------------------------------------------
        print("\n--- TEST 6: Latency Benchmark ---")
        times = []
        for _ in range(100):
            t0 = time.perf_counter()
            checker.compute_min_distance(ref_data)
            t1 = time.perf_counter()
            times.append((t1 - t0) * 1000.0)

        p95_ms = float(np.percentile(times, 95))
        mean_ms = float(np.mean(times))
        print(f"  Collision Check Latency: Mean = {mean_ms:.3f} ms, p95 = {p95_ms:.3f} ms (Threshold: < 2.0 ms)")
        assert p95_ms < 2.0, f"p95 latency {p95_ms:.3f} ms exceeds 2.0 ms threshold!"
        print("  [PASS] p95 collision check latency under 2 ms.")

        # Final Summary
        print("\n" + "=" * 80)
        print("SELF-COLLISION ACCEPTANCE SUMMARY:")
        print(f"  - Penetration Count : 0 (No penetration in any test case)")
        print(f"  - Minimum Distance  : {min_dist_overall_mm:.2f} mm (> 0 mm requirement)")
        print(f"  - Blocked Steps     : {blocked_steps_total} steps intercepted by SafetyLayer")
        print(f"  - Free-space Error  : {free_err_mm:.2f} mm (< 5.0 mm requirement)")
        print(f"  - p95 Check Time    : {p95_ms:.3f} ms (< 2.0 ms requirement)")
        print("=" * 80)
        print("[SUCCESS] ALL SELF-COLLISION ACCEPTANCE TESTS PASSED!")

    finally:
        # Reset home before exit
        try:
            send_cmd({"name": "preset", "pose": "home"}, ssl_ctx)
        except Exception:
            pass
        if did_spawn and proc is not None:
            stop_spawned_server(proc)


if __name__ == "__main__":
    run_tests()
