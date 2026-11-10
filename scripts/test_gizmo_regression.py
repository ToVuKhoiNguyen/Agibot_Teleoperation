#!/usr/bin/env python3
"""scripts/test_gizmo_regression.py: Playwright acceptance test for gizmo & +/-xyz jog."""
import os, sys, time, ssl, json, socket, subprocess, threading, urllib.request
import numpy as np
from pathlib import Path
from playwright.sync_api import sync_playwright

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
from teleop.robot.aimdk_map import HOME

SHOTS_DIR = Path("/tmp/shots")
SHOTS_DIR.mkdir(parents=True, exist_ok=True)
SSL_CTX = ssl.create_default_context()
SSL_CTX.check_hostname, SSL_CTX.verify_mode = False, ssl.CERT_NONE


def get_status(port=8012):
    req = urllib.request.Request(f"https://localhost:{port}/api/status")
    with urllib.request.urlopen(req, context=SSL_CTX, timeout=2.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def wait_ready(port):
    for _ in range(25):
        try:
            get_status(port); return True
        except Exception: time.sleep(0.25)
    return False

def test_sim_mode(browser):
    print("\n--- TEST A: SIM BACKEND (GIZMO & +/-XYZ WORK WITHOUT GATE) ---")
    proc = subprocess.Popen([sys.executable, str(PROJECT_ROOT / "main.py"), "--headless", "--backend", "sim", "--port", "8021"])
    try:
        wait_ready(8021)
        page = browser.new_page(viewport={"width": 1440, "height": 900})
        page.goto("https://localhost:8021/panel", timeout=15000, wait_until="networkidle")
        page.wait_for_timeout(2000)

        st0 = get_status(8021)
        ee0 = np.array(st0["ee_l"])

        # 1. Test +/-X jog button
        jog_btn = page.locator('button.repeatable:has-text("+X")').first
        assert jog_btn.is_visible(), "+X jog button not visible!"
        jog_btn.dispatch_event("mousedown")
        page.wait_for_timeout(250)
        jog_btn.dispatch_event("mouseup")
        page.wait_for_timeout(600)

        st1 = get_status(8021)
        ee1 = np.array(st1["ee_l"])
        jog_disp = np.linalg.norm(ee1 - ee0)
        print(f"  [SIM] Jog +X displacement: {jog_disp*1000.0:.1f} mm")
        assert jog_disp >= 0.005, f"Jog +X moved arm < 5 mm: {jog_disp*1000.0:.1f} mm"
        page.screenshot(path=str(SHOTS_DIR / "sim_jog_xyz.png"))

        # 2. Test mouse drag on left marker
        screen_pos = page.evaluate("""() => {
            const v = targetSphereL.position.clone();
            v.project(camera);
            const r = renderer.domElement.getBoundingClientRect();
            return { x: r.left + ((v.x + 1) * r.width) / 2, y: r.top + ((-v.y + 1) * r.height) / 2 };
        }""")
        page.mouse.move(screen_pos["x"], screen_pos["y"])
        page.mouse.down()
        for dx in range(5, 35, 5):
            page.mouse.move(screen_pos["x"] + dx, screen_pos["y"] - dx * 0.5)
            time.sleep(0.02)
        page.mouse.up()
        page.wait_for_timeout(800)

        st2 = get_status(8021)
        ee2 = np.array(st2["ee_l"])
        drag_disp = np.linalg.norm(ee2 - ee1)
        print(f"  [SIM] Drag displacement: {drag_disp*1000.0:.1f} mm")
        assert drag_disp >= 0.005, f"Drag moved arm < 5 mm: {drag_disp*1000.0:.1f} mm"
        page.screenshot(path=str(SHOTS_DIR / "sim_gizmo_drag.png"))
        print("  [PASS] Sim backend: jog and drag moved arm >= 5 mm.")
        page.close()
    finally:
        proc.terminate()
        proc.wait(timeout=5.0)


class MockRobotMC:
    def __init__(self, feedback_port=5006, cmd_port=5005):
        self.fb_port, self.cmd_port = feedback_port, cmd_port
        self.running = True
        self.cmd_log = []
        self.pinned_offset = 0.0
        self.act_q14 = np.array(HOME, dtype=np.float64)
        self.sock_fb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_cmd.bind(("127.0.0.1", self.cmd_port))
        self.sock_cmd.settimeout(0.05)
        self.thread_fb = threading.Thread(target=self._feedback_loop, daemon=True)
        self.thread_cmd = threading.Thread(target=self._cmd_loop, daemon=True)

    def start(self):
        self.thread_fb.start()
        self.thread_cmd.start()

    def stop(self):
        self.running = False
        self.sock_fb.close()
        self.sock_cmd.close()

    def _feedback_loop(self):
        while self.running:
            q_out = self.act_q14.copy()
            q_out[0] += self.pinned_offset
            payload = json.dumps({"q": q_out.tolist(), "t": time.time()}).encode()
            try:
                self.sock_fb.sendto(payload, ("127.0.0.1", self.fb_port))
            except Exception: pass
            time.sleep(0.02)  # 50 Hz

    def _cmd_loop(self):
        while self.running:
            try:
                data, _ = self.sock_cmd.recvfrom(4096)
                msg = json.loads(data.decode())
                if "q" in msg:
                    cmd_q = np.array(msg["q"], dtype=np.float64)
                    self.cmd_log.append((time.time(), cmd_q))
                    self.act_q14 += (cmd_q - self.act_q14) * 0.35
            except (socket.timeout, Exception): pass


def test_real_udp_mode(browser):
    print("\n--- TEST B: REAL-UDP BACKEND (READY -> ENGAGED -> FAULT LATCH) ---")
    mc = MockRobotMC(feedback_port=5026, cmd_port=5025)
    mc.start()
    run_env = os.environ.copy()
    run_env["ROBOT_IP"] = "127.0.0.1"
    run_env["ROBOT_PORT"] = "5025"
    run_env["ROBOT_RX_PORT"] = "5026"
    cmd = [
        sys.executable, str(PROJECT_ROOT / "main.py"), "--headless", "--backend", "real-udp",
        "--speed-profile", "normal", "--port", "8022"
    ]
    proc = subprocess.Popen(cmd, env=run_env)
    try:
        wait_ready(8022)
        page = browser.new_page(viewport={"width": 1440, "height": 900})
        page.goto("https://localhost:8022/panel", timeout=15000, wait_until="networkidle")
        page.wait_for_timeout(2000)

        # 1. In READY: verify state & drag moves gizmo, 0 commands sent
        st_ready = get_status(8022)
        assert st_ready["sync_state"] == "READY", f"Expected READY, got {st_ready['sync_state']}"
        badge_text = page.locator("#sync-badge").inner_text()
        print(f"  [READY] Badge text: '{badge_text}'")
        assert "READY" in badge_text and "Engage" in badge_text, f"Badge missing guidance: {badge_text}"

        screen_pos = page.evaluate("""() => {
            const v = targetSphereL.position.clone();
            v.project(camera);
            const r = renderer.domElement.getBoundingClientRect();
            return { x: r.left + ((v.x + 1) * r.width) / 2, y: r.top + ((-v.y + 1) * r.height) / 2 };
        }""")
        page.mouse.move(screen_pos["x"], screen_pos["y"])
        page.mouse.down()
        for dx in range(5, 30, 5):
            page.mouse.move(screen_pos["x"] + dx, screen_pos["y"] - dx * 0.5)
            time.sleep(0.02)
        page.mouse.up()
        page.wait_for_timeout(600)

        st_drag = get_status(8022)
        drag_tgt_disp = np.linalg.norm(np.array(st_drag["tgt_l"]) - np.array(st_ready["tgt_l"]))
        print(f"  [READY] Target gizmo displacement: {drag_tgt_disp*1000.0:.1f} mm | Sent cmds: {len(mc.cmd_log)}")
        assert drag_tgt_disp >= 0.005, f"Gizmo did not move in READY! {drag_tgt_disp*1000.0:.1f} mm"
        assert len(mc.cmd_log) == 0, f"Expected 0 commands sent in READY, got {len(mc.cmd_log)}"
        page.screenshot(path=str(SHOTS_DIR / "real_ready_drag.png"))

        # 2. Engage Arms: verify arm follows with max step <= 0.0101 rad/cycle
        engage_btn = page.locator("#btn-engage-arms")
        assert engage_btn.is_visible(), "Engage button not visible in READY!"
        engage_btn.click(force=True)
        page.wait_for_timeout(1000)

        st_eng = get_status(8022)
        assert st_eng["sync_state"] == "ENGAGED", f"Expected ENGAGED after click, got {st_eng['sync_state']}"
        print(f"  [ENGAGED] Transitioned to ENGAGED. Commands received: {len(mc.cmd_log)}")
        assert len(mc.cmd_log) > 0, "No commands received after Engage!"

        steps = [float(np.max(np.abs(mc.cmd_log[i][1] - mc.cmd_log[i - 1][1]))) for i in range(1, len(mc.cmd_log))]
        max_step = max(steps) if steps else 0.0
        print(f"  [ENGAGED] Max step per cycle: {max_step:.5f} rad (cap: 0.0101 rad)")
        assert max_step <= 0.0101, f"Step exceeded rate cap: {max_step:.5f} > 0.0101"
        page.screenshot(path=str(SHOTS_DIR / "real_engaged_move.png"))

        # 3. Test +/-X jog button in ENGAGED: moves gizmo and arm follows
        n_cmds_before = len(mc.cmd_log)
        jog_btn = page.locator('button.repeatable:has-text("+X")').first
        jog_btn.dispatch_event("mousedown")
        page.wait_for_timeout(250)
        jog_btn.dispatch_event("mouseup")
        page.wait_for_timeout(600)
        assert len(mc.cmd_log) > n_cmds_before, "No new commands sent after jog in ENGAGED!"
        print("  [PASS] Jog +X in ENGAGED moves gizmo and advances commands.")

        # 4. Trigger FAULT latch: input accepted, 0 commands sent until Clear + Engage
        mc.pinned_offset = 0.07  # Induce 0.07 rad deviation (>0.05 rad gate)
        t_fault_start = time.time()
        st_fault = None
        while time.time() - t_fault_start < 2.0:
            st_fault = get_status(8022)
            if st_fault["fault_latched"] or st_fault["sync_state"] == "FAULT": break
            time.sleep(0.05)

        print(f"  [FAULT] Latched: {st_fault['fault_latched']}, State: {st_fault['sync_state']}")
        assert st_fault["fault_latched"] or st_fault["sync_state"] == "FAULT", "FAULT should be latched!"
        page.screenshot(path=str(SHOTS_DIR / "real_fault_hold.png"))

        cmds_during_fault = len(mc.cmd_log)
        page.evaluate("() => { sendCmd({cmd:'arm_jog', arm:'left', axis:'x', dir:1}); }")
        time.sleep(0.1)
        assert len(mc.cmd_log) == cmds_during_fault, "Commands must not be sent while in FAULT!"

        # Clear fault
        mc.pinned_offset = 0.0
        page.evaluate("() => { sendCmd({cmd: 'clear_fault', confirmed: true}); }")
        time.sleep(0.5)
        st_cleared = get_status(8022)
        assert not st_cleared["fault_latched"], "Fault did not clear!"
        assert st_cleared["sync_state"] == "READY", f"Expected READY after clear, got {st_cleared['sync_state']}"
        print("  [PASS] Real-UDP state machine, smooth following, and fault safety verified.")
        page.close()
    finally:
        proc.terminate()
        proc.wait(timeout=5.0)
        mc.stop()


def main():
    print("=" * 80 + "\nPLAYWRIGHT HEADLESS ACCEPTANCE TEST: GIZMO & +/-XYZ JOG VERIFICATION\n" + "=" * 80)
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True, args=["--ignore-certificate-errors", "--no-sandbox"])
        test_sim_mode(browser)
        test_real_udp_mode(browser)
        browser.close()
    print("\nALL ACCEPTANCE TESTS PASSED!")


if __name__ == "__main__":
    main()
