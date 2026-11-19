#!/usr/bin/env python3
"""
scripts/test_sil.py - Software-In-the-Loop acceptance suite.

Brings up sil/sil_up.sh (fake MC + UNMODIFIED arm_teleop_rx.py on loopback) and checks the
real receiver's behaviour end to end, then optionally drives the real laptop app (main.py,
backend real-udp, ROBOT_IP=127.0.0.1) through the same stack.

  Layer A (receiver, laptop emulated over the real UDP protocol):
    A1 feedback handshake (sha, proto, rate)         A6 stale target -> hold + hands open
    A2 not engaged -> nothing published to MC         A7 foreign session -> HOLDING
    A3 engage -> first command == measured            A8 NaN packet ignored
    A4 final-line step cap + convergence              A9 stale feedback (frozen MC state)
    A5 hand cap 2000 raw, per-side gating           A10 dual publisher -> FAULT latch
    A11 GetAction != 402 -> receiver refuses to start
    A13 robot standing outside our joint bounds -> engage does not move it
  Layer B (--e2e): main.py SYNC -> READY -> engage -> jog -> fake MC moved by the commanded amount.

Usage:
  venv/bin/python scripts/test_sil.py            # layer A
  venv/bin/python scripts/test_sil.py --e2e      # layer A + B
  venv/bin/python scripts/test_sil.py --only A4,A5
Exit code 0 = all selected checks passed.
"""
import os
import sys
import json
import time
import math
import signal
import socket
import argparse
import subprocess
import urllib.request
import ssl

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "sil"))
os.environ["SIL_CTL_HOST"] = "127.0.0.1"  # loopback SIL fake MC, never the a2sim robot (192.168.2.50)
from sil_ctl import ctl  # noqa: E402

RX_SHA = None  # filled from scripts/arm_teleop_rx.py
TX_PORT, FB_PORT = 5005, 5006
THUMB_REST6 = [400.0, 900.0, 0.0, 0.0, 0.0, 0.0]   # receiver rest hand: --thumb-pose 400,900 (default), fingers open
HTTPS_PORT, CTL_PORT = 8012, 5099
# SIL_ALT_PORTS=1: run beside a live session (main.py on 5006/8012, robot on domain 232/88) on private ports
if os.environ.get("SIL_ALT_PORTS") == "1":
    TX_PORT, FB_PORT, HTTPS_PORT, CTL_PORT = 5015, 5016, 8099, 5098
    os.environ.update(SIL_DOMAIN="89", SIL_RPC_PORT="56399", SIL_CTL_PORT=str(CTL_PORT),
                      SIL_RX_ARGS=(os.environ.get("SIL_RX_ARGS", "") + f" --port {TX_PORT} --tx-state-port {FB_PORT}").strip())
    import sil_ctl as _sil_ctl  # noqa: E402
    _sil_ctl.CTL_PORT = CTL_PORT  # ctl() reads this module global at call time
SPEED_VMAX = {1: 0.40, 2: 0.60, 3: 0.80, 4: 1.00, 5: 1.5, 6: 2.0}
os.environ["no_proxy"] = "127.0.0.1,localhost," + os.environ.get("no_proxy", "")
os.environ["NO_PROXY"] = os.environ["no_proxy"]

RESULTS = []


def check(name, cond, detail="", xfail=None):
    """xfail='reason': known, reported bug -> a failure is printed as XFAIL and does not fail the suite."""
    cond = bool(cond)
    if xfail and not cond:
        tag, ok = "XFAIL", True
        detail = f"{detail}; KNOWN BUG: {xfail}"
    elif xfail and cond:
        tag, ok = "XPASS", True
        detail = f"{detail}; known bug no longer reproduces - remove xfail"
    else:
        tag, ok = ("PASS" if cond else "FAIL"), cond
    RESULTS.append((name, ok, detail))
    print(f"  [{tag}] {name}" + (f"  ({detail})" if detail else ""), flush=True)
    return cond


# ----------------------------------------------------------------------------- stack
class Stack:
    def __init__(self, speed, extra_env=None, tag="sil"):
        self.speed, self.extra_env, self.tag, self.proc = speed, extra_env or {}, tag, None
        self.log_path = None

    def __enter__(self):
        for port in (TX_PORT, CTL_PORT):
            _assert_port_free_udp(port)
        env = dict(os.environ, SPEED=str(self.speed), **self.extra_env)
        os.makedirs(os.path.join(REPO, "logs", "sil"), exist_ok=True)
        self.log_path = os.path.join(REPO, "logs", "sil", f"test_{self.tag}_{time.strftime('%H%M%S')}.log")
        self.log_f = open(self.log_path, "w")
        self.proc = subprocess.Popen(["bash", os.path.join(REPO, "sil", "sil_up.sh")], env=env, cwd=REPO,
                                     stdout=self.log_f, stderr=subprocess.STDOUT, start_new_session=True)
        return self

    def wait_log(self, needle, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            with open(self.log_path) as f:
                if needle in f.read():
                    return True
            if self.proc.poll() is not None:
                return False
            time.sleep(0.2)
        return False

    def log_text(self):
        with open(self.log_path) as f:
            return f.read()

    def __exit__(self, *exc):
        if self.proc and self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGINT)
            try:
                self.proc.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
        self.log_f.close()
        time.sleep(0.5)


def _assert_port_free_udp(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", port))
    except OSError:
        raise SystemExit(f"UDP port {port} busy (another SIL/teleop running?)")
    finally:
        s.close()


# ----------------------------------------------------------------------------- laptop emulator
class Laptop:
    """Speaks the exact UdpBackend wire format (proto v3) and listens for feedback on 5006."""

    def __init__(self, session_id=None):
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("0.0.0.0", FB_PORT))
        self.rx.settimeout(0.05)
        self.session_id = session_id or f"sil_test_{os.getpid()}"
        self.seq = 0
        self.last_fb, self.last_fb_t = None, 0.0

    def close(self):
        self.tx.close(); self.rx.close()

    def send(self, q, engaged=(True, True), hand=None, session_id=None, raw_json=None, rec=None):
        self.seq += 1
        pkt = {"seq": self.seq, "t": time.time(), "session_id": session_id or self.session_id, "proto_ver": 3,
               "q": [float(round(x, 5)) for x in q], "engaged": list(engaged),
               "hand": [int(x) for x in (hand if hand is not None else [0] * 12)]}
        if rec: pkt["rec"] = rec
        data = raw_json.encode() if raw_json else json.dumps(pkt).encode()
        self.tx.sendto(data, ("127.0.0.1", TX_PORT))

    def poll(self, duration=0.0):
        t_end = time.time() + duration
        got = []
        while True:
            try:
                d, _ = self.rx.recvfrom(4096)
                self.last_fb, self.last_fb_t = json.loads(d.decode()), time.time()
                got.append(self.last_fb)
            except socket.timeout:
                pass
            if time.time() >= t_end:
                return got

    def stream(self, q_fn, duration, rate=100.0, **kw):
        """Sends at the laptop wire rate while draining feedback. q_fn(t) -> q14."""
        t0 = time.time(); n = 0
        while time.time() - t0 < duration:
            self.send(q_fn(time.time() - t0), **kw)
            n += 1
            nxt = t0 + n / rate
            while time.time() < nxt:
                self.poll(0.0)
                time.sleep(0.001)
        self.poll(0.0)

    def measured(self):
        return np.array(self.last_fb["q"]) if self.last_fb else None


# ----------------------------------------------------------------------------- layer A
def layer_a(speed, only):
    sel = lambda k: (not only) or (k in only)  # noqa: E731
    vmax = SPEED_VMAX[speed]
    step_cap = vmax / 100.0
    print(f"\n=== Layer A: receiver in SIL (speed {speed}, step cap {step_cap*1000:.1f} mrad/tick) ===")
    with Stack(speed, tag="A") as st:
        ok = st.wait_log("Ready and listening", 25)
        if not check("stack up (receiver passed action/pub/preflight checks)", ok, st.log_path):
            print(st.log_text()[-3000:]); return
        lap = Laptop()
        try:
            fb = lap.poll(1.0)
            q0 = lap.measured()
            if sel("A1"):
                rate = len(fb) / 1.0
                check("A1 feedback stream", q0 is not None and len(q0) == 14, f"{rate:.0f} pkt/s")
                if fb:
                    f = fb[-1]
                    check("A1 sha == scripts/arm_teleop_rx.py", f.get("sha") == RX_SHA[:12], f.get("sha"))
                    check("A1 proto_ver 3 / wire_hz 100", f.get("proto_ver") == 3 and f.get("wire_hz") == 100)
                    check("A1 rx_state NOT_ENGAGED before engage", f.get("rx_state") == "NOT_ENGAGED", f.get("rx_state"))
                    check("A1 feedback rate ~100 Hz", 85 <= rate <= 115, f"{rate:.0f}")
            if q0 is None:
                return

            ctl({"cmd": "reset_stats"})
            if sel("A2"):
                tgt = q0.copy(); tgt[1] += 0.2
                lap.stream(lambda t: tgt, 0.6, engaged=(False, False))
                s = ctl({"cmd": "status"})
                check("A2 not engaged -> 0 arm commands to MC", s["n_arm_cmd"] == 0, f"n_arm_cmd={s['n_arm_cmd']}")
                lap.poll(2.2)  # session lock expires after 2 s silence -> clean state for engage

            q0 = lap.measured()
            ctl({"cmd": "reset_stats"})
            if sel("A3") or sel("A4") or sel("A5"):
                lap.stream(lambda t: q0, 0.5)
                s = ctl({"cmd": "status"})
                if sel("A3"):
                    check("A3 engage: commands flow", s["n_arm_cmd"] > 20, f"n={s['n_arm_cmd']}")
                    d_first = float(np.max(np.abs(np.array(s["first_cmd_q"]) - q0))) if s["first_cmd_q"] else 1.0
                    check("A3 first command == measured pose", d_first < 0.002, f"|first_cmd - measured| = {d_first*1000:.2f} mrad")
                    check("A3 no motion afterwards (target == measured)", s["max_cmd_step_rad"] < 1e-4,
                          f"{s['max_cmd_step_rad']*1000:.3f} mrad")
                    print(f"  [INFO] A3 engage jump vs MC's held setpoint = {s['first_cmd_jump_rad']*1000:.2f} mrad "
                          f"(= measured static offset/quantization; real robot will show the same effect)")
                    check("A3 rx_state RX_OK", lap.last_fb.get("rx_state") == "RX_OK", lap.last_fb.get("rx_state"))

            if sel("A4"):
                ctl({"cmd": "reset_stats"})
                tgt = q0.copy(); tgt[1] += 0.30; tgt[10] -= 0.20
                lap.stream(lambda t: tgt, 0.6 / vmax * 1.0 + 0.8)
                s = ctl({"cmd": "status"})
                err_cmd = float(np.max(np.abs(np.array(s["cmd_q"]) - tgt)))
                err_meas = float(np.max(np.abs(np.array(s["meas_q"]) - tgt)))
                # The guard limits SPEED (max_vel x true tick period), so a late tick may carry a larger step;
                # the hard per-message ceiling is 16 mrad (MC rejects > 20 mrad).
                abs_cap = max(step_cap, 0.016)
                check("A4 per-message step <= 16 mrad ceiling", s["max_cmd_step_rad"] <= abs_cap + 1e-5,
                      f"max {s['max_cmd_step_rad']*1000:.2f} mrad vs ceiling {abs_cap*1000:.1f}")
                check("A4 command reaches target", err_cmd < 0.002, f"{err_cmd*1000:.2f} mrad")
                check("A4 plant (measured) converges", err_meas < 0.010, f"{err_meas*1000:.2f} mrad")
                q0 = np.array(s["meas_q"])

            if sel("A5"):
                hand = [2500] * 12
                lap.stream(lambda t: q0, 0.6, engaged=(False, True), hand=hand)
                s = ctl({"cmd": "status"})
                h = np.array(s["cmd_h"])
                check("A5 hand clamp <= 2000 raw (full travel)", float(np.max(h)) <= 2000.0, f"max {np.max(h):.0f}")
                # rest pose = thumb fixed in the grasp pose (--thumb-pose 400,900 default), fingers open
                check("A5 left hand at rest (thumb rest pose 400/900, fingers 0) when left not engaged", h[:6].tolist() == THUMB_REST6, f"{h[:6].tolist()}")
                check("A5 right fingers reach full travel", float(np.min(h[7:])) >= 1999.0, f"{h[6:].tolist()}")
                check("A5 thumb swing passes when enabled (default)", h[6] >= 1999.0, f"thumb_0={h[6]:.0f}")

            if sel("A6"):
                lap.stream(lambda t: q0, 0.3, hand=[400] * 12)
                s1 = ctl({"cmd": "status"})
                lap.poll(0.7)  # stop sending
                s2 = ctl({"cmd": "status"})
                time.sleep(0.3)
                s3 = ctl({"cmd": "status"})
                rx = lap.last_fb.get("rx_state")
                check("A6 rx_state REJECT:stale_target after 400 ms silence", rx == "REJECT:stale_target", rx)
                hold_move = float(np.max(np.abs(np.array(s3["cmd_q"]) - np.array(s2["cmd_q"]))))
                check("A6 arm holds (cmd unchanged while stale)", hold_move < 1e-6, f"{hold_move*1000:.3f} mrad")
                check("A6 hands open, thumbs in rest pose (400/900) while stale", [float(x) for x in s3["cmd_h"]] == THUMB_REST6 * 2, f"{s3['cmd_h']}")
                check("A6 receiver keeps publishing hold (MC never starves)", s3["n_arm_cmd"] > s1["n_arm_cmd"],
                      f"{s1['n_arm_cmd']} -> {s3['n_arm_cmd']}")

            if sel("A7"):
                lap.stream(lambda t: q0, 0.3)
                lap.send(q0 + 0.1, session_id="intruder")
                lap.stream(lambda t: q0, 0.2)
                rx = lap.last_fb.get("rx_state")
                check("A7 foreign session -> HOLDING", rx == "HOLDING", rx)
                lap.poll(2.3)  # silence -> lock resets

            if sel("A8"):
                q_nan = list(q0); q_nan[2] = float("nan")
                lap.stream(lambda t: q0, 0.3)
                ctl({"cmd": "reset_stats"})
                for _ in range(20):
                    lap.send(q_nan)
                    lap.poll(0.01)
                lap.stream(lambda t: q0, 0.2)
                s = ctl({"cmd": "status"})
                fault = lap.last_fb.get("robot_fault")
                check("A8 NaN packets dropped, no motion, no fault", s["max_cmd_step_rad"] < 1e-4 and not fault,
                      f"step {s['max_cmd_step_rad']*1000:.3f} mrad, fault={fault}")

            if sel("A9"):
                lap.stream(lambda t: q0, 0.3)
                ctl({"cmd": "freeze_state", "on": True})
                t_last = lap.last_fb_t
                lap.stream(lambda t: q0, 0.6)
                gap = time.time() - lap.last_fb_t
                ctl({"cmd": "freeze_state", "on": False})
                lap.stream(lambda t: q0, 0.3)
                check("A9 frozen MC state -> feedback stops reaching laptop", gap > 0.5, f"gap {gap*1000:.0f} ms")
                check("A9 [doc gap] receiver does NOT fault on stale feedback (doc claims >300 ms FAULT)",
                      not lap.last_fb.get("robot_fault"), "documented behaviour not implemented")

            if sel("A10"):
                lap.stream(lambda t: q0, 0.3)
                ctl({"cmd": "dual_publisher", "on": True})
                lap.stream(lambda t: q0, 1.0)
                f = lap.last_fb or {}
                check("A10 dual publisher -> FAULT latched", bool(f.get("robot_fault")) and "DUAL" in f.get("fault_reason", ""),
                      f.get("fault_reason"))
                ctl({"cmd": "dual_publisher", "on": False})
        finally:
            lap.close()

    if sel("A13"):   # robot standing outside OUR joint bounds (real 17/11: elbows at +-0.035 after SetAction)
        rest = [0.0, 1.314, 0.0, -0.035, 1.570, 0.0, 0.0, 0.0, -1.315, 0.0, 0.035, 1.571, 0.40, 0.0]   # + R wrist J6 0.40
        with Stack(speed, extra_env={"SIL_MC_ARGS": "--init-pose " + json.dumps(rest).replace(" ", "")}, tag="A13") as st:
            if not check("A13 stack up", st.wait_log("Ready and listening", 25), st.log_path): return
            lap = Laptop()
            try:
                lap.poll(3.0); q0 = lap.measured()          # the plant settles from the init pose first (gravity sag)
                ctl({"cmd": "reset_stats"})
                lap.stream(lambda t: q0, 0.8)
                s = ctl({"cmd": "status"})
                d_first = float(np.max(np.abs(np.array(s["first_cmd_q"]) - q0))) if s["first_cmd_q"] else 1.0
                check("A13 start outside our bounds: first command == measured (no self-motion)", d_first < 0.002,
                      f"{d_first*1000:.2f} mrad (elbows {q0[3]:+.3f}/{q0[10]:+.3f}, R wrist {q0[12]:+.2f})")
                check("A13 ... and no motion afterwards", s["max_cmd_step_rad"] < 1e-4, f"{s['max_cmd_step_rad']*1000:.3f} mrad")
            finally:
                lap.close()

    if sel("A11"):
        with Stack(speed, extra_env={"SIL_MC_ARGS": "--action 401"}, tag="A11") as st:
            st.wait_log("Must set to 402", 20)
            check("A11 GetAction != 402 -> receiver refuses LIVE", "Must set to 402" in st.log_text())

    if sel("A12"):   # data collection: the laptop's "rec" field starts / stops a rosbag in a per-episode folder
        import re, glob
        with Stack(speed, extra_env={"ORIN_RECORDER_ON": "1"}, tag="A12") as st:
            if not check("A12 stack up", st.wait_log("Ready and listening", 25), st.log_path): return
            m = re.search(r"run dir: (\S+)", st.log_text())
            data_root = os.path.join(m.group(1), "data") if m else ""
            lap = Laptop()
            try:
                lap.poll(1.0); q0 = lap.measured()
                lap.stream(lambda t: q0, 0.8)
                f = (lap.last_fb or {}).get("rec") or {}
                check("A12 recorder idle before a request", f.get("state") == "idle", f)
                # the MC stamp is always sent since 16/11 (exact timing in the laptop logs); the heavy fields are not
                check("A12 plain feedback carries no data-collection fields", all(k not in (lap.last_fb or {}) for k in ("dq", "tau", "cmd")))
                check("A12 plain feedback carries the MC stamp and pacer health",
                      isinstance((lap.last_fb or {}).get("stamp"), float) and isinstance((lap.last_fb or {}).get("pacer"), dict),
                      (lap.last_fb or {}).get("pacer"))
                rec = {"task": "sil test/../etc", "date": "2026-11-13", "ep": "ep_0001_000000"}
                lap.stream(lambda t: q0, 7.0, rec=rec)
                f = (lap.last_fb or {}).get("rec") or {}
                check("A12 robot reports recording with data", f.get("state") == "recording" and f.get("mb", 0) > 0, f)
                fb = lap.last_fb or {}
                check("A12 while recording, feedback adds MC stamp, joint velocity and published command",
                      isinstance(fb.get("stamp"), float) and len(fb.get("dq") or []) == 14 and len(fb.get("cmd") or []) == 14,
                      {k: (len(v) if isinstance(v, list) else v) for k, v in fb.items() if k in ("stamp", "dq", "tau", "cmd", "hand_tau")})
                lap.stream(lambda t: q0, 2.5)
                f = (lap.last_fb or {}).get("rec") or {}
                check("A12 recorder idle after the request ends", f.get("state") == "idle", f)
                eps = glob.glob(os.path.join(data_root, "*", "*", "*", "meta.json"))
                ok_dir = len(eps) == 1 and os.path.realpath(eps[0]).startswith(os.path.realpath(data_root) + os.sep)
                check("A12 one episode folder <task>/<date>/<episode>, name sanitised (no path escape)", ok_dir,
                      [os.path.relpath(e, data_root) for e in eps])
                if eps:
                    meta = json.load(open(eps[0])); ep_dir = os.path.dirname(eps[0])
                    bag = os.path.join(ep_dir, meta.get("bag", "robot"))
                    size = sum(os.path.getsize(os.path.join(bag, x)) for x in os.listdir(bag)) if os.path.isdir(bag) else 0
                    check("A12 meta.json closed (duration, size) and rosbag written", meta.get("duration_s", 0) > 3 and size > 0
                          and os.path.exists(os.path.join(bag, "metadata.yaml")), f"dur={meta.get('duration_s')} size={size}")
                lap.stream(lambda t: q0, 1.5, rec=dict(rec, ep="ep_0002_000001"))
                lap.poll(3.6)   # laptop goes silent while recording -> the robot must stop by itself
                st.wait_log("[rec] STOP  sil_test", 8)
                check("A12 recording stops when the laptop goes silent (3 s)", st.log_text().count("[rec] STOP") >= 2)
            finally:
                lap.close()


# ----------------------------------------------------------------------------- layer B
def _https(path, body=None, port=None):
    port = port or HTTPS_PORT
    ctx = ssl.create_default_context(); ctx.check_hostname = False; ctx.verify_mode = ssl.CERT_NONE
    req = urllib.request.Request(f"https://127.0.0.1:{port}{path}", data=json.dumps(body).encode() if body else None,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, context=ctx, timeout=3.0) as r:
        return json.loads(r.read().decode())


def layer_b(speed, ideal=False):
    print(f"\n=== Layer B: main.py (real-udp) -> fake Orin -> fake MC (speed {speed}, plant {'IDEAL' if ideal else 'mc_plant'}) ===")
    with Stack(speed, extra_env={"SIL_IDEAL": "1"} if ideal else None, tag="B") as st:
        if not check("stack up", st.wait_log("Ready and listening", 25), st.log_path):
            return
        env = dict(os.environ, A2_AUTO_EXPORT="0", ROBOT_IP="127.0.0.1", ROBOT_PORT=str(TX_PORT), ROBOT_RX_PORT=str(FB_PORT),   # no auto LeRobot export of the self-test episode
                   TELEOP_LOCK_FILE="/tmp/a2_teleop_sil.lock")
        app_log = open(os.path.join(REPO, "logs", "sil", f"test_B_main_{time.strftime('%H%M%S')}.log"), "w")
        app = subprocess.Popen([sys.executable, "-u", "main.py", "--headless", "--no-adb", "--no-quest-launch",
                                "--speed", str(speed), "--port", str(HTTPS_PORT)], cwd=REPO, env=env, stdout=app_log, stderr=subprocess.STDOUT,
                               start_new_session=True)
        try:
            s = None
            for _ in range(120):
                try:
                    s = _https("/api/status")
                    if s.get("sync_state") in ("READY", "ENGAGED"):  # READY is transient (auto-engage)
                        break
                except Exception:
                    pass
                time.sleep(0.5)
            if not check("B1 SYNC -> READY/ENGAGED from fake robot feedback", s and s.get("sync_state") in ("READY", "ENGAGED"),
                         s.get("sync_state") if s else "no status"):
                return
            check("B1 feedback age < 100 ms", s.get("feedback_age_ms", 999) < 100, f"{s.get('feedback_age_ms'):.0f} ms")
            mc0 = ctl({"cmd": "status"})
            ee0 = np.array(s["ee_l"])

            _https("/api/command", {"name": "engage_arm", "arm": "both"})
            time.sleep(0.8)
            s = _https("/api/status")
            check("B2 ENGAGED", s.get("sync_state") == "ENGAGED", f"{s.get('sync_state')} / orin {s.get('orin_rx_state')}")
            mc1 = ctl({"cmd": "status"})
            jump = float(np.max(np.abs(np.array(mc1["cmd_q"]) - np.array(mc0["meas_q"]))))
            check("B2 engage causes no jump at MC", jump < 0.005, f"{jump*1000:.2f} mrad")

            ctl({"cmd": "reset_stats"})
            _https("/api/command", {"name": "jog_l", "axis": 0, "step": 0.05})
            time.sleep(5.0)
            s = _https("/api/status")
            mc2 = ctl({"cmd": "status"})
            disp = np.array(s["ee_l"]) - ee0
            check("B3 jog_l +X 5 cm moved real (fake) arm", 0.035 < disp[0] < 0.065, f"disp={np.round(disp, 3).tolist()}")
            check("B3 MC per-tick step <= cap", mc2["max_cmd_step_rad"] <= SPEED_VMAX[speed] / 100 * 1.02 + 1e-5,
                  f"{mc2['max_cmd_step_rad']*1000:.2f} mrad")
            track = float(np.max(np.abs(np.array(mc2["meas_q"]) - np.array(mc2["cmd_q"]))))
            check("B3 MC tracking settled", track < 0.01, f"{track*1000:.2f} mrad")
            check("B3 no fault", not s.get("fault_latched"), s.get("fault_reason"))
            path, net = np.array(mc2["cmd_path_rad"][:7]), np.abs(np.array(mc2["cmd_net_rad"][:7]))
            moving = net > 0.01
            ratio = float(np.max(path[moving] / net[moving])) if moving.any() else 1.0
            check("B4 step target -> monotone command (no oscillation): max path/net <= 1.2", ratio <= 1.2,
                  f"path/net per joint = {np.round(path / np.maximum(net, 1e-6), 2).tolist()}")

            # B5 data collection: an episode recorded through the panel API lands on the laptop with robot data
            import shutil
            task = f"sil_selftest_{os.getpid()}"
            dom = os.environ.get("SIL_DOMAIN", "88")
            cam = subprocess.Popen(["bash", "-c", f"set +u; source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID={dom} ROS_LOCALHOST_ONLY=1; "
                                    f"unset FASTRTPS_DEFAULT_PROFILES_FILE; exec /usr/bin/python3 {os.path.join(REPO, 'sil', 'fake_camera.py')}"],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            time.sleep(1.5)
            _https("/api/command", {"name": "rec_start", "task": task, "instruction": "sil self test"})
            time.sleep(1.0)
            _https("/api/command", {"name": "jog_l", "axis": 2, "step": 0.03})
            time.sleep(5.0)
            _https("/api/command", {"name": "rec_stop"})
            time.sleep(0.8)
            try: os.killpg(cam.pid, signal.SIGKILL)
            except Exception: pass
            ep_dirs = sorted(__import__("glob").glob(os.path.join(REPO, "recordings", task, "*", "ep_*")))
            try:
                ok = len(ep_dirs) == 1 and all(os.path.exists(os.path.join(ep_dirs[0], f)) for f in ("meta.json", "robot.jsonl", "teleop.jsonl"))
                check("B5 episode folder with meta.json, robot.jsonl, teleop.jsonl on the laptop", ok, ep_dirs)
                if ok:
                    meta = json.load(open(os.path.join(ep_dirs[0], "meta.json")))
                    rob = [json.loads(l) for l in open(os.path.join(ep_dirs[0], "robot.jsonl"))]
                    tel = [json.loads(l) for l in open(os.path.join(ep_dirs[0], "teleop.jsonl"))]
                    dur = meta.get("duration_s", 0)
                    check("B5 robot rows ~100 Hz with MC stamp, velocity, command", len(rob) > 60 * dur and
                          all(k in rob[-1] and rob[-1][k] is not None for k in ("t_laptop", "t_robot", "stamp_mc", "q", "dq", "cmd")),
                          f"{len(rob)} rows in {dur} s, keys with data: {[k for k, v in rob[-1].items() if v is not None]}")
                    check("B5 teleop rows with targets and hand pose", len(tel) > 40 * dur and tel[-1]["left"]["ee_pos"] is not None
                          and tel[-1]["left"]["target_quat_xyzw"] is not None and tel[-1]["q_cmd"] is not None, f"{len(tel)} rows")
                    q = np.array([r["q"] for r in rob]); check("B5 the jog is visible in the recorded joint data", float(np.ptp(q, axis=0).max()) > 0.02,
                                                              f"max joint travel {float(np.ptp(q, axis=0).max()):.3f} rad")
                    idx_p = os.path.join(ep_dirs[0], "images.jsonl")
                    imgs = [json.loads(l) for l in open(idx_p)] if os.path.exists(idx_p) else []
                    col = [i for i in imgs if i["cam"] == "head"]; dep = [i for i in imgs if i["cam"] == "head_depth"]
                    stamps = [i["stamp"] for i in col]
                    ok_img = len(col) > 20 and all(os.path.getsize(os.path.join(ep_dirs[0], i["file"])) > 1000 for i in col)
                    check("B5 colour frames streamed from the (fake) robot as JPEG with increasing image stamps",
                          ok_img and all(b > a for a, b in zip(stamps, stamps[1:])) and meta.get("instruction") == "sil self test",
                          f"{len(col)} frames, stored {col[0]['stored_w']}x{col[0]['stored_h']}" if col else "no frames")
                    check("B5 image index keeps every sensor_msgs/Image field", bool(col) and col[0]["encoding"] == "rgb8" and col[0]["height"] == 480
                          and col[0]["width"] == 640 and col[0]["step"] == 1920 and col[0]["is_bigendian"] == 0 and col[0]["frame_id"] == "head_front",
                          {k: col[0].get(k) for k in ("encoding", "height", "width", "step", "is_bigendian", "frame_id")} if col else None)
                    ok_dep = False; dinfo = "no depth frames"
                    if len(dep) > 20:
                        import cv2
                        d = cv2.imread(os.path.join(ep_dirs[0], dep[5]["file"]), cv2.IMREAD_UNCHANGED)
                        ok_dep = d is not None and d.dtype == np.uint16 and d.shape == (480, 640) and int(d[100, 320]) == 500 + 320 * 2500 // 640 \
                            and dep[5]["encoding"] == "16UC1" and dep[5]["stored_fmt"] == "png"
                        dinfo = f"{len(dep)} frames, dtype {d.dtype if d is not None else None}, value at centre {int(d[100, 320]) if d is not None else None} mm"
                    check("B5 depth frames stored as lossless 16-bit PNG with exact values", ok_dep, dinfo)
                    check("B5 robot rows carry the neck joint state", rob[-1].get("neck") is not None and abs(rob[-1]["neck"][0] - 0.10) < 1e-6, rob[-1].get("neck"))
                    check("B5 meta.json: row counts and clock offset", meta.get("rows", {}).get("robot", 0) == len(rob)
                          and meta.get("robot_clock_minus_laptop_s") is not None, {k: meta.get(k) for k in ("rows", "robot_clock_minus_laptop_s")})
            finally:
                if os.environ.get("SIL_KEEP_DATA") != "1": shutil.rmtree(os.path.join(REPO, "recordings", task), ignore_errors=True)

        finally:
            os.killpg(app.pid, signal.SIGINT)
            try:
                app.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(app.pid, signal.SIGKILL)
            app_log.close()


def refuse_if_live_session():
    """The fake Orin streams feedback to 127.0.0.1:5006. A running main.py listens there on 0.0.0.0 and would mix
    the fake robot's joints into a live session (possibly ENGAGED with the real robot). Never start in that case."""
    for port, kind in ((FB_PORT, socket.SOCK_DGRAM), (8012, socket.SOCK_STREAM)):
        s = socket.socket(socket.AF_INET, kind)
        try:
            s.bind(("0.0.0.0", port))
        except OSError:
            sys.exit(f"REFUSING TO RUN: port {port} is in use - a teleop session (main.py) is live on this machine. "
                     f"Stop it first; SIL would inject fake robot feedback into it.")
        finally:
            s.close()
    # a real receiver on the Orin streams feedback to this port even when main.py is not running: it would be
    # mixed into the test's own feedback (seen 13/11: "feedback rate 201 Hz", first command 1.8 rad off)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("0.0.0.0", FB_PORT)); s.settimeout(0.5)
    try:
        _, addr = s.recvfrom(65535)
        sys.exit(f"REFUSING TO RUN: robot feedback is arriving from {addr[0]} on port {FB_PORT} "
                 f"(arm_teleop_rx.py is running on the robot). Stop it first; the results would be meaningless.")
    except socket.timeout:
        pass
    finally:
        s.close()


def main():
    refuse_if_live_session()
    global RX_SHA
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=int, default=3, choices=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--e2e", action="store_true")
    ap.add_argument("--only", default="")
    ap.add_argument("--ideal", action="store_true", help="layer B with pass-through plant (isolates plant-induced effects)")
    a = ap.parse_args()
    import hashlib
    with open(os.path.join(REPO, "scripts", "arm_teleop_rx.py"), "rb") as f:
        RX_SHA = hashlib.sha256(f.read()).hexdigest()
    only = set(x.strip() for x in a.only.split(",") if x.strip())
    if not only or any(k.startswith("A") for k in only):
        layer_a(a.speed, only)
    if a.e2e or "B" in only:
        layer_b(a.speed, a.ideal)
    n_fail = sum(1 for _, ok, _ in RESULTS if not ok)
    print("\n" + "=" * 70)
    print(f"SIL: {len(RESULTS) - n_fail}/{len(RESULTS)} checks passed" + (f", {n_fail} FAILED" if n_fail else ""))
    for name, ok, det in RESULTS:
        if not ok:
            print(f"  FAIL: {name} {det}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
