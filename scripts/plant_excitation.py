#!/usr/bin/env python3
"""
scripts/plant_excitation.py - Collect identification data for the MC plant (arm + hand).

Acts as the laptop (same UDP wire protocol v3 as teleop/robot/udp_backend.py) and drives the
UNMODIFIED Orin receiver with small, slow, pre-verified joint excitations:
  per joint: step +A / 0 / -A / 0 (receiver ramps at its speed cap), then a chirp 0.2 -> 3 Hz
  whose amplitude is reduced so peak velocity stays <= 60 % of the receiver speed cap;
  per hand: 0 -> 400 -> 800 -> 0 raw ramps (receiver clamps at 800).
Writes logs/plant_excitation_<ts>.jsonl (t, sent q, measured q, hand_cmd, hand_meas, rx_state).
The Orin writes trace_orin_<ts>.jsonl (published -> measured at the MC), which is the arm fit input.

SAFETY
  * Rehearse first against SIL:   sil/sil_up.sh  then  venv/bin/python scripts/plant_excitation.py
  * Real robot needs --robot-ip 192.168.2.50 --i-have-estop, a second person on the physical E-stop,
    receiver started with --speed 1, and main.py NOT running (single sender, port 5006).
  * Whole trajectory is checked offline before motion: joint limits (margin 0.05 rad) and armor
    self-collision clearance >= 40 mm at every sample. Any failure -> nothing is sent.
  * Live aborts: Ctrl+C, receiver FAULT/HOLDING/REJECT, feedback older than 300 ms, or
    |sent - measured| > 0.25 rad. On abort the script stops sending; the receiver holds the
    last command and opens the hands after 400 ms (its stale-target path).

After a real run:
  scp agi@192.168.2.50:/agibot/data/home/agi/Desktop/mc/logs/trace_orin_<ts>.jsonl logs/   (or mc/ if no logs/ dir)
  venv/bin/python tools/identify_plant.py --source orin --fit logs/trace_orin_<A>.jsonl \
      --val logs/trace_orin_<B>.jsonl --hand-logs logs/plant_excitation_<A>.jsonl
"""
import os
import sys
import json
import time
import socket
import argparse

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

LIMS_MIN = np.array([-2.91, -0.46, -2.91, -2.00, -2.94, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.10, -2.94, -0.30, -0.20])
LIMS_MAX = np.array([2.91, 1.5359, 2.91, -0.10, 2.94, 0.30, 0.20, 2.91, 0.46, 2.91, 2.00, 2.94, 0.30, 0.20])
SPEED_VMAX = {1: 0.40, 2: 0.60, 3: 0.80, 4: 1.00}
WRISTS = {5, 6, 12, 13}
RATE = 100.0


def safe_amp(q0, j, sign, a, clear_fn, min_clear):
    """Largest amplitude <= a (halving) whose swept path q0 -> q0 + sign*amp keeps clearance."""
    while a >= 0.03:
        ok = True
        for frac in np.linspace(0.25, 1.0, 4):
            q = q0.copy(); q[j] += sign * a * frac
            if clear_fn(q) < min_clear:
                ok = False; break
        if ok:
            return a
        a *= 0.5
    return 0.0


def build_trajectory(q0, joints, amp, vmax, hands, clear_fn=None, min_clear=0.04):
    """Returns list of (q14, hand12, engaged, label) at RATE Hz."""
    traj = []
    hold = lambda q, h, s, lab: traj.extend([(q.copy(), h.copy(), (True, True), lab)] * int(s * RATE))  # noqa: E731
    h0 = np.zeros(12)
    hold(q0, h0, 1.0, "settle")
    for j in joints:
        a = amp * (0.6 if j in WRISTS else 1.0)
        up, dn = min(a, LIMS_MAX[j] - 0.05 - q0[j]), min(a, q0[j] - (LIMS_MIN[j] + 0.05))
        if clear_fn is not None:
            up, dn = safe_amp(q0, j, +1, up, clear_fn, min_clear), safe_amp(q0, j, -1, dn, clear_fn, min_clear)
            if up < a or dn < a:
                print(f"[excite] J{j}: amplitude limited by clearance -> +{up:.3f} / -{dn:.3f} rad")
        for target, lab in ((up, "step+"), (0.0, "back"), (-dn, "step-"), (0.0, "back")):
            if lab.startswith("step") and abs(target) < 0.02:
                continue
            q = q0.copy(); q[j] += target
            hold(q, h0, abs(target) / (vmax * 0.6) + 1.5 if target else 1.5 + a / (vmax * 0.6), f"J{j}:{lab}")
        # chirp 0.2 -> 3 Hz, 15 s, peak velocity <= 0.6 vmax
        T, f0, f1 = 15.0, 0.2, 3.0
        n = int(T * RATE)
        t = np.arange(n) / RATE
        f = f0 * (f1 / f0) ** (t / T)
        phase = 2 * np.pi * f0 * T / np.log(f1 / f0) * ((f1 / f0) ** (t / T) - 1.0)
        # chirp around the centre of the cleared interval [q0-dn, q0+up] (one-sided if needed)
        c_off, c_amp = (up - dn) / 2.0, (up + dn) / 2.0
        if c_amp < 0.02:
            print(f"[excite] J{j}: chirp skipped (cleared interval < 40 mrad)")
            hold(q0, h0, 0.5, f"J{j}:rest"); continue
        qc = q0.copy(); qc[j] += c_off
        if abs(c_off) > 1e-6:
            hold(qc, h0, abs(c_off) / (vmax * 0.6) + 1.0, f"J{j}:to_centre")
        amp_t = np.minimum(c_amp, 0.6 * vmax / (2 * np.pi * f))
        ramp = np.clip(np.minimum(t, T - t) / 1.0, 0.0, 1.0)
        for k in range(n):
            q = qc.copy(); q[j] += amp_t[k] * ramp[k] * np.sin(phase[k])
            traj.append((q, h0.copy(), (True, True), f"J{j}:chirp"))
        hold(q0, h0, 1.0, f"J{j}:rest")
    for side in hands:
        sl = slice(0, 6) if side == "left" else slice(6, 12)
        for lo, hi in ((0, 400), (400, 800), (800, 0)):
            for k in range(int(2.0 * RATE)):
                h = np.zeros(12); h[sl] = lo + (hi - lo) * k / (2.0 * RATE)
                traj.append((q0.copy(), h, (True, True), f"hand_{side}:{lo}->{hi}"))
            h = np.zeros(12); h[sl] = hi
            traj.extend([(q0.copy(), h, (True, True), f"hand_{side}:hold{hi}")] * int(1.5 * RATE))
    hold(q0, h0, 1.0, "end")
    return traj


def make_checker():
    import mujoco
    from teleop.safety.armor_collision import ArmorCollisionChecker
    model = mujoco.MjModel.from_xml_path(os.path.join(REPO, "assets/agibot_a2_ultra/mjcf/scene.xml"))
    return ArmorCollisionChecker(model, config_path=os.path.join(REPO, "config/safety.yaml"))


def verify_offline(traj, min_clear_m, chk):
    worst, worst_lab = 9.9, ""
    for k in range(0, len(traj), 5):
        q, _, _, lab = traj[k]
        if np.any(q < LIMS_MIN + 0.049) or np.any(q > LIMS_MAX - 0.049):
            return False, f"joint limit margin violated at {lab}"
        d, pair, _ = chk.compute_min_distance(q)
        if d < worst:
            worst, worst_lab = d, f"{lab} ({pair})"
    return worst >= min_clear_m, f"min armor clearance {worst*1000:.0f} mm at {worst_lab}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot-ip", default="127.0.0.1")
    ap.add_argument("--speed", type=int, default=1, choices=[1, 2, 3, 4], help="must equal the receiver --speed")
    ap.add_argument("--joints", default="0,1,2,3,4,5,6,7,8,9,10,11,12,13")
    ap.add_argument("--amp", type=float, default=0.10, help="rad (wrists use 60 %%)")
    ap.add_argument("--hands", default="left,right", help="'' to skip")
    ap.add_argument("--min-clearance-mm", type=float, default=40.0)
    ap.add_argument("--i-have-estop", action="store_true")
    ap.add_argument("--plan-only", action="store_true", help="build + verify, do not send")
    a = ap.parse_args()

    real = a.robot_ip not in ("127.0.0.1", "localhost")
    if real and not a.i_have_estop and not a.plan_only:
        print("REFUSED: real robot requires --i-have-estop (second person on the physical E-stop)."); return 2

    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rx.bind(("0.0.0.0", 5006))
    except OSError:
        print("REFUSED: UDP 5006 busy - stop main.py first (single sender)."); return 2
    rx.settimeout(0.05)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fb, fb_t = None, 0.0
    t_end = time.time() + 3.0
    while time.time() < t_end and fb is None:
        try:
            d, _ = rx.recvfrom(4096); fb, fb_t = json.loads(d.decode()), time.time()
        except socket.timeout:
            pass
    if fb is None:
        print(f"REFUSED: no feedback from receiver at {a.robot_ip} within 3 s."); return 2
    if fb.get("speed_level") != a.speed:
        print(f"REFUSED: receiver speed {fb.get('speed_level')} != --speed {a.speed}."); return 2
    q0 = np.array(fb["q"], dtype=np.float64)
    print(f"[excite] receiver sha={fb.get('sha')} speed={fb.get('speed_level')} rx_state={fb.get('rx_state')}")
    print(f"[excite] start pose: {np.round(q0, 3).tolist()}")

    joints = [int(x) for x in a.joints.split(",") if x.strip()]
    hands = [h for h in a.hands.split(",") if h.strip()]
    vmax = SPEED_VMAX[a.speed]
    chk = make_checker()
    clear_fn = lambda q: chk.compute_min_distance(q)[0]  # noqa: E731
    d0 = clear_fn(q0)
    if d0 < a.min_clearance_mm / 1000.0:
        print(f"REFUSED: start pose clearance {d0*1000:.0f} mm < {a.min_clearance_mm:.0f} mm"); return 3
    traj = build_trajectory(q0, joints, a.amp, vmax, hands, clear_fn, a.min_clearance_mm / 1000.0)
    ok, msg = verify_offline(traj, a.min_clearance_mm / 1000.0, chk)
    print(f"[excite] plan: {len(traj)/RATE:.0f} s, joints {joints}, hands {hands} | offline check: {msg} -> {'OK' if ok else 'REJECTED'}")
    if not ok:
        return 3
    if a.plan_only:
        return 0
    if real:
        print("[excite] REAL ROBOT. E-stop in hand? Starting in 5 s (Ctrl+C to cancel)...")
        time.sleep(5.0)

    os.makedirs(os.path.join(REPO, "logs"), exist_ok=True)
    path = os.path.join(REPO, "logs", f"plant_excitation_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    sid = f"excite_{os.getpid()}_{int(time.time())}"
    abort = ""
    with open(path, "w", buffering=1) as log:
        log.write(json.dumps({"event": "START", "robot_ip": a.robot_ip, "speed": a.speed, "q0": q0.tolist(),
                              "joints": joints, "hands": hands, "amp": a.amp, "receiver_sha": fb.get("sha")}) + "\n")
        rx.setblocking(False)
        t0 = time.perf_counter()
        try:
            for k, (q, h, eng, lab) in enumerate(traj):
                pkt = {"seq": k + 1, "t": time.time(), "session_id": sid, "proto_ver": 3,
                       "q": [float(round(x, 5)) for x in q], "engaged": list(eng), "hand": [int(round(x)) for x in h]}
                tx.sendto(json.dumps(pkt).encode(), (a.robot_ip, 5005))
                while True:  # drain all pending feedback, keep newest
                    try:
                        d, _ = rx.recvfrom(4096)
                    except (BlockingIOError, socket.timeout):
                        break
                    fb, fb_t = json.loads(d.decode()), time.time()
                meas = np.array(fb["q"])
                log.write(json.dumps({"t": time.time(), "label": lab, "sent_q": pkt["q"], "meas_q": fb["q"],
                                      "hand_cmd": pkt["hand"], "hand_meas": fb.get("hand"), "rx_state": fb.get("rx_state"),
                                      "fb_age_ms": round((time.time() - fb_t) * 1000, 1)}) + "\n")
                st = str(fb.get("rx_state", ""))
                if k > 50 and (st.startswith("FAULT") or st == "HOLDING" or st.startswith("REJECT")):
                    abort = f"receiver state {st}"; break
                if time.time() - fb_t > 0.300:
                    abort = "feedback stale > 300 ms"; break
                if float(np.max(np.abs(meas - q))) > 0.25 + a.amp:
                    abort = "|sent - measured| too large"; break
                if k % 500 == 0:
                    print(f"[excite] {k/RATE:6.1f}/{len(traj)/RATE:.0f} s  {lab:<18} rx={st}", flush=True)
                nxt = t0 + (k + 1) / RATE
                while time.perf_counter() < nxt:
                    time.sleep(0.0005)
        except KeyboardInterrupt:
            abort = "Ctrl+C"
        log.write(json.dumps({"event": "END", "abort": abort}) + "\n")
    print(f"[excite] {'ABORTED: ' + abort if abort else 'done'} -> {path}")
    print("[excite] stopped sending; receiver holds last command, hands open after 400 ms.")
    return 1 if abort else 0


if __name__ == "__main__":
    sys.exit(main())
