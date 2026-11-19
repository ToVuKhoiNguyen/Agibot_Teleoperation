"""
tests/test_headless_idle.py
Automated headless idle verification script (<=250 lines).
"""

import time, urllib.request, ssl, json, sys, numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from teleop.utils.server_process import ensure_server_running, stop_spawned_server

CTX = ssl.create_default_context()
CTX.check_hostname = False
CTX.verify_mode = ssl.CERT_NONE


def get_json(url: str) -> dict:
    req = urllib.request.Request(url)
    with urllib.request.urlopen(req, context=CTX, timeout=3.0) as resp:
        return json.loads(resp.read().decode())


def post_json(url: str, data: dict) -> dict:
    req = urllib.request.Request(url, data=json.dumps(data).encode("utf-8"), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, context=CTX, timeout=3.0) as resp:
        return json.loads(resp.read().decode())


def _sample_loop(duration_s: float, on_tick=None):
    try: post_json("https://localhost:8012/api/command", {"cmd": "clear_target_trace"})
    except Exception: pass
    time.sleep(0.1)
    st0 = get_json("https://localhost:8012/api/status")
    pos_l0 = np.array(st0.get("ee_l", [0, 0, 0]), dtype=np.float64)
    pos_r0 = np.array(st0.get("ee_r", [0, 0, 0]), dtype=np.float64)
    t_start, prev_t = time.time(), time.time()
    prev_l, prev_r = pos_l0.copy(), pos_r0.copy()
    max_spd_l = max_spd_r = max_drift_l = max_drift_r = 0.0

    while (time.time() - t_start) < duration_s:
        if on_tick: on_tick()
        time.sleep(0.05)
        now_t = time.time()
        dt = max(1e-4, now_t - prev_t)
        prev_t = now_t
        try:
            st = get_json("https://localhost:8012/api/status")
            ee_l = np.array(st.get("ee_l", prev_l), dtype=np.float64)
            ee_r = np.array(st.get("ee_r", prev_r), dtype=np.float64)
            spd_l = (np.linalg.norm(ee_l - prev_l) / dt) * 1000.0
            spd_r = (np.linalg.norm(ee_r - prev_r) / dt) * 1000.0
            drift_l = np.linalg.norm(ee_l - pos_l0) * 1000.0
            drift_r = np.linalg.norm(ee_r - pos_r0) * 1000.0
            max_spd_l, max_spd_r = max(max_spd_l, spd_l), max(max_spd_r, spd_r)
            max_drift_l, max_drift_r = max(max_drift_l, drift_l), max(max_drift_r, drift_r)
            prev_l, prev_r = ee_l.copy(), ee_r.copy()
        except Exception: pass

    trace_resp = get_json("https://localhost:8012/api/target_trace")
    summary = trace_resp.get("summary", {})
    src_l = summary.get("left", {}).get("sources", ["none"])
    src_r = summary.get("right", {}).get("sources", ["none"])
    return {
        "max_speed_l": max(max_spd_l, summary.get("left", {}).get("max_speed_mm_s", 0.0)),
        "max_speed_r": max(max_spd_r, summary.get("right", {}).get("max_speed_mm_s", 0.0)),
        "drift_l": max_drift_l, "drift_r": max_drift_r, "sources_l": src_l, "sources_r": src_r,
    }


def run_idle_sim_test(duration_s: float = 60.0):
    print(f"\nCASE 1: Headless Idle Test - Default Sim, No Inputs for {duration_s:.1f} s")
    res = _sample_loop(duration_s)
    print(f"CASE 1 RESULT: L spd={res['max_speed_l']:.3f} mm/s, drift={res['drift_l']:.3f} mm, src={res['sources_l']}")
    print(f"               R spd={res['max_speed_r']:.3f} mm/s, drift={res['drift_r']:.3f} mm, src={res['sources_r']}")
    return res


def run_mock_invalid_hands_test(duration_s: float = 10.0):
    print(f"\nCASE 2: Mocked XRHand (valid=False) for {duration_s:.1f} s")
    post_json("https://localhost:8012/api/command", {"cmd": "set_input_mode", "mode": "hands"})
    cmd = {"cmd": "xr_handtrack", "left": {"tracking": False, "confidence": 0.0}, "right": {"tracking": False, "confidence": 0.0}}
    res = _sample_loop(duration_s, on_tick=lambda: post_json("https://localhost:8012/api/command", dict(cmd, t=time.time())))
    print(f"CASE 2 RESULT: L spd={res['max_speed_l']:.3f} mm/s, drift={res['drift_l']:.3f} mm, src={res['sources_l']}")
    print(f"               R spd={res['max_speed_r']:.3f} mm/s, drift={res['drift_r']:.3f} mm, src={res['sources_r']}")
    return res


def run_mock_jitter_hands_test(duration_s: float = 10.0, jitter_mm: float = 2.0):
    print(f"\nCASE 3: Mocked Static Hand (+ {jitter_mm}mm jitter) for {duration_s:.1f} s")
    post_json("https://localhost:8012/api/command", {"cmd": "set_input_mode", "mode": "hands"})
    st0 = get_json("https://localhost:8012/api/status")
    tgt_l0, tgt_r0 = np.array(st0.get("tgt_l", [0.235, 0.504, 1.250])), np.array(st0.get("tgt_r", [0.235, -0.504, 1.250]))
    h_pos, ax = np.array([0.0, 1.5, 0.0]), np.array([[0,0,-1],[-1,0,0],[0,1,0]], dtype=float)
    pw_l = h_pos + np.array([-0.20, -0.20, -0.05]) + ax.T @ (tgt_l0 - np.array([-0.038, 0.191, 1.272]))
    pw_r = h_pos + np.array([0.20, -0.20, -0.05]) + ax.T @ (tgt_r0 - np.array([-0.038, -0.191, 1.272]))
    j_l0, j_r0 = [pw_l.copy() for _ in range(25)], [pw_r.copy() for _ in range(25)]
    jit_l, jit_r = [], []

    def on_tick():
        u = np.random.normal(size=3)
        jit_vec = (u / np.linalg.norm(u)) * np.random.uniform(0, jitter_mm / 1000.0)
        pjl, pjr = pw_l + jit_vec, pw_r + jit_vec
        pkt_l = {"tracking": True, "confidence": 0.95, "wrist": {"pos": pjl.tolist(), "quat": [1, 0, 0, 0]}, "joints": [(j + jit_vec).tolist() for j in j_l0]}
        pkt_r = {"tracking": True, "confidence": 0.95, "wrist": {"pos": pjr.tolist(), "quat": [1, 0, 0, 0]}, "joints": [(j + jit_vec).tolist() for j in j_r0]}
        post_json("https://localhost:8012/api/command", {"cmd": "xr_handtrack", "t": time.time(), "left": pkt_l, "right": pkt_r})
        st = get_json("https://localhost:8012/api/status")
        jit_l.append(np.linalg.norm(np.array(st.get("tgt_l", tgt_l0)) - tgt_l0) * 1000.0)
        jit_r.append(np.linalg.norm(np.array(st.get("tgt_r", tgt_r0)) - tgt_r0) * 1000.0)

    res = _sample_loop(duration_s, on_tick=on_tick)
    res["target_jitter_l"] = float(np.max(jit_l)) if jit_l else 0.0
    res["target_jitter_r"] = float(np.max(jit_r)) if jit_r else 0.0
    print(f"CASE 3 RESULT: L tgt_jitter={res['target_jitter_l']:.3f} mm, spd={res['max_speed_l']:.3f} mm/s, drift={res['drift_l']:.3f} mm")
    print(f"               R tgt_jitter={res['target_jitter_r']:.3f} mm, spd={res['max_speed_r']:.3f} mm/s, drift={res['drift_r']:.3f} mm")
    return res


def main():
    proc, did_spawn = ensure_server_running(timeout_s=20.0)
    try:
        run_idle_sim_test(duration_s=60.0)
        run_mock_invalid_hands_test(duration_s=10.0)
        run_mock_jitter_hands_test(duration_s=10.0, jitter_mm=2.0)
    finally:
        if did_spawn and proc is not None:
            stop_spawned_server(proc)


if __name__ == "__main__":
    main()
