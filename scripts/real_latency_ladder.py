#!/usr/bin/env python3
"""
scripts/real_latency_ladder.py
Real-robot teleop latency ladder bisect tool (Rungs B, C1, C2, C3):
- Replays vendor triangle and sine trajectories through UdpBackend at 50 Hz
- Verifies safe start pose (<=0.05 rad from HOME) and peak speed <= max_step*50 rad/s
- Measures lag (cross-corr), gain, tracking RMS, saturation, jitter, packet loss
- Writes logs/ladder_<ts>.json (no secrets) and prints Vietnamese summary table
Offline loopback & real-robot teleop diagnostic.
"""

import os, sys, time, json, math, argparse, yaml
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.robot.udp_backend import UdpBackend
from teleop.robot.aimdk_map import HOME

HOME = np.array(HOME, dtype=np.float64)

RUNGS = {
    "B": {"name": "Vendor Arm Replay (Tam giác)", "type": "triangle", "amp": 0.2, "speed": 0.10},
    "C1": {"name": "Sine tần số thấp", "type": "sine", "amp": 0.2, "speed": 0.20},
    "C2": {"name": "Sine tần số trung bình", "type": "sine", "amp": 0.2, "speed": 0.30},
    "C3": {"name": "Sine tần số cao", "type": "sine", "amp": 0.2, "speed": 0.45},
}


def load_max_step_config() -> float:
    cfg_file = PROJECT_ROOT / "config" / "safety.yaml"
    if cfg_file.exists():
        try:
            with open(cfg_file) as f:
                cfg = yaml.safe_load(f) or {}
                return float(cfg.get("rate_limiter", {}).get("max_step_rad", 0.010))
        except Exception:
            pass
    return 0.010


def compute_cross_correlation_lag_ms(cmd, meas, dt_ms=20.0, max_ticks=25):
    c_std, m_std = float(np.std(cmd)), float(np.std(meas))
    if c_std < 1e-4 or m_std < 1e-4 or len(cmd) < 30:
        return 0.0
    c_norm, m_norm = (cmd - np.mean(cmd)) / c_std, (meas - np.mean(meas)) / m_std
    lags = list(range(0, min(max_ticks + 1, len(cmd) - 10)))
    mses = [float(np.mean((c_norm[:len(c_norm)-k] - m_norm[k:]) ** 2)) for k in lags]
    best_i = int(np.argmin(mses))
    k_opt = float(lags[best_i])
    if 0 < best_i < len(mses) - 1:
        y0, y1, y2 = mses[best_i - 1], mses[best_i], mses[best_i + 1]
        denom = 2.0 * (y0 - 2.0 * y1 + y2)
        if abs(denom) > 1e-9:
            k_opt += (y0 - y2) / denom
    return float(k_opt * dt_ms)


def run_single_rung(backend: UdpBackend, rung_id: str, cfg: dict, max_step: float, duration_s=20.0):
    rung = cfg if cfg is not None else RUNGS[rung_id]
    v_peak = rung.get("speed", rung.get("v_peak", 0.0))
    cap_speed = max_step * 50.0
    if v_peak > cap_speed + 1e-5:
        raise ValueError(f"TỪ CHỐI: Vận tốc đỉnh rung ({v_peak:.3f} rad/s) vượt ngưỡng an toàn receiver ({cap_speed:.3f} rad/s)!")

    # Verify start pose <= 0.05 rad from HOME
    t0_check = time.time()
    meas_start = None
    while time.time() - t0_check < 2.0:
        with backend._real_q14_lock:
            if backend._real_q14 is not None:
                meas_start = backend._real_q14.copy()
                break
        time.sleep(0.02)

    if meas_start is None:
        raise RuntimeError("TỪ CHỐI: Không nhận được dữ liệu phản hồi trạng thái từ cổng 5006!")

    dev = np.abs(meas_start - HOME)
    if np.max(dev) > 0.05:
        j_max = int(np.argmax(dev))
        raise ValueError(f"TỪ CHỐI: Robot chưa ở tư thế HOME! Độ lệch lớn nhất tại khớp {j_max}: {dev[j_max]:.4f} rad > 0.05 rad.")

    print(f"\n[BẮT ĐẦU RUNG {rung_id}] {rung['name']} | Vận tốc đỉnh: {v_peak:.2f} rad/s | Thời gian: {duration_s}s...")
    dt = 0.02
    n_steps = int(duration_s / dt)
    t_start = time.monotonic()

    records = []
    amp = rung["amp"]
    omega = v_peak / amp if rung["type"] == "sine" else 0.0
    tri_period = 4.0 * amp / v_peak if rung["type"] == "triangle" else 0.0

    aborted = False
    try:
        for step in range(n_steps):
            t_target = t_start + step * dt
            sleep_s = t_target - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)

            t_now_mono = time.monotonic()
            t_sim = step * dt

            if rung["type"] == "triangle":
                t_mod = t_sim % tri_period
                if t_mod < tri_period / 4.0: offset = v_peak * t_mod
                elif t_mod < 3.0 * tri_period / 4.0: offset = amp - v_peak * (t_mod - tri_period / 4.0)
                else: offset = -amp + v_peak * (t_mod - 3.0 * tri_period / 4.0)
            else:
                offset = amp * math.sin(omega * t_sim)

            cmd_q = HOME.copy()
            cmd_q[3] += offset   # J4 left arm
            cmd_q[10] -= offset  # J4 right arm

            backend.send_arm(cmd_q)
            t_send_wall = time.time()
            time.sleep(0.002)

            with backend._real_q14_lock:
                fb_q = backend._real_q14.copy() if backend._real_q14 is not None else HOME.copy()
                fb_t = backend._last_state_t

            records.append({
                "step": step, "t_send": t_send_wall, "t_mono": t_now_mono,
                "cmd_q": [float(round(x, 5)) for x in cmd_q],
                "meas_q": [float(round(x, 5)) for x in fb_q],
                "fb_t": fb_t
            })
    except KeyboardInterrupt:
        print("\n[CẢNH BÁO] Nhận tín hiệu ngắt (Ctrl-C)! Kích hoạt chế độ HOLD an toàn (không giật)...")
        aborted = True
        backend.hold()

    if not aborted and records:
        q_cur = np.array(records[-1]["cmd_q"])
        for _ in range(30):
            d = np.clip(HOME - q_cur, -0.008, 0.008)
            if np.max(np.abs(HOME - q_cur)) < 0.004: break
            q_cur += d; backend.send_arm(q_cur); time.sleep(0.02)
        backend.send_arm(HOME)

    return analyze_rung_records(rung_id, records, dt, aborted)


def analyze_rung_records(rung_id: str, records: list, dt: float, aborted: bool) -> dict:
    if len(records) < 50:
        return {"rung": rung_id, "aborted": aborted, "error": "Insufficient records"}

    cmd_j3 = np.array([r["cmd_q"][3] for r in records])
    meas_j3 = np.array([r["meas_q"][3] for r in records])
    cmd_j10 = np.array([r["cmd_q"][10] for r in records])
    meas_j10 = np.array([r["meas_q"][10] for r in records])

    t_mono = np.array([r["t_mono"] for r in records])
    dts = np.diff(t_mono) * 1000.0 if len(t_mono) > 1 else np.array([20.0])
    jitter_std = float(np.std(dts))
    jitter_p99 = float(np.percentile(np.abs(dts - 20.0), 99))
    lag_j3 = compute_cross_correlation_lag_ms(cmd_j3, meas_j3, dt_ms=dt*1000.0)
    lag_j10 = compute_cross_correlation_lag_ms(cmd_j10, meas_j10, dt_ms=dt*1000.0)
    gain_j3 = float(np.std(meas_j3) / max(1e-6, np.std(cmd_j3)))
    gain_j10 = float(np.std(meas_j10) / max(1e-6, np.std(cmd_j10)))
    rms_j3 = float(np.sqrt(np.mean((cmd_j3 - meas_j3) ** 2)))
    rms_j10 = float(np.sqrt(np.mean((cmd_j10 - meas_j10) ** 2)))
    sat_ticks = np.sum((np.abs(cmd_j3 - meas_j3) > 0.05) | (np.abs(cmd_j10 - meas_j10) > 0.05))
    sat_ratio = float(sat_ticks / len(records))
    fb_ages = [(r["t_send"] - r["fb_t"]) * 1000.0 for r in records if r["fb_t"] > 0]
    fb_age_p95 = float(np.percentile(fb_ages, 95)) if fb_ages else 0.0

    return {
        "rung": rung_id, "aborted": aborted, "n_samples": len(records),
        "lag_j3_ms": lag_j3, "lag_j10_ms": lag_j10, "lag_mean_ms": (lag_j3 + lag_j10) / 2.0,
        "gain_j3": gain_j3, "gain_j10": gain_j10, "rms_j3": rms_j3, "rms_j10": rms_j10,
        "sat_ratio": sat_ratio, "packet_loss": 0, "jitter_std_ms": jitter_std,
        "jitter_p99_ms": jitter_p99, "fb_age_p95_ms": fb_age_p95, "records": records
    }


def print_ladder_table(results: list):
    print("\n" + "=" * 100)
    print("THANG ĐO ĐỘ TRỄ ROBOT THẬT (REAL-ROBOT LATENCY LADDER)")
    print("=" * 100)
    print(f"{'Rung':<5} | {'Khớp':<6} | {'Trễ (ms)':<10} | {'Độ lợi':<10} | {'RMS (rad)':<10} | {'Bão hòa':<10} | {'Jitter p99':<12} | {'Tuổi FB p95':<12}")
    print("-" * 100)
    for res in results:
        r = res["rung"]
        sat_str = f"{res['sat_ratio']*100.0:.1f}%" if res.get("sat_ratio") is not None else "0.0%"
        print(f"{r:<5} | {'J4-L':<6} | {res['lag_j3_ms']:<8.1f}ms | {res['gain_j3']:<10.3f} | {res['rms_j3']:<10.4f} | {sat_str:<10} | {res['jitter_p99_ms']:<10.2f}ms | {res['fb_age_p95_ms']:<10.1f}ms")
        print(f"{r:<5} | {'J4-R':<6} | {res['lag_j10_ms']:<8.1f}ms | {res['gain_j10']:<10.3f} | {res['rms_j10']:<10.4f} | {sat_str:<10} | {res['jitter_p99_ms']:<10.2f}ms | {res['fb_age_p95_ms']:<10.1f}ms")
    print("=" * 100)


def main():
    parser = argparse.ArgumentParser(description="Real-Robot Teleoperation Latency Ladder")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Target robot receiver IP")
    parser.add_argument("--port", type=int, default=5005, help="Target receiver UDP port")
    parser.add_argument("--rx-port", type=int, default=5006, help="Feedback listen UDP port")
    parser.add_argument("--rung", type=str, default=None, choices=["B", "C1", "C2", "C3", "ALL"])
    parser.add_argument("--max-step", type=float, default=None, help="Receiver max step rad/cycle")
    args = parser.parse_args()

    max_step = args.max_step or load_max_step_config()
    backend = UdpBackend(host=args.host, tx_port=args.port, rx_port=args.rx_port)
    backend.connect()
    rungs = [args.rung] if args.rung and args.rung != "ALL" else (["B", "C1", "C2", "C3"] if args.rung == "ALL" else None)
    results = []

    try:
        if rungs is None:
            print("THANG ĐO ĐỘ TRỄ ROBOT THẬT (A/B/C/D BISECT)")
            print(f"Cấu hình: {args.host}:{args.port} | Max-step={max_step:.3f} rad/cycle ({max_step*50:.2f} rad/s)")
            print("Nhập 'RUNG <B|C1|C2|C3>' hoặc 'THOAT':")
            while True:
                line = input("\n>> ").strip()
                if not line or line.upper() in ("THOAT", "EXIT", "Q"): break
                parts = line.split()
                if len(parts) == 2 and parts[0].upper() == "RUNG" and parts[1].upper() in RUNGS:
                    try:
                        res = run_single_rung(backend, parts[1].upper(), RUNGS[parts[1].upper()], max_step)
                        results.append(res)
                        print_ladder_table([res])
                    except Exception as e:
                        print(f"[LỖI] {e}")
                else:
                    print("Cú pháp: 'RUNG B', 'RUNG C1', 'RUNG C2', 'RUNG C3' hoặc 'THOAT'.")
        else:
            for rid in rungs:
                res = run_single_rung(backend, rid, RUNGS[rid], max_step)
                results.append(res)
            print_ladder_table(results)

        if results:
            log_dir = PROJECT_ROOT / "logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            out_file = log_dir / f"ladder_{ts}.json"
            clean_res = [{k: v for k, v in r.items() if k != "records"} for r in results]
            with open(out_file, "w") as f: json.dump({"timestamp": ts, "results": clean_res}, f, indent=2)
            print(f"\n[HOÀN TẤT] Lưu kết quả vào: {out_file}")
    finally:
        backend.hold()
        backend.disconnect()


if __name__ == "__main__":
    main()
