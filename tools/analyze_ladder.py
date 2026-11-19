#!/usr/bin/env python3
"""Offline Analyzer for Real-Robot Teleop Latency Ladder and Quest Rung D Logs.

Usage:
    python tools/analyze_ladder.py logs/ladder_20261108_xxxxxx.json
    python tools/analyze_ladder.py logs/real_20261106_165245.jsonl [--cap-step 0.010]
"""

import argparse
import json
import math
import sys
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.real_latency_ladder import compute_cross_correlation_lag_ms, load_max_step_config, print_ladder_table

JOINT_NAMES = [
    "J1-L (Shoulder Pitch)", "J2-L (Shoulder Roll)", "J3-L (Shoulder Yaw)",
    "J4-L (Elbow Pitch)", "J5-L (Wrist Yaw)", "J6-L (Wrist Roll)", "J7-L (Wrist Pitch)",
    "J1-R (Shoulder Pitch)", "J2-R (Shoulder Roll)", "J3-R (Shoulder Yaw)",
    "J4-R (Elbow Pitch)", "J5-R (Wrist Yaw)", "J6-R (Wrist Roll)", "J7-R (Wrist Pitch)"
]


def analyze_ladder_json(json_path: Path):
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    results = data.get("results", [])
    print(f"\n[PHÂN TÍCH LOG THANG ĐO] Tệp: {json_path.name} | Thời gian: {data.get('timestamp', 'N/A')}")
    if results:
        print_ladder_table(results)
    else:
        print("Không tìm thấy kết quả hợp lệ trong tệp.")


def analyze_quest_jsonl(jsonl_path: Path, max_step: float):
    cap_speed = max_step * 50.0  # rad/s at 50 Hz
    records = []
    with open(jsonl_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try:
                records.append(json.loads(line))
            except Exception:
                continue

    if not records:
        print(f"[LỖI] Tệp rỗng hoặc không có bản ghi hợp lệ: {jsonl_path}")
        return

    t_laptops = np.array([r["t_laptop"] for r in records])
    target_q = np.array([r["target_q"] for r in records])
    measured_q = np.array([r["measured_q"] for r in records])
    meas_arrival_t = np.array([r.get("measured_arrival_t", 0.0) for r in records])

    # Time steps
    dts = np.diff(t_laptops)
    valid_dts = dts[(dts > 0.005) & (dts < 0.100)]
    dt_nom = float(np.median(valid_dts)) if len(valid_dts) > 0 else 0.02

    # Identify joints with motion (std > 0.01 rad)
    std_targets = np.std(target_q, axis=0)
    std_meas = np.std(measured_q, axis=0)

    # Rung D Overall Latency & Tracking
    rung_d_results = []
    for j_idx in [3, 10]:  # J4-L and J4-R for standardized comparison
        c = target_q[:, j_idx]
        m = measured_q[:, j_idx]
        lag = compute_cross_correlation_lag_ms(c, m, dt_ms=dt_nom*1000.0)
        gain = float(np.std(m) / max(1e-6, np.std(c)))
        rms = float(np.sqrt(np.mean((c - m) ** 2)))
        sat_ticks = np.sum(np.abs(c - m) > 0.05)
        sat_ratio = float(sat_ticks / len(records))
        rung_d_results.append({
            "joint": f"J{j_idx+1} ({'L' if j_idx < 7 else 'R'})",
            "lag_ms": lag, "gain": gain, "rms": rms, "sat_ratio": sat_ratio
        })

    # Speed calculation per joint
    dq = np.diff(measured_q, axis=0)
    dt_matrix = np.clip(dts[:, None], 0.005, 0.100)
    speeds = np.abs(dq) / dt_matrix  # shape (N-1, 14)

    bins = [0.0, 0.2 * cap_speed, 0.5 * cap_speed, 0.8 * cap_speed, 0.9 * cap_speed, float("inf")]
    bin_labels = ["<20%", "20-50%", "50-80%", "80-90%", ">=90%"]

    print("\n" + "=" * 90)
    print(f"PHÂN TÍCH RUNG D (QUEST HUMAN MOTION) - TỆP: {jsonl_path.name}")
    print(f"Số mẫu: {len(records)} | Thời gian: {t_laptops[-1] - t_laptops[0]:.1f}s | Tốc độ trần (Cap): {cap_speed:.2f} rad/s")
    print("=" * 90)

    print("\n[1] ĐỘ TRỄ VÀ ĐỘ BÁM ĐUỔI KHỚP KHUỶU (J4-L & J4-R):")
    print(f"{'Khớp':<10} | {'Độ trễ (ms)':<12} | {'Độ lợi (Gain)':<14} | {'RMS bám (rad)':<14} | {'Tỷ lệ bão hòa (>0.05 rad)':<25}")
    print("-" * 80)
    for res in rung_d_results:
        print(f"{res['joint']:<10} | {res['lag_ms']:<10.1f}ms | {res['gain']:<14.3f} | {res['rms']:<14.4f} | {res['sat_ratio']*100.0:<23.1f}%")

    # Feedback age
    valid_fb = (meas_arrival_t > 0)
    if np.any(valid_fb):
        fb_ages = (t_laptops[valid_fb] - meas_arrival_t[valid_fb]) * 1000.0
        p95_age = float(np.percentile(fb_ages, 95))
        print(f"\nTuổi gói tin phản hồi 5006 p95: {p95_age:.1f} ms")

    print("\n[2] PHÂN BỐ VẬN TỐC KHỚP VÀ TỶ LỆ TIỆM CẬN TRẦN VẬN TỐC (>= 90% CAP):")
    print(f"{'Khớp':<22} | {'V_max':<8} | {'V_p95':<8} | {'% Time >=90% Cap':<18} | Phân bố Histogram (<20% | 20-50% | 50-80% | 80-90% | >=90%)")
    print("-" * 105)

    for j in range(14):
        v = speeds[:, j]
        v_max = float(np.max(v))
        v_p95 = float(np.percentile(v, 95))
        pct_near_cap = float(np.mean(v >= 0.90 * cap_speed) * 100.0)

        counts, _ = np.histogram(v, bins=bins)
        total = max(1, len(v))
        hist_pcts = [f"{c/total*100.0:4.1f}%" for c in counts]
        hist_str = " | ".join(hist_pcts)

        # Highlight joints with significant movement
        flag = " *" if std_targets[j] > 0.02 else "  "
        print(f"{JOINT_NAMES[j]:<22}{flag} | {v_max:<6.2f} | {v_p95:<6.2f} | {pct_near_cap:<16.2f}% | {hist_str}")

    print("-" * 105)
    print("Ghi chú: (*) Khớp có dịch chuyển đáng kể trong phiên teleop. Trần vận tốc Cap = {:.2f} rad/s.".format(cap_speed))


def main():
    parser = argparse.ArgumentParser(description="Offline Latency Ladder & Quest Rung D Log Analyzer")
    parser.add_argument("log", type=str, help="Path to logs/ladder_*.json or logs/real_*.jsonl")
    parser.add_argument("--cap-step", type=float, default=None, help="Rate limiter max step (default from config)")
    args = parser.parse_args()

    log_path = Path(args.log)
    if not log_path.exists():
        print(f"[LỖI] Tệp không tồn tại: {log_path}")
        sys.exit(1)

    max_step = args.cap_step or load_max_step_config()

    if log_path.suffix == ".jsonl":
        analyze_quest_jsonl(log_path, max_step)
    elif log_path.suffix == ".json":
        analyze_ladder_json(log_path)
    else:
        # Try inspecting content
        with open(log_path, "r", encoding="utf-8") as f:
            first = f.readline()
        if "target_q" in first:
            analyze_quest_jsonl(log_path, max_step)
        else:
            analyze_ladder_json(log_path)


if __name__ == "__main__":
    main()

