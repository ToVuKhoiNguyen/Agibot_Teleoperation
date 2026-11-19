#!/usr/bin/env python3
"""
tools/identify_tau.py
First-order actuator lag identification (tau) and fit quality from real robot logs.
"""

import sys
import json
import argparse
from pathlib import Path
import numpy as np

ARM_JOINT_NAMES = [
    "Left Arm J1", "Left Arm J2", "Left Arm J3", "Left Arm J4", "Left Arm J5", "Left Arm J6", "Left Arm J7",
    "Right Arm J1", "Right Arm J2", "Right Arm J3", "Right Arm J4", "Right Arm J5", "Right Arm J6", "Right Arm J7"
]

def identify_tau(log_path: Path):
    targets, measured, times = [], [], []
    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            if "target_q" in d and "measured_q" in d:
                targets.append(d["target_q"])
                measured.append(d["measured_q"])
                times.append(d.get("t_laptop", 0.0))

    if len(targets) < 50:
        print(f"Log {log_path.name} has insufficient samples ({len(targets)})")
        return None

    tgt = np.array(targets, dtype=np.float64)
    meas = np.array(measured, dtype=np.float64)
    t = np.array(times, dtype=np.float64)
    dt = np.diff(t)
    valid_dt = dt[dt > 0.005]
    dt_med = float(np.median(valid_dt)) if len(valid_dt) > 0 else 0.02

    results = {}
    taus, r2s = [], []
    for j, name in enumerate(ARM_JOINT_NAMES):
        d_meas = np.diff(meas[:, j])
        err = tgt[:-1, j] - meas[:-1, j]
        mask = (np.abs(err) > 0.005) & (np.abs(d_meas) > 1e-4)
        n_samples = int(np.sum(mask))
        if n_samples > 20:
            denom = float(np.sum(err[mask] ** 2))
            if denom > 1e-9:
                alpha = float(np.sum(err[mask] * d_meas[mask])) / denom
                if alpha > 0:
                    tau = dt_med / alpha
                    pred = alpha * err[mask]
                    res = d_meas[mask] - pred
                    var_y = float(np.var(d_meas[mask]))
                    r2 = 1.0 - (float(np.var(res)) / (var_y + 1e-12))
                    results[name] = {"tau": round(tau, 3), "r2": round(r2, 3), "samples": n_samples}
                    taus.append(tau)
                    r2s.append(r2)

    tau_max = float(np.max(taus)) if taus else None
    tau_mean = float(np.mean(taus)) if taus else None
    r2_mean = float(np.mean(r2s)) if r2s else None
    print(f"File: {log_path.name} | Samples: {len(tgt)} | dt: {dt_med*1000:.1f} ms")
    for name, data in results.items():
        print(f"  {name:14s}: tau = {data['tau']:.3f} s (R^2 = {data['r2']:.3f}, N = {data['samples']})")
    if taus:
        print(f"  Summary: tau_mean = {tau_mean:.3f} s, tau_max = {tau_max:.3f} s, R^2_mean = {r2_mean:.3f}")
    return {"tau_arm": {k: v["tau"] for k, v in results.items()}, "tau_max": tau_max, "tau_mean": tau_mean, "r2_mean": r2_mean}

def main():
    parser = argparse.ArgumentParser(description="Identify actuator lag tau from real robot logs.")
    parser.add_argument("log", nargs="?", default=None, help="Path to real_*.jsonl log file")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent
    if args.log:
        p = Path(args.log)
    else:
        logs = sorted((project_root / "logs").glob("real_*.jsonl"), key=lambda f: f.stat().st_mtime, reverse=True)
        substantive = [f for f in logs if f.stat().st_size > 10000]
        if not substantive:
            print("No real logs found.")
            return
        p = substantive[0]
    identify_tau(p)

if __name__ == "__main__":
    main()

