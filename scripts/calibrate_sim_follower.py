#!/usr/bin/env python3
"""
scripts/calibrate_sim_follower.py
Read-only calibration data tool:
- Compares measured q (5006 UDP feedback) vs RealLikeFollower sim predictions from real logs
- Reports per-joint steady-state error and torso-induced offset
- Computes recommended margin inflation parameter for predictive collision gating
Offline analysis tool only; NEVER runs on physical robot or publishes ROS packets.
"""

import sys
import json
import argparse
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.kinematics.real_follower import RealLikeFollower, IDENTIFIED_TAU_S


def find_newest_log() -> Path:
    log_dir = PROJECT_ROOT / "logs"
    real_logs = sorted(log_dir.glob("real_*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not real_logs:
        raise FileNotFoundError("No logs/real_*.jsonl files found.")
    return real_logs[0]


def calibrate_sim_follower(log_path: Path, tau_s: float = IDENTIFIED_TAU_S, scaling_mode: str = "per-joint"):
    print(f"================================================================================")
    print(f"CALIBRATION HELPER: Real-Like Follower vs Measured Robot Feedback")
    print(f"Log: {log_path.name} | tau: {tau_s:.3f} s | Limiter: {scaling_mode}")
    print(f"================================================================================")

    targets, measured, timestamps = [], [], []
    with open(log_path, "r") as f:
        for line in f:
            if not line.strip():
                continue
            entry = json.loads(line)
            if "target_q" in entry and "measured_q" in entry:
                targets.append(entry["target_q"])
                measured.append(entry["measured_q"])
                timestamps.append(entry.get("t_laptop", 0.0))

    targets = np.array(targets, dtype=np.float64)
    measured = np.array(measured, dtype=np.float64)
    timestamps = np.array(timestamps, dtype=np.float64)
    n_frames = len(targets)

    if n_frames < 50:
        raise ValueError(f"Log contains insufficient frames ({n_frames} < 50).")

    # Initialize follower anchored to first measured joint position
    follower = RealLikeFollower(max_step_rad=0.01, scaling_mode=scaling_mode, tau_s=tau_s, dt=0.02, initial_q14=measured[0])
    pred_exec = np.zeros_like(measured)

    for i in range(n_frames):
        dt = (timestamps[i] - timestamps[i-1]) if i > 0 else 0.02
        if dt <= 0 or dt > 0.1:
            dt = 0.02
        pred_exec[i] = follower.step(targets[i], dt=dt)

    # Compute errors between simulated follower and actual measured hardware execution
    errors = measured - pred_exec
    abs_errors = np.abs(errors)

    # Detect steady-state frames (target velocity < 0.01 rad/s across adjacent frames)
    target_vel = np.zeros_like(targets)
    target_vel[1:] = np.abs(targets[1:] - targets[:-1]) / 0.02
    steady_mask = np.max(target_vel, axis=1) < 0.01

    joint_names = [f"L_J{i+1}" for i in range(7)] + [f"R_J{i+1}" for i in range(7)]
    print(f"\n{'Joint':<10} | {'Mean Err (rad)':<14} | {'Max Err (rad)':<14} | {'RMS Err (rad)':<14} | {'Steady Err (rad)':<16}")
    print("-" * 75)

    steady_errs = []
    for j in range(14):
        mean_e = float(np.mean(abs_errors[:, j]))
        max_e = float(np.max(abs_errors[:, j]))
        rms_e = float(np.sqrt(np.mean(errors[:, j] ** 2)))
        st_e = float(np.mean(abs_errors[steady_mask, j])) if np.any(steady_mask) else mean_e
        steady_errs.append(st_e)
        print(f"{joint_names[j]:<10} | {mean_e:<14.4f} | {max_e:<14.4f} | {rms_e:<14.4f} | {st_e:<16.4f}")

    # Estimate Cartesian torso-induced offset approximation (arm length ~0.8m)
    # delta_cartesian ~ L_arm * max_steady_err
    max_steady_err = float(np.max(steady_errs))
    est_cartesian_steady_offset_m = max_steady_err * 0.8
    p95_error_rad = float(np.percentile(abs_errors, 95))
    rec_inflation_m = float(np.clip(p95_error_rad * 0.8 + 0.01, 0.02, 0.05))

    print("\n--------------------------------------------------------------------------------")
    print(f"Calibration Summary:")
    print(f"  Frames Analyzed            : {n_frames} ({n_frames * 0.02:.1f} s)")
    print(f"  Max Joint Steady-State Err : {max_steady_err:.4f} rad ({np.rad2deg(max_steady_err):.2f} deg)")
    print(f"  Est Torso Cartesian Offset : {est_cartesian_steady_offset_m * 1000.0:.1f} mm")
    print(f"  Follower Prediction P95 Err: {p95_error_rad:.4f} rad ({np.rad2deg(p95_error_rad):.2f} deg)")
    print(f"  Recommended Margin Inflation: {rec_inflation_m * 1000.0:.1f} mm (safety default: 30.0 mm)")
    print("--------------------------------------------------------------------------------")
    return {
        "max_steady_err_rad": max_steady_err,
        "est_torso_offset_m": est_cartesian_steady_offset_m,
        "p95_error_rad": p95_error_rad,
        "recommended_inflation_m": rec_inflation_m,
    }


def main():
    parser = argparse.ArgumentParser(description="Calibrate Sim Follower vs Real Log")
    parser.add_argument("--log", type=str, default=None, help="Path to real_*.jsonl log file")
    parser.add_argument("--tau", type=float, default=IDENTIFIED_TAU_S, help="Follower time constant tau (s)")
    parser.add_argument("--mode", type=str, default="per-joint", choices=["per-joint", "uniform"], help="Limiter mode")
    args = parser.parse_args()

    log_path = Path(args.log) if args.log else find_newest_log()
    calibrate_sim_follower(log_path, tau_s=args.tau, scaling_mode=args.mode)


if __name__ == "__main__":
    main()
