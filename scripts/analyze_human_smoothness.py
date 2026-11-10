#!/usr/bin/env python3
"""Offline analyzer for teleop command smoothness and discontinuity sources."""
import sys, json, numpy as np

def analyze_log(log_path):
    entries = []
    with open(log_path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try: entries.append(json.loads(line))
                except Exception: pass
    if not entries:
        print(f"Empty log: {log_path}")
        return

    t = [e["t_laptop"] for e in entries]
    targets = np.array([e["target_q"] for e in entries])
    dts = np.diff(t)
    dt_valid = dts[dts > 1e-4]
    rates = 1.0 / dt_valid if len(dt_valid) > 0 else np.array([0.0])
    
    deltas = np.diff(targets, axis=0) # shape (N-1, 14)
    max_steps = np.max(np.abs(deltas), axis=1) if len(deltas) > 0 else np.array([0.0])
    speeds = max_steps / np.clip(dts, 1e-4, 1.0) if len(dts) > 0 else np.array([0.0])
    
    jumps_gt_002 = int(np.sum(max_steps > 0.02))
    jumps_gt_005 = int(np.sum(max_steps > 0.05))
    
    # Check IK flips: joint jump > 0.1 rad
    ik_flips = int(np.sum(np.any(np.abs(deltas) > 0.1, axis=1)))
    
    rate_mean = float(np.mean(rates)) if len(rates) > 0 else 0.0
    p99_dt = float(np.percentile(dts, 99)) if len(dts) > 0 else 0.0
    jitter = float(np.std(dts)) if len(dts) > 0 else 0.0
    
    print("=== HUMAN-INPUT SMOOTHNESS ANALYSIS ===")
    print(f"Log: {log_path} ({len(entries)} frames)")
    print(f"Packet rate: {rate_mean:.1f} Hz | Jitter std: {jitter*1000:.2f} ms | p99 dt: {p99_dt*1000:.2f} ms")
    print(f"Max step per packet: {float(np.max(max_steps)):.4f} rad ({float(np.max(max_steps))*180/np.pi:.2f} deg)")
    print(f"Count of steps > 0.02 rad: {jumps_gt_002} | > 0.05 rad: {jumps_gt_005}")
    print(f"IK flips (joint change > 0.1 rad): {ik_flips}")
    print("Speed percentiles (rad/s): p50={:.3f}, p90={:.3f}, p99={:.3f}, max={:.3f}".format(
        float(np.percentile(speeds, 50)), float(np.percentile(speeds, 90)),
        float(np.percentile(speeds, 99)), float(np.max(speeds))
    ))

if __name__ == "__main__":
    p = sys.argv[1] if len(sys.argv) > 1 else "logs/real_20261110_094249.jsonl"
    analyze_log(p)

