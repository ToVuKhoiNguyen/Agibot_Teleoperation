#!/usr/bin/env python3
"""
scripts/analyze_handrec.py
Offline analysis tool for WebXR hand tracking logs (logs/handrec_<time>.json).
Evaluates tracking lag, steady-state error, gain, drift, clamping, and
per-finger flexion/gating metrics to produce actionable diagnostic verdicts.
"""

import sys
import json
import glob
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def find_latest_log() -> Path:
    logs = sorted(glob.glob(str(PROJECT_ROOT / "logs" / "handrec_*.json")))
    if not logs:
        print("[ERROR] No handrec_*.json found in logs/ directory.")
        sys.exit(1)
    return Path(logs[-1])


def analyze(file_path: Path):
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    frames = data.get("frames", [])
    if not frames:
        print(f"[WARN] No frames recorded in {file_path.name}")
        return

    print("=" * 80)
    print(f"HAND TRACKING OFFLINE ANALYSIS: {file_path.name}")
    print(f"Total Frames: {len(frames)} | Metadata: {data.get('metadata', {})}")
    print("=" * 80)

    # Separate by side
    for side in ("right", "left"):
        side_frames = [f for f in frames if f.get("side") == side and f.get("valid", True)]
        if not side_frames:
            continue

        print(f"\n>>> SIDE: {side.upper()} ({len(side_frames)} frames)")
        clutched_frames = [f for f in side_frames if f.get("clutch")]

        # 1. TRACKING ANALYSIS
        times = np.array([f["timestamp"] for f in side_frames])
        dt = np.median(np.diff(times)) if len(times) > 1 else 0.0166

        raw_pos = np.array([f["wrist_raw"]["pos"] for f in side_frames])
        filt_pos = np.array([f["wrist_filtered"]["pos"] for f in side_frames])
        tgt_pos = np.array([f["p_target"]["pos"] for f in side_frames])
        act_pos = np.array([f["actual_site"]["pos"] for f in side_frames])

        # Cross-correlation lag (tgt vs act)
        lag_ms = 0.0
        filt_delay_ms = 0.0
        if len(clutched_frames) > 30:
            c_tgt = np.array([f["p_target"]["pos"] for f in clutched_frames])
            c_act = np.array([f["actual_site"]["pos"] for f in clutched_frames])
            # axis with maximum motion
            axis = int(np.argmax(np.var(c_tgt, axis=0)))
            tgt_sig = c_tgt[:, axis] - np.mean(c_tgt[:, axis])
            act_sig = c_act[:, axis] - np.mean(c_act[:, axis])
            if np.std(tgt_sig) > 1e-4 and np.std(act_sig) > 1e-4:
                corr = np.correlate(tgt_sig, act_sig, mode="full")
                shift = np.argmax(corr) - (len(tgt_sig) - 1)
                lag_ms = float(shift * dt * 1000.0)

            # Filter delay: raw vs filt
            raw_sig = raw_pos[:, axis] - np.mean(raw_pos[:, axis])
            filt_sig = filt_pos[:, axis] - np.mean(filt_pos[:, axis])
            if np.std(raw_sig) > 1e-4 and np.std(filt_sig) > 1e-4:
                corr_f = np.correlate(raw_sig, filt_sig, mode="full")
                shift_f = np.argmax(corr_f) - (len(raw_sig) - 1)
                filt_delay_ms = float(shift_f * dt * 1000.0)

        # Steady-state pos & rot error
        pos_errs = [f["ik_residual"] * 1000.0 for f in clutched_frames]
        ss_pos_err = float(np.mean(pos_errs)) if pos_errs else 0.0

        # Gain (slope actual / commanded)
        gain = 1.0
        if len(clutched_frames) > 20:
            std_tgt = np.std(c_tgt, axis=0)
            std_act = np.std(c_act, axis=0)
            valid_axes = std_tgt > 0.005
            if np.any(valid_axes):
                gain = float(np.mean(std_act[valid_axes] / std_tgt[valid_axes]))

        # Clamping
        n_clamped_wl = sum(1 for f in side_frames if f["clamp_flags"].get("workspace"))
        n_clamped_jl = sum(1 for f in side_frames if f["clamp_flags"].get("joint_limit"))
        n_clamped_col = sum(1 for f in side_frames if f["clamp_flags"].get("collision"))
        pct_wl = (n_clamped_wl / len(side_frames)) * 100.0
        pct_jl = (n_clamped_jl / len(side_frames)) * 100.0
        pct_col = (n_clamped_col / len(side_frames)) * 100.0

        # Offset effect: target motion when wrist linear motion is low but angular is high
        offset_effect_mm = 0.0
        if len(clutched_frames) > 10:
            diff_w = np.linalg.norm(np.diff(raw_pos, axis=0), axis=1)
            still_indices = np.where(diff_w < 0.002)[0]
            if len(still_indices) > 0:
                tgt_diffs = np.linalg.norm(tgt_pos[still_indices + 1] - tgt_pos[still_indices], axis=1)
                offset_effect_mm = float(np.max(tgt_diffs) * 1000.0)

        print("1) TRACKING QUALITY:")
        print(f"   Lag (target -> actual)       : {lag_ms:.1f} ms")
        print(f"   Filter delay contribution    : {filt_delay_ms:.1f} ms")
        print(f"   Steady-state pos error       : {ss_pos_err:.2f} mm")
        print(f"   Tracking gain                : {gain:.3f} (ideal: 1.0)")
        print(f"   Wrist-vs-palm offset movement: {offset_effect_mm:.2f} mm")
        print(f"   Clamped time                 : Workspace: {pct_wl:.1f}%, Joint Limits: {pct_jl:.1f}%, Collision: {pct_col:.1f}%")

        # 2. FINGER ANALYSIS
        finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        raw_flex = np.array([f["flexion"]["raw"] for f in side_frames])
        cmd_flex = np.array([f["flexion"]["commanded"] for f in side_frames])
        sim_flex = np.array([f["flexion"]["actual_sim"] for f in side_frames])
        cal = side_frames[0].get("calibration_used", {})
        open_cal = cal.get("open_angles", [0.15, 0.1, 0.1, 0.1, 0.1])
        close_cal = cal.get("closed_angles", [1.7, 2.3, 2.35, 2.3, 2.2])

        print("\n2) FINGERS PER FINGER:")
        bad_cal = False
        wrong_chain = False
        clutch_gated = False

        for i, name in enumerate(finger_names):
            r_min, r_max = float(np.min(raw_flex[:, i])), float(np.max(raw_flex[:, i]))
            cal_range = close_cal[i] - open_cal[i]
            cmd_min, cmd_max = float(np.min(cmd_flex[:, i + 1])), float(np.max(cmd_flex[:, i + 1]))
            sim_min, sim_max = float(np.min(sim_flex[:, i + 1])), float(np.max(sim_flex[:, i + 1]))

            # Correlation raw vs commanded
            corr_val = 0.0
            if np.std(raw_flex[:, i]) > 1e-4 and np.std(cmd_flex[:, i + 1]) > 1e-4:
                corr_val = float(np.corrcoef(raw_flex[:, i], cmd_flex[:, i + 1])[0, 1])

            if cal_range < np.radians(30):
                bad_cal = True
            if r_max < 1.8:  # 3 segments flexed should reach > 2.5 rad
                wrong_chain = True

            print(f"   {name:<6}: Raw [{r_min:.2f}..{r_max:.2f}] rad | Cal Range: {cal_range:.2f} rad | "
                  f"Cmd: [{cmd_min:.2f}..{cmd_max:.2f}] rad | Sim: [{sim_min:.2f}..{sim_max:.2f}] rad | Corr: {corr_val:.2f}")

        # Check clutch gating: were fingers commanded 0 while user flexed when clutch was OFF?
        unclutched = [f for f in side_frames if not f.get("clutch")]
        if unclutched:
            u_raw = np.array([f["flexion"]["raw"] for f in unclutched])
            u_cmd = np.array([f["flexion"]["commanded"] for f in unclutched])
            if np.max(u_raw) > 1.0 and np.max(u_cmd) < 0.05:
                clutch_gated = True

        # VERDICT LINES
        print("\n--- DIAGNOSTIC VERDICTS ---")
        causes_1 = []
        if lag_ms > 60.0 or filt_delay_ms > 25.0:
            causes_1.append("filter lag")
        if offset_effect_mm > 5.0:
            causes_1.append("wrist-vs-palm offset")
        if pct_wl > 10.0 or pct_jl > 10.0:
            causes_1.append("workspace/limit clamp")
        if abs(gain - 1.0) > 0.08:
            causes_1.append("gain mismatch / drift")

        causes_2 = []
        if wrong_chain:
            causes_2.append("wrong joints in formula")
        if clutch_gated:
            causes_2.append("clutch gating")
        if bad_cal:
            causes_2.append("bad/absent calibration")
        if pct_col > 10.0:
            causes_2.append("collision gate")

        v1 = ", ".join(causes_1) if causes_1 else "none (within spec)"
        v2 = ", ".join(causes_2) if causes_2 else "none (within spec)"
        print(f"Issue 1 (Robot does not follow 1:1) : Proven by [{v1}]")
        print(f"Issue 2 (Fingers do not open/close) : Proven by [{v2}]")


if __name__ == "__main__":
    target_file = Path(sys.argv[1]) if len(sys.argv) > 1 else find_latest_log()
    analyze(target_file)

