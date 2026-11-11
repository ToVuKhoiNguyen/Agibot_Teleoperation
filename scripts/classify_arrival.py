#!/usr/bin/env python3
"""scripts/classify_arrival.py - Classifies arrival oscillation origin across laptop, orin, recorder."""
import sys, json, math
import numpy as np

def compute_fft_dominant(series, dt):
    n = len(series)
    if n < 8: return 0.0, 0.0
    detrended = series - np.mean(series)
    fft_vals = np.abs(np.fft.rfft(detrended))
    freqs = np.fft.rfftfreq(n, d=dt)
    if len(fft_vals) <= 1: return 0.0, 0.0
    idx = np.argmax(fft_vals[1:]) + 1
    return float(freqs[idx]), float(np.ptp(series))

def load_jsonl(path):
    records = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try: records.append(json.loads(line))
            except Exception: pass
    return records

def estimate_act_decay(series, dt):
    # Splits series into two halves and compares std
    n = len(series)
    if n < 20: return "persist", 999.0
    h1 = np.std(series[:n//2])
    h2 = np.std(series[n//2:])
    if h1 < 1e-5: return "persist", 999.0
    ratio = h2 / h1
    if ratio < 0.6:
        tau = -(n * dt * 0.5) / math.log(ratio)
        return "decay", tau
    return "persist", 999.0

def main():
    if len(sys.argv) < 4:
        print("Usage: scripts/classify_arrival.py <laptop.jsonl> <orin.jsonl> <recorder.jsonl>")
        sys.exit(1)
    lap_rows, orin_rows, rec_rows = load_jsonl(sys.argv[1]), load_jsonl(sys.argv[2]), load_jsonl(sys.argv[3])
    
    # 1. Find arrival seq in laptop
    arr_seq, arr_t = None, None
    for r in lap_rows:
        if r.get("event") == "ARRIVAL":
            arr_seq, arr_t = r.get("seq"), r.get("t")
            break
    if arr_seq is None:
        for r in lap_rows:
            if r.get("arrived", False):
                arr_seq, arr_t = r.get("seq"), r.get("t")
                break
    if arr_seq is None:
        print("[WARN] No ARRIVAL event or arrived flag found in laptop log; using last 600 ticks.")
        lap_win = [r for r in lap_rows if "seq" in r and "sent_q14" in r][-600:]
    else:
        lap_win = [r for r in lap_rows if "seq" in r and "sent_q14" in r and arr_seq <= r["seq"] <= arr_seq + 600]

    if not lap_win:
        print("Error: Empty laptop window.")
        sys.exit(1)

    t0_lap = lap_win[0]["t"]
    t1_lap = lap_win[-1]["t"]
    seq_min = lap_win[0]["seq"]
    seq_max = lap_win[-1]["seq"]

    # 2. Extract Orin window by seqs_consumed or time
    orin_win = []
    for r in orin_rows:
        sc = r.get("seqs_consumed", [])
        if any(seq_min <= s <= seq_max for s in sc):
            orin_win.append(r)
    if not orin_win:
        orin_ticks = [r for r in orin_rows if "t" in r and "published_q14" in r]
        orin_win = orin_ticks[-len(lap_win):] if len(orin_ticks) >= len(lap_win) else orin_ticks

    # 3. Extract Recorder window (CMD and STATE)
    rec_cmds = [r for r in rec_rows if r.get("type") == "CMD"]
    rec_states = [r for r in rec_rows if r.get("type") == "STATE"]
    max_pubs = max([r.get("pub_count", 1) for r in rec_cmds] or [1])

    # Synchronize recorder by aligning duration
    rec_win_cmd = rec_cmds[-len(lap_win):] if len(rec_cmds) >= len(lap_win) else rec_cmds
    rec_win_state = rec_states[-len(lap_win):] if len(rec_states) >= len(lap_win) else rec_states

    # Compute per-joint ptp and frequencies
    dt = 0.01
    sent_arr = np.array([r["sent_q14"] for r in lap_win])
    pub_arr = np.array([r["published_q14"] for r in orin_win]) if orin_win else sent_arr
    rec_cmd_arr = np.array([r["q14"] for r in rec_win_cmd]) if rec_win_cmd else pub_arr
    act_arr = np.array([r["q14"] for r in rec_win_state]) if rec_win_state else (
        np.array([r["measured_q14"] for r in orin_win if r.get("measured_q14")]) if orin_win else None
    )

    stats = []
    verdicts_per_joint = []
    for j in range(14):
        f_s, ptp_s = compute_fft_dominant(sent_arr[:, j], dt)
        f_p, ptp_p = compute_fft_dominant(pub_arr[:, j], dt) if len(pub_arr) > 0 else (0, 0)
        f_rc, ptp_rc = compute_fft_dominant(rec_cmd_arr[:, j], dt) if len(rec_cmd_arr) > 0 else (0, 0)
        f_a, ptp_a = compute_fft_dominant(act_arr[:, j], dt) if act_arr is not None and len(act_arr) > 0 else (0, 0)
        stats.append((j, ptp_s, f_s, ptp_p, f_p, ptp_rc, f_rc, ptp_a, f_a))

        if ptp_s > 0.003:
            verdicts_per_joint.append("V1")
        elif ptp_p > 0.003:
            verdicts_per_joint.append("V2")
        elif ptp_rc > 0.003 or max_pubs > 1:
            verdicts_per_joint.append("V3")
        elif ptp_a > 0.003:
            verdicts_per_joint.append("V4")
        else:
            verdicts_per_joint.append("CLEAN")

    # Overall Verdict
    unique_v = set(v for v in verdicts_per_joint if v != "CLEAN")
    if len(unique_v) == 0:
        verdict = "V4 all commands constant (ptp<0.001) but ACT oscillates -> robot inner loop/mechanical"
    elif len(unique_v) == 1:
        v = list(unique_v)[0]
        if v == "V1": verdict = "V1 SENT still oscillates after arrival (ptp>0.003 rad) -> software command circling"
        elif v == "V2": verdict = "V2 SENT constant but PUBLISHED oscillates -> Orin playout/interpolation"
        elif v == "V3": verdict = "V3 PUBLISHED constant but RECORDED command oscillates, or publisher count>1 -> second publisher"
        elif v == "V4": verdict = "V4 all commands constant (ptp<0.001) but ACT oscillates -> robot inner loop/mechanical"
        else: verdict = "CLEAN"
    else:
        verdict = f"V5 mixed: {dict((v, [j for j, vj in enumerate(verdicts_per_joint) if vj == v]) for v in unique_v)}"

    print("VERDICT:", verdict)
    
    # ACT decay vs persist
    if act_arr is not None and len(act_arr) > 0:
        worst_j = int(np.argmax([s[7] for s in stats]))
        decay_type, tau = estimate_act_decay(act_arr[:, worst_j], dt)
        print(f"ACT on J{worst_j}: {decay_type.upper()} (tau={tau:.2f}s, ptp={stats[worst_j][7]:.5f} rad)")

    # Packet timing stats
    t_pkt = np.array([r["t"] for r in lap_win])
    dts = np.diff(t_pkt)
    seqs = [r["seq"] for r in lap_win]
    gaps = sum(1 for i in range(len(seqs)-1) if seqs[i+1] - seqs[i] > 1)
    dups = sum(1 for i in range(len(seqs)-1) if seqs[i+1] == seqs[i])
    print(f"Packet Timing (window {len(lap_win)} pkts): dt_mean={np.mean(dts)*1000:.2f}ms, dt_std={np.std(dts)*1000:.2f}ms, gaps={gaps}, duplicates={dups}")

    # Top 6 worst joints table
    stats.sort(key=lambda x: max(x[1], x[3], x[5], x[7]), reverse=True)
    print("\n--- Top 6 Worst Joints Table (ptp in rad, f in Hz) ---")
    print(f"{'Joint':<6} | {'SENT ptp':<10} | {'PUB ptp':<10} | {'REC ptp':<10} | {'ACT ptp':<10} | {'Dom Freq':<8}")
    for s in stats[:6]:
        j, ptp_s, f_s, ptp_p, f_p, ptp_rc, f_rc, ptp_a, f_a = s
        dom_f = max([f_s, f_p, f_rc, f_a], key=lambda f: f)
        print(f"J{j:<5} | {ptp_s:<10.5f} | {ptp_p:<10.5f} | {ptp_rc:<10.5f} | {ptp_a:<10.5f} | {dom_f:<8.1f}")

if __name__ == "__main__":
    main()
