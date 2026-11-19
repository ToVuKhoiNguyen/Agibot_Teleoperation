#!/usr/bin/env python3
"""tools/plot_cmd_state.py - Plot command (input) vs measured state (output) from a cmd_recorder trace.

Record on the Orin while arm.py (or arm_teleop_rx) runs:
    python3 cmd_recorder.py            # writes logs/trace_recorder_<ts>.jsonl
Then on the laptop:
    python tools/plot_cmd_state.py logs/trace_recorder_<ts>.jsonl            # arm.py joints (L/R J1, J4)
    python tools/plot_cmd_state.py <trace> --joints all                       # all 14 arm joints
    python tools/plot_cmd_state.py <trace> --joints 0,3,7,10 --t0 5 --t1 30   # time window [s]

Per joint: cmd vs state, tracking error, and the lag that best aligns state to cmd (cross-correlation).
Writes <trace>_cmd_state.png next to the trace (or --out).
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

NAMES = [f"L J{i}" for i in range(1, 8)] + [f"R J{i}" for i in range(1, 8)]


def load(path):
    cmd_t, cmd_q, st_t, st_q = [], [], [], []
    with open(path) as f:
        for line in f:
            try: r = json.loads(line)
            except ValueError: continue
            q = r.get("q14")
            if not q or len(q) != 14: continue
            if r.get("type") == "CMD": cmd_t.append(r["t_recv"]); cmd_q.append(q)
            elif r.get("type") == "STATE": st_t.append(r["t_recv"]); st_q.append(q)
    if not cmd_t or not st_t:
        sys.exit(f"{path}: {len(cmd_t)} CMD / {len(st_t)} STATE records - need both (was a commander running?)")
    t0 = min(cmd_t[0], st_t[0])
    return np.array(cmd_t) - t0, np.array(cmd_q), np.array(st_t) - t0, np.array(st_q)


def estimate_lag(ct, c, st, s, dt=0.005, max_lag=1.0):
    """Shift [s] of state that minimises RMS(cmd(t) - state(t+lag)) on a uniform grid; None if the joint barely moves.
    (RMS-of-shift, not derivative cross-correlation: the cmd stream arrives jittery/stair-stepped.)"""
    if np.ptp(c) < 0.02: return None
    g = np.arange(max(ct[0], st[0]), min(ct[-1], st[-1]), dt)
    cg, sg = np.interp(g, ct, c), np.interp(g, st, s)
    lags = range(0, min(int(max_lag / dt), len(g) // 2))
    rms = [np.mean((cg[: len(g) - k] - sg[k:]) ** 2) for k in lags]
    return lags[int(np.argmin(rms))] * dt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trace")
    ap.add_argument("--joints", default="0,3,7,10", help="indices 0-13 (0-6 left, 7-13 right) or 'all'")
    ap.add_argument("--t0", type=float, default=None); ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    ct, cq, st, sq = load(a.trace)
    lo = a.t0 if a.t0 is not None else -np.inf; hi = a.t1 if a.t1 is not None else np.inf
    cm, sm = (ct >= lo) & (ct <= hi), (st >= lo) & (st <= hi)
    ct, cq, st, sq = ct[cm], cq[cm], st[sm], sq[sm]
    joints = list(range(14)) if a.joints == "all" else [int(x) for x in a.joints.split(",")]

    cmd_hz = (len(ct) - 1) / max(ct[-1] - ct[0], 1e-9); st_hz = (len(st) - 1) / max(st[-1] - st[0], 1e-9)
    fig, axes = plt.subplots(len(joints), 2, figsize=(14, 2.6 * len(joints)), squeeze=False,
                             gridspec_kw={"width_ratios": [3, 1.2]}, sharex="col")
    print(f"{Path(a.trace).name}: cmd {len(ct)} @ {cmd_hz:.0f} Hz, state {len(st)} @ {st_hz:.0f} Hz, {ct[-1]-ct[0]:.1f} s")
    print(f"{'joint':6} {'cmd range':>17} {'state range':>17} {'RMS err':>8} {'max err':>8} {'lag':>7}")
    for row, j in enumerate(joints):
        s_on_c = np.interp(ct, st, sq[:, j])          # state resampled at cmd times
        err = cq[:, j] - s_on_c
        lag = estimate_lag(ct, cq[:, j], st, sq[:, j])
        lag_s = f"{lag*1000:.0f} ms" if lag is not None else "-"
        print(f"{NAMES[j]:6} [{cq[:,j].min():+.3f},{cq[:,j].max():+.3f}] [{sq[:,j].min():+.3f},{sq[:,j].max():+.3f}]"
              f" {np.sqrt(np.mean(err**2)):8.4f} {np.abs(err).max():8.4f} {lag_s:>7}")
        ax = axes[row][0]
        ax.plot(ct, cq[:, j], lw=1.2, label="cmd (input)")
        ax.plot(st, sq[:, j], lw=1.2, label="state (output)")
        ax.set_ylabel(f"{NAMES[j]} [rad]"); ax.grid(alpha=0.3)
        ax.set_title(f"{NAMES[j]}  lag≈{lag_s}  RMS err {np.sqrt(np.mean(err**2)):.4f} rad", fontsize=9, loc="left")
        if row == 0: ax.legend(loc="upper right", fontsize=8)
        ax = axes[row][1]
        ax.plot(ct, err, lw=1.0, color="tab:red"); ax.axhline(0, color="k", lw=0.5)
        ax.set_ylabel("cmd - state"); ax.grid(alpha=0.3)
    axes[-1][0].set_xlabel("t [s]"); axes[-1][1].set_xlabel("t [s]")
    fig.suptitle(f"{Path(a.trace).name} - cmd {cmd_hz:.0f} Hz / state {st_hz:.0f} Hz", fontsize=10)
    fig.tight_layout()
    out = Path(a.out) if a.out else Path(a.trace).with_name(Path(a.trace).stem + "_cmd_state.png")
    fig.savefig(out, dpi=110)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
