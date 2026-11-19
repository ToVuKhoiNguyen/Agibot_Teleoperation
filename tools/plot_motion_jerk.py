#!/usr/bin/env python3
"""tools/plot_motion_jerk.py - Is a measured arm motion smooth or jerky? (position / velocity / acceleration from STATE)

Works on any trace with STATE records: trace_armpy_* (arm.py),
trace_recorder_* (cmd_recorder / arm_teleop_rx). CMD records, if present, are overlaid on the position plot;
EVENT "PLAN" records (PlanningMove requests) are drawn as vertical lines and used to cut the trace into moves.

    python tools/plot_motion_jerk.py logs/trace_planning_<ts>.jsonl
    python tools/plot_motion_jerk.py <trace> --joints all --t0 5 --t1 30

Velocity / acceleration: Savitzky-Golay derivatives of the measured position on a 100 Hz grid (window 0.11 s).
Jerkiness metric "ripple %": RMS of (velocity - its 0.4 s smoothed version) while the joint moves, in % of the
peak speed - the high-frequency shake on top of the intended motion (same idea as arm_stream_test.py).
Rough reading: < 3 % smooth, 3-8 % visible ripple, > 8 % jerky. Vendor limits drawn: 3 rad/s, 6.28 rad/s^2.
"""
import argparse, json, sys
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

NAMES = [f"L J{i}" for i in range(1, 8)] + [f"R J{i}" for i in range(1, 8)]
V_LIM, A_LIM, DT = 3.0, 6.28, 0.01


def load(path):
    st, sq, ct, cq, ev, stamps = [], [], [], [], [], []
    for line in open(path):
        try: r = json.loads(line)
        except ValueError: continue
        k = r.get("type")
        if k == "STATE" and len(r.get("q14", [])) == 14: st.append(r["t_recv"]); sq.append(r["q14"]); stamps.append(r.get("stamp"))
        elif k == "CMD" and len(r.get("q14", [])) == 14: ct.append(r["t_recv"]); cq.append(r["q14"])
        elif k == "EVENT": ev.append(r)
    if len(st) < 50: sys.exit(f"{path}: only {len(st)} STATE records")
    # Time base of the measured states = the MC's own stamp, not the arrival time. Arrival on the Orin jitters by tens
    # of ms (p99 ~25 ms, worst 65 ms on a real trace), which turns a smooth motion into fake velocity / acceleration
    # ripple (measured: 1.5 % with the stamp vs 7-11 % with arrival time on the same PlanningMove). The stamp is
    # shifted by the smallest (arrival - stamp) so events and CMD records, which only have arrival time, stay aligned.
    if all(isinstance(x, (int, float)) and x > 0 for x in stamps):
        sa, ra = np.array(stamps, dtype=float), np.array(st, dtype=float)
        if np.all(np.diff(sa) > 0):
            st = list(sa + float(np.min(ra - sa)))
            print(f"{Path(path).name}: time base = MC stamp (arrival jitter p99 {np.percentile(np.diff(ra), 99) * 1000:.0f} ms ignored)")
    else:
        print(f"{Path(path).name}: WARNING no MC stamp in STATE records - using arrival time, ripple / peak a are inflated by receive jitter")
    t0 = st[0]
    ev = [dict(e, t=e["t_recv"] - t0) for e in ev if "t_recv" in e]
    return np.array(st) - t0, np.array(sq), (np.array(ct) - t0 if ct else None), (np.array(cq) if cq else None), ev


def derivs(t, q):
    g = np.arange(t[0], t[-1], DT)
    p = np.interp(g, t, q)
    v = savgol_filter(p, 11, 3, deriv=1, delta=DT)
    a = savgol_filter(p, 11, 3, deriv=2, delta=DT)
    return g, p, v, a


def ripple(v):
    vs = savgol_filter(v, 41, 2)
    moving = np.abs(vs) > 0.05
    peak = np.abs(vs).max()
    if moving.sum() < 20 or peak < 0.05: return None
    return 100.0 * np.sqrt(np.mean((v - vs)[moving] ** 2)) / peak


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trace")
    ap.add_argument("--joints", default="0,3,7,10", help="0-6 left, 7-13 right, or 'all' (default: arm.py's J1/J4)")
    ap.add_argument("--t0", type=float); ap.add_argument("--t1", type=float)
    ap.add_argument("--out")
    a = ap.parse_args()
    st, sq, ct, cq, ev = load(a.trace)
    lo, hi = (a.t0 if a.t0 is not None else -np.inf), (a.t1 if a.t1 is not None else np.inf)
    m = (st >= lo) & (st <= hi); st, sq = st[m], sq[m]
    if ct is not None:
        m = (ct >= lo) & (ct <= hi); ct, cq = ct[m], cq[m]
    joints = list(range(14)) if a.joints == "all" else [int(x) for x in a.joints.split(",")]
    plans = [e for e in ev if e.get("name") == "PLAN" and lo <= e["t"] <= hi]

    fig, ax = plt.subplots(3, len(joints), figsize=(4.2 * len(joints), 9), squeeze=False, sharex=True)
    print(f"{Path(a.trace).name}: state {len(st)} @ {(len(st)-1)/(st[-1]-st[0]):.0f} Hz, {st[-1]-st[0]:.1f} s"
          + (f", cmd {len(ct)}" if ct is not None and len(ct) else "") + (f", {len(plans)} PlanningMove" if plans else ""))
    print(f"{'joint':6} {'travel':>7} {'peak v':>7} {'peak a':>7} {'ripple%':>8}   (v rad/s, a rad/s^2)")
    D = {}
    for c, j in enumerate(joints):
        g, p, v, acc = derivs(st, sq[:, j]); D[j] = (g, v, acc)
        rp = ripple(v)
        print(f"{NAMES[j]:6} {np.ptp(p):7.3f} {np.abs(v).max():7.3f} {np.abs(acc).max():7.2f} "
              f"{(f'{rp:7.1f}' if rp is not None else '      -')}")
        if ct is not None and len(ct): ax[0][c].plot(ct, cq[:, j], lw=1, color="0.6", label="cmd")
        ax[0][c].plot(st, sq[:, j], lw=1.2, label="state"); ax[0][c].set_title(
            f"{NAMES[j]}" + (f"  ripple {rp:.1f}%" if rp is not None else ""), fontsize=10)
        ax[1][c].plot(g, v, lw=1, color="tab:green"); ax[2][c].plot(g, acc, lw=1, color="tab:red")
        for r, lim in ((1, V_LIM), (2, A_LIM)):
            if np.abs(ax[r][c].get_ylim()).max() > 0.5 * lim:
                for s in (1, -1): ax[r][c].axhline(s * lim, color="k", ls="--", lw=0.7)
        for e in plans:
            for r in range(3): ax[r][c].axvline(e["t"], color="tab:purple", lw=0.6, alpha=0.6)
        for r, lab in enumerate(("q [rad]", "v [rad/s]", "a [rad/s²]")):
            ax[r][c].grid(alpha=0.3)
            if c == 0: ax[r][0].set_ylabel(lab)
        ax[2][c].set_xlabel("t [s]")
    ax[0][0].legend(fontsize=8)

    if plans:   # per-move table (from one PLAN to the next)
        print(f"\n{'move':12} {'group':26} {'joint':6} {'dur':>5} {'peak v':>7} {'peak a':>7} {'ripple%':>8}")
        bounds = [e["t"] for e in plans] + [st[-1]]
        for k, e in enumerate(plans):
            grp = e.get("group", "")
            js = [j for j in joints if ("LEFT" in grp and j < 7) or ("RIGHT" in grp and j >= 7) or "DUAL" in grp]
            for j in js:
                g, v, acc = D[j]; w = (g >= bounds[k]) & (g < bounds[k + 1])
                if w.sum() < 20: continue
                mv = np.abs(savgol_filter(v[w], 41, 2)) > 0.02
                dur = (mv.sum() * DT) if mv.any() else 0.0
                rp = ripple(v[w])
                print(f"{e.get('label',''):12} {grp:26} {NAMES[j]:6} {dur:5.2f} {np.abs(v[w]).max():7.3f}"
                      f" {np.abs(acc[w]).max():7.2f} {(f'{rp:7.1f}' if rp is not None else '      -')}")
    fig.suptitle(f"{Path(a.trace).name}  (purple = PlanningMove request, dashed = vendor limit 3 rad/s, 6.28 rad/s²)",
                 fontsize=10)
    fig.tight_layout()
    out = Path(a.out) if a.out else Path(a.trace).with_name(Path(a.trace).stem + "_jerk.png")
    fig.savefig(out, dpi=100)
    print(f"saved {out}")


if __name__ == "__main__":
    main()
