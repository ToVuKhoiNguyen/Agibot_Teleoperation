#!/usr/bin/env python3
"""tools/vibration_report.py - How much does the arm shake, and does the shake come from our command or the robot?

Input: a trace recorded ON THE ORIN while teleoperating:
    trace_recorder_<ts>.jsonl  from tools/cmd_recorder.py  (best: every 100 Hz joint state + every published command)
    trace_orin_<ts>.jsonl      from arm_teleop_rx.py       (works, but the state is only sampled at the receiver tick)

    python tools/vibration_report.py logs/trace_recorder_<ts>.jsonl
    python tools/vibration_report.py logs/trace_recorder_A.jsonl logs/trace_recorder_B.jsonl      # compare runs
    python tools/vibration_report.py <trace> --t0 10 --t1 40 --plot                                # a time window + PNG

What it reports, per arm and per speed band (speed = fastest of J1-J5, 0.2 s average):
  shake  = RMS of the velocity left after removing its 0.2 s moving average (rad/s): the 5-50 Hz part, i.e. what you
           see as vibration on top of the intended motion. Computed on the MC's own timestamps (arrival time jitter
           would add fake shake), for the MEASURED joints and for the COMMAND we published.
  freq   = strongest frequency of the measured shake while moving.
  verdict per band: command shake << measured shake -> the robot shakes on a smooth command (robot side / mode 402);
           command shake comparable -> our stream is rough (network, buffer, filter settings).
Reference (real robot, same measure): PlanningMove (MC-generated motion, 16/11) J1 0.017 rad/s at ~0.55 rad/s with the
wrist at 0.0001; teleop 16/11 15:48 (filter on) J1-J5 0.013 slow / 0.034 mid, wrist 0.0005 / 0.003.
"""
import argparse, json, sys
from pathlib import Path

import numpy as np

DT = 0.01
BANDS = [("rest", 0.0, 0.02), ("slow", 0.05, 0.3), ("mid", 0.3, 0.7), ("fast", 0.7, 9.0)]
NAMES = [f"J{i}" for i in range(1, 8)]
REF = {"PlanningMove J1 @0.55 rad/s": 0.017, "PlanningMove wrist": 0.0001}


def ma(x, n):
    return np.convolve(np.pad(x, (n // 2, n - 1 - n // 2), mode="edge"), np.ones(n) / n, mode="valid")


def load(path):
    """-> state (stamp, q), command (stamp, q) and, for each, the receive time on ONE local clock, used only to put
    the two streams on the same time axis (state stamps come from the MC, command stamps from the publisher)."""
    st, sq, ct, cq, sr, cr = [], [], [], [], [], []
    for line in open(path):
        try: r = json.loads(line)
        except ValueError: continue
        if r.get("type") == "STATE" and len(r.get("q14", [])) == 14 and r.get("stamp"):
            st.append(r["stamp"]); sq.append(r["q14"]); sr.append(r.get("t_recv", r["stamp"]))
        elif r.get("type") == "CMD" and len(r.get("q14", [])) == 14 and r.get("stamp"):
            ct.append(r["stamp"]); cq.append(r["q14"]); cr.append(r.get("t_recv", r["stamp"]))
        elif "measured_q14" in r and r.get("measured_q14") and r.get("measured_stamp"):     # trace_orin row
            st.append(r["measured_stamp"]); sq.append(r["measured_q14"]); sr.append(r["t"])
            if r.get("published_q14") and r.get("t"):
                ct.append(r["t"]); cq.append(r["published_q14"]); cr.append(r["t"])
    if len(st) < 200: sys.exit(f"{path}: only {len(st)} joint states")
    st, ct, sr, cr = np.array(st), np.array(ct), np.array(sr), np.array(cr)
    # command time expressed on the state (MC stamp) clock: shift by the difference of the two stamp-vs-receive offsets
    if len(ct): ct = ct + (np.median(st - sr) - np.median(ct - cr))
    return st, np.array(sq), (ct if len(ct) else None), (np.array(cq) if cq else None)


def grid(t, q):
    o = np.argsort(t); t, q = t[o], q[o]
    keep = np.concatenate([[True], np.diff(t) > 1e-4]); t, q = t[keep], q[keep]
    g = np.arange(t[0], t[-1], DT)
    Q = np.stack([np.interp(g, t, q[:, j]) for j in range(14)], 1)
    V = np.stack([np.gradient(ma(Q[:, j], 3), DT) for j in range(14)], 1)
    return g - t[0], Q, V, t


def shake(V):
    return np.stack([V[:, j] - ma(V[:, j], 20) for j in range(V.shape[1])], 1)


def dominant_hz(x):
    if len(x) < 64: return float("nan")
    f = np.fft.rfftfreq(len(x), DT); p = np.abs(np.fft.rfft(x - x.mean())) ** 2
    m = f > 3.0
    return float(f[m][np.argmax(p[m])]) if m.any() else float("nan")


def report(path, t0=None, t1=None, plot=False):
    st, sq, ct, cq = load(path)
    g, Q, V, raw_t = grid(st, sq)
    gaps = np.diff(raw_t)
    m = np.ones(len(g), bool)
    if t0 is not None: m &= g >= t0
    if t1 is not None: m &= g <= t1
    H = shake(V)
    have_cmd = ct is not None and len(ct) > 200
    if have_cmd:
        cg, CQ, CV, _ = grid(ct, cq)
        CH = shake(CV)
        off = ct.min() - st.min()               # both already on the MC-stamp clock (see load); grids start at 0
        CHs = np.stack([np.interp(g, cg + off, CH[:, j], left=np.nan, right=np.nan) for j in range(14)], 1)
    print(f"\n=== {Path(path).name}: {g[-1]:.0f} s, state gaps p99 {np.percentile(gaps, 99) * 1000:.0f} ms / max {gaps.max() * 1000:.0f} ms"
          f"{'' if have_cmd else '  (no command records: only the measured shake)'}")
    if have_cmd and len(ct) > 1000 and np.median(np.diff(np.sort(ct))) < 0.008:      # full-rate command stream (cmd_recorder)
        o = np.argsort(ct); cts, cqs = ct[o], cq[o]
        cd = np.diff(cts) * 1000
        rate = len(ct) / (ct.max() - ct.min())
        # A gap in the RECORDED stream is either the pacer not publishing, or the recorder (normal priority, best-effort
        # DDS) missing messages. The pacer caps each message to <= max_vel x 2 ticks after a gap, so a jump across the
        # gap that matches the speed before it (full distance covered) means the messages were published but not recorded.
        pub_gap, rec_loss, at_rest = 0, 0, 0
        for i in np.where(cd > 30)[0]:
            j0 = max(0, i - 40)
            v_before = float(np.max(np.abs(cqs[i] - cqs[j0]))) / max(cts[i] - cts[j0], 1e-3)
            jump = float(np.max(np.abs(cqs[i + 1] - cqs[i])))
            expected = v_before * cd[i] / 1000.0          # distance the arm would cover during the gap
            if expected < 0.005: at_rest += 1               # too slow to tell a capped step from a full one
            elif jump >= 0.5 * expected: rec_loss += 1
            else: pub_gap += 1
        print(f"  COMMAND STREAM: {rate:.0f} msg/s, gap p99 {np.percentile(cd, 99):.1f} ms, max {cd.max():.0f} ms, "
              f"{int((cd > 30).sum())} gaps > 30 ms in the recording: {pub_gap} publisher stalls while moving, "
              f"{rec_loss} recorder losses (arm kept moving = published, not recorded), {at_rest} at rest / too slow to tell (harmless)"
              + ("  <-- the Orin publisher is starved: run the receiver real-time (sudo prlimit ... --rtprio)" if pub_gap else ""))
    if np.percentile(gaps, 99) > 0.015:
        print("  WARNING: joint states are missing / irregular (trace_orin samples them at the receiver tick): shake numbers"
              " are rough. Record with tools/cmd_recorder.py on the Orin for exact values.")
    rows = []
    for arm, sl in (("LEFT", slice(0, 7)), ("RIGHT", slice(7, 14))):
        sp = np.max(np.abs(np.stack([ma(V[:, j], 20) for j in range(sl.start, sl.start + 5)], 1)), 1)
        print(f"  {arm} arm          time   measured shake rad/s (J1..J7)                          | command J1-5 / J6-7 | freq | verdict")
        for name, lo, hi in BANDS:
            b = m & (sp >= lo) & (sp < hi)
            if b.sum() < 50: continue
            ms = np.sqrt(np.mean(H[b, sl] ** 2, axis=0))
            main, wrist = float(np.mean(ms[:5])), float(np.mean(ms[5:]))
            cs = np.sqrt(np.nanmean(CHs[b, sl] ** 2, axis=0)) if have_cmd else None
            cmain = float(np.mean(cs[:5])) if have_cmd else float("nan")
            cwr = float(np.mean(cs[5:])) if have_cmd else float("nan")
            hz = dominant_hz(H[b, sl.start]) if name != "rest" else float("nan")
            sparse = np.percentile(gaps, 99) > 0.015
            if main < 0.006 and wrist < 0.006: verdict = "quiet"
            elif sparse: verdict = "(trace too sparse to split command / robot)"
            elif not have_cmd: verdict = ""
            elif cmain < 0.3 * main: verdict = "robot shakes on a smooth command"
            elif cmain < 0.8 * main: verdict = "partly our command"
            else: verdict = "our command is rough"
            print(f"    {name:5} {b.sum() * DT:6.1f}s  " + " ".join(f"{x:.4f}" for x in ms)
                  + (f" | {cmain:.4f} / {cwr:.4f}" if have_cmd else " |   -   ") + f" | {hz:4.0f} Hz | {verdict}")
            rows.append({"arm": arm, "band": name, "seconds": round(b.sum() * DT, 1), "main": main, "wrist": wrist,
                         "cmd_main": cmain, "cmd_wrist": cwr, "freq_hz": hz})
    print(f"  reference: PlanningMove J1 0.017 rad/s at ~0.55 rad/s, wrist 0.0001")
    if plot:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        j = int(np.argmax(np.sqrt(np.mean(H[m, :] ** 2, axis=0))))
        fig, ax = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
        ax[0].plot(g[m], V[m, j], lw=0.7, label=f"measured velocity {'L' if j < 7 else 'R'} {NAMES[j % 7]}")
        if have_cmd: ax[0].plot(g[m], np.interp(g[m], cg + off, CV[:, j]), lw=0.7, label="command velocity")
        ax[0].set_ylabel("rad/s"); ax[0].legend(loc="upper right")
        ax[1].plot(g[m], H[m, j], lw=0.6, label="measured shake (velocity - 0.2 s average)")
        if have_cmd: ax[1].plot(g[m], CHs[m, j], lw=0.6, label="command shake")
        ax[1].set_ylabel("rad/s"); ax[1].set_xlabel("s"); ax[1].legend(loc="upper right")
        out = str(path).rsplit(".", 1)[0] + "_vibration.png"
        fig.tight_layout(); fig.savefig(out, dpi=110); print(f"  plot: {out}")
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("traces", nargs="+")
    ap.add_argument("--t0", type=float); ap.add_argument("--t1", type=float)
    ap.add_argument("--plot", action="store_true", help="PNG of the joint that shakes most")
    a = ap.parse_args()
    allrows = {p: report(p, a.t0, a.t1, a.plot) for p in a.traces}
    if len(a.traces) > 1:
        print("\n=== comparison (measured shake J1-J5 / wrist, rad/s)")
        for band in ("slow", "mid", "fast"):
            line = [f"{band:5}"]
            for p, rows in allrows.items():
                r = [x for x in rows if x["band"] == band]
                line.append(f"{Path(p).name[:30]:30} " + (" ".join(f"{x['arm'][0]} {x['main']:.4f}/{x['wrist']:.4f}" for x in r) or "-"))
            print("  " + " | ".join(line))


if __name__ == "__main__":
    main()
