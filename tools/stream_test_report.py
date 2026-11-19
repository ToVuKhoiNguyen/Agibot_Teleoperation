#!/usr/bin/env python3
"""tools/stream_test_report.py - ripple per phase of an arm_stream_test_<time>.jsonl log (laptop).

Uses the MC's own stamp as time base (arrival time inflates ripple 2-3x, 16/11), elbows only (joint 4, both arms),
whole phase.
  ripple % = tools/plot_motion_jerk.py metric: rms(velocity - 0.4 s smoothed) while moving, % of peak speed
  (command rows have no stamp: placed on the phase's nominal publish grid)
Reference (same metric): PlanningMove 16/11 elbow 1.2-1.4 %. The sweep is a triangle, so the command itself shows some ripple at the reversals.
Usage: venv/bin/python tools/stream_test_report.py logs/arm_stream_test_<time>.jsonl
"""
import sys, json
import numpy as np

ELBOWS = (3, 10)


sys.path.insert(0, __file__.rsplit("/", 1)[0])
from plot_motion_jerk import derivs, ripple as _ripple   # SAME metric as the PlanningMove reference (16/11)


def ripple(t, q, j, nominal_dt=None):
    """-> (ripple % of peak speed, peak speed) of joint j; t None = even grid at nominal_dt (command rows)."""
    if len(q) < 60: return None
    if t is None: t = np.arange(len(q)) * nominal_dt
    keep = np.concatenate([[True], np.diff(t) > 1e-6]); t, q = t[keep], q[keep]
    _, _, v, _ = derivs(t, q[:, j])
    r = _ripple(v)
    return (r / 100.0, float(np.abs(v).max())) if r is not None else None


def main(path):
    S, C = {}, {}
    for line in open(path):
        try: d = json.loads(line)
        except Exception: continue
        ph = d.get("ph", "")
        if ph in ("init", "pause", "homing", ""): continue
        if d.get("k") == "S" and "stamp" in d: S.setdefault(ph, []).append((d["stamp"], d["q"]))
        elif d.get("k") == "C": C.setdefault(ph, []).append((None, d["q"]))
    print(f"{path}\n  phase                       peak speed | measured ripple L / R elbow | command ripple L / R")
    for ph in S:
        t = np.array([s[0] for s in S[ph]]); q = np.array([s[1] for s in S[ph]])
        m = [ripple(t, q, j) for j in ELBOWS]
        cm = [None, None]
        if ph in C:
            qc = np.array([c[1] for c in C[ph]])
            hz = next((float(x[:-2]) for x in ph.split(":") if x.endswith("Hz")), None)
            cm = [ripple(None, qc, j, 1.0 / hz) for j in ELBOWS] if hz else [None, None]
        f = lambda r: f"{r[0] * 100:5.1f} %" if r else "   -   "
        spd = np.mean([r[1] for r in m if r]) if any(m) else float("nan")
        print(f"  {ph:26} {spd:6.2f} rad/s |    {f(m[0])} / {f(m[1])}       |   {f(cm[0])} / {f(cm[1])}")
    print("  reference (same metric): PlanningMove 16/11 elbow J4 1.2-1.4 %, J1 1.4-1.5 % (trace_planning_20261116_145115)")


if __name__ == "__main__":
    for p in sys.argv[1:]: main(p)
