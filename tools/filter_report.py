#!/usr/bin/env python3
"""tools/filter_report.py - What does each Orin-side filter do to a REAL teleop trajectory? (report + plot)

Replays the joint targets the laptop actually sent (logs/trace_laptop_<ts>.jsonl, written by every real session) through
the exact Orin pacer chain - jitter buffer (PlayoutBuffer) -> filter (JointSmoother 2-stage low-pass or OneEuroFilter)
-> optional acceleration guard -> speed / step guard, published at 400 Hz - with a realistic packet-arrival jitter, and
compares filter settings:

    python tools/filter_report.py logs/trace_laptop_20261116_162448.jsonl
    python tools/filter_report.py <trace> --configs none,lpf10,lpf20,lpf30,oneeuro --jitter none
    python tools/filter_report.py <trace> --t0 20 --t1 40         # plot window [s]

Columns (on what the MC sees: the newest command every 10 ms):
  lag ms        time shift that best aligns the output with the laptop target (cost of filtering)
  shake         RMS of command velocity minus its 0.2 s average while moving (rad/s): steps / jitter left in the command
  peak acc      largest command acceleration over 10 ms (vendor limit 6.28 rad/s^2)
  rest motion   RMS command velocity while the target is at rest (rad/s): ringing / sway at stops (should be ~0)
  overshoot     largest excursion past a target that came to rest (mrad)
This only measures OUR command. The robot can still shake on a smooth command: measure that with tools/cmd_recorder.py
on the Orin + tools/vibration_report.py.
"""
import argparse, importlib, json, math, sys, types
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
RAW_INPUT = False
SPEED = {1: (0.40, 1.00), 2: (0.60, 1.50), 3: (0.80, 2.00), 4: (1.00, 2.50), 5: (1.50, 4.00), 6: (2.00, 6.28)}
CONFIGS = {   # name: (filter, param1, param2, accel_guard)
    "none": ("lpf", 0.0, None, 0.0),                       # 13/11 checkpoint
    "lpf10": ("lpf", 10.0, None, 0.0), "lpf20": ("lpf", 20.0, None, 0.0), "lpf30": ("lpf", 30.0, None, 0.0),
    "lpf40": ("lpf", 40.0, None, 0.0), "lpf50": ("lpf", 50.0, None, 0.0), "lpf60": ("lpf", 60.0, None, 0.0),
    "oneeuro": ("oneeuro", 1.5, 0.5, 0.0), "oneeuro-strong": ("oneeuro", 1.0, 0.3, 0.0),
    "lpf10+guard": ("lpf", 10.0, None, 6.28),
    # Kalman (q rad^2/s^3, r rad^2, optional low-pass ms after it)
    "kalman": ("kalman", 40.0, 4e-5, 0.0, 0.0), "kalman-soft": ("kalman", 10.0, 1e-4, 0.0, 0.0),
    "kalman+lpf20": ("kalman", 40.0, 4e-5, 0.0, 20.0), "kalman+lpf30": ("kalman", 20.0, 1e-4, 0.0, 30.0),
    # per joint (17/11, env JOINT_VMAX / JOINT_AMAX / JOINT_TAU_MS on the Orin): ("joint", vmax J1..J7, amax, tau ms)
    "joint-k1": ("joint", [1.2, 1.5, 1.2, 1.5, 2.0, 2.0, 2.0], [8.0] * 7, None),
    "joint-k3": ("joint", [1.2, 1.5, 1.2, 1.5, 2.0, 2.0, 2.0], [6.0] * 7, None),
    "joint-k2": ("joint", [1.0, 1.2, 1.0, 1.2, 1.5, 2.0, 2.0], [8.0] * 7, None),
    "acc8": ("joint", None, [8.0] * 7, None),
}


def load_core():
    """Import scripts/orin_teleop_core.py without ROS (it imports rclpy at module level)."""
    for name in ("rclpy", "rclpy.node", "rclpy.qos", "sensor_msgs", "sensor_msgs.msg"):
        if name not in sys.modules:
            try: importlib.import_module(name)
            except Exception:
                m = types.ModuleType(name)
                m.Node = object; m.JointState = object
                m.QoSProfile = m.QoSHistoryPolicy = m.QoSReliabilityPolicy = object
                sys.modules[name] = m
    sys.path.insert(0, str(REPO / "scripts"))
    return importlib.import_module("orin_teleop_core")


def load_trace(path):
    t, q, speed = [], [], 6
    for line in open(path):
        try: r = json.loads(line)
        except ValueError: continue
        if r.get("event") == "CONFIG": speed = int(r.get("speed", speed))
        elif "sent_q14" in r and "t" in r and len(r["sent_q14"]) == 14: t.append(r["t"]); q.append(r["sent_q14"])
    if len(t) < 500: sys.exit(f"{path}: only {len(t)} sent targets")
    if RAW_INPUT:                     # IK output before the laptop speed limiter = what the fast config (speed 6) sends
        q = [r["raw_target_q14"] for r in map(json.loads, (l for l in open(path) if '"sent_q14"' in l))
             if "t" in r and len(r.get("sent_q14", [])) == 14]
        if len(q) != len(t): sys.exit(f"{path}: raw_target_q14 missing")
    return np.array(t) - t[0], np.array(q), speed


def arrivals(t_send, mode, rng):
    n = len(t_send)
    if mode == "none": return t_send + 0.002
    # real-like (Orin 11-13/11): ~2 ms base delay with an exponential tail, packets delivered in bursts of 2 and,
    # about every 0.5 s, a run of 3-4 packets held 20-50 ms
    a = t_send + 0.002 + rng.exponential(0.003, n)
    a[1::2] = np.maximum(a[1::2], a[:-1:2] + 0.0)                   # pairs arrive together
    for k in rng.choice(n - 5, max(1, n // 50), replace=False): a[k:k + 4] += rng.uniform(0.02, 0.05)
    return np.maximum.accumulate(a)


def replay(core, t_send, q_send, cfg, speed, playout_ms, jitter, seed=1):
    kind, p1, p2, acc = cfg[:4]
    vmax, _ = SPEED[speed]; step_abs = min(0.018, max(vmax / 100.0, 0.016))
    if kind == "joint":                                       # same wiring as pacer_main with JOINT_* set
        jv = None if p1 is None else np.minimum(np.array(list(p1) * 2), vmax)
        acc = np.array(list(p2) * 2) if p2 is not None else 0.0
        kind, p1 = "lpf", (np.array(list(acc_tau) * 2) if (acc_tau := cfg[3]) is not None else 0.0)
        if jv is not None: vmax = jv if np.ndim(acc) else vmax
    rng = np.random.default_rng(seed)
    arr = arrivals(t_send, jitter, rng)
    buf = core.PlayoutBuffer(100, playout_ms / 1000.0)
    if kind == "oneeuro": flt = core.OneEuroFilter(p1, p2)
    elif kind == "kalman": flt = core.KalmanJointFilter(p1, p2, cfg[4] if len(cfg) > 4 else 0.0)
    else: flt = core.JointSmoother(p1 / 1000.0)
    cur = q_send[0].copy(); flt.reset(cur); vp = np.zeros(14); tp = None
    dt, k, out_t, out_q = 1.0 / 400, 0, [], []
    t = 0.0
    while t < t_send[-1] + 0.5:
        while k < len(arr) and arr[k] <= t: buf.push(arr[k], q_send[k], k); k += 1
        g, _ = buf.target(t)
        g = cur if g is None else g
        g = flt.step(cur, g, dt)
        st = None
        if np.any(np.asarray(acc) > 0): st, _ = core.accel_guard(cur, vp, g, tp, dt, vmax, acc, step_abs); tp = g
        if st is None:
            e = g - cur; d = float(np.max(np.abs(e))); cap = min(step_abs, float(np.max(vmax)) * dt)
            st = e * (cap / d) if d > cap else e
        nx = cur + st; vp = (nx - cur) / dt; cur = nx
        out_t.append(t); out_q.append(cur.copy()); t += dt
    return np.array(out_t), np.array(out_q)


def ma(x, n): return np.convolve(np.pad(x, (n // 2, n - 1 - n // 2), mode="edge"), np.ones(n) / n, mode="valid")


def metrics(t_in, q_in, t_out, q_out):
    g = np.arange(0.5, min(t_in[-1], t_out[-1]) - 0.5, 0.01)          # what the MC latches every 10 ms
    I = np.stack([np.interp(g, t_in, q_in[:, j]) for j in range(14)], 1)
    O = np.stack([np.interp(g, t_out, q_out[:, j]) for j in range(14)], 1)
    vI, vO = np.gradient(I, 0.01, axis=0), np.gradient(O, 0.01, axis=0)
    aO = np.gradient(vO, 0.01, axis=0)
    spd_in = np.max(np.abs(np.stack([ma(vI[:, j], 20) for j in range(14)], 1)), 1)
    moving, rest = spd_in > 0.05, spd_in < 0.005
    shake = np.sqrt(np.mean([(vO[moving, j] - ma(vO[:, j], 20)[moving]) ** 2 for j in range(14)])) if moving.any() else 0.0
    rest_mo = np.sqrt(np.mean(vO[rest] ** 2)) if rest.sum() > 50 else 0.0
    best = (0, 1e9)
    for s in range(0, 40):                                           # 0..390 ms
        e = float(np.mean(np.abs(O[s:] - I[:len(I) - s])))
        if e < best[1]: best = (s, e)
    # overshoot past targets that came to rest: within rest windows, excursion beyond the rest value in the direction
    # of the preceding motion
    over = 0.0
    idx = np.where(np.diff(rest.astype(int)) == 1)[0]
    for i in idx:
        j_end = min(len(g) - 1, i + 60)
        if not rest[i + 1:j_end].all(): continue        # only real stops (target still for 0.6 s), not brief pauses
        for j in range(14):
            d = np.sign(I[i, j] - I[max(0, i - 30), j])
            if d == 0: continue
            over = max(over, float(np.max(d * (O[i:j_end, j] - I[i:j_end, j]))))
    return {"lag_ms": best[0] * 10, "shake": shake, "peak_acc": float(np.max(np.abs(aO[5:-5]))), "rest_motion": rest_mo,
            "overshoot_mrad": over * 1000}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trace")
    ap.add_argument("--configs", default="none,lpf10,lpf20,lpf30,oneeuro,oneeuro-strong,lpf10+guard")
    ap.add_argument("--jitter", choices=["real", "none"], default="real")
    ap.add_argument("--playout", type=float, default=30.0, help="jitter buffer [ms] (receiver default 30)")
    ap.add_argument("--speed", type=int, default=None, help="speed level (default: from the trace CONFIG)")
    ap.add_argument("--t0", type=float); ap.add_argument("--t1", type=float)
    ap.add_argument("--raw", action="store_true", help="replay the IK output (raw_target_q14) = fast-config input")
    a = ap.parse_args()
    global RAW_INPUT; RAW_INPUT = a.raw
    core = load_core()
    t_in, q_in, speed = load_trace(a.trace)
    speed = a.speed or speed
    names = [c.strip() for c in a.configs.split(",") if c.strip()]
    print(f"{Path(a.trace).name}: {t_in[-1]:.0f} s of targets, speed {speed}, jitter {a.jitter}, buffer {a.playout:.0f} ms")
    print(f"{'config':15} {'lag ms':>6} {'shake':>8} {'peak acc':>9} {'rest motion':>12} {'overshoot':>10}")
    outs = {}
    for n in names:
        if n not in CONFIGS: print(f"  unknown config {n}; choose from {', '.join(CONFIGS)}"); continue
        to, qo = replay(core, t_in, q_in, CONFIGS[n], speed, a.playout, a.jitter)
        m = metrics(t_in, q_in, to, qo)
        outs[n] = (to, qo, m)
        print(f"{n:15} {m['lag_ms']:6.0f} {m['shake']:8.4f} {m['peak_acc']:9.2f} {m['rest_motion']:12.5f} {m['overshoot_mrad']:10.1f}")
    # plot: the joint that moves most, around the strongest stop (or the given window)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    j = int(np.argmax(np.ptp(q_in, axis=0)))
    if a.t0 is not None: w0, w1 = a.t0, a.t1 if a.t1 else a.t0 + 6
    else:
        v = np.abs(np.gradient(q_in[:, j], t_in)); c = t_in[int(np.argmax(ma(v, 30)))]; w0, w1 = c - 2, c + 4
    fig, ax = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    mi = (t_in >= w0) & (t_in <= w1)
    ax[0].plot(t_in[mi], q_in[mi, j], "k", lw=2, alpha=0.35, label="laptop target")
    for n, (to, qo, m) in outs.items():
        mo = (to >= w0) & (to <= w1)
        g = np.arange(w0, w1, 0.01); x = np.interp(g, to, qo[:, j]); v = np.gradient(x, 0.01); acc = np.gradient(v, 0.01)
        ax[0].plot(to[mo], qo[mo, j], lw=1, label=f"{n} (lag {m['lag_ms']:.0f} ms)")
        ax[1].plot(g, v, lw=0.9, label=n); ax[2].plot(g, acc, lw=0.8, label=n)
    ax[2].axhline(6.28, color="r", ls=":", lw=1); ax[2].axhline(-6.28, color="r", ls=":", lw=1)
    jn = f"{'L' if j < 7 else 'R'} J{j % 7 + 1}"
    ax[0].set_ylabel(f"{jn} position rad"); ax[1].set_ylabel("velocity rad/s (10 ms)"); ax[2].set_ylabel("accel rad/s^2 (red: vendor limit)")
    ax[2].set_xlabel("s"); ax[0].legend(fontsize=8, loc="best"); ax[1].legend(fontsize=7, ncol=4, loc="best")
    ax[0].set_title(f"Orin filter replay of {Path(a.trace).name} (speed {speed}, jitter {a.jitter}, buffer {a.playout:.0f} ms)")
    out = str(Path(a.trace).with_suffix("")) + "_filters.png"
    fig.tight_layout(); fig.savefig(out, dpi=110); print(f"plot: {out}")


if __name__ == "__main__":
    main()
