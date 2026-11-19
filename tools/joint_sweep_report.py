#!/usr/bin/env python3
"""tools/joint_sweep_report.py - Per-joint speed / acceleration limits from a joint_limit_sweep log (laptop).

Input: joint_sweep_<time>.jsonl written on the Orin by tools/joint_limit_sweep.py (one joint moving at a time).
For every phase (joint, speed, accel) the measured joint angles (MC stamp, 100 Hz) are put through the robot model
(MuJoCo forward kinematics) and the HAND vibration is measured:
  cruise mm   rms of the hand position high-passed above 4 Hz while the joint is at constant speed
  stop mm     same, in the 0.4 s after each stop (the arm settling = acceleration / braking effect)
  joint %     rms ripple of the joint speed at cruise, % of the cruise speed (tools/plot_motion_jerk metric)
  10 Hz %     share of the hand vibration energy at 7-14 Hz (the 402 oscillation)
Proposed limits: per joint, the highest tested speed whose cruise vibration is <= --cruise-mm and the highest accel
whose stop vibration is <= --stop-mm (defaults: the worst level measured at config D's speeds, which the operator
rated "slight shake, very good"). Prints JOINT_VMAX / JOINT_AMAX lines for arm_teleop_rx.

Usage: venv/bin/python tools/joint_sweep_report.py logs/joint_sweep_<time>.jsonl [--cruise-mm 0.3 --stop-mm 0.6] [--plot]
"""
import sys, json, argparse
from pathlib import Path
import numpy as np
import mujoco
from scipy.signal import butter, sosfiltfilt

REPO = Path(__file__).resolve().parents[1]
SCENE = REPO / "assets" / "agibot_a2_ultra" / "mjcf" / "scene.xml"
DT = 0.01
D_CONFIG = [1.2, 1.5, 1.2, 1.5, 2.0, 2.0, 2.0]          # current default JOINT_VMAX (config D)


def load(path):
    cfg, rows = {}, {}
    for line in open(path):
        try: d = json.loads(line)
        except ValueError: continue
        if d.get("k") == "CONFIG": cfg = d
        elif d.get("k") == "S" and ":" in d.get("ph", ""):
            rows.setdefault(d["ph"], []).append((d["stamp"], d["vc"], d["q"]))
    return cfg, rows


class Hand:
    def __init__(self, arm):
        self.m = mujoco.MjModel.from_xml_path(str(SCENE)); self.d = mujoco.MjData(self.m)
        names = [f"idx{13 + i}_left_arm_joint{i + 1}" for i in range(7)] + [f"idx{20 + i}_right_arm_joint{i + 1}" for i in range(7)]
        self.adr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
        # grip point = mean of the fingertips (what the operator watches); the ee site sits on the wrist axes, so
        # wrist joints J6/J7 would show no vibration there
        p = "L" if arm == "left" else "R"
        self.tips = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, f"{p}_{f}") for f in ("thumb_3", "index_2", "middle_2", "ring_2")]

    def pos(self, Q):
        out = np.empty((len(Q), 3))
        for k, q in enumerate(Q):
            self.d.qpos[self.adr] = q; mujoco.mj_kinematics(self.m, self.d); out[k] = self.d.xpos[self.tips].mean(axis=0)
        return out


def analyse_phase(rows, hand, idx, sp):
    t = np.array([r[0] for r in rows]); vc = np.array([r[1] for r in rows]); Q = np.array([r[2] for r in rows])
    o = np.argsort(t); t, vc, Q = t[o], vc[o], Q[o]
    keep = np.concatenate([[True], np.diff(t) > 1e-6]); t, vc, Q = t[keep], vc[keep], Q[keep]
    if len(t) < 100: return None
    g = np.arange(t[0], t[-1], DT)
    Qg = np.stack([np.interp(g, t, Q[:, j]) for j in range(14)], 1); vcg = np.interp(g, t, vc)
    P = hand.pos(Qg) * 1000.0                                             # mm
    sos = butter(4, 4.0, "highpass", fs=1.0 / DT, output="sos")
    H = sosfiltfilt(sos, P, axis=0); hv = np.linalg.norm(H, axis=1)
    cruise = np.abs(np.abs(vcg) - sp) < 0.02 * sp + 1e-3
    moving = np.abs(vcg) > 1e-3
    stop = np.zeros_like(moving)
    for k in np.where(moving[:-1] & ~moving[1:])[0]: stop[k + 1:k + 41] = True   # 0.4 s after each stop
    edge = np.zeros_like(cruise); edge[30:-30] = True                         # filter edge effects
    qj = Qg[:, idx]; vj = np.gradient(qj, DT)
    vs = sosfiltfilt(butter(2, 3.0, "lowpass", fs=1.0 / DT, output="sos"), vj)
    c, s = cruise & edge, stop & edge
    rms = lambda m: float(np.sqrt(np.mean(hv[m] ** 2))) if m.sum() > 10 else float("nan")
    F = np.abs(np.fft.rfft(np.where(c, H[:, 0], 0) ** 1)); fr = np.fft.rfftfreq(len(H), DT)
    band = float(F[(fr > 7) & (fr < 14)].sum() / max(1e-9, F[fr > 4].sum()))
    return {"cruise_mm": rms(c), "stop_mm": rms(s), "cruise_s": c.sum() * DT,
            "joint_pct": 100.0 * float(np.sqrt(np.mean((vj[c] - vs[c]) ** 2))) / sp if c.sum() > 10 else float("nan"),
            "band10": band, "peak_v": float(np.max(np.abs(vs)))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log"); ap.add_argument("--cruise-mm", type=float, default=None); ap.add_argument("--stop-mm", type=float, default=None)
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    cfg, rows = load(a.log)
    arm = cfg.get("arm", "right"); off = 0 if arm == "left" else 7
    hand = Hand(arm)
    res = {}
    print(f"{Path(a.log).name}: {arm} arm, {len(rows)} phases")
    print(f"  {'phase':14} {'cruise s':>8} {'hand cruise mm':>14} {'hand stop mm':>12} {'joint ripple %':>14} {'7-14 Hz':>8} {'peak v':>7}")
    for ph, r in rows.items():
        j, sp, acc = ph[1:].split(":"); j, sp, acc = int(j), float(sp), float(acc)
        m = analyse_phase(r, hand, off + j - 1, sp)
        if m is None: continue
        res[(j, sp, acc)] = m
        short = "  short" if m["cruise_s"] < 0.5 else ""
        print(f"  {ph:14} {m['cruise_s']:8.1f} {m['cruise_mm']:14.3f} {m['stop_mm']:12.3f} {m['joint_pct']:13.1f}% {100 * m['band10']:7.0f}% {m['peak_v']:7.2f}{short}")
    if not res: sys.exit("no usable phases")
    # thresholds: default = vibration at config D's speeds (operator: slight shake, very good)
    ref_c = [m["cruise_mm"] for (j, sp, acc), m in res.items() if abs(sp - D_CONFIG[j - 1]) < 0.21 and np.isfinite(m["cruise_mm"])]
    ref_s = [m["stop_mm"] for (j, sp, acc), m in res.items() if abs(sp - D_CONFIG[j - 1]) < 0.21 and abs(acc - 6) < 0.6 and np.isfinite(m["stop_mm"])]
    # default = the WORST hand vibration among the joints at config D's speeds: the operator accepted D, so every joint
    # may go as fast as keeps its own vibration at or below that level
    thr_c = a.cruise_mm if a.cruise_mm is not None else (float(np.max(ref_c)) if ref_c else 0.3)
    thr_s = a.stop_mm if a.stop_mm is not None else (float(np.max(ref_s)) if ref_s else 0.6)
    print(f"\n  thresholds: hand cruise <= {thr_c:.3f} mm, hand after stop <= {thr_s:.3f} mm"
          f"{'' if a.cruise_mm is not None else '  (worst joint at config D speeds = what the operator accepted)'}")
    vmax, amax = [], []
    for j in range(1, 8):
        ok_v = [sp for (jj, sp, acc), m in res.items() if jj == j and m["cruise_s"] >= 0.5 and m["cruise_mm"] <= thr_c]
        all_v = [sp for (jj, sp, acc), m in res.items() if jj == j]
        ok_a = [acc for (jj, sp, acc), m in res.items() if jj == j and np.isfinite(m["stop_mm"]) and m["stop_mm"] <= thr_s]
        testable = [sp for (jj, sp, acc), m in res.items() if jj == j and m["cruise_s"] >= 0.5]
        if ok_v and len(ok_v) == len(testable):      # every speed that could be reached was fine: no evidence against D
            v = max(max(ok_v), D_CONFIG[j - 1])
        else: v = max(ok_v) if ok_v else (min(all_v) if all_v else D_CONFIG[j - 1])
        vmax.append(v); amax.append(max(ok_a) if ok_a else 6.0)
        tested = "tested" if all_v else "NOT tested, config D kept"
        print(f"  J{j}: speed limit {v:.1f} rad/s, accel {amax[-1]:.0f} rad/s^2   ({tested})")
    print("\n  arm_teleop_rx (Orin):")
    print(f"    JOINT_VMAX={','.join(format(v, 'g') for v in vmax)} JOINT_AMAX={','.join(format(x, 'g') for x in amax)} python arm_teleop_rx.py")
    if a.plot:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        for j in range(1, 8):
            pts = sorted((sp, m["cruise_mm"]) for (jj, sp, acc), m in res.items() if jj == j and abs(acc - 6) < 0.6)
            if pts: ax[0].plot(*zip(*pts), "o-", label=f"J{j}")
            pa = sorted((acc, m["stop_mm"]) for (jj, sp, acc), m in res.items() if jj == j)
            if len({p[0] for p in pa}) > 1: ax[1].plot(*zip(*pa), "o-", label=f"J{j}")
        ax[0].axhline(thr_c, color="r", ls=":"); ax[1].axhline(thr_s, color="r", ls=":")
        ax[0].set_xlabel("joint speed rad/s"); ax[0].set_ylabel("hand vibration at cruise, mm rms"); ax[0].legend()
        ax[1].set_xlabel("accel rad/s^2"); ax[1].set_ylabel("hand vibration after stop, mm rms"); ax[1].legend()
        out = str(Path(a.log).with_suffix("")) + "_limits.png"; fig.tight_layout(); fig.savefig(out, dpi=110); print(f"  plot: {out}")


if __name__ == "__main__":
    main()
