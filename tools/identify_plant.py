"""
tools/identify_plant.py
Data-driven plant identification for AGIBOT A2 Ultra (SENT -> ACT).

Two plant scopes (pick with --source):
  laptop : laptop logs logs/real_*.jsonl  (target_q -> measured_q, both seen at the laptop).
           Lumps network + Orin playout/guard + MC + feedback path. Used by main.py --sim
           with SIM_PLANT=1.                                 -> config/sim_plant.json
  orin   : Orin traces trace_orin_*.jsonl (published_q14 -> measured_q14 at the Orin).
           MC + motors only. Used by the SIL fake MC (sil/fake_mc_node.py), where network,
           playout and the final-line guard run as real code.  -> config/mc_plant.json

Logs are resampled onto a uniform native grid (--dt, default 0.010 s) from their real
timestamps (command: zero-order hold, measurement: linear), so 50 Hz and 100 Hz logs
can be mixed. The discrete model is teleop.simulation.real_plant.joint_step, the same
code the simulator runs.

Hand (optional): --hand-logs plant_excitation_*.jsonl (hand_cmd/hand_meas raw 0..2000).

Usage:
  python tools/identify_plant.py --source orin --fit logs/trace_orin_A.jsonl --val logs/trace_orin_B.jsonl
  python tools/identify_plant.py --source laptop --fit logs/real_A.jsonl --val logs/real_B.jsonl
"""

import os
import sys
import json
import argparse
import subprocess
import numpy as np
from scipy import optimize, signal

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from teleop.simulation.real_plant import joint_step, MAX_PLAUSIBLE_NOISE_RAD  # noqa: E402

WRIST_JOINTS = {5, 6, 12, 13}
VAL_OK_RAD = 0.010


# ----------------------------------------------------------------------------- loading
def _read_jsonl(path):
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            try:
                yield json.loads(line)
            except Exception:
                continue


def load_log(path, source):
    """Returns (t[s], sent[N,14], act[N,14], meta) with raw (non-uniform) timestamps."""
    t, s, a, speed = [], [], [], None
    for d in _read_jsonl(path):
        if source == "laptop":
            if "target_q" in d and "measured_q" in d and any(abs(x) > 0 for x in d["measured_q"]):
                t.append(float(d["t_laptop"])); s.append(d["target_q"]); a.append(d["measured_q"])
        else:
            if d.get("published_q14") and d.get("measured_q14") and "t" in d:
                t.append(float(d["t"])); s.append(d["published_q14"]); a.append(d["measured_q14"])
        if "speed" in d:
            speed = int(d["speed"])
    if len(t) < 50:
        raise ValueError(f"{path}: only {len(t)} usable samples for source={source}")
    t, s, a = np.array(t), np.array(s, dtype=np.float64), np.array(a, dtype=np.float64)
    order = np.argsort(t, kind="stable")
    t, s, a = t[order], s[order], a[order]
    dts = np.diff(t)
    meta = {"path": path, "samples": int(len(t)), "median_dt_s": float(np.median(dts)),
            "max_gap_s": float(np.max(dts)), "speed": speed}
    return t, s, a, meta


def resample(t, s, a, dt):
    """Uniform grid: command zero-order hold (what the MC actually sees), measurement linear."""
    grid = np.arange(t[0], t[-1], dt)
    idx = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)
    s_u = s[idx]
    a_u = np.stack([np.interp(grid, t, a[:, j]) for j in range(a.shape[1])], axis=1)
    return s_u, a_u


# ----------------------------------------------------------------------------- model
def simulate(u, cfg, dt, y0, apply_limits):
    d = max(0, int(round(cfg.get("delay_ticks", 0))))
    u_del = np.concatenate([np.full(d, u[0]), u[:-d]]) if d > 0 else u
    y, v = y0 - float(cfg.get("static_offset_rad", 0.0)), 0.0
    out = np.empty_like(u)
    for k in range(len(u)):
        y, v = joint_step(cfg, y, v, float(u_del[k]), dt, apply_limits=apply_limits)
        out[k] = y
    return out + float(cfg.get("static_offset_rad", 0.0))


def fit_delay(u, y, max_ticks):
    du, dy = np.diff(u), np.diff(y)
    best_lag, best_c = 0, -1.0
    for lag in range(0, max_ticks + 1):
        a, b = (du, dy) if lag == 0 else (du[:-lag], dy[lag:])
        if np.std(a) < 1e-7 or np.std(b) < 1e-7:
            continue
        c = np.corrcoef(a, b)[0, 1]
        if np.isfinite(c) and c > best_c:
            best_c, best_lag = c, lag
    return best_lag


def crop_active(u, y, dt, pad_s=1.0):
    """Restricts a fit to where this joint's command actually moves (per-joint excitation logs)."""
    mv = np.where(np.abs(np.diff(u)) > 1e-6)[0]
    if len(mv) == 0:
        return u, y
    pad = int(pad_s / dt)
    a, b = max(0, mv[0] - pad), min(len(u), mv[-1] + pad)
    return u[a:b], y[a:b]


def robust_noise(u, y):
    """Noise std from measurement increments while the command is static (MAD, robust to motion)."""
    du, dy = np.abs(np.diff(u)), np.diff(y)
    still = du < 1e-7
    still = still[:-1] & still[1:] if len(still) > 1 else still
    seg = dy[1:][still] if len(dy) > 1 else dy
    if len(seg) < 50:
        return 0.0
    sigma = 1.4826 * np.median(np.abs(seg - np.median(seg))) / np.sqrt(2.0)
    return float(min(sigma, MAX_PLAUSIBLE_NOISE_RAD))


def metrics(y_true, y_sim, dt):
    rmse = float(np.sqrt(np.mean((y_sim - y_true) ** 2)))
    ss = float(np.sum((y_true - np.mean(y_true)) ** 2))
    r2 = float(1.0 - np.sum((y_true - y_sim) ** 2) / ss) if ss > 1e-9 else 1.0
    fs = 1.0 / dt
    nper = min(512, len(y_true))
    f1, p1 = signal.welch(y_true - np.mean(y_true), fs=fs, nperseg=nper)
    _, p2 = signal.welch(y_sim - np.mean(y_sim), fs=fs, nperseg=nper)
    band = f1 <= 15.0
    nrm = np.linalg.norm(p1[band]) * np.linalg.norm(p2[band])
    spec = float(np.dot(p1[band], p2[band]) / nrm) if nrm > 1e-12 else 1.0
    return {"rmse_rad": round(rmse, 5), "r2": round(r2, 4), "spectral_match_0_15hz": round(spec, 4),
            "peak_err_rad": round(float(np.max(np.abs(y_sim - y_true))), 5)}


def fit_joint(j, fit_sets, val_sets, dt, max_delay):
    """fit_sets/val_sets: list of (u[N], y[N]) for this joint. Uses the set with the most motion."""
    u_f, y_f = crop_active(*max(fit_sets, key=lambda p: np.ptp(p[0])), dt)
    if np.ptp(u_f) < 0.005:
        return {"name": f"joint_{j}", "model_type": "IDEAL", "confidence": "IDEAL",
                "delay_ticks": 0, "reason": f"fit command ptp {np.ptp(u_f):.4f} rad < 5 mrad"}

    d0 = fit_delay(u_f, y_f, max_delay)
    dy = np.abs(np.diff(y_f)) / dt
    v_max = float(np.percentile(dy, 99.9))
    a_max = float(np.percentile(np.abs(np.diff(np.diff(y_f))) / dt / dt, 99.9))
    nz = np.abs(np.diff(y_f)); nz = nz[nz > 2e-5]
    deadband = float(np.percentile(nz, 1)) if len(nz) > 20 else 0.0

    # Velocity cross-correlation peaks near delay + lag, so it over-estimates the pure delay:
    # scan every delay from 0 up to just past that peak and let the model fit decide.
    best = None
    for d in range(0, min(max_delay, d0 + 2) + 1):
        if j in WRIST_JOINTS:
            def mk(p): return {"model_type": "2nd_order", "delay_ticks": d, "fn_hz": p[0], "zeta": p[1], "static_offset_rad": p[2]}
            p0, bnds = [4.0, 0.7, 0.0], [(0.8, 15.0), (0.2, 1.5), (-0.03, 0.03)]
        else:
            def mk(p): return {"model_type": "1st_order", "delay_ticks": d, "tau_s": p[0], "static_offset_rad": p[1]}
            p0, bnds = [0.05, 0.0], [(0.001, 0.5), (-0.03, 0.03)]
        res = optimize.minimize(lambda p: np.mean((simulate(u_f, mk(p), dt, y_f[0], False) - y_f) ** 2),
                                p0, bounds=bnds, method="Nelder-Mead",
                                options={"xatol": 1e-4, "fatol": 1e-12, "maxiter": 400})
        if best is None or res.fun < best[0]:
            best = (res.fun, mk(res.x), res.x, bnds)

    _, cfg, x, bnds = best
    at_bound = [i for i, (lo, hi) in enumerate(bnds) if np.isclose(x[i], lo, rtol=0, atol=1e-6) or np.isclose(x[i], hi, rtol=0, atol=1e-6)]
    cfg.update({
        "name": f"joint_{j}",
        "delay_s": round(cfg["delay_ticks"] * dt, 4),
        "max_vel_rad_s": round(max(v_max * 1.05, 0.5), 3),
        "max_acc_rad_s2": round(max(a_max * 1.05, 5.0), 2),
        "deadband_rad": round(deadband, 6),
        "noise_std_rad": round(robust_noise(u_f, y_f), 6),
    })
    for k in ("tau_s", "fn_hz", "zeta", "static_offset_rad"):
        if k in cfg:
            cfg[k] = round(float(cfg[k]), 5)

    fitq = {"fit": metrics(y_f, simulate(u_f, cfg, dt, y_f[0], True), dt)}
    val_rmse = None
    if val_sets:
        u_v, y_v = crop_active(*max(val_sets, key=lambda p: np.ptp(p[0])), dt)
        fitq["val"] = metrics(y_v, simulate(u_v, cfg, dt, y_v[0], True), dt)
        fitq["val"]["cmd_ptp_rad"] = round(float(np.ptp(u_v)), 4)
        val_rmse = fitq["val"]["rmse_rad"]
    cfg["fit_quality"] = fitq
    if at_bound:
        cfg["fit_warning"] = f"parameters at bounds: {at_bound}"
    if val_rmse is None:
        cfg["confidence"] = "UNVALIDATED"
    else:
        if fitq["val"]["cmd_ptp_rad"] < 0.05:
            cfg["confidence"] = "UNVALIDATED"
            cfg["fit_warning"] = (cfg.get("fit_warning", "") + " validation data barely moves this joint").strip()
        else:
            cfg["confidence"] = "OK" if (val_rmse < VAL_OK_RAD and not at_bound) else "POOR"
    return cfg


def fit_hand(paths, dt):
    """Single shared 1st-order + delay model for all 12 hand DoF in raw units."""
    t, c, m = [], [], []
    for p in paths:
        for d in _read_jsonl(p):
            if d.get("hand_cmd") and d.get("hand_meas") and "t" in d:
                t.append(float(d["t"])); c.append(d["hand_cmd"]); m.append(d["hand_meas"])
    if len(t) < 200:
        return {"model_type": "IDEAL", "confidence": "IDEAL", "reason": f"only {len(t)} hand samples"}
    t, c, m = np.array(t), np.array(c, dtype=float), np.array(m, dtype=float)
    cu, mu = resample(t, c, m, dt)
    sets = [(cu[:, i], mu[:, i]) for i in range(12) if np.ptp(cu[:, i]) > 50]
    if not sets:
        return {"model_type": "IDEAL", "confidence": "IDEAL", "reason": "no hand motion > 50 raw"}
    u = np.concatenate([s[0] for s in sets]); y = np.concatenate([s[1] for s in sets])
    d0 = fit_delay(u, y, 40)
    def mk(p): return {"model_type": "1st_order", "delay_ticks": d0, "tau_s": p[0], "static_offset_rad": p[1]}
    res = optimize.minimize(lambda p: np.mean((simulate(u, mk(p), dt, y[0], False) - y) ** 2),
                            [0.1, 0.0], bounds=[(0.001, 2.0), (-100, 100)], method="L-BFGS-B")
    cfg = mk(res.x)
    v = np.abs(np.diff(y)) / dt
    cfg.update({"max_vel_rad_s": round(float(np.percentile(v, 99.9)) * 1.05, 1), "max_acc_rad_s2": 1e6,
                "units": "raw_0_2000", "tau_s": round(float(res.x[0]), 4), "static_offset_rad": round(float(res.x[1]), 2)})
    cfg["fit_quality"] = {"rmse_raw": round(float(np.sqrt(res.fun)), 2), "samples": int(len(t))}
    cfg["confidence"] = "OK" if np.sqrt(res.fun) < 40 else "POOR"
    return cfg


# ----------------------------------------------------------------------------- main
def mirror_missing(joints):
    """Copies the fitted model of the contralateral joint (i <-> i+7) into IDEAL joints (marked)."""
    for j in range(14):
        k = (j + 7) % 14
        if joints[str(j)]["model_type"] == "IDEAL" and joints[str(k)]["model_type"] != "IDEAL":
            src = {kk: vv for kk, vv in joints[str(k)].items() if kk != "fit_quality"}
            src.update({"name": f"joint_{j}", "confidence": "MIRRORED", "mirrored_from": k,
                        "static_offset_rad": 0.0})
            joints[str(j)] = src


def identify(source, fit_files, val_files, out_path, dt, hand_logs=None, mirror=False):
    print(f"[IdentifyPlant] source={source} dt={dt*1000:.0f}ms fit={len(fit_files)} val={len(val_files)}")
    fit_raw = [load_log(p, source) for p in fit_files]
    val_raw = [load_log(p, source) for p in val_files]
    for t, s, a, meta in fit_raw + val_raw:
        print(f"  {meta['path']}: {meta['samples']} samples, median dt {meta['median_dt_s']*1000:.1f} ms, max gap {meta['max_gap_s']*1000:.0f} ms")
    fit_u = [resample(t, s, a, dt) for t, s, a, _ in fit_raw]
    val_u = [resample(t, s, a, dt) for t, s, a, _ in val_raw]

    try:
        sha = subprocess.check_output(["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        sha = "unknown"

    max_delay = int(round(0.300 / dt))
    joints = {}
    for j in range(14):
        joints[str(j)] = fit_joint(j, [(s[:, j], a[:, j]) for s, a in fit_u],
                                   [(s[:, j], a[:, j]) for s, a in val_u], dt, max_delay)

    if mirror:
        mirror_missing(joints)

    ok_val = [c["fit_quality"]["val"]["rmse_rad"] for c in joints.values() if c.get("confidence") == "OK"]
    n_poor = sum(1 for c in joints.values() if c.get("confidence") == "POOR")
    validated = bool(val_files) and n_poor == 0 and len(ok_val) > 0
    cfg = {
        "metadata": {
            "scope": "end_to_end_laptop" if source == "laptop" else "mc_only_orin",
            "source": source, "dt_s": dt, "git_sha": sha,
            "fit_files": [m for *_, m in fit_raw], "val_files": [m for *_, m in val_raw],
            "overall_val_rms_rad": round(float(np.mean(ok_val)), 5) if ok_val else None,
            "validated": validated,
        },
        "mc_step_reject": {"enabled": False, "threshold_rad_20ms": 0.02, "status": "UNKNOWN (not measured)"},
        "hand": fit_hand(hand_logs, dt) if hand_logs else {"model_type": "IDEAL", "confidence": "IDEAL", "reason": "no hand logs given"},
        "assumptions": {
            "cmd_timeout": "ASSUMED: MC holds last command when arm_joint_command stops",
            "large_step": "ASSUMED: MC tracks any step (no rejection modelled)",
        },
        "joints": joints,
    }
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)

    print("=" * 96)
    print(f"{'J':<4}{'type':<11}{'delay':>7}  {'dynamics':<18}{'fit rms':>9}{'val rms':>9}{'val R2':>8}{'spec':>7}  conf")
    print("-" * 96)
    for k, c in joints.items():
        if c.get("confidence") == "MIRRORED":
            print(f"J{int(k):02d} {c['model_type']:<11}{c['delay_s']*1000:5.0f}ms  MIRRORED from J{c['mirrored_from']:02d}"); continue
        if c["model_type"] == "IDEAL":
            print(f"J{int(k):02d} IDEAL      ({c.get('reason','')})"); continue
        dyn = f"tau={c['tau_s']*1000:.1f}ms" if c["model_type"] == "1st_order" else f"fn={c['fn_hz']:.2f} z={c['zeta']:.2f}"
        fq = c["fit_quality"]; v = fq.get("val", {})
        vr = f"{v['rmse_rad']*1000:8.2f}" if v else "     n/a"
        vr2 = f"{v['r2']:8.3f}" if v else "     n/a"
        vs = f"{v['spectral_match_0_15hz']:7.3f}" if v else "    n/a"
        print(f"J{int(k):02d} {c['model_type']:<11}{c['delay_s']*1000:5.0f}ms  {dyn:<18}{fq['fit']['rmse_rad']*1000:9.2f}{vr}{vr2}{vs}  {c['confidence']}"
              + (f"  [{c['fit_warning']}]" if c.get("fit_warning") else ""))
    print("=" * 96)
    print(f"[IdentifyPlant] Saved {out_path} | validated={validated} | hand={cfg['hand']['model_type']}")
    return cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["laptop", "orin"], required=True)
    ap.add_argument("--fit", nargs="+", required=True)
    ap.add_argument("--val", nargs="*", default=[])
    ap.add_argument("--hand-logs", nargs="*", default=None)
    ap.add_argument("--dt", type=float, default=0.010)
    ap.add_argument("--out", default=None)
    ap.add_argument("--mirror-missing", action="store_true", help="Fill un-excited joints from the opposite arm (marked MIRRORED)")
    a = ap.parse_args()
    out = a.out or ("config/sim_plant.json" if a.source == "laptop" else "config/mc_plant.json")
    identify(a.source, a.fit, a.val, out, a.dt, a.hand_logs, a.mirror_missing)


if __name__ == "__main__":
    main()
