#!/usr/bin/env python3
"""tools/view_episode.py - Look at recorded teleop episodes (laptop, teleop venv).

For each episode folder recordings/<task>/<date>/ep_*/ it writes, next to the data:
  preview.mp4  head colour camera | head depth (colour-mapped), real time, with the joint plot underneath and a moving
               time cursor; text overlay: time, hand closure L/R, ENGAGED / warnings
  plots.png    whole episode: arm joints commanded (dashed) vs measured (solid) for both arms, fingers, hand speed
and prints a one-line summary. Nothing in the episode is changed.

  venv/bin/python tools/view_episode.py                       # the newest episode under recordings/
  venv/bin/python tools/view_episode.py --task pick_cube      # every episode of a task
  venv/bin/python tools/view_episode.py recordings/pick_cube/2026-11-17/ep_0003_101500 --open
  venv/bin/python tools/view_episode.py --all                 # every episode (skips ones that already have a preview)
"""
import argparse, json, subprocess, sys
from pathlib import Path

import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DATA = Path(__file__).resolve().parent.parent / "recordings"
JN = [f"J{i}" for i in range(1, 8)]


def jl(path):
    out = []
    if path.exists():
        for line in open(path):
            try: out.append(json.loads(line))
            except ValueError: pass
    return out


def load(ep):
    meta = json.loads((ep / "meta.json").read_text())
    tele = [r for r in jl(ep / "teleop.jsonl") if r.get("q_cmd") and r.get("q_meas")]
    imgs = jl(ep / "images.jsonl")
    off = meta.get("robot_clock_minus_laptop_s") or 0.0
    if not tele: return None
    t = np.array([r["t"] for r in tele]); t0 = t[0]
    d = {"meta": meta, "t": t - t0, "qc": np.array([r["q_cmd"] for r in tele]), "qm": np.array([r["q_meas"] for r in tele]),
         "hc": np.array([r.get("hand_raw_cmd") or [np.nan] * 12 for r in tele], dtype=float),
         "clo": np.array([[(r.get("left") or {}).get("hand_closure", np.nan), (r.get("right") or {}).get("hand_closure", np.nan)] for r in tele], dtype=float),
         "eng": np.array([r.get("sync_state") == "ENGAGED" for r in tele]),
         "warn": [((r.get("safety") or {}).get("armor_warning") or "") for r in tele],
         "ee": np.array([((r.get("left") or {}).get("ee_pos") or [np.nan] * 3) + ((r.get("right") or {}).get("ee_pos") or [np.nan] * 3) for r in tele], dtype=float)}
    cams = {}
    for c in sorted({i.get("cam", "head") for i in imgs}):
        rows = [i for i in imgs if i.get("cam", "head") == c]
        ts = np.array([(i["stamp"] - off) if i.get("stamp") is not None else i["t_laptop"] for i in rows]) - t0
        o = np.argsort(ts)
        cams[c] = {"t": ts[o], "files": [ep / rows[k]["file"] for k in o], "kind": rows[0].get("kind", "color")}
    d["cams"] = cams
    return d


def plots(d, out):
    t = d["t"]
    fig, ax = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    for a, off, side in ((ax[0], 0, "LEFT"), (ax[1], 7, "RIGHT")):
        for j in range(7):
            l, = a.plot(t, d["qm"][:, off + j], lw=1.1, label=JN[j])
            a.plot(t, d["qc"][:, off + j], "--", lw=0.8, color=l.get_color())
        a.set_ylabel(f"{side} arm rad\n(solid robot, dashed cmd)"); a.legend(ncol=7, fontsize=8, loc="upper right")
    for k, nm in enumerate(["L thumb0", "L thumb1", "L index", "L middle", "L ring", "L pinky", "R thumb0", "R thumb1", "R index", "R middle", "R ring", "R pinky"]):
        if k in (0, 1, 2, 6, 7, 8): ax[2].plot(t, d["hc"][:, k], lw=1, label=nm)
    ax[2].set_ylabel("finger cmd (raw 0..2000)"); ax[2].legend(ncol=6, fontsize=8)
    dt = np.maximum(np.diff(t), 1e-3)
    for j, nm in ((0, "left"), (3, "right")):
        v = np.linalg.norm(np.diff(d["ee"][:, j:j + 3], axis=0), axis=1) / dt
        ax[3].plot(t[1:], np.convolve(np.nan_to_num(v), np.ones(9) / 9, "same"), lw=1, label=f"{nm} hand")
    ax[3].set_ylabel("hand speed m/s"); ax[3].legend(); ax[3].set_xlabel("s")
    ne = ~d["eng"]
    for a in ax:
        if ne.any(): a.fill_between(t, *a.get_ylim(), where=ne, color="grey", alpha=0.15, step="mid")
    m = d["meta"]
    fig.suptitle(f"{m.get('task')} / {m.get('episode')}  -  \"{m.get('instruction', '')}\"  ({t[-1]:.1f} s{', DISCARDED' if m.get('discarded') else ''})")
    fig.tight_layout(); fig.savefig(out, dpi=90); plt.close(fig)


def strip(d, w, h=220):
    """Joint plot image (both arms measured) for the video; returns (img, x_of_time)."""
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100)
    ax = fig.add_axes([0.05, 0.12, 0.93, 0.8])
    for j in range(7): ax.plot(d["t"], d["qm"][:, j], lw=0.8)
    for j in range(7): ax.plot(d["t"], d["qm"][:, 7 + j], lw=0.8, ls="--")
    ax.set_xlim(d["t"][0], d["t"][-1]); ax.tick_params(labelsize=7); ax.set_title("arm joints (solid left, dashed right)", fontsize=8)
    fig.canvas.draw()
    img = cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba())[..., :3], cv2.COLOR_RGB2BGR); plt.close(fig)
    x0, x1 = 0.05 * w, 0.98 * w
    return img, lambda tt: int(x0 + (x1 - x0) * (tt - d["t"][0]) / max(1e-6, d["t"][-1] - d["t"][0]))


def video(d, out, fps=15):
    cams = d["cams"]
    col = next((c for c in cams if cams[c]["kind"] == "color"), None)
    dep = next((c for c in cams if cams[c]["kind"] == "depth"), None)
    if col is None: return False
    im0 = cv2.imread(str(cams[col]["files"][0])); H, W = im0.shape[:2]
    width = W * (2 if dep else 1)
    bg, xt = strip(d, width)
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, H + bg.shape[0]))
    def pick(c, tt):
        k = int(np.clip(np.searchsorted(cams[c]["t"], tt, side="right") - 1, 0, len(cams[c]["t"]) - 1)); return cams[c]["files"][k]
    for tt in np.arange(d["t"][0], d["t"][-1], 1.0 / fps):
        a = cv2.imread(str(pick(col, tt)))
        a = cv2.resize(a, (W, H)) if a is not None else np.zeros((H, W, 3), np.uint8)
        if dep:
            z = cv2.imread(str(pick(dep, tt)), cv2.IMREAD_UNCHANGED)
            if z is not None:
                z = cv2.resize(z, (W, H), interpolation=cv2.INTER_NEAREST).astype(np.float32)
                zz = np.clip(z / 3000.0 * 255.0, 0, 255).astype(np.uint8)          # 0..3 m
                zc = cv2.applyColorMap(zz, cv2.COLORMAP_TURBO); zc[z == 0] = 0
            else: zc = np.zeros((H, W, 3), np.uint8)
            a = np.hstack([a, zc])
        k = int(np.clip(np.searchsorted(d["t"], tt) , 0, len(d["t"]) - 1))
        txt = f"t {tt:5.1f}s  closure L {d['clo'][k, 0]:.2f} R {d['clo'][k, 1]:.2f}  {'ENGAGED' if d['eng'][k] else 'not engaged'}"
        if d["warn"][k]: txt += "  ! " + d["warn"][k][:40]
        cv2.putText(a, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3); cv2.putText(a, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        s = bg.copy(); x = xt(tt); cv2.line(s, (x, 0), (x, s.shape[0] - 1), (0, 0, 255), 2)
        vw.write(np.vstack([a, s]))
    vw.release()
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*", help="episode folders (default: the newest one)")
    ap.add_argument("--task"); ap.add_argument("--all", action="store_true")
    ap.add_argument("--open", action="store_true", help="open the result(s) with the desktop's default player")
    ap.add_argument("--force", action="store_true", help="redo episodes that already have preview.mp4")
    a = ap.parse_args()
    eps = [Path(p) for p in a.paths]
    if not eps:
        eps = sorted({p.parent for p in DATA.glob(f"{a.task or '*'}/*/ep_*/meta.json")}, key=lambda p: (p / "meta.json").stat().st_mtime)
        if not (a.task or a.all): eps = eps[-1:]
    if not eps: sys.exit(f"no episodes under {DATA}")
    for ep in eps:
        if (ep / "preview.mp4").exists() and not a.force and (a.all or a.task):
            print(f"[have] {ep}"); continue
        d = load(ep)
        if d is None: print(f"[skip] {ep}: no teleop rows"); continue
        plots(d, ep / "plots.png")
        ok = video(d, ep / "preview.mp4")
        m = d["meta"]; cams = {c: f"{len(v['t'])} img" for c, v in d["cams"].items()}
        print(f"[ok] {ep}  {d['t'][-1]:.1f} s  task '{m.get('task')}'  \"{m.get('instruction', '')}\"  cams {cams}  "
              f"{'DISCARDED  ' if m.get('discarded') else ''}-> plots.png{', preview.mp4' if ok else ' (no colour camera: no video)'}")
        if a.open:
            for f in (["preview.mp4"] if ok else []) + ["plots.png"]:
                subprocess.Popen(["xdg-open", str(ep / f)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
