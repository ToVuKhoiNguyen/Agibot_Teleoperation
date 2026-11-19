#!/usr/bin/env python3
"""tools/dataset_cache.py - Decode ONE episode of a LeRobot dataset for the dataset viewer (/dataset on the panel).

Run with the exporter environment (needs the `lerobot` library), normally started by the teleop web server:
    venv_lerobot/bin/python tools/dataset_cache.py --root lerobot_datasets/a2_pick --episode 0 --out lerobot_datasets/a2_pick/.viewer_cache/ep_000000

Writes into --out:
    data.json          every non-image feature of the episode as arrays (frame-major), names, fps, timestamps, task
    <cam>/000000.jpg   colour frames (JPEG); depth frames colour-mapped 0..3 m (TURBO, black = no depth)
    done               written last: the cache is complete
The dataset itself is only read.
"""
import argparse, json, sys
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True); ap.add_argument("--episode", type=int, default=None)
    ap.add_argument("--out", required=True); ap.add_argument("--repo-id", default=None)
    ap.add_argument("--jpeg-q", type=int, default=82)
    ap.add_argument("--list", action="store_true", help="write the episode list (index, length, task) to --out as JSON")
    a = ap.parse_args()
    root = Path(a.root)
    info = json.loads((root / "meta" / "info.json").read_text())
    if a.list:
        from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
        meta = LeRobotDatasetMetadata(a.repo_id or f"local/{root.name}", root=root)
        if meta.episodes is None: sys.exit(f"{root}: dataset incomplete (no meta/episodes): export still running or interrupted")
        eps = [{"index": int(e["episode_index"]), "length": int(e["length"]), "tasks": list(e.get("tasks") or [])}
               for e in meta.episodes]
        Path(a.out).parent.mkdir(parents=True, exist_ok=True)
        Path(a.out).write_text(json.dumps({"fps": info.get("fps"), "episodes": eps}))
        print(f"[cache] {len(eps)} episodes listed -> {a.out}")
        return
    if a.episode is None: sys.exit("--episode is required (or --list)")
    import cv2
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ds = LeRobotDataset(a.repo_id or f"local/{root.name}", root=root, episodes=[a.episode])
    feats = ds.meta.features
    img_keys = [k for k, f in feats.items() if f.get("dtype") in ("video", "image")]
    num_keys = [k for k, f in feats.items() if f.get("dtype") not in ("video", "image", "string")
                and k not in ("timestamp", "frame_index", "episode_index", "index", "task_index")]
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    cams = {}
    for k in img_keys:
        short = k.split(".")[-1]
        (out / short).mkdir(exist_ok=True)
        cams[k] = {"dir": short, "depth": bool((feats[k].get("info") or {}).get("is_depth_map")) or "depth" in short}
    arrays = {k: [] for k in num_keys}
    ts, task = [], ""
    n = len(ds)
    for i in range(n):
        f = ds[i]
        ts.append(float(f["timestamp"]))
        task = task or f.get("task", "")
        for k in num_keys:
            v = f[k]
            arrays[k].append([round(float(x), 5) for x in np.atleast_1d(v.numpy() if hasattr(v, "numpy") else v)])
        for k, c in cams.items():
            img = f[k].numpy() if hasattr(f[k], "numpy") else np.asarray(f[k])
            img = np.transpose(img, (1, 2, 0)) if img.ndim == 3 and img.shape[0] in (1, 3) else img
            if c["depth"]:
                z = img[..., 0].astype(np.float32)                         # mm
                zz = cv2.applyColorMap(np.clip(z / 3000.0 * 255.0, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
                zz[(z <= 10.0) | (z >= 9999.0)] = 0                        # no depth
                bgr = zz
            else:
                rgb = np.clip(img * 255.0 if img.dtype != np.uint8 else img, 0, 255).astype(np.uint8)
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(out / c["dir"] / f"{i:06d}.jpg"), bgr, [cv2.IMWRITE_JPEG_QUALITY, a.jpeg_q])
        if i % 100 == 0: print(f"[cache] {i}/{n}", flush=True)
    names = {k: (feats[k].get("names") if isinstance(feats[k].get("names"), list) else None) for k in num_keys}
    (out / "data.json").write_text(json.dumps({
        "episode": a.episode, "fps": info.get("fps"), "frames": n, "t": ts, "task": task,
        "features": arrays, "names": names,
        "cams": [{"key": k, "dir": c["dir"], "depth": c["depth"]} for k, c in cams.items()]}))
    (out / "done").write_text("ok")
    print(f"[cache] episode {a.episode}: {n} frames -> {out}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
