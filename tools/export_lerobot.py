#!/usr/bin/env python3
"""tools/export_lerobot.py - Convert recorded teleop episodes into a LeRobot dataset (v3.0) for VLA training.

Input : recordings/<task>/<YYYY-MM-DD>/ep_*/{meta.json, robot.jsonl, teleop.jsonl, images.jsonl, images/<cam>/*.jpg}
Output: a LeRobot dataset written with the official `lerobot` library (so the format is whatever that library reads).

Run it with the exporter environment, NOT the teleop venv:
    venv_lerobot/bin/python tools/export_lerobot.py --out lerobot_datasets/a2_pick --repo-id local/a2_pick
    venv_lerobot/bin/python tools/export_lerobot.py --task pick_cube --out lerobot_datasets/a2_pick --repo-id local/a2_pick --fps 30

Per frame (default 30 Hz, resampled by "latest sample at or before the frame time"):
    observation.state         62  measured arm joints (14, rad: left 1-7, right 1-7) + measured fingers (12, raw units)
                                  + measured pose of both robot hands (2 x [x, y, z, 6D rotation], robot base frame)
                                  + pose of both Quest controllers (2 x [x, y, z, 6D rotation], headset/XR frame)
                                  (44 without the controllers: --no-controller-pose, or when an episode has none)
    action                    26  joint command the Orin PUBLISHED to the robot (14, rad; after the laptop + pacer speed /
                                  acceleration limits = what the robot executed, it follows within ~60 ms) + finger command
                                  (12, raw units). Episodes without it fall back to the laptop IK target (warning printed).
    action.ik_target          14  the laptop's raw IK target (operator intent before the limiters; runs ahead of the
                                  robot and jumps up to ~14 rad/s on fast moves)
    action.ee_pose            16  commanded hand pose: left pos(3)+quat xyzw(4), right pos(3)+quat(4), left/right closure
    observation.ee_pose       14  measured hand pose: left pos(3)+quat xyzw(4), right pos(3)+quat xyzw(4)
    observation.velocity      14  joint velocity (only if every exported episode has it)
    observation.effort        14  joint torque   (only if every exported episode has it)
    observation.neck           2  neck yaw, pitch (rad)                 (only if every exported episode has it)
    observation.hand_effort_raw N the hand state's whole effort array as published: finger torque and, per the vendor
                                  guide, the pressure-sensor arrays (only if present with one length everywhere)
    observation.images.<cam>      colour camera frames (only if every exported episode has them)
    observation.images.<cam>      depth frames, uint16 as published, flagged is_depth_map (lerobot treats integers as mm)
    task                          the episode's language instruction (meta.json "instruction")
Episodes flagged "discarded" are skipped unless --include-discarded.
Writes meta/a2_sources.json (source episode + first-frame time per dataset episode) for the dataset viewer; a cut set in
the viewer (meta.json "trim" = kept range, "cuts" = removed ranges such as idle pauses, laptop clock) is honoured; --from-dataset DIR re-exports exactly that dataset's episodes; --append adds only new episodes to an existing dataset
(automatic export after each recording).
Idle lead-in / tail is cut automatically (robot hands, finger command and hand target all still; 0.5 s kept at each end,
--trim-pad / --idle-speed to tune, --no-trim to keep everything). An episode in which nothing ever moved is skipped.
"""
import argparse, json, shutil, sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]


def source_dir(rel: str) -> Path:
    """Source episode folder recorded in meta/a2_sources.json (datasets exported before the 17/11 rename say data/...)."""
    p = REPO / rel
    if not p.exists() and rel.startswith("data/") and (REPO / "recordings" / rel[5:]).exists(): p = REPO / "recordings" / rel[5:]
    return p


ARM = [f"{s}_arm_joint{i}" for s in ("left", "right") for i in range(1, 8)]
FINGERS = [f"{s}_{f}" for s in ("left", "right") for f in ("thumb_0", "thumb_1", "index", "middle", "ring", "pinky")]
POSE7 = ["x", "y", "z", "qx", "qy", "qz", "qw"]
POSE9 = ["x", "y", "z", "r00", "r10", "r20", "r01", "r11", "r21"]     # position + first two columns of the rotation matrix


def quat_xyzw_to_6d(q):
    """Nx4 quaternions (x, y, z, w) -> Nx6: the first two columns of the rotation matrix (continuous, no sign flips)."""
    q = np.asarray(q, dtype=np.float64)
    q = q / np.maximum(np.linalg.norm(q, axis=-1, keepdims=True), 1e-12)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    c0 = np.stack([1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w)], axis=-1)
    c1 = np.stack([2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)], axis=-1)
    return np.concatenate([c0, c1], axis=-1).astype(np.float32)


def pose7_to_pose9(p7):
    """N x (2 x [pos3 + quat xyzw]) -> N x (2 x [pos3 + rot6d])."""
    p7 = np.asarray(p7, dtype=np.float32)
    return np.concatenate([p7[:, 0:3], quat_xyzw_to_6d(p7[:, 3:7]), p7[:, 7:10], quat_xyzw_to_6d(p7[:, 10:14])], axis=1)


def read_jsonl(path):
    rows = []
    if path.exists():
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    try: rows.append(json.loads(line))
                    except json.JSONDecodeError: pass      # a torn last line after an unclean stop
    return rows


def load_episode(ep_dir: Path):
    """-> dict with time-aligned arrays, or (None, reason)."""
    meta = json.loads((ep_dir / "meta.json").read_text())
    robot, teleop, images = read_jsonl(ep_dir / "robot.jsonl"), read_jsonl(ep_dir / "teleop.jsonl"), read_jsonl(ep_dir / "images.jsonl")
    robot = [r for r in robot if r.get("q") and len(r["q"]) == 14]
    teleop = [r for r in teleop if r.get("q_cmd") and len(r["q_cmd"]) == 14]
    # The first few packets of an episode arrive before the robot has seen the "recording" flag and lack the extra
    # fields (MC stamp, velocity, torque). If most rows have them, keep only the complete rows.
    for key in ("stamp_mc", "dq", "tau", "hand_tau"):
        have = [r for r in robot if r.get(key) is not None]
        if len(have) >= 0.9 * len(robot): robot = have
    if len(robot) < 10: return None, f"only {len(robot)} robot rows (was the receiver running?)"
    if len(teleop) < 10: return None, f"only {len(teleop)} teleop rows"

    # Robot rows on the laptop time axis. The MC stamp is the best clock when it is the same clock as the Orin's
    # (checked against the Orin send time); otherwise fall back to the laptop arrival time.
    off = meta.get("robot_clock_minus_laptop_s")
    t_arr = np.array([r["t_laptop"] for r in robot])
    stamp = np.array([r["stamp_mc"] if r.get("stamp_mc") is not None else np.nan for r in robot])
    t_rob = np.array([r["t_robot"] if r.get("t_robot") is not None else np.nan for r in robot])
    if off is not None and np.all(np.isfinite(stamp)) and np.nanmedian(np.abs(stamp - t_rob)) < 0.2:
        t_robot_l, robot_clock = stamp - off, "mc_stamp"
    else:
        t_robot_l, robot_clock = t_arr, "laptop_arrival"
    order = np.argsort(t_robot_l, kind="stable")
    robot, t_robot_l = [robot[i] for i in order], t_robot_l[order]
    t_tel = np.array([r["t"] for r in teleop])

    def field(rows, key, n):
        ok = all(r.get(key) is not None and len(r[key]) == n for r in rows)
        return np.array([r[key] for r in rows], dtype=np.float32) if ok else None

    def pose(rows, pos_key, quat_key):
        out = np.zeros((len(rows), 14), dtype=np.float32)
        for k, r in enumerate(rows):
            for j, side in enumerate(("left", "right")):
                p, q = r[side].get(pos_key), r[side].get(quat_key)
                if p is None or q is None: return None
                out[k, 7 * j:7 * j + 3], out[k, 7 * j + 3:7 * j + 7] = p, q
        return out

    hand_meas = field(robot, "hand", 12)
    hand_cmd = field(teleop, "hand_raw_cmd", 12)
    ee_meas, ee_cmd = pose(teleop, "ee_pos", "ee_quat_xyzw"), pose(teleop, "target_pos", "target_quat_xyzw")
    if hand_meas is None: return None, "robot rows have no finger positions ('hand')"
    if hand_cmd is None: return None, "teleop rows have no finger command ('hand_raw_cmd')"
    if ee_meas is None or ee_cmd is None: return None, "teleop rows miss hand poses"
    closure = np.array([[r["left"]["hand_closure"], r["right"]["hand_closure"]] for r in teleop], dtype=np.float32)

    # Quest controllers: position + quaternion (logged w, x, y, z) in the XR frame. A controller that is not tracked
    # yet has no pose; hold the last tracked pose (first tracked pose for the leading rows). None if never tracked.
    ctrl = np.full((len(teleop), 14), np.nan, dtype=np.float64)
    for k, r in enumerate(teleop):
        for j, side in enumerate(("left", "right")):
            p, q = r[side].get("ctrl_pos"), r[side].get("ctrl_quat_wxyz")
            if p is not None and q is not None and len(q) == 4:
                ctrl[k, 7 * j:7 * j + 3] = p
                ctrl[k, 7 * j + 3:7 * j + 7] = [q[1], q[2], q[3], q[0]]
    ctrl_ok = True
    for j in range(2):
        col = ctrl[:, 7 * j:7 * j + 7]
        good = np.flatnonzero(np.isfinite(col).all(axis=1))
        if len(good) == 0: ctrl_ok = False; break
        idx = np.maximum.accumulate(np.where(np.isfinite(col).all(axis=1), np.arange(len(col)), -1))
        idx[idx < 0] = good[0]
        ctrl[:, 7 * j:7 * j + 7] = col[idx]
    ep = {"dir": ep_dir, "meta": meta, "robot_clock": robot_clock, "ctrl": pose7_to_pose9(ctrl) if ctrl_ok else None,
          "t_robot": t_robot_l, "q": field(robot, "q", 14), "hand": hand_meas, "dq": field(robot, "dq", 14), "tau": field(robot, "tau", 14),
          "t_tel": t_tel, "q_cmd": field(teleop, "q_cmd", 14), "hand_cmd": hand_cmd, "ee_meas": ee_meas,
          "orin_cmd": field(robot, "cmd", 14),          # what the Orin PUBLISHED to the MC (after speed / accel limits)
          "ee_cmd": np.concatenate([ee_cmd, closure], axis=1), "cams": {}}
    neck = [r.get("neck") for r in robot]
    ep["neck"] = np.array([n[:2] for n in neck], dtype=np.float32) if all(n is not None and len(n) >= 2 for n in neck) else None
    eff = [r.get("hand_effort_raw") for r in robot]
    ep["hand_effort_raw"] = (np.array(eff, dtype=np.float32) if eff and all(e is not None and len(e) == len(eff[0]) and len(e) > 0 for e in eff) else None)
    for cam in sorted({i.get("cam", "head") for i in images}):
        rows = [i for i in images if i.get("cam", "head") == cam]
        if len(rows) < 2: continue
        stamps = [i.get("stamp", i.get("stamp_robot")) for i in rows]
        if off is not None and all(x is not None for x in stamps):
            t_img = np.array(stamps) - off                                    # image stamp is on the Orin clock
        else:
            t_img = np.array([i["t_laptop"] for i in rows])
        o = np.argsort(t_img, kind="stable")
        ep["cams"][cam] = {"rows": [rows[i] for i in o], "t": t_img[o], "kind": rows[0].get("kind", "color")}
    return ep, ""


def active_window(ep, idle_speed=0.005, finger_rate=50.0, pad=0.5):
    """-> (t_first, t_last) of activity on the teleop time axis, or None if nothing ever moved.
    Active = a measured robot hand moves faster than idle_speed (m/s), or a finger command changes faster than
    finger_rate (raw units/s), or a commanded hand target moves faster than idle_speed. Everything before the first /
    after the last active moment except `pad` seconds is idle lead-in / tail (recording started early / stopped late)."""
    t = ep["t_tel"]
    if len(t) < 3: return None
    dt = np.maximum(np.diff(t), 1e-3)
    k = 9                                                          # ~0.1 s moving average against sensor noise
    sm = lambda x: np.apply_along_axis(lambda c: np.convolve(np.pad(c, (k // 2, k - 1 - k // 2), mode="edge"), np.ones(k) / k, mode="valid"), 0, x)
    act = np.zeros(len(t) - 1, dtype=bool)
    for arr in (ep["ee_meas"], ep["ee_cmd"]):
        P = sm(np.asarray(arr, dtype=np.float64))
        for j in (0, 7):
            act |= np.linalg.norm(np.diff(P[:, j:j + 3], axis=0), axis=1) / dt > idle_speed
    H = sm(np.asarray(ep["hand_cmd"], dtype=np.float64))
    act |= np.abs(np.diff(H, axis=0)).max(axis=1) / dt > finger_rate
    idx = np.flatnonzero(act)
    if len(idx) == 0: return None
    return t[idx[0]] - pad, t[idx[-1] + 1] + pad


def frame_times(ep, fps):
    starts, ends = [ep["t_robot"][0], ep["t_tel"][0]], [ep["t_robot"][-1], ep["t_tel"][-1]]
    for cam in ep.get("use_cams", ep["cams"]):
        starts.append(ep["cams"][cam]["t"][0]); ends.append(ep["cams"][cam]["t"][-1])
    if ep.get("trim") is not None:                                # idle lead-in / tail removed (active_window)
        starts.append(ep["trim"][0]); ends.append(ep["trim"][1])
    mt = ep["meta"].get("trim")                                   # cut set by hand in the dataset viewer (laptop clock)
    if isinstance(mt, (list, tuple)) and len(mt) == 2 and mt[0] is not None and mt[1] is not None:
        starts.append(float(mt[0])); ends.append(float(mt[1]))
    t0, t1 = max(starts), min(ends)
    n = int(np.floor((t1 - t0) * fps)) + 1
    t = t0 + np.arange(max(n, 0)) / fps
    cuts = ep["meta"].get("cuts")                                 # ranges removed in the dataset viewer (laptop clock): frames inside are dropped
    if cuts:
        keep = np.ones(len(t), dtype=bool)
        for c in cuts:
            if isinstance(c, (list, tuple)) and len(c) == 2: keep &= ~((t >= float(c[0])) & (t < float(c[1])))
        t = t[keep]
    return t


def latest(t_src, t):
    return np.clip(np.searchsorted(t_src, t, side="right") - 1, 0, len(t_src) - 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default="recordings", help="root of the recorded episodes (default: recordings)")
    ap.add_argument("--task", default=None, help="only this task folder (default: all tasks)")
    ap.add_argument("--out", required=True, help="output dataset folder (must not exist)")
    ap.add_argument("--repo-id", required=True, help="LeRobot repo id, e.g. local/a2_pick (nothing is uploaded)")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--robot-type", default="agibot_a2_ultra")
    ap.add_argument("--include-discarded", action="store_true")
    ap.add_argument("--no-images", action="store_true", help="export state/action only even if images exist")
    ap.add_argument("--no-depth", action="store_true", help="leave the depth stream out")
    ap.add_argument("--no-controller-pose", action="store_true",
                    help="leave the Quest controller poses out of observation.state (they do not exist when a trained "
                         "policy runs without an operator)")
    ap.add_argument("--overwrite", action="store_true", help="delete --out first if it exists")
    ap.add_argument("--from-dataset", default=None,
                    help="export exactly the source episodes of an earlier export (its meta/a2_sources.json): re-export "
                         "after edits made in the dataset viewer (discarded / instruction / trim in the episodes' meta.json)")
    ap.add_argument("--append", action="store_true",
                    help="add only the episodes not in --out yet (by meta/a2_sources.json); creates --out if missing. Used "
                         "by the automatic export after each recording (teleop/app/episode_recorder.py)")
    ap.add_argument("--size", default="1280x720", help="WxH of every camera video, colour and depth (default 1280x720; frames recorded at another size are resized)")
    ap.add_argument("--no-trim", action="store_true", help="keep the idle lead-in / tail of every episode")
    ap.add_argument("--trim-pad", type=float, default=0.5, help="seconds of stillness kept before the first / after the last motion (default 0.5)")
    ap.add_argument("--idle-speed", type=float, default=0.005, help="hand speed (m/s) below which the robot counts as still (default 0.005)")
    a = ap.parse_args()

    root = Path(a.data)
    if a.from_dataset:
        src = Path(a.from_dataset) / "meta" / "a2_sources.json"
        if not src.exists(): sys.exit(f"{src} not found: that dataset was exported before sources were recorded (17/11) - export it again from recordings/")
        ep_dirs = [source_dir(e["source"]) for e in json.loads(src.read_text())["episodes"]]
        missing = [d for d in ep_dirs if not (d / "meta.json").exists()]
        if missing: sys.exit(f"source episodes no longer on disk: {', '.join(map(str, missing[:5]))}")
    else:
        ep_dirs = sorted(p.parent for p in root.glob(f"{a.task or '*'}/*/ep_*/meta.json"))
    if not ep_dirs: sys.exit(f"no episodes under {root}/{a.task or '*'}/<date>/ep_*")
    out = Path(a.out)
    prev_sources = []
    if a.append and (out / "meta" / "info.json").exists():
        sf = out / "meta" / "a2_sources.json"
        if not sf.exists(): sys.exit(f"{out} has no meta/a2_sources.json (exported before 17/11): cannot append - export it again without --append")
        prev_sources = json.loads(sf.read_text())["episodes"]
        done = {source_dir(e["source"]).resolve() for e in prev_sources}
        ep_dirs = [d for d in ep_dirs if d.resolve() not in done]
        if not ep_dirs:
            print(f"nothing new to append to {out} ({len(prev_sources)} episodes already there)"); return
    episodes, skipped = [], []
    for d in ep_dirs:
        meta = json.loads((d / "meta.json").read_text())
        if meta.get("discarded") and not a.include_discarded: skipped.append((d, "flagged discarded" + (f" ({meta['discard_reason']})" if meta.get("discard_reason") else ""))); continue
        ep, why = load_episode(d)
        if ep is None: skipped.append((d, why)); continue
        episodes.append(ep)
    for d, why in skipped: print(f"[skip] {d}: {why}")
    if not episodes: sys.exit("nothing to export")

    if not a.no_trim:
        kept = []
        for ep in episodes:
            full = ep["t_tel"][-1] - ep["t_tel"][0]
            w = active_window(ep, idle_speed=a.idle_speed, pad=max(0.0, a.trim_pad))
            if w is None: print(f"[skip] {ep['dir']}: the robot never moved (nothing to train on; --no-trim keeps it)"); continue
            ep["trim"] = w
            lead, tail = max(0.0, w[0] - ep["t_tel"][0]), max(0.0, ep["t_tel"][-1] - w[1])
            if lead > 0.05 or tail > 0.05:
                print(f"[trim] {ep['dir']}: cut {lead:.1f} s idle at the start, {tail:.1f} s at the end (kept {full - lead - tail:.1f} of {full:.1f} s)")
            kept.append(ep)
        episodes = kept
        if not episodes: sys.exit("nothing to export")
    for ep in episodes:
        if ep["orin_cmd"] is None:
            print(f"[warn] {ep['dir']}: no Orin command in robot.jsonl -> action uses the laptop IK target for this episode")
    cams = [] if a.no_images else sorted(set.intersection(*[set(ep["cams"]) for ep in episodes]))
    if a.no_depth: cams = [c for c in cams if episodes[0]["cams"][c]["kind"] != "depth"]
    for ep in episodes: ep["use_cams"] = cams
    short = [ep for ep in episodes if len(frame_times(ep, a.fps)) < 2]
    for ep in short: print(f"[skip] {ep['dir']}: shorter than two frames")
    episodes = [ep for ep in episodes if ep not in short]
    if not episodes: sys.exit("nothing to export")
    use_images = bool(cams)
    use_dq, use_tau = all(ep["dq"] is not None for ep in episodes), all(ep["tau"] is not None for ep in episodes)
    use_neck = all(ep["neck"] is not None for ep in episodes)
    n_eff = {ep["hand_effort_raw"].shape[1] if ep["hand_effort_raw"] is not None else 0 for ep in episodes}
    use_eff = len(n_eff) == 1 and 0 not in n_eff
    if not any(episodes[0]["cams"][c]["kind"] == "color" for c in cams) and not a.no_images:
        print("[warn] no colour camera present in every episode -> exporting WITHOUT colour frames (not usable for a VLA)")
    use_ctrl = (not a.no_controller_pose) and all(ep["ctrl"] is not None for ep in episodes)
    if not use_ctrl and not a.no_controller_pose:
        print("[warn] at least one episode has no Quest controller pose -> observation.state exported WITHOUT controller poses")
    state_names = ARM + FINGERS + [f"left_hand_{k}" for k in POSE9] + [f"right_hand_{k}" for k in POSE9]
    if use_ctrl: state_names += [f"left_controller_{k}" for k in POSE9] + [f"right_controller_{k}" for k in POSE9]
    features = {
        "observation.state": {"dtype": "float32", "shape": (len(state_names),), "names": state_names},
        "action": {"dtype": "float32", "shape": (26,), "names": ARM + FINGERS},
        "action.ik_target": {"dtype": "float32", "shape": (14,), "names": ARM},
        "action.ee_pose": {"dtype": "float32", "shape": (16,), "names": [f"left_{k}" for k in POSE7] + [f"right_{k}" for k in POSE7] + ["left_closure", "right_closure"]},
        "observation.ee_pose": {"dtype": "float32", "shape": (14,), "names": [f"left_{k}" for k in POSE7] + [f"right_{k}" for k in POSE7]},
    }
    if use_dq: features["observation.velocity"] = {"dtype": "float32", "shape": (14,), "names": ARM}
    if use_tau: features["observation.effort"] = {"dtype": "float32", "shape": (14,), "names": ARM}
    if use_neck: features["observation.neck"] = {"dtype": "float32", "shape": (2,), "names": ["neck_yaw", "neck_pitch"]}
    if use_eff:
        n = n_eff.pop()
        features["observation.hand_effort_raw"] = {"dtype": "float32", "shape": (n,), "names": [f"hand_effort_{i}" for i in range(n)]}
    cam_keys = {}
    if use_images:
        from PIL import Image
        first = episodes[0]
        for cam in cams:
            # Fixed output size (--size, default 1280x720) for colour and depth, whatever size each episode was recorded at.
            sizes = set()
            for ep in episodes:
                with Image.open(ep["dir"] / ep["cams"][cam]["rows"][0]["file"]) as im: sizes.add(im.size)
            try: w, h = (int(v) for v in a.size.lower().split("x"))
            except ValueError: sys.exit(f"--size must look like 1280x720, got {a.size!r}")
            if sizes != {(w, h)}: print(f"[note] {cam}: episodes recorded at {sorted(sizes)} -> all frames at {w}x{h}")
            key = f"observation.images.{cam}"
            if first["cams"][cam]["kind"] == "depth":
                features[key] = {"dtype": "video", "shape": (h, w, 1), "names": ["height", "width", "channels"], "info": {"is_depth_map": True}}
            else:
                features[key] = {"dtype": "video", "shape": (h, w, 3), "names": ["height", "width", "channels"]}
            cam_keys[cam] = key

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    if prev_sources:                                              # --append onto an existing dataset
        ds = LeRobotDataset.resume(a.repo_id, root=out)
        have = {k: tuple(v["shape"]) for k, v in ds.meta.features.items() if k not in ("timestamp", "frame_index", "episode_index", "index", "task_index")}
        want = {k: tuple(v["shape"]) for k, v in features.items()}
        if have != want or ds.meta.fps != a.fps:
            diff = sorted(set(have.items()) ^ set(want.items()))
            sys.exit(f"cannot append: the new episodes do not have the same features as {out} ({diff[:6]}{' fps' if ds.meta.fps != a.fps else ''}). "
                     f"Export them to another dataset, or re-export everything with --overwrite.")
    else:
        if out.exists():
            if not a.overwrite and not a.append: sys.exit(f"{out} already exists (use --overwrite to replace it)")
            if a.append and any(out.iterdir()): sys.exit(f"{out} exists but is not a dataset: move it away first")
            shutil.rmtree(out)
        ds = LeRobotDataset.create(repo_id=a.repo_id, fps=a.fps, features=features, root=out, robot_type=a.robot_type, use_videos=use_images)

    total, sources = 0, list(prev_sources)
    for ep in episodes:
        t = frame_times(ep, a.fps)
        ir, it = latest(ep["t_robot"], t), latest(ep["t_tel"], t)
        ii = {cam: latest(ep["cams"][cam]["t"], t) for cam in cams}
        ee9 = pose7_to_pose9(ep["ee_meas"])
        task = (ep["meta"].get("instruction") or ep["meta"].get("task", "")).strip() or "teleoperation"
        for k in range(len(t)):
            frame = {
                "observation.state": np.concatenate([ep["q"][ir[k]], ep["hand"][ir[k]], ee9[it[k]]] + ([ep["ctrl"][it[k]]] if use_ctrl else [])).astype(np.float32),
                "action": np.concatenate([ep["orin_cmd"][ir[k]] if ep["orin_cmd"] is not None else ep["q_cmd"][it[k]],
                                          ep["hand_cmd"][it[k]]]).astype(np.float32),
                "action.ik_target": ep["q_cmd"][it[k]].astype(np.float32),
                "action.ee_pose": ep["ee_cmd"][it[k]],
                "observation.ee_pose": ep["ee_meas"][it[k]],
                "task": task,
            }
            if use_dq: frame["observation.velocity"] = ep["dq"][ir[k]]
            if use_tau: frame["observation.effort"] = ep["tau"][ir[k]]
            if use_neck: frame["observation.neck"] = ep["neck"][ir[k]]
            if use_eff: frame["observation.hand_effort_raw"] = ep["hand_effort_raw"][ir[k]]
            for cam in cams:
                c = ep["cams"][cam]
                with Image.open(ep["dir"] / c["rows"][ii[cam][k]]["file"]) as im:
                    fh, fw = features[cam_keys[cam]]["shape"][:2]
                    if im.size != (fw, fh):                       # episodes recorded at another camera size (e.g. 640 vs 1280 px)
                        im = im.resize((fw, fh), Image.NEAREST if c["kind"] == "depth" else Image.BILINEAR)
                    if c["kind"] == "depth": frame[cam_keys[cam]] = np.asarray(im).astype(np.uint16)[..., None]
                    else: frame[cam_keys[cam]] = np.asarray(im.convert("RGB"))
            ds.add_frame(frame)
        ds.save_episode()
        try: rel = str(Path(ep["dir"]).resolve().relative_to(REPO))
        except ValueError: rel = str(Path(ep["dir"]).resolve())
        sources.append({"episode_index": len(sources), "source": rel, "t0_laptop": float(t[0]), "frames": int(len(t)),
                        "fps": a.fps, "t_laptop": [round(float(x), 3) for x in t], "auto_trim": list(map(float, ep["trim"])) if ep.get("trim") is not None else None})
        total += len(t)
        print(f"[ok] {ep['dir']}: {len(t)} frames ({len(t) / a.fps:.1f} s), task '{task}', robot clock: {ep['robot_clock']}")
    if hasattr(ds, "finalize"): ds.finalize()
    # which recorded episode each dataset episode came from + its first frame on the laptop clock: the dataset viewer
    # (/dataset on the teleop panel) reads safety events / quality from there and writes edits back to it
    (out / "meta").mkdir(parents=True, exist_ok=True)
    (out / "meta" / "a2_sources.json").write_text(json.dumps({"data_root": str(root), "episodes": sources}, indent=1))
    print(f"\nLeRobot dataset written: {out}  ({len(episodes)} episodes, {total} frames, {a.fps} fps, "
          f"observation.state: {len(state_names)} (controller poses: {use_ctrl}), cameras: {cams or 'NONE'}, velocity: {use_dq}, effort: {use_tau}, neck: {use_neck}, hand_effort_raw: {use_eff})")


if __name__ == "__main__":
    main()
