#!/usr/bin/env python3
"""tools/check_episode.py - Screen recorded episodes for ones that are probably not worth training on.

Reads recordings/<task>/<date>/ep_*/{meta.json, robot.jsonl, teleop.jsonl, images.jsonl} and prints one verdict per episode:
    OK     nothing suspicious
    CHECK  something worth looking at (possible drop, jerky, long idle, ...)
    BAD    almost certainly unusable (robot fault, feedback lost, never engaged, no motion, ...)

It only READS by default. Nothing is decided for you: look at the CHECK ones, then flag the real failures with
the X button / Discard last, or let `--mark` set the discarded flag on BAD episodes (reason saved in meta.json).

    python tools/check_episode.py                                   # every episode under recordings/
    python tools/check_episode.py --task pick_cup                   # one task
    python tools/check_episode.py recordings/pick_cup/2026-11-16          # a day, a task, or one episode folder
    python tools/check_episode.py --task pick_cup -v                # also print every metric
    python tools/check_episode.py --task pick_cup --mark            # set discarded=true on BAD episodes
    python tools/check_episode.py --json report.json                # machine-readable report

All thresholds are the constants below (or CLI flags where noted). They are starting points, not ground truth:
the drop and jerk checks are heuristics on raw robot units. Jerk is also compared with the other episodes of the
same run (median + 3 MAD), which is more reliable than the absolute number.
"""
import argparse, json, sys
from pathlib import Path

import numpy as np

DATA_ROOT = Path(__file__).resolve().parent.parent / "recordings"

# ---- thresholds --------------------------------------------------------------------------------------------
# 17/11: the CHECK (warning) thresholds were loosened after the operator found the robot responding well while the
# episode was still flagged (random arm-waving test: reach edge 24 %, fast moves). The BAD thresholds that mean the data
# is unusable or unsafe (fault, E-STOP, never engaged, too short, data holes, robot far from its command, motion held back
# by the collision filters) are unchanged or only slightly relaxed. `--strict` restores the former values (STRICT below).
THRESHOLDS = dict(
    MIN_DURATION_S=3.0,            # shorter than this: accidental tap
    MIN_PATH_M=0.05,               # both hands together moved less than this: nothing was demonstrated
    FEEDBACK_GAP_BAD_S=0.5,        # robot feedback silent this long: data has a hole
    FEEDBACK_GAP_WARN_S=0.2,       # was 0.1
    MIN_ROBOT_HZ=50.0,             # expected ~100 Hz
    ENGAGED_MIN_FRAC=0.80,         # was 0.90: fraction of laptop samples in sync_state ENGAGED
    TRACK_ERR_WARN_RAD=0.50,       # was 0.35: |Orin command - measured| max over joints (normal < 0.15; a joint the MC stopped driving is > 1)
    TRACK_ERR_BAD_RAD=1.00,        # was 0.80
    IDLE_SPEED_MPS=0.005,          # hand speed below this = standing still
    IDLE_EDGE_WARN_S=5.0,          # was 3.0: still at the very start / end (the exporter trims it anyway)
    IDLE_MID_WARN_S=8.0,           # was 5.0: both hands still in the middle: hesitation
    REACH_GAIN_WARN=0.5,           # reach_gain below this = near the edge of the workspace
    REACH_GAIN_WARN_FRAC=0.40,     # was 0.20
    CLOSURE_ON=0.5,                # hand_closure above this = a grasp was commanded
    DROP_FORCE_FRAC=0.30,          # was 0.40: grip force falls below this fraction of its recent peak ...
    DROP_WINDOW_S=0.5,             # ... within this window ...
    DROP_MIN_PEAK_FRAC=0.30,       # ... and the peak was at least this fraction of the episode's max grip force
    CAM_GAP_WARN_S=1.0,            # was 0.5
    JERK_MIN_FLAG=200.0,           # was 100: rad/s^3: never call an episode jerky below this (uncalibrated)
    BLOCKED_WARN_FRAC=0.05,        # was 0.02: fraction of time a collision filter held the motion back
    BLOCKED_BAD_FRAC=0.40,         # was 0.30
    SLOWED_SCALE=0.4,              # was 0.5: proxy_scale below this = soft collision slowed the arms hard
    SLOWED_WARN_FRAC=0.25,         # was 0.10
    CLOSE_WARN_MM=5.0,             # was 10: armor / proxy distance below this at any time
    IK_ERR_WARN_MM=30.0,           # was 20: p95 IK position miss
    MAD_K=4.0,                     # was 3: outlier = median + MAD_K * MAD across the batch (needs >= 5 episodes)
)
STRICT = dict(MIN_DURATION_S=3.0, MIN_PATH_M=0.05, FEEDBACK_GAP_BAD_S=0.5, FEEDBACK_GAP_WARN_S=0.1, MIN_ROBOT_HZ=50.0, ENGAGED_MIN_FRAC=0.90,
              TRACK_ERR_WARN_RAD=0.35, TRACK_ERR_BAD_RAD=0.80, IDLE_SPEED_MPS=0.005, IDLE_EDGE_WARN_S=3.0, IDLE_MID_WARN_S=5.0,
              REACH_GAIN_WARN=0.5, REACH_GAIN_WARN_FRAC=0.20, CLOSURE_ON=0.5, DROP_FORCE_FRAC=0.40, DROP_WINDOW_S=0.5,
              DROP_MIN_PEAK_FRAC=0.30, CAM_GAP_WARN_S=0.5, JERK_MIN_FLAG=100.0, BLOCKED_WARN_FRAC=0.02, BLOCKED_BAD_FRAC=0.30,
              SLOWED_SCALE=0.5, SLOWED_WARN_FRAC=0.10, CLOSE_WARN_MM=10.0, IK_ERR_WARN_MM=20.0, MAD_K=3.0)
globals().update(THRESHOLDS)


def set_strict(on: bool = True):
    """Switch every threshold above between the loosened (default) and the former strict values."""
    globals().update(STRICT if on else THRESHOLDS)


def read_jsonl(path: Path):
    rows = []
    if not path.exists():
        return rows
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass          # last line may be cut if the laptop was killed mid-write
    return rows


def find_episodes(paths):
    found = []
    for p in paths:
        p = Path(p)
        if (p / "meta.json").exists():
            found.append(p)
        else:
            found.extend(sorted(m.parent for m in p.rglob("meta.json")))
    seen, out = set(), []
    for e in found:
        if e not in seen:
            seen.add(e); out.append(e)
    return out


def _smooth(x, n=5):
    if len(x) < n:
        return x
    k = np.ones(n) / n
    pad = n // 2
    xp = np.pad(x, ((pad, pad), (0, 0)), mode="edge")
    return np.stack([np.convolve(xp[:, j], k, mode="valid") for j in range(x.shape[1])], axis=1)


def _longest_run(mask, t):
    """Longest duration of consecutive True in mask. Returns (seconds, start_index)."""
    best, best_i, i, n = 0.0, 0, 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            d = t[j] - t[i]
            if d > best:
                best, best_i = d, i
            i = j + 1
        else:
            i += 1
    return best, best_i


def analyse(ep: Path):
    meta = json.loads((ep / "meta.json").read_text())
    robot = read_jsonl(ep / "robot.jsonl")
    tele = read_jsonl(ep / "teleop.jsonl")
    imgs = read_jsonl(ep / "images.jsonl")
    m, bad, warn = {}, [], []
    dur = float(meta.get("duration_s") or 0.0)
    m["duration_s"] = dur
    m["robot_rows"], m["teleop_rows"], m["image_rows"] = len(robot), len(tele), len(imgs)

    if "stop_unix" not in meta:
        bad.append("episode never closed (laptop stopped while recording?)")
    if dur and dur < MIN_DURATION_S:
        bad.append(f"too short ({dur:.1f}s)")
    if not robot:
        bad.append("no robot.jsonl rows (no robot feedback recorded)")
    if not tele:
        bad.append("no teleop.jsonl rows")

    # ---- robot feedback: holes, rate, faults -------------------------------------------------------------
    if len(robot) > 2:
        tl = np.array([r["t_laptop"] for r in robot], dtype=float)
        gaps = np.diff(tl)
        m["feedback_max_gap_s"] = float(gaps.max())
        span = tl[-1] - tl[0]
        m["feedback_hz"] = float(len(tl) / span) if span > 0 else 0.0
        if m["feedback_max_gap_s"] > FEEDBACK_GAP_BAD_S:
            bad.append(f"robot feedback gap {m['feedback_max_gap_s']:.2f}s")
        elif m["feedback_max_gap_s"] > FEEDBACK_GAP_WARN_S:
            warn.append(f"robot feedback gap {m['feedback_max_gap_s']*1000:.0f}ms")
        if 0 < m["feedback_hz"] < MIN_ROBOT_HZ:
            warn.append(f"robot feedback only {m['feedback_hz']:.0f} Hz")
        faults = [r for r in robot if r.get("robot_fault")]
        states = {str(r.get("rx_state")) for r in robot if r.get("rx_state")}
        m["robot_fault_rows"] = len(faults)
        if faults:
            bad.append(f"robot fault latched ({len(faults)} rows)")
        fs = sorted(s for s in states if s.startswith("FAULT"))
        if fs:
            bad.append("receiver state " + ",".join(fs))

        # jerk of the measured joints: smooth, then third difference (rad/s^3)
        q = [r.get("q") for r in robot]
        if all(isinstance(v, list) and len(v) == 14 for v in q):
            q = _smooth(np.array(q, dtype=float))
            v = np.gradient(q, tl, axis=0)
            a = np.gradient(v, tl, axis=0)
            j = np.gradient(a, tl, axis=0)
            moving = np.linalg.norm(v, axis=1) > 0.02
            hole = np.zeros(len(tl), dtype=bool)          # samples next to a feedback hole: derivative is meaningless there
            for k in np.where(gaps > FEEDBACK_GAP_WARN_S)[0]:
                hole[max(k - 3, 0):k + 5] = True
            moving &= ~hole
            if moving.sum() > 20:
                jm = np.abs(j[moving])
                m["jerk_p99"] = float(np.percentile(jm.max(axis=1), 99))
                m["jerk_rms"] = float(np.sqrt((jm ** 2).mean()))

    # ---- laptop-side stream: engaged, tracking, motion, idle, reach --------------------------------------
    if len(tele) > 10:
        tt = np.array([r["t"] for r in tele], dtype=float)
        eng = np.array([r.get("sync_state") == "ENGAGED" for r in tele])
        m["engaged_frac"] = float(eng.mean())
        if m["engaged_frac"] < 0.05:
            bad.append("never ENGAGED (arms were not being driven)")
        elif m["engaged_frac"] < ENGAGED_MIN_FRAC:
            warn.append(f"only {m['engaged_frac']*100:.0f}% of time ENGAGED")

        # tracking error: the command the Orin actually PUBLISHED to the MC (robot.jsonl "cmd") vs the measured joints.
        # (17/11: comparing with the laptop's raw IK target - teleop q_cmd, which jumps at up to 14 rad/s before the
        # speed limiters - reported 0.4-0.8 s "lag" / BAD on episodes where the robot followed its command within
        # 0-60 ms and 0.1 rad.) Falls back to the IK target only for old episodes without "cmd".
        err = []
        for r in robot:
            qc, qm = r.get("cmd"), r.get("q")
            if qc and qm and len(qc) == 14 and len(qm) == 14:
                err.append(np.max(np.abs(np.array(qc, dtype=float) - np.array(qm, dtype=float))))
        m["track_ref"] = "orin_cmd" if err else "ik_target"
        if not err:
            for r in tele:
                qc, qm = r.get("q_cmd"), r.get("q_meas")
                if qc and qm and len(qc) == 14 and len(qm) == 14:
                    err.append(np.max(np.abs(np.array(qc, dtype=float) - np.array(qm, dtype=float))))
        if err:
            m["track_err_max_rad"] = float(np.max(err))
            m["track_err_p95_rad"] = float(np.percentile(err, 95))
            if m["track_err_p95_rad"] > TRACK_ERR_BAD_RAD:
                bad.append(f"robot far from command (p95 {m['track_err_p95_rad']:.2f} rad): blocked?")
            elif m["track_err_p95_rad"] > TRACK_ERR_WARN_RAD:
                warn.append(f"robot lags command (p95 {m['track_err_p95_rad']:.2f} rad)")

        # hand motion from the measured end-effector position
        speeds, path = [], 0.0
        for side in ("left", "right"):
            pos = [(r.get(side) or {}).get("ee_pos") for r in tele]
            if all(isinstance(p, list) and len(p) == 3 for p in pos):
                p = _smooth(np.array(pos, dtype=float), 9)
                dt = np.maximum(np.diff(tt), 1e-3)
                s = np.linalg.norm(np.diff(p, axis=0), axis=1) / dt
                speeds.append(np.concatenate([[0.0], s]))
                path += float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
        if speeds:
            m["path_len_m"] = path
            if path < MIN_PATH_M:
                bad.append(f"hands barely moved (path {path*100:.1f} cm)")
            still = np.all(np.stack(speeds) < IDLE_SPEED_MPS, axis=0)   # BOTH hands still (was all(speeds) < x: one still hand = idle)
            tt0 = tt - tt[0]
            moving_idx = np.where(~still)[0]
            if len(moving_idx) and path >= MIN_PATH_M:
                lead = tt0[moving_idx[0]]
                tail = tt0[-1] - tt0[moving_idx[-1]]
                m["idle_start_s"], m["idle_end_s"] = float(lead), float(tail)
                if lead > IDLE_EDGE_WARN_S:
                    warn.append(f"still for first {lead:.1f}s (trim start)")
                if tail > IDLE_EDGE_WARN_S:
                    warn.append(f"still for last {tail:.1f}s (trim end)")
                core = still.copy()
                core[:moving_idx[0]] = False
                core[moving_idx[-1] + 1:] = False
                mid, at = _longest_run(core, tt0)
                m["idle_mid_s"] = float(mid)
                if mid > IDLE_MID_WARN_S:
                    warn.append(f"both hands still {mid:.1f}s at t={tt0[at]:.1f}s (hesitation)")

        # safety stack (rows recorded before this field existed simply skip these checks)
        sf = [r.get("safety") for r in tele]
        sf = [x for x in sf if isinstance(x, dict)]
        if sf:
            n_sf = len(sf)
            blocked = np.array([bool(x.get("collision_blocked") or x.get("hand_collision_blocked") or x.get("armor_blocked")) for x in sf])
            m["blocked_frac"] = float(blocked.mean())
            pairs = sorted({x["armor_pair"] for x in sf if x.get("armor_blocked") and x.get("armor_pair")})
            if m["blocked_frac"] > BLOCKED_BAD_FRAC:
                bad.append(f"motion held back by collision filters {m['blocked_frac']*100:.0f}% of the time" + (f" ({', '.join(pairs)})" if pairs else ""))
            elif m["blocked_frac"] > BLOCKED_WARN_FRAC:
                warn.append(f"collision filter held motion back {m['blocked_frac']*100:.1f}% of the time" + (f" ({', '.join(pairs)})" if pairs else ""))
            d = [x[k] for x in sf for k in ("armor_min_dist_mm", "proxy_min_dist_mm") if isinstance(x.get(k), (int, float)) and x[k] < 1000.0]
            if d:
                m["min_clearance_mm"] = float(min(d))
                if m["min_clearance_mm"] < CLOSE_WARN_MM:
                    warn.append(f"arm came within {m['min_clearance_mm']:.0f} mm of another link")
            slow = np.mean([float(x.get("proxy_scale", 1.0)) < SLOWED_SCALE for x in sf])
            m["slowed_frac"] = float(slow)
            if slow > SLOWED_WARN_FRAC:
                warn.append(f"soft collision slowed the arms {slow*100:.0f}% of the time")
            if any(x.get("estop") for x in sf):
                bad.append("E-STOP was active during the episode")
            if any(x.get("fault") for x in sf):
                bad.append("robot safety fault was latched during the episode")
            ik = [max(x["ik_pos_err_mm"]) for x in sf if isinstance(x.get("ik_pos_err_mm"), list) and x["ik_pos_err_mm"]]
            if ik:
                m["ik_err_p95_mm"] = float(np.percentile(ik, 95))
                if m["ik_err_p95_mm"] > IK_ERR_WARN_MM:
                    warn.append(f"IK missed the target by {m['ik_err_p95_mm']:.0f} mm (p95)")
            notes = sorted({x["status"] for x in sf if x.get("status") and x["status"] != "TARGET_FROZEN"})
            if notes:
                m["status_messages"] = notes
                warn.append("status messages: " + " | ".join(n[:60] for n in notes[:3]))

        # near the workspace edge
        gains = [[(r.get(side) or {}).get("reach_gain", 1.0) for r in tele] for side in ("left", "right")]
        low = np.mean([np.mean(np.array(g, dtype=float) < REACH_GAIN_WARN) for g in gains])
        m["reach_limited_frac"] = float(low)
        if low > REACH_GAIN_WARN_FRAC:
            warn.append(f"{low*100:.0f}% of time near workspace edge (slowed reach)")

        # possible drop: grip force collapses while the hand is still commanded closed
        tau = [r.get("hand_tau") for r in robot]
        if len(robot) > 20 and all(isinstance(v, list) and len(v) == 12 for v in tau):
            tau = np.abs(np.array(tau, dtype=float))
            tr = np.array([r["t_laptop"] for r in robot], dtype=float)
            hz = len(tr) / max(tr[-1] - tr[0], 1e-6)
            w = max(int(DROP_WINDOW_S * hz), 3)
            drops = []
            for hand, sl in (("left", slice(0, 6)), ("right", slice(6, 12))):
                force = _smooth(tau[:, sl].sum(axis=1, keepdims=True), 5)[:, 0]
                if force.max() <= 0:
                    continue
                clos = np.interp(tr, tt, [(r.get(hand) or {}).get("hand_closure", 0.0) for r in tele])
                for i in range(w, len(force)):
                    peak = force[i - w:i].max()
                    if peak < DROP_MIN_PEAK_FRAC * force.max():
                        continue
                    closed_now = clos[i] > CLOSURE_ON and clos[i] >= clos[i - w] - 0.1
                    if closed_now and force[i] < DROP_FORCE_FRAC * peak:
                        drops.append((hand, float(tr[i] - tr[0])))
                        break
            m["possible_drops"] = [{"hand": h, "t_s": round(t, 1)} for h, t in drops]
            for h, t in drops:
                warn.append(f"possible DROP: {h} grip force collapsed at t={t:.1f}s while hand still closed")

    # ---- camera ------------------------------------------------------------------------------------------
    cams = meta.get("cameras") or []
    if cams and not imgs:
        warn.append("camera enabled but no images recorded")
    elif imgs:
        for name in sorted({i["cam"] for i in imgs if i.get("cam")}):
            ts = np.array([i["t_laptop"] for i in imgs if i.get("cam") == name and "t_laptop" in i], dtype=float)
            if len(ts) > 2 and np.diff(ts).max() > CAM_GAP_WARN_S:
                warn.append(f"camera '{name}' gap {np.diff(ts).max():.2f}s")

    return {"episode": str(ep), "task": meta.get("task"), "discarded": bool(meta.get("discarded")),
            "discard_reason": meta.get("discard_reason"), "metrics": m, "bad": bad, "warn": warn}


def flag_batch_outliers(results):
    """Jerk compared with the other episodes of the same run: more robust than one absolute number."""
    for key, label in (("jerk_p99", "jerk"), ("jerk_rms", "jerk (rms)")):
        vals = [(r, r["metrics"][key]) for r in results if key in r["metrics"]]
        if len(vals) < 5:
            continue
        x = np.array([v for _, v in vals])
        med = np.median(x)
        mad = np.median(np.abs(x - med)) or 1e-9
        thr = med + MAD_K * 1.4826 * mad
        for r, v in vals:
            if v > thr and v > JERK_MIN_FLAG and key == "jerk_p99":
                r["warn"].append(f"jerky: {label} {v:.0f} vs batch median {med:.0f} (rad/s^3)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="*", help="episode / day / task folders (default: recordings/)")
    ap.add_argument("--task", help="only this task under recordings/")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every metric")
    ap.add_argument("--json", metavar="FILE", help="also write a JSON report")
    ap.add_argument("--mark", action="store_true", help="set discarded=true (+ discard_reason) in meta.json of BAD episodes")
    ap.add_argument("--show-discarded", action="store_true", help="also list episodes already flagged discarded")
    ap.add_argument("--strict", action="store_true", help="use the former, tighter thresholds (before 17/11)")
    a = ap.parse_args()
    if a.strict: set_strict(True)

    roots = a.paths or [str(DATA_ROOT / a.task) if a.task else str(DATA_ROOT)]
    eps = find_episodes(roots)
    if not eps:
        print("no episodes found under:", ", ".join(roots)); return 1

    results, skipped, by = [], 0, {}
    for ep in eps:
        try:
            r = analyse(ep)
        except Exception as e:           # a damaged folder must not stop the scan
            r = {"episode": str(ep), "task": None, "discarded": False, "metrics": {}, "bad": [f"cannot analyse: {type(e).__name__}: {e}"], "warn": []}
        if r["discarded"] and not a.show_discarded:
            skipped += 1
            k = r.get("discard_reason") or "no reason"; by[k] = by.get(k, 0) + 1
            continue
        results.append(r)
    flag_batch_outliers(results)

    counts = {"OK": 0, "CHECK": 0, "BAD": 0}
    for r in results:
        r["verdict"] = "BAD" if r["bad"] else ("CHECK" if r["warn"] else "OK")
        counts[r["verdict"]] += 1
        try:
            name = str(Path(r["episode"]).relative_to(DATA_ROOT))
        except ValueError:
            name = r["episode"]
        mt = r["metrics"]
        head = f"{r['verdict']:<5} {name}" + (f"  [{mt['duration_s']:.0f}s]" if mt.get("duration_s") else "")
        if r["discarded"]:
            head += f"  (already discarded: {r.get('discard_reason') or 'no reason'})"
        print(head)
        for s in r["bad"]:
            print(f"        ! {s}")
        for s in r["warn"]:
            print(f"        - {s}")
        if a.verbose:
            for k, v in mt.items():
                print(f"        . {k} = {v:.3f}" if isinstance(v, float) else f"        . {k} = {v}")
        if a.mark and r["bad"] and not r["discarded"]:
            p = Path(r["episode"]) / "meta.json"
            meta = json.loads(p.read_text())
            meta["discarded"] = True
            meta["discard_reason"] = "auto_check"
            meta["discard_note"] = "; ".join(r["bad"])[:300]
            p.write_text(json.dumps(meta, indent=2))
            print("        -> marked discarded")

    print(f"\n{len(results)} episodes: {counts['OK']} OK, {counts['CHECK']} CHECK, {counts['BAD']} BAD"
          + (f"  ({skipped} already discarded, hidden)" if skipped else ""))
    for r in results:
        if r["discarded"]: k = r.get("discard_reason") or "no reason"; by[k] = by.get(k, 0) + 1
    if by: print("discarded by reason: " + ", ".join(f"{k} {v}" for k, v in sorted(by.items())))
    if a.json:
        Path(a.json).write_text(json.dumps(results, indent=2, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
