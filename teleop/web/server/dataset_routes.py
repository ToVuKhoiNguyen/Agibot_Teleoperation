"""
teleop/web/server/dataset_routes.py
Dataset viewer for exported LeRobot datasets (page /dataset on the teleop panel).

- Lists datasets under <repo>/lerobot_datasets/ (folders with meta/info.json) and their episodes.
- Decoding a LeRobot episode needs the `lerobot` library, which lives in venv_lerobot, not in the teleop venv: the
  server runs tools/dataset_cache.py in a SEPARATE low-priority process (nice 19, 2 threads), so the teleop control
  loop is not disturbed. Results go to <dataset>/.viewer_cache/ (deletable at any time).
- Robot 3D replay: per-frame mesh poses computed here with MuJoCo from observation.state (arms + fingers), same
  geom ids as the panel's 3D view.
- Events: computed from the dataset (tracking error, idle, grasp / release) and, when the source episode is still on
  disk (meta/a2_sources.json, written by tools/export_lerobot.py since 17/11), from the recorded safety log and the
  tools/check_episode.py verdict.
- Edits (discarded, task, instruction, cut) are written to the SOURCE episode's meta.json; "re-export" runs
  tools/export_lerobot.py --from-dataset into a NEW dataset folder. The viewed dataset is never modified.
"""
import asyncio, importlib.util, json, os, re, struct, subprocess, threading, time
from pathlib import Path

import numpy as np
from aiohttp import web

REPO = Path(__file__).resolve().parents[3]
DATASETS = Path(os.environ.get("A2_DATASETS_DIR") or (REPO / "lerobot_datasets"))   # override for tests
PY_LEROBOT = REPO / "venv_lerobot" / "bin" / "python"
NAME_RE = re.compile(r"^[A-Za-z0-9_.\-]+$")
LOW_PRIO_ENV = {"OMP_NUM_THREADS": "2", "MKL_NUM_THREADS": "2", "OPENBLAS_NUM_THREADS": "2", "TORCH_NUM_THREADS": "2"}


def _ds_dir(name: str) -> Path:
    if not name or not NAME_RE.match(name): raise web.HTTPBadRequest(text="bad dataset name")
    d = DATASETS / name
    if not (d / "meta" / "info.json").exists(): raise web.HTTPNotFound(text=f"dataset {name} not found")
    return d


def _ep_cache(d: Path, ep: int) -> Path:
    return d / ".viewer_cache" / f"ep_{int(ep):06d}"


def source_dir(rel: str) -> Path:
    """Source episode folder from a2_sources.json; datasets exported before the rename (data/ -> recordings/) say data/..."""
    p = REPO / rel
    if not p.exists() and rel.startswith("data/") and (REPO / "recordings" / rel[5:]).exists(): p = REPO / "recordings" / rel[5:]
    return p


def _sources(d: Path):
    f = d / "meta" / "a2_sources.json"
    if not f.exists(): return {}
    try: return {int(e["episode_index"]): e for e in json.loads(f.read_text())["episodes"]}
    except Exception: return {}


def _read_jsonl(p: Path):
    out = []
    if p.exists():
        for line in open(p):
            try: out.append(json.loads(line))
            except ValueError: pass
    return out


def _segments(mask, t, min_s=0.2):
    """Boolean mask over frames -> [(t0, t1)] runs longer than min_s."""
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]: j += 1
            if t[j] - t[i] >= min_s: out.append((float(t[i]), float(t[j])))
            i = j + 1
        else: i += 1
    return out


def _idle_cuts(events, t, pad=0.5, min_len=1.0):
    """Proposed removals [[a, b, where]] (episode seconds) from the grey 'idle' events: each idle pause is shortened by
    `pad` s on the sides that touch moving parts, and removed completely up to the episode start / end. Only a proposal:
    the operator reviews them one by one in the viewer."""
    if len(t) < 2: return []
    t0, t1 = float(t[0]), float(t[-1]); out = []
    for e in events:
        if e["kind"] != "idle": continue
        head, tail = e["t0"] <= t0 + 0.3, e["t1"] >= t1 - 0.3
        a = t0 if head else e["t0"] + pad
        b = t1 if tail else e["t1"] - pad
        if b - a >= min_len: out.append([round(a, 2), round(b, 2), "đầu" if head else ("cuối" if tail else "giữa")])
    return out


def _merge_ranges(rs):
    out = []
    for a, b in sorted(rs):
        if out and a <= out[-1][1]: out[-1][1] = max(out[-1][1], b)
        else: out.append([a, b])
    return out


def _view_to_laptop(s, v):
    """Seconds of the exported episode -> laptop clock (uses the per-frame times the exporter recorded, so earlier cuts are honoured)."""
    tl = s.get("t_laptop")
    if tl and len(tl) > 1:
        fps = float(s.get("fps") or 30)
        return float(np.interp(v, np.arange(len(tl)) / fps, tl))
    return float(s["t0_laptop"]) + float(v)


class DatasetService:
    def __init__(self, server):
        self.server = server
        self.jobs = {}                 # key -> {"proc", "log": [..], "state", "kind", "started"}
        self.lock = threading.Lock()
        self._fk = None
        self._fk_lock = threading.Lock()
        self._check_mod = None

    # ---------------------------------------------------------------- routes
    def register(self, app):
        app.router.add_get("/dataset", self.page)
        app.router.add_get("/api/ds/list", self.api_list)
        app.router.add_get("/api/ds/episodes", self.api_episodes)
        app.router.add_post("/api/ds/prepare", self.api_prepare)
        app.router.add_get("/api/ds/job", self.api_job)
        app.router.add_get("/api/ds/episode", self.api_episode)
        app.router.add_get("/api/ds/poses", self.api_poses)
        app.router.add_get("/api/ds/manifest", self.api_manifest)
        app.router.add_get("/api/ds/frame/{ds}/{ep}/{cam}/{file}", self.api_frame)
        app.router.add_post("/api/ds/edit", self.api_edit)
        app.router.add_get("/api/ds/idle_scan", self.api_idle_scan)
        app.router.add_post("/api/ds/reexport", self.api_reexport)

    async def page(self, request):
        html = (REPO / "teleop" / "web" / "static" / "dataset.html").read_text()
        return web.Response(text=html, content_type="text/html", headers={"Cache-Control": "no-cache"})

    async def api_manifest(self, request):
        return web.json_response(self.server.visual_manifest)

    async def api_list(self, request):
        out = []
        if DATASETS.exists():
            for d in sorted(DATASETS.iterdir()):
                f = d / "meta" / "info.json"
                if not f.exists(): continue
                try: info = json.loads(f.read_text())
                except Exception: continue
                done = any((d / "meta" / "episodes").rglob("*.parquet")) if (d / "meta" / "episodes").exists() else False
                out.append({"name": d.name, "complete": done, "episodes": info.get("total_episodes"), "frames": info.get("total_frames"),
                            "fps": info.get("fps"), "has_sources": (d / "meta" / "a2_sources.json").exists(),
                            "cameras": [k.split(".")[-1] for k, v in info.get("features", {}).items() if v.get("dtype") in ("video", "image")],
                            "mtime": f.stat().st_mtime})
        return web.json_response({"datasets": out, "root": str(DATASETS), "lerobot_env": PY_LEROBOT.exists()})

    async def api_episodes(self, request):
        d = _ds_dir(request.query.get("ds", ""))
        if not (d / "meta" / "episodes").exists() or not any((d / "meta" / "episodes").rglob("*.parquet")):
            return web.json_response({"state": "incomplete", "episodes": [], "fps": None,
                                      "msg": "Dataset chưa xuất xong (đang xuất, hoặc lần xuất bị dừng giữa chừng)."})
        lst = d / ".viewer_cache" / "episodes.json"
        if not lst.exists():
            key = f"list:{d.name}"
            self._start_job(key, "list", [str(PY_LEROBOT), str(REPO / "tools" / "dataset_cache.py"), "--root", str(d), "--list", "--out", str(lst)])
            return web.json_response({"state": "building", "job": key})
        data = json.loads(lst.read_text())
        src = _sources(d)
        for e in data["episodes"]:
            s = src.get(e["index"])
            e["cached"] = (_ep_cache(d, e["index"]) / "done").exists()
            e["source"] = s["source"] if s else None
            if s:
                m = self._source_meta(s)
                if m is not None:
                    e["discarded"], e["instruction"], e["task_label"] = bool(m.get("discarded")), m.get("instruction", ""), m.get("task", "")
                    e["edited_trim"] = m.get("trim") is not None
                else: e["source_missing"] = True
        data["state"] = "ready"
        return web.json_response(data)

    async def api_prepare(self, request):
        body = await request.json()
        d = _ds_dir(body.get("ds", "")); ep = int(body.get("ep", 0))
        c = _ep_cache(d, ep)
        if (c / "done").exists(): return web.json_response({"state": "ready"})
        key = f"ep:{d.name}:{ep}"
        self._start_job(key, "cache", [str(PY_LEROBOT), str(REPO / "tools" / "dataset_cache.py"), "--root", str(d),
                                       "--episode", str(ep), "--out", str(c)])
        return web.json_response({"state": "building", "job": key})

    async def api_job(self, request):
        key = request.query.get("id", "")
        with self.lock: j = self.jobs.get(key)
        if j is None: return web.json_response({"state": "unknown"})
        return web.json_response({"state": j["state"], "log": j["log"][-40:], "kind": j["kind"], "out": j.get("out")})

    async def api_frame(self, request):
        d = _ds_dir(request.match_info["ds"]); ep = int(request.match_info["ep"])
        cam, fn = request.match_info["cam"], request.match_info["file"]
        if not NAME_RE.match(cam) or not re.match(r"^\d{6}\.jpg$", fn): raise web.HTTPBadRequest()
        p = _ep_cache(d, ep) / cam / fn
        if not p.exists(): raise web.HTTPNotFound()
        return web.FileResponse(p, headers={"Cache-Control": "public, max-age=3600"})

    async def api_episode(self, request):
        d = _ds_dir(request.query.get("ds", "")); ep = int(request.query.get("ep", 0))
        c = _ep_cache(d, ep)
        if not (c / "done").exists(): raise web.HTTPConflict(text="episode not prepared")
        data = await asyncio.get_event_loop().run_in_executor(None, self._episode_payload, d, ep)
        return web.json_response(data)

    async def api_poses(self, request):
        d = _ds_dir(request.query.get("ds", "")); ep = int(request.query.get("ep", 0))
        c = _ep_cache(d, ep)
        if not (c / "done").exists(): raise web.HTTPConflict(text="episode not prepared")
        f = c / "poses.bin"
        if not f.exists():
            await asyncio.get_event_loop().run_in_executor(None, self._compute_poses, c, f)
        return web.FileResponse(f, headers={"Cache-Control": "no-cache"})

    async def api_edit(self, request):
        b = await request.json()
        d = _ds_dir(b.get("ds", "")); ep = int(b.get("ep", 0))
        s = _sources(d).get(ep)
        if s is None: return web.json_response({"ok": False, "error": "this dataset has no source record (exported before 17/11): re-export it from data/ to edit"}, status=400)
        mp = source_dir(s["source"]) / "meta.json"
        if not mp.exists(): return web.json_response({"ok": False, "error": f"source episode not on disk: {s['source']}"}, status=400)
        meta = json.loads(mp.read_text())
        bak = mp.with_name("meta.json.viewer_bak")
        if not bak.exists(): bak.write_text(json.dumps(meta, indent=2))
        if "discarded" in b:
            meta["discarded"] = bool(b["discarded"])
            if meta["discarded"]: meta["discard_reason"] = b.get("discard_reason") or "marked in the dataset viewer"
            else: meta.pop("discard_reason", None)
        for k in ("task", "instruction"):
            if k in b and isinstance(b[k], str): meta[k] = b[k].strip()
        if "trim" in b:
            tr = b["trim"]
            if tr is None: meta.pop("trim", None)
            else:
                a0, a1 = float(tr[0]), float(tr[1])
                if not (0 <= a0 < a1): return web.json_response({"ok": False, "error": "cut start must be before cut end"}, status=400)
                meta["trim"] = [_view_to_laptop(s, a0), _view_to_laptop(s, a1)]
                meta["trim_note"] = "laptop clock; set in the dataset viewer (seconds of the exported episode + t0_laptop)"
        if b.get("cuts_clear"): meta.pop("cuts", None)
        if b.get("cuts"):
            new = []
            for c in b["cuts"]:
                a0, a1 = float(c[0]), float(c[1])
                if not (0 <= a0 < a1): return web.json_response({"ok": False, "error": "cut start must be before cut end"}, status=400)
                new.append([_view_to_laptop(s, a0), _view_to_laptop(s, a1)])
            meta["cuts"] = _merge_ranges(list(meta.get("cuts") or []) + new)
            meta["cuts_note"] = "laptop clock; ranges removed at export (dataset viewer, idle pauses)"
        tmp = mp.with_suffix(".json.tmp"); tmp.write_text(json.dumps(meta, indent=2)); os.replace(tmp, mp)
        return web.json_response({"ok": True, "source": s["source"], "n_cuts": len(meta.get("cuts") or []),
                                  "meta": {k: meta.get(k) for k in ("discarded", "task", "instruction", "trim", "cuts")}})

    async def api_idle_scan(self, request):
        """Whole dataset: proposed idle lead-in / tail cut per episode (nothing is written; the viewer applies the ones the
        operator ticks through /api/ds/edit)."""
        d = _ds_dir(request.query.get("ds", ""))
        src = _sources(d)
        def scan():
            out = []
            for ep, s in sorted(src.items()):
                row = {"ep": ep, "cached": (_ep_cache(d, ep) / "data.json").exists()}
                if row["cached"]:
                    try:
                        p = self._episode_payload(d, ep)
                        data = json.loads((_ep_cache(d, ep) / "data.json").read_text()); tt = data["t"]
                        row.update(T=round(float(tt[-1]), 1), cuts=_idle_cuts(p["events"], tt), n_cuts=p["source"].get("n_cuts", 0),
                                   discarded=bool(p["source"].get("discarded")), editable=bool(p["source"].get("available")))
                    except Exception as e: row["error"] = str(e)[:120]
                out.append(row)
            return out
        return web.json_response({"ok": True, "episodes": await asyncio.get_event_loop().run_in_executor(None, scan)})

    async def api_reexport(self, request):
        b = await request.json()
        d = _ds_dir(b.get("ds", ""))
        new = (b.get("new_name") or "").strip()
        if not new or not NAME_RE.match(new): return web.json_response({"ok": False, "error": "bad new dataset name"}, status=400)
        out = DATASETS / new
        if out.exists(): return web.json_response({"ok": False, "error": f"lerobot_datasets/{new} already exists"}, status=400)
        if not (d / "meta" / "a2_sources.json").exists():
            return web.json_response({"ok": False, "error": "no source record in this dataset (exported before 17/11)"}, status=400)
        key = f"export:{new}"
        cmd = [str(PY_LEROBOT), str(REPO / "tools" / "export_lerobot.py"), "--from-dataset", str(d), "--out", str(out), "--repo-id", f"local/{new}"]
        info = json.loads((d / "meta" / "info.json").read_text())
        if info.get("fps"): cmd += ["--fps", str(info["fps"])]
        self._start_job(key, "export", cmd, out=new)
        return web.json_response({"ok": True, "job": key})

    # ---------------------------------------------------------------- jobs
    def _start_job(self, key, kind, cmd, out=None):
        with self.lock:
            j = self.jobs.get(key)
            if j is not None and j["state"] == "running": return
            j = {"state": "running", "log": [], "kind": kind, "started": time.time(), "out": out}
            self.jobs[key] = j
        if not PY_LEROBOT.exists():
            j["state"] = "error"; j["log"].append(f"{PY_LEROBOT} missing: create venv_lerobot (README 3.4)"); return

        def run():
            env = dict(os.environ, **LOW_PRIO_ENV)
            try:
                p = subprocess.Popen(["nice", "-n", "19"] + cmd, cwd=str(REPO), env=env, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1)
                j["proc"] = p
                for line in p.stdout:
                    line = line.rstrip()
                    if line and "Warn" not in line and "torchcodec" not in line.lower() and "x265 [info]" not in line:
                        j["log"].append(line[-300:])
                j["state"] = "done" if p.wait() == 0 else "error"
            except Exception as e:
                j["log"].append(f"failed: {e}"); j["state"] = "error"
        threading.Thread(target=run, daemon=True).start()

    # ---------------------------------------------------------------- data
    def _source_meta(self, s):
        p = source_dir(s["source"]) / "meta.json"
        try: return json.loads(p.read_text()) if p.exists() else None
        except Exception: return None

    def _episode_payload(self, d: Path, ep: int):
        c = _ep_cache(d, ep)
        data = json.loads((c / "data.json").read_text())
        t = np.array(data["t"], dtype=float)
        F = {k: np.array(v, dtype=float) for k, v in data["features"].items()}
        events, quality = [], None
        st, ac = F.get("observation.state"), F.get("action")
        names = data.get("names", {})
        s = _sources(d).get(ep)
        # The command the Orin actually published to the MC, from the source episode (robot.jsonl "cmd"), on the dataset
        # time axis. The dataset "action" is the laptop's raw IK target, which runs ahead of the speed limiters (up to
        # ~14 rad/s): comparing the robot with it shows a large fake "lag" (17/11). Tracking events use the Orin command.
        rcmd = self._robot_cmd(s, np.array(data["t"], dtype=float)) if s is not None else None
        if rcmd is not None: data["features"]["robot.cmd"] = np.round(rcmd, 5).tolist()
        ref = rcmd if rcmd is not None else (ac[:, :14] if ac is not None else None)
        if st is not None and ref is not None and st.shape[1] >= 14:
            err = np.abs(ref[:, :14] - st[:, :14])
            jn = [f"{'L' if j < 7 else 'R'} J{j % 7 + 1}" for j in range(14)]
            for j in range(14):
                for a0, a1 in _segments(err[:, j] > 0.5, t, 0.3):
                    m = (t >= a0) & (t <= a1)
                    events.append({"t0": a0, "t1": a1, "kind": "track", "label": f"lệch lệnh {jn[j]} (tới {err[m, j].max():.2f} rad)",
                                   "why": "Khớp thực đo lệch xa lệnh Orin đã gửi: robot đi theo chậm hoặc bị giới hạn tốc độ / gia tốc."})
        ee = F.get("observation.ee_pose")
        if ee is not None and len(t) > 3:
            dt = np.maximum(np.diff(t), 1e-3)
            sp = np.maximum(np.linalg.norm(np.diff(ee[:, 0:3], axis=0), axis=1), np.linalg.norm(np.diff(ee[:, 7:10], axis=0), axis=1)) / dt
            sp = np.convolve(np.r_[sp[0], sp], np.ones(9) / 9, "same")
            for a0, a1 in _segments(sp < 0.005, t, 2.0):
                events.append({"t0": a0, "t1": a1, "kind": "idle", "label": f"đứng yên {a1 - a0:.1f} s",
                                   "why": "Cả hai tay gần như không chuyển động trong đoạn này (không có thao tác). Có thể cắt bỏ đoạn đầu / cuối."})
        ae = F.get("action.ee_pose")
        if ae is not None and ae.shape[1] >= 16:
            for col, side in ((14, "trái"), (15, "phải")):
                cl = ae[:, col]; closed = False
                for k in range(len(cl)):
                    if not closed and cl[k] >= 0.45: closed = True; events.append({"t0": float(t[k]), "t1": float(t[k]), "kind": "grasp", "label": f"nắm tay {side}", "why": "Mức nắm lệnh của tay vượt 0,45: bắt đầu nắm vật."})
                    elif closed and cl[k] <= 0.30: closed = False; events.append({"t0": float(t[k]), "t1": float(t[k]), "kind": "release", "label": f"mở tay {side}", "why": "Mức nắm lệnh của tay xuống dưới 0,30: thả vật."})
        src = {"available": False}
        if s is not None:
            sd = source_dir(s["source"])
            meta = self._source_meta(s)
            if meta is not None:
                t0 = float(s["t0_laptop"])
                src = {"available": True, "path": s["source"], "discarded": bool(meta.get("discarded")),
                       "discard_reason": meta.get("discard_reason"), "task": meta.get("task", ""), "instruction": meta.get("instruction", ""),
                       "trim": [meta["trim"][0] - t0, meta["trim"][1] - t0] if meta.get("trim") else None,
                       "duration_s": meta.get("duration_s"), "n_cuts": len(meta.get("cuts") or [])}
                tele = _read_jsonl(sd / "teleop.jsonl")
                if tele:
                    tt = np.array([r.get("t", 0.0) for r in tele]) - t0
                    sf = [r.get("safety") or {} for r in tele]
                    for key, lab in (("armor_blocked", "armor chặn"), ("collision_blocked", "va chạm chặn"), ("hand_collision_blocked", "ngón tay bị chặn")):
                        for a0, a1 in _segments(np.array([bool(x.get(key)) for x in sf]), tt, 0.05):
                            events.append({"t0": a0, "t1": a1, "kind": "safety", "label": lab, "why": "Một bộ lọc an toàn đã giữ chuyển động lại trong đoạn này (lệnh bị chặn hoặc làm chậm)."})
                    def _rw_list(x):
                        w = x.get("reach_warning")
                        return [str(v) for v in w if v] if isinstance(w, list) else ([str(w)] if w else [])
                    rw = np.array([bool(_rw_list(x)) for x in sf])
                    for a0, a1 in _segments(rw, tt, 0.2):
                        m = (tt >= a0) & (tt <= a1)
                        txt = {w for k in np.nonzero(m)[0] for w in _rw_list(sf[k])}
                        events.append({"t0": a0, "t1": a1, "kind": "reach",
                                       "label": "gần / ngoài tầm với" + (": " + "; ".join(sorted(txt))[:160] if txt else ""),
                                       "why": "Mục tiêu tay nằm sát hoặc ngoài vùng với tới của cánh tay; đích bị chiếu vào vùng với tới nên tay không tới đúng chỗ operator chỉ."})
                    ik2 = np.array([[(x.get("ik_pos_err_mm") or [0.0, 0.0])[i] if len(x.get("ik_pos_err_mm") or []) > i else 0.0 for i in (0, 1)] for x in sf], dtype=float)
                    for a0, a1 in _segments(ik2.max(axis=1) > 30.0, tt, 0.2):
                        m = (tt >= a0) & (tt <= a1)
                        pk = ik2[m].max(axis=0)
                        events.append({"t0": a0, "t1": a1, "kind": "ik",
                                       "label": f"IK không tới đích (> 30 mm): trái tới {pk[0]:.0f} mm, phải tới {pk[1]:.0f} mm",
                                       "why": "Bộ giải IK không đưa được cổ tay tới đúng vị trí mục tiêu (sai số > 30 mm): đích ngoài tầm với, gần giới hạn khớp, hoặc bị chặn va chạm. Tay robot lệch so với ý định của operator trong đoạn này."})
                quality = self._quality(sd)
            else:
                src = {"available": False, "missing": s["source"]}
        events = [e for e in events if e["t1"] >= t[0] - 0.5 and e["t0"] <= t[-1] + 0.5]
        events.sort(key=lambda e: e["t0"])
        return {"ds": d.name, "episode": ep, "fps": data["fps"], "frames": data["frames"], "t": data["t"], "task": data["task"],
                "features": data["features"], "names": names, "cams": data["cams"], "events": events,
                "idle_cuts": _idle_cuts(events, t), "source": src, "quality": quality}

    def _robot_cmd(self, s, t_ds):
        """Orin-published command of the source episode resampled onto the dataset frames (None if unavailable)."""
        meta = self._source_meta(s)
        if meta is None: return None
        off = meta.get("robot_clock_minus_laptop_s")
        rows = [r for r in _read_jsonl(source_dir(s["source"]) / "robot.jsonl") if r.get("cmd") and r.get("stamp_mc") and len(r["cmd"]) == 14]
        if off is None or len(rows) < 10: return None
        tr = np.array([r["stamp_mc"] for r in rows]) - off - float(s["t0_laptop"])
        o = np.argsort(tr); tr = tr[o]; C = np.array([rows[i]["cmd"] for i in o], dtype=float)
        k = np.clip(np.searchsorted(tr, t_ds - t_ds[0], side="right") - 1, 0, len(tr) - 1)   # frame 0 = t0_laptop
        return C[k]

    def _quality(self, ep_dir: Path):
        try:
            if self._check_mod is None:
                spec = importlib.util.spec_from_file_location("check_episode", REPO / "tools" / "check_episode.py")
                mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); self._check_mod = mod
            r = self._check_mod.analyse(ep_dir)
            verdict = "BAD" if r["bad"] else ("CHECK" if r["warn"] else "OK")
            return {"verdict": verdict, "bad": r["bad"], "warn": r["warn"]}
        except Exception as e:
            return {"verdict": "?", "bad": [], "warn": [f"check_episode failed: {e}"]}

    def fk_poses(self, rows):
        """Live view: [(q14 rad, hand12 raw), ...] -> (mesh geom ids, float32 array [n, G, 7]). Same FK as _compute_poses."""
        import mujoco
        from teleop.robot.aimdk_map import robot_raw_to_sim_hand
        with self._fk_lock:
            if self._fk is None:
                from teleop.simulation.mujoco_env import AgibotSimEnv
                self._fk = AgibotSimEnv(self.server.scene_xml or str(REPO / "assets" / "agibot_a2_ultra" / "mjcf" / "scene.xml"))
            env, ids, buf = self._fk, None, []
            for q, hand in rows:
                q = np.asarray(q, dtype=float); env.set_arm_commands(q[:7], q[7:14])
                h = robot_raw_to_sim_hand(np.asarray(hand, dtype=float)) if hand is not None and len(hand) >= 12 else np.zeros(12)
                env.set_hand_commands(h[:6], h[6:])
                mujoco.mj_kinematics(env.model, env.data)
                poses = env.get_visual_body_poses()
                if ids is None: ids = sorted(int(k) for k in poses if str(k).isdigit())
                buf.append(np.array([poses[str(g)] for g in ids], dtype=np.float32))
            return ids or [], (np.stack(buf) if buf else np.zeros((0, 0, 7), np.float32))

    def _compute_poses(self, c: Path, out: Path):
        """observation.state (arm 14 rad + fingers 12 raw) -> per-frame mesh poses of the panel's 3D robot."""
        with self._fk_lock:
            if self._fk is None:
                from teleop.simulation.mujoco_env import AgibotSimEnv
                self._fk = AgibotSimEnv(self.server.scene_xml or str(REPO / "assets" / "agibot_a2_ultra" / "mjcf" / "scene.xml"))
            env = self._fk
            import mujoco
            from teleop.robot.aimdk_map import robot_raw_to_sim_hand
            data = json.loads((c / "data.json").read_text())
            st = np.array(data["features"]["observation.state"], dtype=float)
            ids = None; buf = []
            for row in st:
                env.set_arm_commands(row[:7], row[7:14])
                h = robot_raw_to_sim_hand(row[14:26]) if st.shape[1] >= 26 else np.zeros(12)
                env.set_hand_commands(h[:6], h[6:])
                mujoco.mj_kinematics(env.model, env.data)
                poses = env.get_visual_body_poses()
                if ids is None: ids = sorted(int(k) for k in poses if str(k).isdigit())
                buf.append(np.array([poses[str(g)] for g in ids], dtype=np.float32))
            arr = np.stack(buf) if buf else np.zeros((0, 0, 7), np.float32)
            with open(out, "wb") as f:
                f.write(struct.pack("<II", len(ids or []), arr.shape[0]))
                f.write(np.array(ids or [], dtype=np.int32).tobytes())
                f.write(arr.astype(np.float32).tobytes())
