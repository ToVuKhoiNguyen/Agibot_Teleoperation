"""
teleop/app/episode_recorder.py
Data-collection episodes. Everything is written on THIS machine, in one folder per episode:

    recordings/<task>/<YYYY-MM-DD>/ep_<n>_<HHMMSS>/
        meta.json      what / when / settings / clock offset / discard flag / row counts
        robot.jsonl    one row per robot feedback packet (100 Hz): joint position, velocity, torque, finger position
                       and effort, the command the robot published, with three timestamps (MC stamp, robot clock,
                       laptop arrival)
        teleop.jsonl   100 Hz: operator controllers (pose, trigger, grip), Cartesian targets, measured hand pose
                       (position + orientation), joint command, hand command, teleop context

While an episode is active every command packet carries a "rec" field; the robot then adds velocity / torque /
effort / its own timestamp to the feedback packets (see scripts/arm_teleop_rx.py). No video is recorded.
"""
import collections, json, os, socket, struct, subprocess, threading, time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from teleop.utils.async_writer import AsyncJsonlWriter

DATA_ROOT = Path(__file__).resolve().parents[2] / "recordings"   # raw episodes (was data/ until 17/11)
LOG_HZ = 100.0

# Camera frames: while an episode records, the robot compresses the listed streams and sends them here over TCP.
# Colour -> JPEG, depth -> lossless 16-bit PNG; shrunk on the robot to the given width (0 = keep the native size).
# 17/11 (operator): NATIVE size by default (head D415 = 1280x720, as in the vendor topic list) instead of 640x360.
# Cost ~4x: ~110 KB colour + ~380 KB depth per frame, ~6-7 MB/s Orin -> laptop, ~180 MB per 25 s episode.
# If the robot drops frames (camera fps well below 15 in images.jsonl / /dataset) shrink the depth only:
# REC_DEPTH_WIDTH=640 (or everything: REC_CAM_WIDTH=640).
CAM_PORT = int(os.environ.get("REC_CAM_PORT", "5007"))
CAM_WIDTH = int(os.environ.get("REC_CAM_WIDTH", "1280"))   # 1280 = the head camera native width (never upscaled on the Orin)
DEPTH_WIDTH = int(os.environ.get("REC_DEPTH_WIDTH", str(CAM_WIDTH)))
CAM_QUALITY = int(os.environ.get("REC_CAM_QUALITY", "85"))
CAM_FPS = float(os.environ.get("REC_CAM_FPS", "15"))
CAM_ENABLED = os.environ.get("REC_CAM", "1") != "0"
CAM_STREAMS = [
    {"name": "head", "kind": "color", "topic": os.environ.get("REC_CAM_TOPIC", "/aima/hal/rgbd_camera/head_front/color")},
    {"name": "head_depth", "kind": "depth", "topic": os.environ.get("REC_DEPTH_TOPIC", "/aima/hal/rgbd_camera/head_front/depth")},
]
if os.environ.get("REC_DEPTH", "1") == "0": CAM_STREAMS = CAM_STREAMS[:1]
for _st in CAM_STREAMS: _st.update(width=DEPTH_WIDTH if _st["kind"] == "depth" else CAM_WIDTH, q=CAM_QUALITY, fps=CAM_FPS)
CAM_MAGIC, CAM_STATUS = b"A2I2", b"A2ST"


def safe_name(text, default="default", max_len=48):
    out = "".join(ch if (ch.isalnum() and ch.isascii()) or ch in "_-" else "_" for ch in str(text or ""))[:max_len].strip("_")
    return out or default


def _quat_xyzw(rot):
    try: return [round(float(x), 5) for x in R.from_matrix(np.asarray(rot, dtype=np.float64)).as_quat()]
    except Exception: return None


def _vec(v, nd=5):
    return None if v is None else [round(float(x), nd) for x in np.asarray(v, dtype=np.float64).ravel()]


# Why an episode was thrown away. The operator picks one in the web panel before pressing X / Discard last
# (X on the controller uses the one currently selected). "auto_check" is written by tools/check_episode.py --mark.
DISCARD_REASONS = ("dropped_object", "collision", "bad_trajectory", "task_failed", "robot_issue", "other")


class LiveBus:
    """In-memory copy of the running episode for the live view (/dataset, "Trực tiếp").

    Every robot row, teleop row and image index row gets a per-stream sequence number and is appended to a bounded ring.
    Readers (web sockets) keep their own cursor and read by sequence number, so a slow reader never makes the recorder
    wait and never silently loses rows: if it falls behind by more than RING rows it is told how many it missed.
    Nothing here touches the disk files; it is a second, read-only tap on what is already being written.
    """
    RING = 30000                                   # 300 s of a 100 Hz stream
    IMG_RING = 120                                 # ~4 s of camera rows (preview rows carry the JPEG bytes in memory)
    PREVIEW_AFTER_END_S = 1.0                      # after REC stops, the stopped episode stays on screen this long

    # Preview (no episode recording): while at least one live page is open, robot rows, teleop rows and camera frames
    # are pushed too, in a "preview" session (info["preview"] = True). Nothing is written to disk for the preview.

    def __init__(self):
        self.lock = threading.Lock()
        self.session, self.info, self.active = 0, None, False
        self.last_info, self.viewers, self.end_t = None, 0, 0.0
        self.ring = {k: collections.deque(maxlen=self.IMG_RING if k == "img" else self.RING) for k in ("robot", "tel", "img")}
        self.seq = {k: 0 for k in self.ring}

    def begin(self, info):
        with self.lock:
            self.session += 1; self.info, self.active = dict(info), True
            for k in self.ring: self.ring[k].clear(); self.seq[k] = 0

    def end(self, summary=None):
        with self.lock:
            self.active, self.end_t = False, time.time()
            if self.info is not None:
                self.last_info = dict(self.info, **(summary or {}))
                self.info = dict(self.info, **(summary or {}))

    def push(self, stream, row):
        with self.lock:
            if not self.active: return
            n = self.seq[stream] + 1; self.seq[stream] = n
            row["s"] = n; self.ring[stream].append(row)

    def preview_wanted(self):
        return self.viewers > 0 and not self.active

    def push_preview(self, stream, row, t_abs):
        """Preview row (no episode recording, a live page is open). row["t"] is set from t_abs (laptop clock)."""
        if self.viewers <= 0 or self.active: return
        with self.lock:
            if self.active or time.time() - self.end_t < self.PREVIEW_AFTER_END_S: return
            if not (self.info and self.info.get("preview")):          # first preview row: new session
                self.session += 1; self.info = {"preview": True, "t0": t_abs}
                for k in self.ring: self.ring[k].clear(); self.seq[k] = 0
            n = self.seq[stream] + 1; self.seq[stream] = n
            row["s"], row["t"] = n, round(t_abs - self.info["t0"], 4); self.ring[stream].append(row)

    def read(self, stream, after, limit=2000):
        """-> (rows with seq > after, missed, newest seq). missed > 0 means the ring overflowed before the reader came."""
        with self.lock:
            d, newest = self.ring[stream], self.seq[stream]
            if newest <= after or not d: return [], 0, newest
            first = d[0]["s"]
            missed = max(0, first - after - 1)
            start = max(after + 1, first)
            i0 = start - first
            out = [d[i] for i in range(i0, min(len(d), i0 + limit))]
            return out, missed, newest

    def state(self):
        with self.lock:
            return self.session, self.active, (dict(self.info) if self.info else None), dict(self.seq)


LIVE = LiveBus()


def init_recording_state(app):
    app.rec_discard_reason = "other"
    app.rec_active, app.rec_task, app.rec, app.rec_instruction = False, "default", None, ""
    app.rec_last, app.rec_count, app.rec_msg = None, 0, ""
    app._rec_writer, app._rec_robot_writer, app._rec_last_log_t = None, None, 0.0
    app._rec_rows = {"teleop": 0, "robot": 0, "images": 0}
    app._rec_img_index, app._rec_cam_server, app._rec_img_counts = None, None, {}


def rec_packet_field(app):
    """Field for the UDP command packet: which episode the robot must be recording (None = not recording)."""
    r = getattr(app, "rec", None)
    if not (getattr(app, "rec_active", False) and r):
        # Live page open, no episode: ask the robot for camera frames only. Without "ep" the robot records nothing
        # (orin_teleop_core.EpisodeRecorder.request needs "ep"); its CameraStreamer only needs "cam".
        if CAM_ENABLED and LIVE.preview_wanted():
            _ensure_cam_server(app)
            return {"cam": {"port": CAM_PORT, "streams": CAM_STREAMS}}
        return None
    out = {"task": r["task"], "date": r["date"], "ep": r["ep"]}
    if CAM_ENABLED: out["cam"] = {"port": CAM_PORT, "streams": CAM_STREAMS}
    return out


def _git_sha():
    try: return subprocess.check_output(["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL, cwd=str(DATA_ROOT.parent)).decode().strip()
    except Exception: return "unknown"


def _next_index(day_dir: Path) -> int:
    nums = []
    if day_dir.is_dir():
        for p in day_dir.iterdir():
            parts = p.name.split("_")
            if len(parts) >= 2 and parts[0] == "ep" and parts[1].isdigit(): nums.append(int(parts[1]))
    return (max(nums) + 1) if nums else 1


def start_episode(app, task=None, instruction=None) -> bool:
    if getattr(app, "rec_active", False): return False
    if task: app.rec_task = safe_name(task)
    if instruction is not None: app.rec_instruction = str(instruction).strip()[:300]
    date = time.strftime("%Y-%m-%d")
    day_dir = DATA_ROOT / app.rec_task / date
    ep = f"ep_{_next_index(day_dir):04d}_{time.strftime('%H%M%S')}"
    ep_dir = day_dir / ep
    try:
        ep_dir.mkdir(parents=True, exist_ok=False)
        meta = {
            "task": app.rec_task, "date": date, "episode": ep, "start_unix": time.time(),
            "start_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "git_sha": _git_sha(),
            "backend": getattr(app, "backend", ""), "speed_level": getattr(app, "speed_level", None),
            "xr_scale": float(getattr(app, "xr_scale", 1.0)), "axis_lock": bool(getattr(app, "xr_axis_lock", False)),
            "laptop_log_hz": LOG_HZ, "discarded": False,
            "time": {"t": "laptop wall clock, unix seconds", "t_mono": "laptop monotonic clock",
                     "t_robot": "robot (Orin) wall clock when it sent the packet", "stamp_mc": "motion controller's stamp of the joint state",
                     "t_laptop": "laptop wall clock when the packet arrived",
                     "robot_clock_minus_laptop_s": "smallest (t_robot - t_laptop) seen in the episode; add it to laptop times to get robot time"},
            "units": {"q/q_cmd/q_meas/cmd": "rad, 14 arm joints: left 1-7 then right 1-7", "dq": "rad/s", "tau": "joint effort as reported by the MC",
                      "hand/hand_tau": "raw finger position / effort, 12 values: left then right", "pos": "m, robot base frame", "quat": "see field name (xyzw or wxyz)"},
            "safety": "teleop.jsonl rows carry a 'safety' dict: *_blocked = a safety filter held the motion back this tick, *_dist_mm = closest distance, proxy_scale < 1 = soft collision slowed the arms, ik_pos_err_mm = [left, right] IK miss",
                        "streams": {"robot.jsonl": "one row per robot feedback packet (~100 Hz)", "teleop.jsonl": f"{LOG_HZ:.0f} Hz teleop + operator stream",
                        "images.jsonl": "one row per image file: every field of the original sensor_msgs/Image (stamp, frame_id, height, width, "
                                        "encoding, is_bigendian, step), the stored size/format, robot send time and laptop arrival time"},
            "cameras": [dict(st, stored="jpeg" if st["kind"] == "color" else "png 16-bit (lossless, nearest-neighbour resize)") for st in CAM_STREAMS] if CAM_ENABLED else [],
            "instruction": getattr(app, "rec_instruction", "") or app.rec_task.replace("_", " "),
        }
        (ep_dir / "meta.json").write_text(json.dumps(meta, indent=2))
        app._rec_writer = AsyncJsonlWriter(ep_dir / "teleop.jsonl")
        app._rec_robot_writer = AsyncJsonlWriter(ep_dir / "robot.jsonl")
        app._rec_rows, app._rec_clock_off = {"teleop": 0, "robot": 0, "images": 0}, None
        if CAM_ENABLED:
            for st in CAM_STREAMS: (ep_dir / "images" / st["name"]).mkdir(parents=True, exist_ok=True)
            app._rec_img_counts = {st["name"]: 0 for st in CAM_STREAMS}
            app._rec_img_index = open(ep_dir / "images.jsonl", "a", buffering=1)
            _ensure_cam_server(app)
    except Exception as e:
        app.rec_msg = f"cannot create episode folder: {e}"
        return False
    app.rec = {"task": app.rec_task, "date": date, "ep": ep, "dir": str(ep_dir), "t0": time.time(), "meta": meta}
    app.rec_active, app.rec_msg, app._rec_last_log_t = True, "", 0.0
    LIVE.begin({"task": app.rec_task, "date": date, "ep": ep, "dir": str(ep_dir), "t0": app.rec["t0"],
                "instruction": meta["instruction"], "cams": [st["name"] for st in CAM_STREAMS] if CAM_ENABLED else []})
    if hasattr(app, "_log_xr_event"): app._log_xr_event(f"REC start {app.rec_task}/{date}/{ep}")
    print(f"[REC] START {ep_dir}")
    return True


def stop_episode(app) -> bool:
    if not getattr(app, "rec_active", False) or not app.rec: return False
    r = app.rec
    app.rec_active = False
    for attr in ("_rec_writer", "_rec_robot_writer", "_rec_img_index"):
        w = getattr(app, attr, None)
        setattr(app, attr, None)
        if w is not None:
            try: w.close()
            except Exception: pass
    meta = r["meta"]
    meta.update(stop_unix=time.time(), duration_s=round(time.time() - r["t0"], 2), rows=dict(app._rec_rows),
                robot_clock_minus_laptop_s=getattr(app, "_rec_clock_off", None))
    try: (Path(r["dir"]) / "meta.json").write_text(json.dumps(meta, indent=2))
    except Exception as e: app.rec_msg = f"meta.json: {e}"
    app.rec_last, app.rec_count = r, app.rec_count + 1
    LIVE.end({"duration_s": meta["duration_s"], "rows": dict(app._rec_rows)})
    if hasattr(app, "_log_xr_event"): app._log_xr_event(f"REC stop {r['ep']} ({meta['duration_s']} s)")
    print(f"[REC] STOP  {r['dir']}  ({meta['duration_s']} s)")
    _exporter(app).schedule(r["task"])
    return True


# ---------------------------------------------------------------- automatic LeRobot export (17/11)
# After an episode stops, the laptop appends it to lerobot_datasets/<task> (LeRobot v3.0) by itself: no script to run.
# Waits AUTO_EXPORT_DELAY_S after the last stop (time to tap X / Discard last) and until no episode is being
# recorded, then runs tools/export_lerobot.py --append in venv_lerobot as a separate nice-19 process (2 threads),
# one export at a time. Discarded episodes and episodes already in the dataset are skipped by the exporter.
# A2_AUTO_EXPORT=0 turns it off; A2_AUTO_EXPORT_DELAY_S changes the wait.
REPO = Path(__file__).resolve().parents[2]
AUTO_EXPORT = os.environ.get("A2_AUTO_EXPORT", "1") != "0"
AUTO_EXPORT_DELAY_S = float(os.environ.get("A2_AUTO_EXPORT_DELAY_S", "10"))
PY_LEROBOT = REPO / "venv_lerobot" / "bin" / "python"


class AutoExporter:
    def __init__(self, app):
        self.app, self.pending, self.last_stop, self.lock = app, set(), 0.0, threading.Lock()
        self.status = {"state": "idle", "msg": "", "dataset": ""}
        self.thread = None

    def schedule(self, task):
        if not AUTO_EXPORT: return
        if not PY_LEROBOT.exists():
            self.status = {"state": "off", "msg": "auto export off: venv_lerobot missing (README 3.4)", "dataset": ""}; return
        with self.lock:
            self.pending.add(task); self.last_stop = time.time()
            self.status = {"state": "waiting", "msg": f"LeRobot: export of '{task}' in {AUTO_EXPORT_DELAY_S:.0f} s", "dataset": f"lerobot_datasets/{task}"}
            if self.thread is None or not self.thread.is_alive():
                self.thread = threading.Thread(target=self._run, daemon=True, name="auto_export"); self.thread.start()

    def _run(self):
        while True:
            with self.lock:
                if not self.pending: return
                ready = time.time() - self.last_stop >= AUTO_EXPORT_DELAY_S and not getattr(self.app, "rec_active", False)
                task = next(iter(self.pending)) if ready else None
                if task: self.pending.discard(task)
            if task is None: time.sleep(0.5); continue
            out = REPO / "lerobot_datasets" / task
            cmd = ["nice", "-n", "19", str(PY_LEROBOT), str(REPO / "tools" / "export_lerobot.py"), "--task", task,
                   "--out", str(out), "--repo-id", f"local/{task}", "--append"]
            env = dict(os.environ, OMP_NUM_THREADS="2", MKL_NUM_THREADS="2", OPENBLAS_NUM_THREADS="2")
            self.status = {"state": "running", "msg": f"LeRobot: exporting '{task}' -> lerobot_datasets/{task} ...", "dataset": f"lerobot_datasets/{task}"}
            print(f"[EXPORT] auto: {task} -> {out}", flush=True)
            last = ""
            try:
                p = subprocess.Popen(cmd, cwd=str(REPO), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                for line in p.stdout:
                    line = line.strip()
                    if line.startswith(("[ok]", "[skip]", "[trim]", "[warn]", "LeRobot dataset written", "nothing new", "cannot append", "no episodes")):
                        last = line; print(f"[EXPORT] {line[:200]}", flush=True)
                rc = p.wait()
            except Exception as e:
                rc, last = 1, str(e)
            ok = rc == 0
            short = ("added to" if "written" in last else "up to date:" if "nothing new" in last else "") if ok else "FAILED:"
            self.status = {"state": "done" if ok else "error", "dataset": f"lerobot_datasets/{task}",
                           "msg": f"LeRobot: {short} lerobot_datasets/{task}" + ("" if ok else f" ({last[:120]})") + (" - view it on /dataset" if ok else "")}


def _exporter(app):
    if getattr(app, "_auto_exporter", None) is None: app._auto_exporter = AutoExporter(app)
    return app._auto_exporter


def discard_last(app, reason=None, note=None) -> bool:
    """Flag the episode being recorded (it is stopped first) or the last finished one as discarded. Nothing is deleted.
    reason: one of DISCARD_REASONS (default: the one selected in the panel). Calling again on an already discarded
    episode just changes its reason, so you can fix the reason after pressing X."""
    if getattr(app, "rec_active", False): stop_episode(app)
    r = getattr(app, "rec_last", None)
    if not r: return False
    reason = reason or getattr(app, "rec_discard_reason", "other")
    if reason not in DISCARD_REASONS: reason = "other"
    meta = r["meta"]
    meta["discarded"], meta["discard_reason"], meta["discard_unix"] = True, reason, time.time()
    if note is not None: meta["discard_note"] = str(note).strip()[:300]
    try: (Path(r["dir"]) / "meta.json").write_text(json.dumps(meta, indent=2))
    except Exception as e:
        app.rec_msg = f"meta.json: {e}"; return False
    if hasattr(app, "_log_xr_event"): app._log_xr_event(f"REC discard {r['ep']} ({reason})")
    print(f"[REC] DISCARD flag set on {r['dir']} (reason: {reason})")
    return True


def set_discard_reason(app, reason) -> bool:
    if reason not in DISCARD_REASONS: return False
    app.rec_discard_reason = reason
    return True


def _robot_status(app):
    tel = getattr(getattr(app, "robot_backend", None), "rx_telemetry", None)
    return tel.get("rec") if isinstance(tel, dict) else None


def rec_telemetry(app) -> dict:
    r = app.rec if getattr(app, "rec_active", False) else None
    last = getattr(app, "rec_last", None)
    return {
        "active": bool(getattr(app, "rec_active", False)), "task": getattr(app, "rec_task", "default"),
        "instruction": getattr(app, "rec_instruction", ""),
        "ep": r["ep"] if r else (last["ep"] if last else ""), "secs": round(time.time() - r["t0"], 1) if r else 0.0,
        "count": int(getattr(app, "rec_count", 0)), "last_discarded": bool(last and last["meta"].get("discarded")),
        "last_discard_reason": (last["meta"].get("discard_reason", "") if last else ""),
        "discard_reason": getattr(app, "rec_discard_reason", "other"), "discard_reasons": list(DISCARD_REASONS),
        "msg": getattr(app, "rec_msg", ""), "rows": dict(getattr(app, "_rec_rows", {})),
        "images": dict(getattr(app, "_rec_img_counts", {}) or {}),
        "export": dict(getattr(getattr(app, "_auto_exporter", None), "status", None) or {"state": "idle" if AUTO_EXPORT else "off", "msg": ""}),
    }


def _safety_flags(app) -> dict:
    """What the safety stack was doing this tick, so a bad episode can be told apart afterwards (all read-only)."""
    lim = getattr(app, "mirror_limiter", None)
    rs = getattr(app, "robot_safety", None)
    f = lambda v, nd=1: round(float(v), nd)
    return {
        "collision_blocked": bool(getattr(app, "collision_blocked", False)),            # arm IK gate (self-collision) refused a step
        "hand_collision_blocked": bool(getattr(app, "hand_collision_blocked", False)),  # hand-vs-hand filter clamped the step
        "armor_blocked": bool(getattr(lim, "armor_blocked", False)),                    # limiter held back by the armor check
        "armor_pair": str(getattr(lim, "armor_pair", "")) if getattr(lim, "armor_blocked", False) else "",
        "armor_min_dist_mm": f(getattr(lim, "armor_min_dist", 999.0) * 1000.0),
        "proxy_min_dist_mm": f(getattr(app, "proxy_min_distance_m", 0.999) * 1000.0),
        "proxy_scale": f(getattr(app, "proxy_collision_scale", 1.0), 3),                # soft collision speed scale (<1 = slowed)
        "min_hand_dist_mm": f(getattr(app, "min_hand_distance_m", 999.0) * 1000.0),
        "status": str(getattr(app, "status_warning", "") or ""),
        "armor_warning": str(getattr(app, "armor_warning", "") or ""),
        "reach_warning": [str(getattr(app, f"reach_warning_{s}", "") or "") for s in ("l", "r")],
        "ik_pos_err_mm": [f(getattr(app, f"last_ik_pos_err_{s}", 0.0) * 1000.0) for s in ("l", "r")],
        "estop": bool(getattr(app, "state_estop", False)),
        "fault": bool(getattr(rs, "fault_latched", False)),
    }


def live_preview_tick(app):
    """Teleop row for the live page while NOT recording (rate-limited to LOG_HZ). Called from the control loop."""
    if not LIVE.preview_wanted(): return
    now = time.time()
    if now - getattr(app, "_live_prev_t", 0.0) < 1.0 / LOG_HZ: return
    app._live_prev_t = now
    try:
        ee = app.env.get_ee_poses() if hasattr(app, "env") and hasattr(app.env, "get_ee_poses") else {}
        LIVE.push_preview("tel", {"ee": [_vec((ee.get("left") or {}).get("pos")), _vec((ee.get("right") or {}).get("pos"))],
                                  "clos": [round(float(getattr(app, "hand_closure_l", 0.0)), 3), round(float(getattr(app, "hand_closure_r", 0.0)), 3)],
                                  "sf": [int(bool(getattr(getattr(app, "mirror_limiter", None), "armor_blocked", False))),
                                         int(bool(getattr(app, "collision_blocked", False))), int(bool(getattr(app, "hand_collision_blocked", False)))]}, now)

        # In simulation mode ONLY, push simulated robot state and camera to preview
        is_sim_backend = str(getattr(app, "backend", "sim")).lower() in ("sim", "mujoco")
        if is_sim_backend and hasattr(app, "env"):
            ql = app.env.get_left_arm_qpos()
            qr = app.env.get_right_arm_qpos()
            q14 = np.concatenate([ql, qr])
            cmd14 = np.concatenate([getattr(app, "last_arm_cmd_left", ql), getattr(app, "last_arm_cmd_right", qr)])
            hl = getattr(app, "last_hand_cmd_left", np.zeros(6))
            hr = getattr(app, "last_hand_cmd_right", np.zeros(6))
            h12 = np.concatenate([hl, hr])
            LIVE.push_preview("robot", {"q": _vec(q14), "hand": _vec(h12), "cmd": _vec(cmd14), "fault": False}, now)

            # Simulated camera rendering (rate-limited ~10 Hz)
            if now - getattr(app, "_sim_cam_t", 0.0) >= 0.10:
                app._sim_cam_t = now
                if hasattr(app.env, "render_head_camera"):
                    rgb_bytes, depth_bytes, seq = app.env.render_head_camera()
                    if rgb_bytes is not None:
                        LIVE.push_preview("img", {"cam": "head_cam_color", "i": seq, "kind": "color", "fmt": "jpg", "data": rgb_bytes}, now)
                    if depth_bytes is not None:
                        LIVE.push_preview("img", {"cam": "head_cam_depth", "i": seq, "kind": "depth", "fmt": "png", "data": depth_bytes}, now)
    except Exception:
        pass


def live_preview_robot(pkt: dict, arrival_t: float):
    """Robot row for the live page while NOT recording (called from the UDP receive thread)."""
    if not LIVE.preview_wanted(): return
    LIVE.push_preview("robot", {"q": pkt.get("q"), "hand": pkt.get("hand"), "cmd": pkt.get("cmd"), "fault": pkt.get("robot_fault")}, arrival_t)


def log_tick(app):
    """One laptop-side sample of the running episode (rate-limited to LOG_HZ). Called from the control loop."""
    w = getattr(app, "_rec_writer", None)
    if w is None or not getattr(app, "rec_active", False): return
    now = time.time()
    if now - app._rec_last_log_t < 1.0 / LOG_HZ: return
    app._rec_last_log_t = now
    try:
        be = getattr(app, "robot_backend", None)
        tel = getattr(be, "rx_telemetry", None) if be is not None else None
        meas = getattr(be, "_real_q14", None) if be is not None else None
        ee = app.env.get_ee_poses() if hasattr(app, "env") and hasattr(app.env, "get_ee_poses") else {}
        rec = {
            "t": round(now, 4), "t_mono": round(time.perf_counter(), 4), "sync_state": getattr(app, "sync_state", ""),
            "xr_scale": float(getattr(app, "xr_scale", 1.0)),
            "q_cmd": _vec(np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])),
            "q_meas": _vec(meas), "robot_clock_minus_laptop_s": getattr(app, "_rec_clock_off", None),
            "hand_raw_cmd": getattr(be, "_last_hand_raw", None),
            "safety": _safety_flags(app),
        }
        for side, s in (("left", "l"), ("right", "r")):
            e = ee.get(side) or {}
            rec[side] = {
                "ctrl_pos": _vec(getattr(app, f"xr_ctrl_pos_{s}", None)), "ctrl_quat_wxyz": _vec(getattr(app, f"xr_ctrl_quat_{s}", None)),
                "trigger": round(float(getattr(app, f"xr_trig_val_{s}", 0.0)), 3), "grip": round(float(getattr(app, f"xr_grip_val_{s}", 0.0)), 3),
                "clutch": getattr(app, f"xr_clutch_state_{s}", ""),
                "target_pos": _vec(getattr(app, f"cur_target_pos_{s}", None)), "target_quat_xyzw": _quat_xyzw(getattr(app, f"cur_target_rot_{s}", np.eye(3))),
                "ee_pos": _vec(e.get("pos")), "ee_quat_xyzw": _quat_xyzw(e["rot"]) if "rot" in e else None,
                "hand_closure": round(float(getattr(app, f"hand_closure_{s}", 0.0)), 3),
                "hand_cmd": _vec(getattr(app, f"last_hand_cmd_{side}", None), 4),
                "reach_gain": round(float(getattr(app, f"xr_soft_gain_{s}", 1.0)), 3),
            }
        w.log(rec)
        app._rec_rows["teleop"] += 1
        t0 = app.rec["t0"]
        LIVE.push("tel", {"t": round(now - t0, 4), "ee": [rec["left"]["ee_pos"], rec["right"]["ee_pos"]],
                          "clos": [rec["left"]["hand_closure"], rec["right"]["hand_closure"]],
                          "sf": [int(bool(rec["safety"]["armor_blocked"])), int(bool(rec["safety"]["collision_blocked"])),
                                 int(bool(rec["safety"]["hand_collision_blocked"]))]})
    except Exception as e:
        app.rec_msg = f"teleop log: {e}"


def log_robot(app, pkt: dict, arrival_t: float):
    """One row per robot feedback packet (called from the UDP receive thread while an episode is active)."""
    w = getattr(app, "_rec_robot_writer", None)
    if w is None: return
    try:
        t_robot = pkt.get("t")
        if t_robot is not None:        # robot clock minus ours; the minimum over the episode removes the network delay
            off = float(t_robot) - arrival_t
            app._rec_clock_off = off if getattr(app, "_rec_clock_off", None) is None else min(app._rec_clock_off, off)
        w.log({"t_laptop": round(arrival_t, 5), "t_robot": t_robot, "stamp_mc": pkt.get("stamp"),
               "q": pkt.get("q"), "dq": pkt.get("dq"), "tau": pkt.get("tau"), "cmd": pkt.get("cmd"),
               "hand": pkt.get("hand"), "hand_tau": pkt.get("hand_tau"), "hand_effort_raw": pkt.get("hand_effort_raw"),
               "hand_type": pkt.get("hand_type"), "neck": pkt.get("neck"),
               "rx_state": pkt.get("rx_state"), "robot_fault": pkt.get("robot_fault")})
        app._rec_rows["robot"] += 1
        LIVE.push("robot", {"t": round(arrival_t - app.rec["t0"], 4), "q": pkt.get("q"), "hand": pkt.get("hand"), "cmd": pkt.get("cmd"),
                            "fault": pkt.get("robot_fault")})
    except Exception as e:
        app.rec_msg = f"robot log: {e}"


# ---------------------------------------------------------------------------------------------------------------
# Camera frames from the robot (TCP server; the robot connects only while an episode is recording)
# ---------------------------------------------------------------------------------------------------------------
def _recv_exact(conn, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk: return None
        buf += chunk
    return bytes(buf)


def _cam_client(app, conn):
    try:
        conn.settimeout(5.0)
        while True:
            head = _recv_exact(conn, 6)
            if head is None: return
            magic, (hlen,) = head[:4], struct.unpack("<H", head[4:])
            if magic not in (CAM_MAGIC, CAM_STATUS): return
            hraw = _recv_exact(conn, hlen)
            nraw = _recv_exact(conn, 4) if hraw is not None else None
            if nraw is None: return
            (n,) = struct.unpack("<I", nraw)
            if n > 30_000_000: return
            body = _recv_exact(conn, n) if n else b""
            if body is None: return
            t_arr = time.time()
            hdr = json.loads(hraw.decode("utf-8", "replace"))
            if magic == CAM_STATUS:
                app.rec_msg = "camera: " + str(hdr.get("msg", "")); continue
            r, idx = app.rec, getattr(app, "_rec_img_index", None)
            cam = safe_name(hdr.get("cam"), "cam", 24)
            if not getattr(app, "rec_active", False):          # preview: memory only, nothing on disk
                LIVE.push_preview("img", {"cam": cam, "i": hdr.get("seq"), "kind": hdr.get("kind"), "fmt": hdr.get("fmt"), "data": body}, t_arr)
                continue
            counts = getattr(app, "_rec_img_counts", None)
            if not getattr(app, "rec_active", False) or r is None or idx is None or counts is None or cam not in counts: continue
            k = counts[cam]
            name = f"{k:06d}.{'png' if hdr.get('fmt') == 'png' else 'jpg'}"
            try:
                with open(Path(r["dir"]) / "images" / cam / name, "wb") as f: f.write(body)
                row = {"cam": cam, "i": k, "file": f"images/{cam}/{name}", "t_laptop": round(t_arr, 5), "t_robot": hdr.get("t_send"),
                       "stamp": hdr.get("stamp"), "frame_id": hdr.get("frame_id"), "height": hdr.get("height"), "width": hdr.get("width"),
                       "encoding": hdr.get("encoding"), "is_bigendian": hdr.get("is_bigendian"), "step": hdr.get("step"),
                       "stored_w": hdr.get("out_w"), "stored_h": hdr.get("out_h"), "stored_fmt": hdr.get("fmt"), "kind": hdr.get("kind"),
                       "note": hdr.get("note"), "seq": hdr.get("seq")}
                idx.write(json.dumps(row) + "\n")
                LIVE.push("img", {"cam": cam, "i": k, "kind": hdr.get("kind"), "fmt": hdr.get("fmt"), "t": round(t_arr - r["t0"], 4),
                                  "path": str(Path(r["dir"]) / "images" / cam / name)})
                counts[cam] = k + 1
                app._rec_rows["images"] = sum(counts.values())
            except Exception as e:
                app.rec_msg = f"image write: {e}"
    except (OSError, struct.error, ValueError): pass
    finally:
        try: conn.close()
        except OSError: pass


def _ensure_cam_server(app):
    if getattr(app, "_rec_cam_server", None) is not None: return
    if time.time() - getattr(app, "_rec_cam_fail_t", 0.0) < 5.0: return     # called every tick while previewing: retry slowly
    app._rec_cam_fail_t = time.time()
    try:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", CAM_PORT)); srv.listen(2)
    except OSError as e:
        app.rec_msg = f"camera port {CAM_PORT}: {e}"; return
    app._rec_cam_server = srv

    def loop():
        while True:
            try: conn, _ = srv.accept()
            except OSError: return
            threading.Thread(target=_cam_client, args=(app, conn), daemon=True).start()
    threading.Thread(target=loop, name="rec_cam_server", daemon=True).start()
