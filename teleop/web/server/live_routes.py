"""
teleop/web/server/live_routes.py
Live view of the episode that is being recorded right now (page /dataset, button "Trực tiếp"). Web socket /ws/live.

Source: episode_recorder.LIVE, an in-memory tap on the rows the recorder writes to disk (robot 100 Hz, teleop 100 Hz,
image index). Each connection keeps its own cursor per stream and reads by sequence number, so

- every robot / teleop row is delivered, in order, exactly once (web sockets are TCP; the server awaits the send, a slow
  browser slows only its own connection, never the recorder or the control loop);
- if a browser falls behind by more than the ring (300 s of rows) it is told how many rows it missed ("missed");
- images are also sent in order, one by one. If more than IMG_BACKLOG are waiting (browser or network too slow) the oldest
  ones are skipped for DISPLAY ONLY and counted in "img_skipped" - they are complete on disk in the episode folder.

Preview: while this page is open and NO episode records, the same streams are sent as a "preview" session
(info.preview = true): robot / teleop rows from the control loop and camera frames the robot sends because the command
packets ask for the camera only (no "ep" -> the robot records nothing). Preview frames stay in memory, nothing is saved.

Messages server -> browser:
  text   {"type":"hello"|"episode"|"idle"|"ended"|"rows"|"status", ...}
  binary byte 1: robot 3D poses  | u32 n | u32 seq[n] | f32 poses[n * G * 7]      (geom ids sent in "hello" as "ids")
         byte 2: camera frame    | u32 header_len | header json {cam, kind, i, t, s} | jpeg bytes
Depth frames (16-bit PNG on disk) are colour-mapped 0..3 m (TURBO, black = no depth), as in the exported dataset viewer.
The FK for the 3D view runs in a worker thread, so the event loop and the control loop are not blocked.
"""
import asyncio, json, struct, time
from pathlib import Path

import numpy as np
from aiohttp import web

from teleop.app.episode_recorder import LIVE, DATA_ROOT

TICK_S = 0.03
IMG_BACKLOG = 45                      # waiting image rows (all cameras) before the oldest are skipped for display
IMG_PER_TICK = 6
POSE_MAX = 30                         # FK rows per tick (normally 3 at 100 Hz); a bigger backlog keeps only the newest
DEPTH_W = 640                         # depth is shrunk to this width for display (nearest neighbour)


def _jpeg_for_display(row: dict) -> bytes:
    kind = row.get("kind")
    if row.get("data") is not None: raw = row["data"]                 # preview frame (memory only)
    else:
        p = Path(row["path"])
        if not p.is_relative_to(DATA_ROOT): raise ValueError("path outside recordings")
        raw = p.read_bytes()
    if kind != "depth": return raw
    import cv2
    z = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_UNCHANGED)
    if z is None: return b""
    if z.shape[1] > DEPTH_W: z = cv2.resize(z, (DEPTH_W, int(z.shape[0] * DEPTH_W / z.shape[1])), interpolation=cv2.INTER_NEAREST)
    z = z.astype(np.float32)
    col = cv2.applyColorMap(np.clip(z / 3000.0 * 255.0, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    col[(z <= 10.0) | (z >= 9999.0)] = 0
    ok, enc = cv2.imencode(".jpg", col, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return enc.tobytes() if ok else b""


class LiveService:
    def __init__(self, server, dataset_service):
        self.server, self.ds = server, dataset_service

    def register(self, app):
        app.router.add_get("/ws/live", self.ws_live)

    async def ws_live(self, request):
        ws = web.WebSocketResponse(heartbeat=20, max_msg_size=0)
        await ws.prepare(request)
        loop = asyncio.get_running_loop()
        session = -1
        LIVE.viewers += 1          # while > 0 and no episode records, robot / teleop / camera are pushed as a preview

        async def drain():         # read the browser side so a closed tab is noticed at once (else only on the next send)
            async for _ in ws: pass
        reader = asyncio.ensure_future(drain())
        try:        # load MuJoCo once now (takes seconds the first time) so no 3D pose is lost when the episode starts
            ids, _ = await loop.run_in_executor(None, self.ds.fk_poses, [([0.0] * 14, [0.0] * 12)])
            await ws.send_str(json.dumps({"type": "hello", "ids": ids}))
        except Exception as e:
            await ws.send_str(json.dumps({"type": "status", "error": f"3D: {e}"}))
        cur = {"robot": 0, "tel": 0, "img": 0}
        img_skipped, last_idle = 0, 0.0
        try:
            while not ws.closed and not reader.done():
                sess, active, info, seq = LIVE.state()
                if sess != session:                                    # a new episode started (or first look)
                    session, cur = sess, {"robot": 0, "tel": 0, "img": 0}; img_skipped = 0
                    if info is None:
                        await ws.send_str(json.dumps({"type": "idle"}))
                    else:
                        await ws.send_str(json.dumps({"type": "episode", "info": info, "active": active}))
                if info is None:
                    await asyncio.sleep(0.25); continue
                robot, m_r, _ = LIVE.read("robot", cur["robot"]); tel, m_t, _ = LIVE.read("tel", cur["tel"])
                if robot: cur["robot"] = robot[-1]["s"]
                if tel: cur["tel"] = tel[-1]["s"]
                msg = {"type": "rows", "robot": robot, "tel": tel, "missed": {"robot": m_r, "tel": m_t},
                       "newest": {"robot": seq["robot"], "tel": seq["tel"], "img": seq["img"]}, "active": active}
                if robot or tel or m_r or m_t: await ws.send_str(json.dumps(msg))
                # robot 3D poses for every robot row of this batch (FK in a worker thread)
                pose_rows = [(r["s"], r["q"], r.get("hand")) for r in robot if r.get("q") and len(r["q"]) >= 14]
                pose_rows = pose_rows[-POSE_MAX:]                      # catching up after a reconnect: only the newest poses
                if pose_rows:
                    ids, arr = await loop.run_in_executor(None, self.ds.fk_poses, [(q, h) for _, q, h in pose_rows])
                    head = struct.pack("<BI", 1, len(pose_rows)) + np.array([s for s, _, _ in pose_rows], dtype=np.uint32).tobytes()
                    await ws.send_bytes(head + arr.astype(np.float32).tobytes())
                # camera frames, in order
                imgs, m_i, newest_i = LIVE.read("img", cur["img"], limit=500)
                if len(imgs) > IMG_BACKLOG:
                    drop = len(imgs) - IMG_BACKLOG // 3
                    img_skipped += drop; imgs = imgs[drop:]
                    await ws.send_str(json.dumps({"type": "status", "img_skipped": img_skipped}))
                for row in imgs[:IMG_PER_TICK]:
                    cur["img"] = row["s"]
                    try: jpg = await loop.run_in_executor(None, _jpeg_for_display, row)
                    except Exception: continue
                    if not jpg: continue
                    hdr = json.dumps({"cam": row["cam"], "kind": row.get("kind"), "i": row["i"], "t": row["t"], "s": row["s"]}).encode()
                    await ws.send_bytes(struct.pack("<BI", 2, len(hdr)) + hdr + jpg)
                if len(imgs) > IMG_PER_TICK: continue                  # still behind: no sleep
                streaming = active or bool(info.get("preview"))
                if not streaming and not (robot or tel or imgs):
                    if time.time() - last_idle > 1.0:
                        last_idle = time.time()
                        await ws.send_str(json.dumps({"type": "ended", "info": info}))
                    await asyncio.sleep(0.25)
                else:
                    await asyncio.sleep(TICK_S)
        except (ConnectionResetError, asyncio.CancelledError): pass
        finally:
            LIVE.viewers = max(0, LIVE.viewers - 1)
            reader.cancel()
            if not ws.closed: await ws.close()
        return ws
