"""
tests/test_live_stream.py
Live view (/ws/live): every robot and teleop row of a simulated 3 s / 100 Hz episode reaches the client in order, none
missing, every image is delivered, and a 3D pose arrives for every robot row. Own port, no robot, no running server needed.
Run: venv/bin/python tests/test_live_stream.py
"""
import asyncio, json, struct, sys, tempfile, threading, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np, cv2
from aiohttp import web, ClientSession
import teleop.web.server.live_routes as lr
from teleop.web.server.dataset_routes import DatasetService
from teleop.app.episode_recorder import LIVE

tmp = Path(tempfile.mkdtemp()); lr.DATA_ROOT = tmp
(tmp/"img").mkdir()
class Srv: scene_xml=None
async def main():
    app = web.Application(); ds = DatasetService(Srv()); lr.LiveService(Srv(), ds).register(app)
    runner = web.AppRunner(app); await runner.setup(); site = web.TCPSite(runner, "127.0.0.1", 0); await site.start()
    port = site._server.sockets[0].getsockname()[1]
    N = 300  # rows at 100 Hz = 3 s
    def producer():
        t0 = time.time(); LIVE.begin({"task":"t","ep":"e","dir":str(tmp),"t0":t0,"instruction":"x","cams":["head","head_depth"],"date":"d"})
        z = (np.random.rand(720,1280)*3000).astype(np.uint16); ok, dp = cv2.imencode(".png", z)
        col = np.zeros((720,1280,3),np.uint8); ok, cj = cv2.imencode(".jpg", col)
        for i in range(N):
            LIVE.push("robot", {"t": i*0.01, "q":[0.01*i]*14, "hand":[500]*12, "cmd":[0.0]*14, "fault": None})
            LIVE.push("tel", {"t": i*0.01, "ee":[[0.3,0.2,1.0],[0.3,-0.2,1.0]], "clos":[0.1,0.2], "sf":[0,0,0]})
            if i % 7 == 0:
                for cam,kind,data,ext in (("head","color",cj,"jpg"),("head_depth","depth",dp,"png")):
                    p = tmp/"img"/f"{cam}_{i}.{ext}"; p.write_bytes(data.tobytes())
                    LIVE.push("img", {"cam":cam,"i":i,"kind":kind,"fmt":ext,"t":i*0.01,"path":str(p)})
            time.sleep(0.01)
        LIVE.end({"duration_s":3.0})
    got = {"robot": [], "tel": [], "poses": 0, "imgs": [], "types": set(), "hello": None, "missed": 0}
    async with ClientSession() as cs:
        async with cs.ws_connect(f"http://127.0.0.1:{port}/ws/live", max_msg_size=0) as ws:
            threading.Thread(target=producer, daemon=True).start()
            t_end = time.time()+8
            while time.time() < t_end:
                try: m = await ws.receive(timeout=1.5)
                except asyncio.TimeoutError: break
                if m.type == web.WSMsgType.TEXT:
                    d = json.loads(m.data); got["types"].add(d["type"])
                    if d["type"]=="rows":
                        got["robot"] += [r["s"] for r in d["robot"]]; got["tel"] += [r["s"] for r in d["tel"]]
                        got["missed"] += d["missed"]["robot"]+d["missed"]["tel"]
                    if d["type"]=="hello": got["hello"]=len(d["ids"])
                elif m.type == web.WSMsgType.BINARY:
                    b = m.data
                    if b[0]==1: got["poses"] += struct.unpack("<I", b[1:5])[0]
                    elif b[0]==2:
                        hl = struct.unpack("<I", b[1:5])[0]; h = json.loads(b[5:5+hl]); got["imgs"].append((h["cam"], len(b)-5-hl))
                if "ended" in got["types"] and len(got["robot"])>=N: break
    print("robot rows", len(got["robot"]), "contiguous", got["robot"]==list(range(1,N+1)))
    print("tel rows", len(got["tel"]), "contiguous", got["tel"]==list(range(1,N+1)), "missed", got["missed"])
    print("pose frames", got["poses"], "geoms", got["hello"]); print("images", len(got["imgs"]), got["imgs"][:2], "expected", 2*len(range(0,N,7)))
    print("types", sorted(got["types"]))
    await runner.cleanup()
    assert got["robot"] == list(range(1, N + 1)), "robot rows lost or out of order"
    assert got["tel"] == list(range(1, N + 1)), "teleop rows lost or out of order"
    assert got["missed"] == 0
    assert got["poses"] == N, "3D poses: %d of %d" % (got["poses"], N)
    assert len(got["imgs"]) == 2 * len(range(0, N, 7)), "images lost"
    print("PASS")


asyncio.run(main())


async def preview():
    """No episode recording, one live page open: robot / teleop / camera rows arrive as a preview session."""
    app = web.Application(); ds = DatasetService(Srv()); lr.LiveService(Srv(), ds).register(app)
    runner = web.AppRunner(app); await runner.setup(); site = web.TCPSite(runner, "127.0.0.1", 0); await site.start()
    port = site._server.sockets[0].getsockname()[1]
    from teleop.app.episode_recorder import live_preview_robot
    ok, jpg = cv2.imencode(".jpg", np.zeros((90, 160, 3), np.uint8))
    got = {"robot": [], "imgs": 0, "preview": False, "poses": 0}
    async with ClientSession() as cs:
        async with cs.ws_connect(f"http://127.0.0.1:{port}/ws/live", max_msg_size=0) as ws:
            await ws.receive(timeout=20)                      # hello (MuJoCo loaded)
            assert LIVE.viewers == 1 and LIVE.preview_wanted()
            time.sleep(1.1)                                   # past PREVIEW_AFTER_END_S of the episode above
            def feed():
                for i in range(100):
                    live_preview_robot({"q": [0.01 * i] * 14, "hand": [400] * 12, "cmd": [0.0] * 14}, time.time())
                    if i % 10 == 0: LIVE.push_preview("img", {"cam": "head", "i": i, "kind": "color", "data": jpg.tobytes()}, time.time())
                    time.sleep(0.01)
            threading.Thread(target=feed, daemon=True).start()
            t_end = time.time() + 6
            while time.time() < t_end and len(got["robot"]) < 100:
                try: m = await ws.receive(timeout=1.5)
                except asyncio.TimeoutError: break
                if m.type == web.WSMsgType.TEXT:
                    d = json.loads(m.data)
                    if d["type"] == "episode": got["preview"] = bool(d["info"].get("preview")); got["robot"] = []; got["imgs"] = 0; got["poses"] = 0
                    if d["type"] == "rows" and got["preview"]: got["robot"] += [r["s"] for r in d["robot"]]
                elif m.type == web.WSMsgType.BINARY and got["preview"]:
                    if m.data[0] == 2: got["imgs"] += 1
                    elif m.data[0] == 1: got["poses"] += struct.unpack("<I", m.data[1:5])[0]
            await asyncio.sleep(0.3)
            while True:
                try: m = await ws.receive(timeout=0.5)
                except asyncio.TimeoutError: break
                if m.type == web.WSMsgType.BINARY and m.data[0] == 2: got["imgs"] += 1
    for _ in range(20):
        if LIVE.viewers == 0: break
        await asyncio.sleep(0.1)
    assert LIVE.viewers == 0, "closed page still counted as a viewer (robot would keep sending camera)"
    print("preview: session", got["preview"], "robot rows", len(got["robot"]), "poses", got["poses"], "images", got["imgs"], "viewers after close", LIVE.viewers)
    await runner.cleanup()
    assert got["preview"], "no preview session"
    assert got["robot"] == list(range(1, 101)), "preview robot rows lost or out of order"
    assert got["imgs"] == 10, "preview images lost"
    assert LIVE.viewers == 0 and not LIVE.preview_wanted()
    print("PASS preview")

asyncio.run(preview())
