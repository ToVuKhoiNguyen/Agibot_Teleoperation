"""
teleop/web/server/ws_broadcaster.py
WebSocket telemetry, robot poses streaming, and incoming command handling.
"""

import time
import json
import struct
import queue
import asyncio
from aiohttp import web


async def handle_ws_impl(server, request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    try:
        sock = request.transport.get_extra_info("socket")
        if sock is not None:
            import socket
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass
    server.ws_clients.add(ws)

    # Immediately send static robot manifest to initialize 3D scene
    try:
        await ws.send_str(json.dumps(server.visual_manifest))
        with server.status_lock:
            latest_p = getattr(server, "_latest_full_poses_dict", None)
            if latest_p:
                full_json = json.dumps({"type": "poses", "poses": latest_p})
            else:
                full_json = server._latest_full_json
        if full_json:
            await ws.send_str(full_json)
    except Exception:
        pass

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                try:
                    data = json.loads(msg.data)
                    if data.get("cmd") == "request_full_pose":
                        with server.status_lock:
                            fj = server._latest_full_json
                        if fj and not ws.closed:
                            asyncio.ensure_future(ws.send_str(fj), loop=server.loop)
                    elif data.get("cmd") == "xr_controllers":
                        if hasattr(server, "xr_receiver") and server.xr_receiver is not None:
                            server.xr_receiver.handle_message(data)
                        with server._xr_lock:
                            server._latest_xr_cmd = data
                    else:
                        if hasattr(server, "xr_receiver") and server.xr_receiver is not None:
                            server.xr_receiver.handle_message(data)
                        server.cmd_queue.put(data)
                except Exception:
                    pass
            elif msg.type == web.WSMsgType.BINARY:
                pass
    finally:
        server.ws_clients.discard(ws)
    return ws


def _json_default(o):
    if hasattr(o, "item"): return o.item()
    if hasattr(o, "tolist"): return o.tolist()
    return str(o)


def set_status_impl(server, status_dict: dict):
    with server.status_lock:
        server.latest_status = status_dict.copy()
    if server.loop is not None and len(server.ws_clients) > 0:
        msg = json.dumps({"type": "status", "status": status_dict}, default=_json_default)
        for ws in list(server.ws_clients):
            if not ws.closed:
                asyncio.run_coroutine_threadsafe(ws.send_str(msg), server.loop)


def broadcast_event_impl(server, event_dict: dict):
    if server.loop is not None and len(server.ws_clients) > 0:
        msg = json.dumps(event_dict, default=_json_default)
        for ws in list(server.ws_clients):
            if not ws.closed:
                asyncio.run_coroutine_threadsafe(ws.send_str(msg), server.loop)


def broadcast_poses_impl(server, poses_dict: dict):
    now_ts = time.time()
    arr = []
    for gid in server._moving_geom_ids:
        p = poses_dict.get(str(gid))
        if p and len(p) == 7:
            arr.extend(p)
        else:
            arr.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    hdr = struct.pack("<BBBBd", 0xAA, 0x02, len(server._moving_geom_ids), 0, now_ts)
    f_bytes = struct.pack(f"<{len(arr)}f", *arr)
    bin_payload = hdr + f_bytes

    full_json = None
    if now_ts - server._last_full_broadcast_t >= 2.0:
        server._last_full_broadcast_t = now_ts
        full_json = json.dumps({"type": "poses", "poses": poses_dict})

    with server.status_lock:
        server._latest_moving_binary = bin_payload
        server._latest_full_poses_dict = poses_dict
        if full_json:
            server._pending_full_json = full_json
            server._latest_full_json = full_json
        elif server._latest_full_json is None:
            server._latest_full_json = json.dumps({"type": "poses", "poses": poses_dict})


async def pose_stream_worker_impl(server):
    """Broadcasts latest binary poses at a steady 60 Hz independent of control loop."""
    interval = 1.0 / 60.0
    while True:
        t0 = asyncio.get_event_loop().time()
        try:
            if len(server.ws_clients) > 0:
                with server.status_lock:
                    bin_payload = getattr(server, "_latest_moving_binary", None)
                    full_json = getattr(server, "_pending_full_json", None)
                    server._pending_full_json = None
                if bin_payload is not None:
                    for ws in list(server.ws_clients):
                        if not ws.closed:
                            transport = ws._req.transport if hasattr(ws, "_req") and ws._req else None
                            if transport and hasattr(transport, "get_write_buffer_size") and transport.get_write_buffer_size() > 0:
                                continue  # Drop stale frame on write backlog
                            try:
                                if full_json:
                                    asyncio.ensure_future(ws.send_str(full_json), loop=server.loop)
                                asyncio.ensure_future(ws.send_bytes(bin_payload), loop=server.loop)
                            except Exception:
                                pass
        except Exception:
            pass
        elapsed = asyncio.get_event_loop().time() - t0
        await asyncio.sleep(max(0.001, interval - elapsed))


def dispatch_immediate_stream_impl(server, bin_data, full_json):
    for ws in list(server.ws_clients):
        if not ws.closed:
            transport = ws._req.transport if hasattr(ws, "_req") and ws._req else None
            if transport and hasattr(transport, "get_write_buffer_size") and transport.get_write_buffer_size() > 0:
                continue
            try:
                if full_json:
                    asyncio.ensure_future(ws.send_str(full_json), loop=server.loop)
                asyncio.ensure_future(ws.send_bytes(bin_data), loop=server.loop)
            except Exception:
                pass


def get_command_impl(server):
    try:
        return server.cmd_queue.get_nowait()
    except queue.Empty:
        with server._xr_lock:
            pkt = server._latest_xr_cmd
            server._latest_xr_cmd = None
            return pkt

