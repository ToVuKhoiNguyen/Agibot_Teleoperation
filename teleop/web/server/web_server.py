"""
teleop/web/server/web_server.py
Core WebPanelServer hosting HTTPS, WebSocket streaming, and static assets.
"""

import os
import ssl
import queue
import asyncio
import threading
from aiohttp import web

from teleop.web.server.manifest_builder import (
    build_visual_manifest,
    load_bundle_cache,
    handle_bundle_bin_response,
)
from teleop.web.server.panel_routes import (
    handle_health_impl,
    handle_panel_page,
    handle_vr_page,
    handle_post_quest_launch_impl,
    handle_post_cmd_impl,
    handle_get_status_impl,
)
from teleop.web.server.ws_broadcaster import (
    handle_ws_impl,
    set_status_impl,
    broadcast_poses_impl,
    dispatch_immediate_stream_impl,
    get_command_impl,
)
from teleop.web.server.html_builder import HTML_PANEL

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
STATIC_DIR = os.path.join(PROJECT_ROOT, "teleop", "web", "static")
STATIC_JS_DIR = os.path.join(STATIC_DIR, "js")
STATIC_CSS_DIR = os.path.join(STATIC_DIR, "css")
WEB_MESHES_DIR = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "web_meshes")
BUNDLE_BIN_PATH = os.path.join(WEB_MESHES_DIR, "bundle.bin")
BUNDLE_GZ_PATH = os.path.join(WEB_MESHES_DIR, "bundle.bin.gz")

MOVING_GEOM_IDS = [
    44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 67, 69, 71, 73,
    75, 77, 79, 81, 83, 85, 87, 89, 91, 93, 95, 97, 99, 101, 103, 105,
    107, 109, 111, 113, 114, 116, 118, 120, 122, 124, 126, 128, 130, 132, 134, 136
]


class WebPanelServer:
    """Lightweight HTTPS server providing 3D WebGL/WebXR control panel and WebSocket streaming."""

    def __init__(self, port: int = 8012, cert_file: str = None, key_file: str = None, scene_xml: str = None):
        self.port = port
        self.cert_file = cert_file
        self.key_file = key_file
        self.scene_xml = scene_xml

        self.latest_status = {}
        self.status_lock = threading.Lock()
        self.cmd_queue = queue.Queue()
        self.ws_clients = set()
        self.visual_manifest = build_visual_manifest(self.scene_xml, PROJECT_ROOT)

        self._moving_geom_ids = MOVING_GEOM_IDS
        self._bundle_raw, self._bundle_gz, self._bundle_etag = load_bundle_cache(BUNDLE_BIN_PATH, BUNDLE_GZ_PATH)

        self._last_full_broadcast_t = 0.0
        self._latest_full_json = None
        self._latest_moving_binary = None
        self._xr_lock = threading.Lock()
        self._latest_xr_cmd = None

        @web.middleware
        async def static_cache_middleware(request, handler):
            resp = await handler(request)
            if request.path.startswith("/web_meshes/") or request.path.startswith("/static/"):
                resp.headers["Cache-Control"] = "public, max-age=86400"
            return resp

        self.app = web.Application(middlewares=[static_cache_middleware])
        self.app.router.add_get("/", self.handle_panel)
        self.app.router.add_get("/panel", self.handle_panel)
        self.app.router.add_get("/vr", self.handle_vr)
        self.app.router.add_get("/health", self.handle_health)
        self.app.router.add_get("/bundle.bin", self.handle_bundle_bin)
        self.app.router.add_get("/ws_panel", self.handle_ws)
        self.app.router.add_get("/api/status", self.handle_get_status)
        self.app.router.add_post("/api/command", self.handle_post_cmd)
        self.app.router.add_post("/api/launch_quest", self.handle_post_quest_launch)

        # Serve static assets
        if os.path.exists(STATIC_DIR):
            self.app.router.add_static("/static", STATIC_DIR)
        if os.path.exists(WEB_MESHES_DIR):
            self.app.router.add_static("/web_meshes", WEB_MESHES_DIR)

        self.loop = None
        self.thread = None

    def _build_visual_manifest(self):
        return build_visual_manifest(self.scene_xml, PROJECT_ROOT)

    def _load_bundle_cache(self):
        self._bundle_raw, self._bundle_gz, self._bundle_etag = load_bundle_cache(BUNDLE_BIN_PATH, BUNDLE_GZ_PATH)

    async def handle_bundle_bin(self, request):
        return await handle_bundle_bin_response(request, self._bundle_raw, self._bundle_gz, self._bundle_etag)

    async def handle_root_redirect(self, request):
        return await handle_panel_page(HTML_PANEL, request)

    async def handle_health(self, request):
        return await handle_health_impl(self, request)

    async def handle_panel(self, request):
        return await handle_panel_page(HTML_PANEL, request)

    async def handle_vr(self, request):
        return await handle_vr_page(HTML_PANEL, request)

    async def handle_post_quest_launch(self, request):
        qm = getattr(self, "quest_manager", None)
        return await handle_post_quest_launch_impl(qm, request)

    async def handle_ws(self, request):
        return await handle_ws_impl(self, request)

    async def handle_post_cmd(self, request):
        return await handle_post_cmd_impl(self.cmd_queue, request)

    async def handle_get_status(self, request):
        return await handle_get_status_impl(self.status_lock, self.latest_status, request)

    def set_status(self, status_dict: dict):
        set_status_impl(self, status_dict)

    def broadcast_poses(self, poses_dict: dict):
        broadcast_poses_impl(self, poses_dict)

    def _dispatch_immediate_stream(self, bin_data, full_json):
        dispatch_immediate_stream_impl(self, bin_data, full_json)

    def get_command(self):
        return get_command_impl(self)

    def start(self):
        def _runner():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

            def _exception_handler(loop, context):
                pass
            self.loop.set_exception_handler(_exception_handler)

            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(self.cert_file, self.key_file)

            runner = web.AppRunner(self.app)
            self.loop.run_until_complete(runner.setup())
            site_v4 = web.TCPSite(runner, "0.0.0.0", self.port, ssl_context=ssl_context)
            self.loop.run_until_complete(site_v4.start())
            try:
                site_v6 = web.TCPSite(runner, "::", self.port, ssl_context=ssl_context)
                self.loop.run_until_complete(site_v6.start())
            except Exception:
                pass
            self.loop.run_forever()

        self.thread = threading.Thread(target=_runner, daemon=True)
        self.thread.start()

    def stop(self):
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=1.0)

