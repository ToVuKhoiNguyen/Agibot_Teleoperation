"""
teleop/simulation/web_panel.py
Facade preserving backwards compatibility for WebPanelServer.
Implementation lives in teleop.web.server.web_server.
"""

from teleop.web.server.web_server import (
    WebPanelServer,
    MOVING_GEOM_IDS,
    PROJECT_ROOT,
    STATIC_DIR,
    STATIC_JS_DIR,
    STATIC_CSS_DIR,
    WEB_MESHES_DIR,
    BUNDLE_BIN_PATH,
    BUNDLE_GZ_PATH,
)
from teleop.web.server.html_builder import HTML_PANEL
from .vr_page import HTML_VR

__all__ = [
    "WebPanelServer",
    "MOVING_GEOM_IDS",
    "HTML_PANEL",
    "HTML_VR",
    "PROJECT_ROOT",
    "STATIC_DIR",
    "STATIC_JS_DIR",
    "STATIC_CSS_DIR",
    "WEB_MESHES_DIR",
    "BUNDLE_BIN_PATH",
    "BUNDLE_GZ_PATH",
]
