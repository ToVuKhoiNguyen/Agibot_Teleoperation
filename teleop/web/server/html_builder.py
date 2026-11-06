"""
teleop/web/server/html_builder.py
Loads HTML partials from teleop/web/static/partials and composes the unified panel HTML
with cache-busting build IDs (git rev + startup timestamp).
"""

import os
import re
import subprocess
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
PARTIALS_DIR = os.path.join(PROJECT_ROOT, "teleop", "web", "static", "partials")

PARTIAL_ORDER = [
    "head.html",
    "viewport.html",
    "sidebar_teleop.html",
    "sidebar_status.html",
    "scripts.html",
]


def get_current_build_id() -> str:
    try:
        res = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=PROJECT_ROOT, capture_output=True, text=True, timeout=1)
        rev = res.stdout.strip() or "dev"
    except Exception:
        rev = "dev"
    return f"{rev}_{int(time.time())}"


def load_panel_html(partials_dir: str = PARTIALS_DIR, build_id: str = None) -> str:
    """Concatenates HTML partials in standard display order with dynamic build query."""
    if build_id is None:
        build_id = get_current_build_id()
    chunks = []
    for filename in PARTIAL_ORDER:
        path = os.path.join(partials_dir, filename)
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                chunks.append(f.read())
        else:
            chunks.append(f"<!-- Missing partial {filename} -->")
    html = "".join(chunks)
    html = re.sub(r'(src="/static/js/[^"?]+)(\?[^"]*)?"', rf'\1?v={build_id}"', html)
    html = re.sub(r'(href="/static/css/[^"?]+)(\?[^"]*)?"', rf'\1?v={build_id}"', html)
    return f'<script>window.BUILD_ID = "{build_id}";</script>\n' + html


HTML_PANEL = load_panel_html()
