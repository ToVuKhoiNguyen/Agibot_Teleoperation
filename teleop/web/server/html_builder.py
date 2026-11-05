"""
teleop/web/server/html_builder.py
Loads HTML partials from teleop/web/static/partials and composes the unified panel HTML.
"""

import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
PARTIALS_DIR = os.path.join(PROJECT_ROOT, "teleop", "web", "static", "partials")

PARTIAL_ORDER = [
    "head.html",
    "viewport.html",
    "sidebar_teleop.html",
    "sidebar_status.html",
    "scripts.html",
]


def load_panel_html(partials_dir: str = PARTIALS_DIR) -> str:
    """Concatenates HTML partials in standard display order."""
    chunks = []
    for filename in PARTIAL_ORDER:
        path = os.path.join(partials_dir, filename)
        if os.path.exists(path):
            with open(path, "r", encoding="utf-8") as f:
                chunks.append(f.read())
        else:
            chunks.append(f"<!-- Missing partial {filename} -->")
    return "".join(chunks)


HTML_PANEL = load_panel_html()

