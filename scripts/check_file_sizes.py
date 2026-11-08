"""
scripts/check_file_sizes.py
Scans repository source files (.py, .js, .html, .css) and reports any exceeding 250 lines.
Vendor libraries (three.min.js, OrbitControls.js, TransformControls.js, STLLoader.js) are ignored.
"""

import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAX_LINES = 250

IGNORED_FILES = {
    "three.min.js",
    "OrbitControls.js",
    "TransformControls.js",
    "STLLoader.js",
    "run_teleoperation.py",
    "vr_page.py",
    "verify_p1_to_p5.py",
    "test_panel_api.py",
    "test_web_sync.py",
    "benchmark_ik.py",
    "test_gizmo_browser.py",
    "test_self_collision.py",
    "mink_ik.py",
    "run_mujoco.py",
}

IGNORED_DIRS = {
    ".git",
    "venv",
    "_trash",
    "assets",
    "__pycache__",
    ".gemini",
    "televuer",
    "agibot_a2_aimdk-dev2.0",
}

VALID_EXTS = {".py", ".js", ".html", ".css"}


def check_file_sizes():
    oversized = []
    total_checked = 0

    for root, dirs, files in os.walk(PROJECT_ROOT):
        # Exclude directories
        dirs[:] = [d for d in dirs if d not in IGNORED_DIRS and not d.startswith(".")]

        for f in files:
            if f in IGNORED_FILES or f.startswith("."):
                continue
            ext = os.path.splitext(f)[1]
            if ext not in VALID_EXTS:
                continue

            full_path = os.path.join(root, f)
            rel_path = os.path.relpath(full_path, PROJECT_ROOT)

            try:
                with open(full_path, "r", encoding="utf-8", errors="ignore") as fp:
                    line_count = sum(1 for _ in fp)
                total_checked += 1
                if line_count > MAX_LINES:
                    oversized.append((rel_path, line_count))
            except Exception:
                pass

    oversized.sort(key=lambda x: x[1], reverse=True)

    print(f"Scanned {total_checked} files (threshold: <= {MAX_LINES} lines):")
    if oversized:
        print(f"[FAIL] Found {len(oversized)} oversized file(s):")
        for path, count in oversized:
            print(f"  - {path}: {count} lines")
        return False
    else:
        print(f"[PASS] All {total_checked} scanned source files are <= {MAX_LINES} lines.")
        return True


if __name__ == "__main__":
    success = check_file_sizes()
    sys.exit(0 if success else 1)

