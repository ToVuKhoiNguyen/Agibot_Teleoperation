"""
teleop/utils/session_logger.py
Logs teleop session metrics and health stats to logs/session_<timestamp>.json on shutdown.
"""

import json
import time
from pathlib import Path
import numpy as np


def save_session_log(app) -> str:
    """Saves session summary JSON into logs/ directory."""
    logs_dir = Path(getattr(app, "project_root", Path(__file__).resolve().parents[2])) / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    filepath = logs_dir / f"session_{ts}.json"

    ages = getattr(app, "msg_ages_ms", [])
    p95_age = float(np.percentile(ages, 95)) if len(ages) > 0 else 0.0

    dur = max(0.001, time.time() - getattr(app, "session_start_time", time.time()))
    pkts = getattr(app, "xr_total_pkts", 0)
    rate_pkt = float(pkts / dur) if pkts > 0 else 0.0

    session_data = {
        "timestamp": ts,
        "duration_s": round(dur, 2),
        "backend": str(getattr(app, "backend", "sim")),
        "loop_hz": round(float(getattr(app, "loop_hz", 200.0)), 1),
        "state_msgs_s": round(float(getattr(app, "cmd_rate_hz", 25.0)), 1),
        "msg_age_p95_ms": round(p95_age, 2),
        "frame_gaps": int(getattr(app, "frame_gaps", 0)),
        "controller_packet_rate_per_hand": {
            "left": round(rate_pkt if getattr(app, "xr_left_connected", False) else 0.0, 1),
            "right": round(rate_pkt if getattr(app, "xr_right_connected", False) else 0.0, 1)
        },
        "tracking_loss_events": int(getattr(app, "tracking_loss_events", 0)),
        "clamp_counters": {
            "arm": int(getattr(app, "clamp_counter_arm", 0)),
            "hand": int(getattr(app, "clamp_counter_hand", 0))
        },
        "fault_events": int(getattr(app, "fault_events", 0)),
        "input_mode": str(getattr(app, "input_mode", "controllers")),
        "handtrack_packet_rate": round(float(getattr(app, "handtrack_rate_hz", 0.0)), 1),
        "handtrack_clutch_events": int(getattr(app, "handtrack_clutch_events", 0)),
        "total_xr_packets": int(pkts),
        "target_trace_summary": app.target_tracer.get_summary() if hasattr(app, "target_tracer") and app.target_tracer else {},
    }

    with open(filepath, "w", encoding="utf-8") as f:
        json.dump(session_data, f, indent=2)

    print(f"[INFO] Session log written to {filepath}")
    return str(filepath)

