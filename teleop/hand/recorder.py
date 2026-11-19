"""
teleop/hand/recorder.py
Ring buffer recording 20 s @ 60 Hz of WebXR hand-tracking telemetry:
raw/filtered poses, finger chain flexions, calibrations, sim EE sites,
and safety clamp flags for offline diagnostics.
"""

import json
import time
from collections import deque
from pathlib import Path
from typing import Dict, Any, List

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
LOGS_DIR = PROJECT_ROOT / "logs"


class HandRecorder:
    """Maintains a 20 s (1200 frames @ 60 Hz) per-hand rolling telemetry buffer."""

    def __init__(self, maxlen: int = 2400):
        # 1200 frames per hand (20s @ 60Hz) = 2400 total
        self.maxlen = maxlen
        self.buffer = deque(maxlen=maxlen)
        self.enabled = True
        LOGS_DIR.mkdir(parents=True, exist_ok=True)

    def record_frame(
        self,
        timestamp: float,
        side: str,
        wrist_raw: Dict[str, Any],
        wrist_filtered: Dict[str, Any],
        clutch: bool,
        p_target: Dict[str, Any],
        actual_site: Dict[str, Any],
        raw_joints_25: List[List[float]],
        flexion_raw: List[float],
        flexion_norm: List[float],
        flexion_cmd: List[float],
        flexion_sim: List[float],
        calibration_used: Dict[str, Any],
        valid: bool,
        confidence: float,
        ik_residual: float,
        clamp_flags: Dict[str, bool],
    ):
        if not self.enabled:
            return

        frame = {
            "timestamp": round(float(timestamp), 4),
            "side": str(side).lower(),
            "wrist_raw": wrist_raw,
            "wrist_filtered": wrist_filtered,
            "clutch": bool(clutch),
            "p_target": p_target,
            "actual_site": actual_site,
            "raw_joints_25": raw_joints_25,
            "flexion": {
                "raw": [round(float(v), 4) for v in flexion_raw],
                "normalized": [round(float(v), 4) for v in flexion_norm],
                "commanded": [round(float(v), 4) for v in flexion_cmd],
                "actual_sim": [round(float(v), 4) for v in flexion_sim],
            },
            "calibration_used": calibration_used,
            "valid": bool(valid),
            "confidence": round(float(confidence), 3),
            "ik_residual": round(float(ik_residual), 5),
            "clamp_flags": {k: bool(v) for k, v in clamp_flags.items()},
        }
        self.buffer.append(frame)

    def save_to_file(self, filename: str = None) -> str:
        """Dumps current ring buffer to logs/handrec_<time>.json."""
        if not filename:
            t_str = time.strftime("%Y%m%d_%H%M%S")
            filename = f"handrec_{t_str}.json"

        out_path = LOGS_DIR / filename
        data = {
            "metadata": {
                "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "sample_count": len(self.buffer),
                "duration_seconds": round(len(self.buffer) / 120.0, 2),  # 2 hands @ 60 Hz
            },
            "frames": list(self.buffer),
        }

        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

        return str(out_path)

    def get_summary(self) -> Dict[str, Any]:
        """Provides brief summary stats for make_report.py."""
        count = len(self.buffer)
        if count == 0:
            return {"count": 0, "status": "empty"}

        clutch_on = sum(1 for f in self.buffer if f["clutch"])
        residuals = [f["ik_residual"] for f in self.buffer if f["clutch"]]
        mean_res = float(sum(residuals) / len(residuals)) if residuals else 0.0

        return {
            "count": count,
            "clutch_ratio": round(clutch_on / max(1, count), 3),
            "mean_ik_residual_mm": round(mean_res * 1000.0, 2),
            "latest_timestamp": self.buffer[-1]["timestamp"] if count > 0 else 0.0,
        }

