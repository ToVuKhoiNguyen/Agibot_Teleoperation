"""
teleop/app/target_tracer.py
Server-side 20 Hz target source tracer ring buffer.
Tracks target provenance, deltas, hand-site speed, follower residual, and IK residual.
"""

import time
import collections
import numpy as np

VALID_SOURCES = {
    "gizmo",
    "mouse panel step",
    "controller clutch",
    "hand tracking",
    "preset/demo/sequence",
    "home/smooth-home",
    "replay",
    "none",
}


class TargetSourceTracer:
    """Ring buffer maintaining 20 Hz trace of arm target sources and tracking residuals."""

    def __init__(self, maxlen: int = 2400):
        self.ring_buffer = collections.deque(maxlen=maxlen)
        self.last_source_l = "none"
        self.last_source_r = "none"
        self._prev_target_pos_l = None
        self._prev_target_pos_r = None
        self._prev_target_rot_l = None
        self._prev_target_rot_r = None
        self._prev_actual_pos_l = None
        self._prev_actual_pos_r = None
        self._prev_tick_time = None

    def record_target_source(self, arm: str, source: str):
        """Records which subsystem wrote the arm target."""
        if source not in VALID_SOURCES:
            source = "none"
        if arm in ("left", "both"):
            self.last_source_l = source
        if arm in ("right", "both"):
            self.last_source_r = source

    def reset_sources(self):
        self.last_source_l = "none"
        self.last_source_r = "none"

    def clear(self):
        self.ring_buffer.clear()
        self.last_source_l = "none"
        self.last_source_r = "none"

    def tick(self, app):
        """Samples state at 20 Hz and pushes into ring buffer."""
        now = time.perf_counter()
        dt = (now - self._prev_tick_time) if self._prev_tick_time is not None else 0.05
        if dt <= 0.0:
            dt = 0.05
        self._prev_tick_time = now

        # Actual hand-site positions from environment
        ee = app.env.get_ee_poses()
        act_pos_l = np.array(ee["left"]["pos"], dtype=np.float64)
        act_pos_r = np.array(ee["right"]["pos"], dtype=np.float64)

        # Target positions & rotations
        tgt_pos_l = np.array(app.cur_target_pos_l, dtype=np.float64)
        tgt_pos_r = np.array(app.cur_target_pos_r, dtype=np.float64)
        tgt_rot_l = np.array(app.cur_target_rot_l, dtype=np.float64)
        tgt_rot_r = np.array(app.cur_target_rot_r, dtype=np.float64)

        # Target position delta since last tick (mm)
        if self._prev_target_pos_l is not None:
            tgt_delta_mm_l = float(np.linalg.norm(tgt_pos_l - self._prev_target_pos_l) * 1000.0)
        else:
            tgt_delta_mm_l = 0.0
        if self._prev_target_pos_r is not None:
            tgt_delta_mm_r = float(np.linalg.norm(tgt_pos_r - self._prev_target_pos_r) * 1000.0)
        else:
            tgt_delta_mm_r = 0.0

        # Target rotation delta since last tick (deg)
        tgt_delta_deg_l = self._rot_diff_deg(tgt_rot_l, self._prev_target_rot_l)
        tgt_delta_deg_r = self._rot_diff_deg(tgt_rot_r, self._prev_target_rot_r)

        # Actual hand-site velocity (mm/s)
        if self._prev_actual_pos_l is not None and dt > 1e-4:
            vel_l = float((np.linalg.norm(act_pos_l - self._prev_actual_pos_l) / dt) * 1000.0)
        else:
            vel_l = 0.0
        if self._prev_actual_pos_r is not None and dt > 1e-4:
            vel_r = float((np.linalg.norm(act_pos_r - self._prev_actual_pos_r) / dt) * 1000.0)
        else:
            vel_r = 0.0

        # Follower residual: target minus actual (mm)
        res_vec_l = (tgt_pos_l - act_pos_l) * 1000.0
        res_vec_r = (tgt_pos_r - act_pos_r) * 1000.0
        follower_res_l = float(np.linalg.norm(res_vec_l))
        follower_res_r = float(np.linalg.norm(res_vec_r))

        # IK residual (mm)
        ik_res_l = float(getattr(app, "last_ik_residual_l", 0.0) * 1000.0)
        ik_res_r = float(getattr(app, "last_ik_residual_r", 0.0) * 1000.0)

        # Update previous tracking state
        self._prev_target_pos_l = tgt_pos_l.copy()
        self._prev_target_pos_r = tgt_pos_r.copy()
        self._prev_target_rot_l = tgt_rot_l.copy()
        self._prev_target_rot_r = tgt_rot_r.copy()
        self._prev_actual_pos_l = act_pos_l.copy()
        self._prev_actual_pos_r = act_pos_r.copy()

        input_mode = str(getattr(app, "input_mode", "controllers"))

        entry = {
            "t": round(now, 4),
            "input_mode": input_mode,
            "left": {
                "source": self.last_source_l,
                "target_delta_mm": round(tgt_delta_mm_l, 4),
                "target_delta_deg": round(tgt_delta_deg_l, 4),
                "hand_velocity_mm_s": round(vel_l, 3),
                "follower_residual_mm": round(follower_res_l, 3),
                "follower_residual_vec": [round(float(x), 3) for x in res_vec_l],
                "ik_residual_mm": round(ik_res_l, 4),
            },
            "right": {
                "source": self.last_source_r,
                "target_delta_mm": round(tgt_delta_mm_r, 4),
                "target_delta_deg": round(tgt_delta_deg_r, 4),
                "hand_velocity_mm_s": round(vel_r, 3),
                "follower_residual_mm": round(follower_res_r, 3),
                "follower_residual_vec": [round(float(x), 3) for x in res_vec_r],
                "ik_residual_mm": round(ik_res_r, 4),
            },
        }
        self.ring_buffer.append(entry)
        self.last_source_l = "none"
        self.last_source_r = "none"

    def _rot_diff_deg(self, r_curr, r_prev) -> float:
        if r_prev is None or r_curr is None:
            return 0.0
        try:
            r_diff = r_curr @ r_prev.T
            tr = float(np.trace(r_diff))
            cos_th = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
            return float(np.degrees(np.arccos(cos_th)))
        except Exception:
            return 0.0

    def get_summary(self) -> dict:
        """Returns statistics summary over currently buffered entries."""
        if not self.ring_buffer:
            return {
                "samples": 0,
                "left": {"max_speed_mm_s": 0.0, "max_tgt_delta_mm": 0.0, "sources": []},
                "right": {"max_speed_mm_s": 0.0, "max_tgt_delta_mm": 0.0, "sources": []},
            }
        entries = list(self.ring_buffer)
        speeds_l = [e["left"]["hand_velocity_mm_s"] for e in entries]
        speeds_r = [e["right"]["hand_velocity_mm_s"] for e in entries]
        tgt_deltas_l = [e["left"]["target_delta_mm"] for e in entries]
        tgt_deltas_r = [e["right"]["target_delta_mm"] for e in entries]
        sources_l = sorted(list(set(e["left"]["source"] for e in entries)))
        sources_r = sorted(list(set(e["right"]["source"] for e in entries)))

        return {
            "samples": len(entries),
            "left": {
                "max_speed_mm_s": round(float(np.max(speeds_l)), 3) if speeds_l else 0.0,
                "mean_speed_mm_s": round(float(np.mean(speeds_l)), 3) if speeds_l else 0.0,
                "max_tgt_delta_mm": round(float(np.max(tgt_deltas_l)), 3) if tgt_deltas_l else 0.0,
                "sources": sources_l,
            },
            "right": {
                "max_speed_mm_s": round(float(np.max(speeds_r)), 3) if speeds_r else 0.0,
                "mean_speed_mm_s": round(float(np.mean(speeds_r)), 3) if speeds_r else 0.0,
                "max_tgt_delta_mm": round(float(np.max(tgt_deltas_r)), 3) if tgt_deltas_r else 0.0,
                "sources": sources_r,
            },
        }
