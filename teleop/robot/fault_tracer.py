"""
teleop/robot/fault_tracer.py
Ring buffer (5 s @ 50 Hz) and logger for SafetyLayer position deviation faults.
"""

import os
import time
import json
import collections
from pathlib import Path
from teleop.robot.aimdk_map import ARM_JOINT_NAMES_AIMDK

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent


class FaultTracer:
    """Maintains a 50 Hz rolling trace of joint commands, clamps, and deviations."""

    def __init__(self, maxlen: int = 250):
        self.ring_buffer = collections.deque(maxlen=maxlen)
        self.last_trip_event = None
        self._last_sample_t = 0.0

    def sample(
        self,
        backend_name: str,
        raw_q: list,
        post_q: list,
        pub_q: list,
        act_q: list,
        qvel: list,
        active_clamps: list,
        limits: list,
        dt_jitter: float,
        input_mode: str,
        collision_gate: bool,
        worst_joint: int,
        t_now: float = None,
    ):
        now = t_now if t_now is not None else time.time()
        # Sample at ~50 Hz (interval >= 18 ms)
        if (now - self._last_sample_t) < 0.018:
            return
        self._last_sample_t = now

        j = int(worst_joint)
        record = {
            "t": round(now, 4),
            "backend": backend_name,
            "joint_idx_0based": j,
            "joint_idx_1based": j + 1,
            "joint_name": ARM_JOINT_NAMES_AIMDK[j] if j < len(ARM_JOINT_NAMES_AIMDK) else f"joint_{j}",
            "raw_ik_target": round(float(raw_q[j]), 5) if j < len(raw_q) else 0.0,
            "post_clamp_cmd": round(float(post_q[j]), 5) if j < len(post_q) else 0.0,
            "published_cmd": round(float(pub_q[j]), 5) if j < len(pub_q) else 0.0,
            "actual_readback": round(float(act_q[j]), 5) if j < len(act_q) else 0.0,
            "deviation": round(abs(float(post_q[j]) - float(act_q[j])), 5) if j < min(len(post_q), len(act_q)) else 0.0,
            "active_clamp": active_clamps[j] if j < len(active_clamps) else "none",
            "joint_limit": [round(float(limits[j][0]), 3), round(float(limits[j][1]), 3)] if j < len(limits) else [0.0, 0.0],
            "joint_velocity": round(float(qvel[j]), 4) if j < len(qvel) else 0.0,
            "input_mode": input_mode,
            "collision_gate": bool(collision_gate),
            "loop_dt_jitter_ms": round(float(dt_jitter) * 1000.0, 3),
        }
        self.ring_buffer.append(record)

    def format_trip(self, j: int, err: float, thresh: float, target_q, q_act, lower, upper, clamp: str, backend: str, ctx: dict, t_now: float):
        j_name = ARM_JOINT_NAMES_AIMDK[j] if j < len(ARM_JOINT_NAMES_AIMDK) else f"joint_{j}"
        low_lim, high_lim = float(lower[j]), float(upper[j])
        cmd_v, act_v = float(target_q[j]), float(q_act[j])
        hint = "joint at physical limit" if (act_v <= low_lim + 0.02 or act_v >= high_lim - 0.02) else (
            "sudden target step" if err > 0.15 else "motor lag / obstruction"
        )
        reason = (
            f"Position deviation >{thresh:.2f} rad (>200ms) on joint {j} (0-based: {j_name}) "
            f"[cmd: {cmd_v:+.3f}, act: {act_v:+.3f}, lim: ({low_lim:.2f}, {high_lim:.2f})] (hint: {hint})"
        )
        event = {
            "t": t_now, "reason": reason, "joint_0based": j, "joint_1based": j + 1, "joint_name": j_name,
            "cmd": cmd_v, "act": act_v, "err": float(err), "thresh": float(thresh),
            "limits": [low_lim, high_lim], "clamp": clamp, "backend": backend,
            "input_mode": ctx.get("input_mode", "mouse"), "collision": bool(ctx.get("collision_gate", False)),
        }
        return reason, event

    def record_trip(self, event_dict: dict):
        """Called when deviation gate trips to latch event and write to log."""
        self.last_trip_event = event_dict
        logs_dir = PROJECT_ROOT / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        t_str = time.strftime("%Y%m%d_%H%M%S")
        log_path = logs_dir / f"fault_trace_{t_str}.json"
        try:
            with open(log_path, "w", encoding="utf-8") as fp:
                json.dump({
                    "trip_event": event_dict,
                    "buffer_count": len(self.ring_buffer),
                    "trace": list(self.ring_buffer),
                }, fp, indent=2)
            event_dict["log_file"] = str(log_path)
        except Exception:
            pass

    def get_summary(self) -> dict:
        return {
            "buffer_count": len(self.ring_buffer),
            "last_trip_event": self.last_trip_event,
            "latest_sample": self.ring_buffer[-1] if self.ring_buffer else None,
        }
