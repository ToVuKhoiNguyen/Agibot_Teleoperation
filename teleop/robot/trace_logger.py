"""Manages trace_laptop_<ts>.jsonl async logging and config headers."""
import os, time, json, subprocess
import numpy as np
from teleop.utils.async_writer import AsyncJsonlWriter
from teleop.robot.arrival_detector import ArrivalDetector

class LaptopTraceLogger:
    def __init__(self, logs_dir, wire_hz: int, speed_level: int):
        self.logs_dir = logs_dir
        self.wire_hz = wire_hz
        self.speed_level = speed_level
        self.writer = None
        self.detector = ArrivalDetector()
        self.last_sent = None
        self._init_writer()

    def _init_writer(self):
        try:
            self.logs_dir.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            p = self.logs_dir / f"trace_laptop_{ts}.jsonl"
            self.writer = AsyncJsonlWriter(p)
            git_sha = subprocess.check_output(["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL).decode().strip()
        except Exception:
            git_sha = "unknown"
        cfg = {
            "event": "CONFIG",
            "git_sha": git_sha,
            "speed": self.speed_level,
            "wire_hz": self.wire_hz,
            "env_switches": {
                "CART_FOLLOWER": os.environ.get("CART_FOLLOWER", "1"),
                "FIR_N": os.environ.get("FIR_N", "6"),
                "END_APPROACH": os.environ.get("END_APPROACH", "0"),
                "WRIST_SCALE": os.environ.get("WRIST_SCALE", "0.5"),
                "WIRE_HZ": str(self.wire_hz),
            }
        }
        if self.writer: self.writer.log(cfg)

    def log_tick(self, seq: int, t_perf: float, raw_target: np.ndarray,
                 after_follower: np.ndarray, plan_q: np.ndarray, sent_q: np.ndarray):
        if not self.writer: return
        dq = list(np.round(sent_q - (self.last_sent if self.last_sent is not None else sent_q), 6))
        self.last_sent = sent_q.copy()
        arrived, first_arr = self.detector.update(sent_q, raw_target, seq)
        if first_arr:
            self.writer.log({"event": "ARRIVAL", "seq": seq, "t": t_perf})
        entry = {
            "seq": seq, "t": t_perf,
            "raw_target_q14": [float(round(x, 5)) for x in raw_target],
            "after_follower_q14": [float(round(x, 5)) for x in after_follower],
            "plan_q14": [float(round(x, 5)) for x in plan_q],
            "sent_q14": [float(round(x, 5)) for x in sent_q],
            "dq": dq, "arrived": arrived
        }
        self.writer.log(entry)

    def close(self):
        if self.writer: self.writer.close(); self.writer = None
