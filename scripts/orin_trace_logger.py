"""Manages trace_orin_<ts>.jsonl and trace_recorder_<ts>.jsonl async logging."""
import os, time, json
from pathlib import Path
try:
    from teleop.utils.async_writer import AsyncJsonlWriter
except ImportError:
    from async_writer import AsyncJsonlWriter

class OrinTraceLogger:
    def __init__(self, logs_dir, file_sha: str, wire_hz: int, playout_ms: int):
        self.logs_dir = Path(logs_dir)
        self.file_sha = file_sha
        self.wire_hz = wire_hz
        self.playout_ms = playout_ms
        self.writer = None
        self.rec_writer = None
        self._init_writers()

    def _init_writers(self):
        try:
            self.logs_dir.mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            p_orin = self.logs_dir / f"trace_orin_{ts}.jsonl"
            self.writer = AsyncJsonlWriter(p_orin)
            self.writer.log({
                "event": "START", "node_sha256": self.file_sha,
                "wire_hz": self.wire_hz, "playout_ms": self.playout_ms
            })
            # The topic recorder (every CMD + STATE message = 200 JSON lines/s) competes with the control tick for
            # the interpreter; trace_orin already holds the published command and measured state per tick.
            if os.environ.get("ORIN_RECORDER", "0") != "1":
                print(f"[OrinTraceLogger] Logging to {p_orin.name} (topic recorder off; ORIN_RECORDER=1 to enable)")
                return
            p_rec = self.logs_dir / f"trace_recorder_{ts}.jsonl"
            self.rec_writer = AsyncJsonlWriter(p_rec)
            self.rec_writer.log({"event": "START", "node": "builtin_recorder"})
            print(f"[OrinTraceLogger] Logging to {p_orin.name} and {p_rec.name}")
        except Exception as e:
            print(f"[OrinTraceLogger] Init error: {e}")

    def log_tick(self, t_mono: float, seqs_consumed: list, published_q14: list,
                 buf_depth: int, extrapolating: bool, holding: bool, meas_q: list, meas_stamp: float, tm: list = None):
        if not self.writer: return
        self.writer.log({
            "t": t_mono, "seqs_consumed": seqs_consumed,
            "published_q14": [float(round(x, 5)) for x in published_q14],
            "buffer_depth": buf_depth,
            "extrapolating": extrapolating, "holding": holding,
            "measured_q14": [float(round(x, 5)) for x in meas_q] if meas_q else None,
            "measured_stamp": meas_stamp, "tm": tm
        })

    def log_recorded_cmd(self, t_recv: float, stamp_sec: float, q14: list, pub_count: int):
        if not self.rec_writer: return
        self.rec_writer.log({
            "type": "CMD", "t_recv": t_recv, "stamp": stamp_sec,
            "q14": [float(round(x, 5)) for x in q14],
            "pub_count": pub_count
        })

    def log_recorded_state(self, t_recv: float, stamp_sec: float, q14: list):
        if not self.rec_writer: return
        self.rec_writer.log({
            "type": "STATE", "t_recv": t_recv, "stamp": stamp_sec,
            "q14": [float(round(x, 5)) for x in q14]
        })

    def close(self):
        if self.writer: self.writer.close(); self.writer = None
        if self.rec_writer: self.rec_writer.close(); self.rec_writer = None
