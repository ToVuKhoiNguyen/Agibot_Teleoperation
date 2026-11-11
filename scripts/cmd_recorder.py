#!/usr/bin/env python3
"""scripts/cmd_recorder.py - Read-only command & state recorder for Orin."""
import os, sys, time, json
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
try:
    from teleop.utils.async_writer import AsyncJsonlWriter
except ImportError:
    from async_writer import AsyncJsonlWriter

ARM_NAMES_LONG = [f"idx{i}_{'left' if i<20 else 'right'}_arm_joint{i-12 if i<20 else i-19}" for i in range(13, 27)]

class CmdRecorder(Node):
    def __init__(self):
        super().__init__("cmd_recorder")
        logs_dir = Path(__file__).resolve().parents[1] / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.file_path = logs_dir / f"trace_recorder_{ts}.jsonl"
        self.writer = AsyncJsonlWriter(self.file_path)
        self.writer.log({"event": "START", "node": "cmd_recorder"})
        
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=20, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.cmd_sub = self.create_subscription(JointState, "/motion/control/arm_joint_command", self._on_cmd, qos)
        self.state_sub = self.create_subscription(JointState, "/motion/control/arm_joint_state", self._on_state, qos)
        self.pub_count_timer = self.create_timer(1.0, self._check_publishers)
        self.current_pub_count = 0
        self._check_publishers()
        print(f"[cmd_recorder] Logging to {self.file_path}. Press Ctrl-C to stop.")

    def _check_publishers(self):
        try:
            pubs = self.get_publishers_info_by_topic("/motion/control/arm_joint_command")
            self.current_pub_count = len(pubs)
        except Exception: pass

    def _extract_q14(self, msg: JointState):
        name_map = dict(zip(msg.name, msg.position))
        if all(k in name_map for k in ARM_NAMES_LONG): return [name_map[k] for k in ARM_NAMES_LONG]
        elif any("idx13" in k for k in name_map): return [name_map[k] for k in sorted(name_map.keys()) if "idx" in k][:14]
        elif len(msg.position) >= 14: return list(msg.position[:14])
        return []

    def _on_cmd(self, msg: JointState):
        t_recv = time.monotonic()
        q14 = self._extract_q14(msg)
        if len(q14) != 14: return
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.writer.log({
            "type": "CMD", "t_recv": t_recv, "stamp": stamp_sec,
            "q14": [float(round(x, 5)) for x in q14],
            "pub_count": self.current_pub_count
        })

    def _on_state(self, msg: JointState):
        t_recv = time.monotonic()
        q14 = self._extract_q14(msg)
        if len(q14) != 14: return
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.writer.log({
            "type": "STATE", "t_recv": t_recv, "stamp": stamp_sec,
            "q14": [float(round(x, 5)) for x in q14]
        })

    def close(self):
        self.writer.close()

def main():
    rclpy.init()
    node = CmdRecorder()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
        print("\n[cmd_recorder] Flushed and stopped safely.")

if __name__ == "__main__": main()
