"""Core logic and preflight checks for arm_teleop_rx."""
import sys, time, json, urllib.request
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy

ARM_NAMES_LONG = [f"idx{i}_{'left' if i<20 else 'right'}_arm_joint{i-12 if i<20 else i-19}" for i in range(13, 27)]
HAND_NAMES = [f"{s}_{f}" for s in ("left", "right") for f in ("thumb_0", "thumb_1", "index", "middle", "ring", "pinky")]
LIMS_MIN = [-2.91, -0.46, -2.91, -2.00, -2.94, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.10, -2.94, -0.30, -0.20]
LIMS_MAX = [ 2.91,  1.5359, 2.91, -0.10,  2.94,  0.30,  0.20,  2.91,  0.46,   2.91, 2.00,  2.94,  0.30,  0.20]
HOME = [0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0]
SPEED_LEVELS = {1: (0.40, 1.00), 2: (0.60, 1.50), 3: (0.80, 2.00), 4: (1.00, 2.50)}

def rpc_get_action(host="192.168.100.100", port=56322):
    try:
        req = urllib.request.Request(f"http://{host}:{port}/rpc/aimdk.protocol.McActionService/GetAction", data=b"{}", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            data = json.loads(resp.read().decode())
            info = data.get("info", {}) if isinstance(data, dict) else {}
            return info.get("current_action", "") or info.get("current_ext_action", "") or data.get("action", "") or data.get("ext_action", "") or str(data)
    except Exception: return ""

def check_topic_rate_preflight(node: Node, topic: str, duration_sec: float = 1.5) -> float:
    count = [0]
    qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    sub = node.create_subscription(JointState, topic, lambda msg: count.__setitem__(0, count[0] + 1), qos)
    t0 = time.time()
    while time.time() - t0 < duration_sec: rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return count[0] / duration_sec

def check_other_publishers(node: Node, topic: str) -> list[str]:
    return [p.node_name for p in node.get_publishers_info_by_topic(topic) if p.node_name != node.get_name()]

def extract_q14_from_msg(msg: JointState) -> list:
    name_map = dict(zip(msg.name, msg.position))
    if all(k in name_map for k in ARM_NAMES_LONG): return [float(name_map[k]) for k in ARM_NAMES_LONG]
    idx_keys = [k for k in sorted(name_map.keys()) if "idx" in k]
    if len(idx_keys) >= 14: return [float(name_map[k]) for k in idx_keys[:14]]
    if len(msg.position) >= 14: return [float(x) for x in msg.position[:14]]
    return []
