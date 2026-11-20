"""
teleop/robot/aimdk_map.py
Constants and mapping specifications extracted verbatim from Agibot A2 AimDK documentation.
Citations reference docs/vendor/aimdk_a2_dev.pdf [AIMDK] and docs/A2_REAL_ROBOT_SPEC.md.

SPEC CROSS-CHECK STATUS (Verified 2026-11-05):
- 14 arm joint names+order: MATCH (7 left J1..J7, 7 right J1..J7; AIMDK pp. 48, 50-51)
- 12 hand joint names+order: MATCH (AIMDK p. 49, lines 4938-4947)
- Open/closed ends 0..2000: MATCH for fingers (0=open, 2000=closed; AIMDK p. 49)
  Thumb swing polarity: UNKNOWN / unstated in spec (TODO_THUMB_SWING_SIGN_CALIBRATION)
- Effort scale: MATCH (0..5700, 5700 == ~57 N force; AIMDK p. 49)
- Topic names: MATCH (/motion/control/{arm,hand}_joint_{command,state}; AIMDK pp. 48-50)
- Message types: MATCH (sensor_msgs/msg/JointState for all 4 topics; AIMDK pp. 48-50)
- JointState alignment: MATCH (positional order standard; align helper maps by name when present)
"""

import numpy as np

# -----------------------------------------------------------------------------
# 1. ROS 2 TOPICS & MESSAGE TYPES
# -----------------------------------------------------------------------------
# [AIMDK: p. 48, lines 4832-4835]
ARM_JOINT_COMMAND_TOPIC = "/motion/control/arm_joint_command"
ARM_JOINT_COMMAND_MSG_TYPE = "sensor_msgs/msg/JointState"

# [AIMDK: p. 49, lines 4927-4930]
HAND_JOINT_COMMAND_TOPIC = "/motion/control/hand_joint_command"
HAND_JOINT_COMMAND_MSG_TYPE = "sensor_msgs/msg/JointState"

# [AIMDK: p. 49, lines 4984-4990]
ARM_JOINT_STATE_TOPIC = "/motion/control/arm_joint_state"
ARM_JOINT_STATE_MSG_TYPE = "sensor_msgs/msg/JointState"

# [AIMDK: p. 50, lines 5040-5046]
HAND_JOINT_STATE_TOPIC = "/motion/control/hand_joint_state"
HAND_JOINT_STATE_MSG_TYPE = "sensor_msgs/msg/JointState"


# -----------------------------------------------------------------------------
# 2. 14 ARM JOINT NAMES & ORDER
# -----------------------------------------------------------------------------
# [AIMDK: p. 48, lines 4846-4849: first 7 for left arm, last 7 for right arm]
# [AIMDK: pp. 50-51, lines 5083-5160 & p. 117: Table of limits]
ARM_JOINT_NAMES_AIMDK = [
    "Left Arm J1", "Left Arm J2", "Left Arm J3", "Left Arm J4",
    "Left Arm J5", "Left Arm J6", "Left Arm J7",
    "Right Arm J1", "Right Arm J2", "Right Arm J3", "Right Arm J4",
    "Right Arm J5", "Right Arm J6", "Right Arm J7",
]

# Simulation URDF joint name correspondences [SPEC: p. 55, lines 57-70]
ARM_JOINT_NAMES_URDF = [
    "idx13_left_arm_joint1", "idx14_left_arm_joint2", "idx15_left_arm_joint3",
    "idx16_left_arm_joint4", "idx17_left_arm_joint5", "idx18_left_arm_joint6",
    "idx19_left_arm_joint7", "idx20_right_arm_joint1", "idx21_right_arm_joint2",
    "idx22_right_arm_joint3", "idx23_right_arm_joint4", "idx24_right_arm_joint5",
    "idx25_right_arm_joint6", "idx26_right_arm_joint7",
]
ARM_JOINT_COUNT = 14
ARM_HOME_14 = [0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0]
HOME = ARM_HOME_14
# Elbow J4 bound = AimDK software limit +-0.03 since 17/11 (was +-0.10): the robot itself stands with the elbows at
# +-0.035 after SetAction, so +-0.10 made the first command after engage bend the elbow 0.065 rad on its own.
ARM_LOWER_14 = np.array([-2.91, -0.46, -2.91, -2.00, -2.88, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.03, -2.88, -0.30, -0.20], dtype=np.float64)
ARM_UPPER_14 = np.array([2.91, 1.5359, 2.91, -0.03, 2.88, 0.30, 0.20, 2.91, 0.46, 2.91, 2.00, 2.88, 0.30, 0.20], dtype=np.float64)
# AimDK software limits (vendor, pp. 50-51; docs/A2_REAL_ROBOT_SPEC.md): the outer bound for start_limits()
VENDOR_LOWER_14 = np.array([-2.91, -0.46, -2.91, -2.00, -2.94, -0.45, -0.35, -2.91, -1.60, -2.91, 0.03, -2.94, -0.45, -0.35], dtype=np.float64)
VENDOR_UPPER_14 = np.array([2.91, 1.60, 2.91, -0.03, 2.94, 0.45, 0.35, 2.91, 0.46, 2.91, 2.00, 2.94, 0.45, 0.35], dtype=np.float64)


def start_limits(q14) -> tuple:
    """Joint bounds for a session that starts at the measured pose q14: our bounds, widened (never past the vendor
    limits) to include q14. A joint that stands outside our tighter bounds at SYNC is therefore NOT pulled in by the
    first command - the robot stays exactly where it is until the operator moves it."""
    q = np.clip(np.asarray(q14, dtype=np.float64), VENDOR_LOWER_14, VENDOR_UPPER_14)
    return np.minimum(ARM_LOWER_14, q), np.maximum(ARM_UPPER_14, q)


# -----------------------------------------------------------------------------
# 3. 12 HAND JOINT NAMES & ORDER
# -----------------------------------------------------------------------------
# [AIMDK: p. 49, lines 4935-4947: JointState includes 12 joints for left and right hands]
HAND_JOINT_NAMES = [
    "left_thumb_0", "left_thumb_1", "left_index",
    "left_middle", "left_ring", "left_pinky",
    "right_thumb_0", "right_thumb_1", "right_index",
    "right_middle", "right_ring", "right_pinky",
]
HAND_JOINT_COUNT = 12

SIM_HAND_JOINT_NAMES = [
    "L_thumb_swing_joint", "L_thumb_1_joint", "L_index_1_joint",
    "L_middle_1_joint", "L_ring_1_joint", "L_pinky_1_joint",
    "R_thumb_swing_joint", "R_thumb_1_joint", "R_index_1_joint",
    "R_middle_1_joint", "R_ring_1_joint", "R_pinky_1_joint",
]

# Comfortable flexion limits (rad) [AgibotHandRetargeting.COMFORTABLE_LIMITS]
# thumb_swing=1.60, thumb_1=0.35, fingers=1.30
SIM_HAND_COMFORTABLE_LIMITS = np.array([
    1.60, 0.35, 1.30, 1.30, 1.30, 1.30,
    1.60, 0.35, 1.30, 1.30, 1.30, 1.30,
], dtype=np.float64)


# -----------------------------------------------------------------------------
# 4. HAND VALUE RANGE, EFFORT & CONVERSION FUNCTIONS
# -----------------------------------------------------------------------------
# [AIMDK: p. 49, lines 4948-4967]
HAND_POS_RAW_MIN = 0
HAND_POS_RAW_MAX = 2000
HAND_SAFE_RAW_MAX = 2000  # full vendor travel (was 800 = 40 % for the first real-robot tests; --hand-raw-max lowers it)
THUMB_REST_RAW = 500      # default thumb pose: both thumb joints (thumb_0, thumb_1) folded this far when the hand is open
THUMB_IDX_12 = (0, 1, 6, 7)
HAND_FRAME_ID = "AgiHand"  # Hand message frame_id required by vendor spec
HAND_EFFORT_RAW_MIN = 0
HAND_EFFORT_RAW_MAX = 5700
HAND_EFFORT_DEFAULT = 1140  # 20% of 5700 as safe default torque limit
HAND_NORM_TO_RAW_SCALE = 2000.0
HAND_NORM_TO_RAW_OFFSET = 0.0


def sim_hand_to_robot_raw(
    sim_h12: np.ndarray,
    raw_max: int = HAND_POS_RAW_MAX,
    enable_thumb_swing: bool = True,
) -> np.ndarray:
    """Maps 12 sim hand joint angles (rad) to robot raw positions [0..raw_max]."""
    angles = np.asarray(sim_h12, dtype=np.float64)
    norm = np.clip(angles / SIM_HAND_COMFORTABLE_LIMITS, 0.0, 1.0)
    raw = np.round(norm * 2000.0).astype(int)
    clamped_raw = np.clip(raw, HAND_POS_RAW_MIN, raw_max)
    if not enable_thumb_swing:
        clamped_raw[0] = 0  # Left thumb swing held neutral
        clamped_raw[6] = 0  # Right thumb swing held neutral
    return clamped_raw


def apply_thumb_rest(q6: np.ndarray, rest_raw: float = THUMB_REST_RAW) -> np.ndarray:
    """Default (open-hand) pose with the thumb slightly folded: remaps the two thumb joints of one hand (rad) from
    [0 .. limit] onto [rest .. limit], so closure 0 % rests at rest_raw / 2000 of the travel and 100 % is unchanged
    in proportion. Continuous: no dead zone at the start of the closure."""
    out = np.array(q6, dtype=np.float64)
    f = float(np.clip(rest_raw, 0.0, HAND_POS_RAW_MAX)) / HAND_POS_RAW_MAX
    if f > 0.0: out[:2] = f * SIM_HAND_COMFORTABLE_LIMITS[:2] + (1.0 - f) * out[:2]
    return out


THUMB_POSE_RAW = (800, 0, 1750, 1760)   # thumb (swing, flex) raw 0..2000 with hand OPEN, then with it CLOSED
# Grasp-ready default: swing 800 (open outward by 800 raw, open aperture ~92 mm to index and ~91 mm to middle finger).
# When closing, folds to (1750, 1760) meeting the index finger tip squarely (~0.2-2.2 mm distance) without folding into middle finger.


def parse_thumb_pose(text):
    """'1700,0' (hand open; closed = default closed pose) or '1700,0,1700,1500' (open, closed) -> 4-tuple;
    'off' / '' / 'none' -> None (thumb follows the hand from fully open, rest --thumb-rest)."""
    t = str(text or "").strip().lower()
    if t in ("", "off", "none", "follow"): return None
    v = [max(0, min(2000, int(round(float(x))))) for x in t.replace(";", ",").split(",") if x.strip()]
    if len(v) == 2: v += [max(v[0], THUMB_POSE_RAW[2]), max(v[1], THUMB_POSE_RAW[3])]
    if len(v) != 4: raise ValueError(f"--thumb-pose needs 2 or 4 numbers, got {text!r}")
    return tuple(v)


def apply_thumb_pose(q6: np.ndarray, pose_raw=THUMB_POSE_RAW) -> np.ndarray:
    """Thumb resting in a grasp pose (17/11, operator request) and still closing with the hand: the two thumb joints of
    the RAW retargeting output (0 = open .. limit = closed) are remapped onto [open pose .. closed pose] (pose_raw =
    swing, flex open, swing, flex closed). Hand open -> thumb at the open pose (opposed, ready to grasp); closing the
    hand folds it to the closed pose together with the fingers; opening brings it back. Continuous, no dead zone. Use on the raw output INSTEAD of apply_thumb_rest.
    None = no-op (thumb follows the hand from fully open)."""
    out = np.array(q6, dtype=np.float64)
    if pose_raw is None: return out
    lim = SIM_HAND_COMFORTABLE_LIMITS[:2]
    p = list(pose_raw) + ([] if len(pose_raw) >= 4 else list(THUMB_POSE_RAW[2:]))
    to_rad = lambda r: np.clip(np.asarray(r, dtype=np.float64), 0.0, HAND_POS_RAW_MAX) / HAND_POS_RAW_MAX * lim
    p_open, p_closed = to_rad(p[:2]), to_rad(p[2:4])
    f = float(np.clip(max(out[1] / lim[1], out[2] / (0.60 * 1.70)), 0.0, 1.0)) if (len(out) > 2 and out[2] > 0) else float(np.clip(out[1] / lim[1], 0.0, 1.0))
    out[:2] = p_open + (p_closed - p_open) * f
    return out


def hand_thumb(q6: np.ndarray, rest_raw: float = THUMB_REST_RAW, pose_raw=THUMB_POSE_RAW) -> np.ndarray:
    """Thumb shaping of a raw retargeting output: grasp pose (--thumb-pose, default) or the old rest (--thumb-pose off)."""
    return apply_thumb_pose(q6, pose_raw) if pose_raw is not None else apply_thumb_rest(q6, rest_raw)


def robot_raw_to_sim_hand(raw12: np.ndarray) -> np.ndarray:
    """Maps 12 robot raw positions [0..2000] to sim hand joint angles (rad)."""
    raw = np.clip(np.asarray(raw12, dtype=np.float64), 0.0, 2000.0)
    return (raw / 2000.0) * SIM_HAND_COMFORTABLE_LIMITS


def sim_hand_to_norm(sim_h12: np.ndarray) -> np.ndarray:
    """Maps 12 sim hand joint angles (rad) to normalized [0.0..1.0]."""
    angles = np.asarray(sim_h12, dtype=np.float64)
    return np.clip(angles / SIM_HAND_COMFORTABLE_LIMITS, 0.0, 1.0)


def norm_to_sim_hand(norm12: np.ndarray) -> np.ndarray:
    """Maps 12 normalized [0.0..1.0] to sim hand joint angles (rad)."""
    norm = np.clip(np.asarray(norm12, dtype=np.float64), 0.0, 1.0)
    return norm * SIM_HAND_COMFORTABLE_LIMITS


def align_joint_state_positions(msg, expected_names: list[str], expected_len: int) -> np.ndarray:
    """Aligns JointState positions by name if names exist; else slices by expected_len."""
    if hasattr(msg, "name") and msg.name and len(msg.position) >= len(msg.name):
        n2p = dict(zip(msg.name, msg.position))
        if all(k in n2p for k in expected_names):
            return np.array([n2p[k] for k in expected_names], dtype=np.float64)
    return np.array(msg.position[:expected_len], dtype=np.float64)


# -----------------------------------------------------------------------------
# 5. RPC ENDPOINTS & FORMATS
# -----------------------------------------------------------------------------
A2_DEFAULT_IP = "192.168.100.100"       # [AIMDK: pp. 8, 25, 48, 68]
A2_MOTION_PORT = 56322                  # [AIMDK: p. 54, line 5474]
A2_MOTION_PLAYER_PORT = 56444           # [AIMDK: pp. 25, 68, 114, line 12884]

ACTION_ID_SERVO_402 = 402
ACTION_NAME_SERVO_402 = "McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO"
ACTION_ID_STAND_DEFAULT = 400           # McAction_RL_JOINT_DEFAULT [AIMDK: p. 59, line 5863]

RPC_SET_ACTION_ENDPOINT = f"http://{A2_DEFAULT_IP}:{A2_MOTION_PORT}/rpc/aimdk.protocol.McActionService/SetAction"
RPC_GET_ACTION_ENDPOINT = f"http://{A2_DEFAULT_IP}:{A2_MOTION_PORT}/rpc/aimdk.protocol.McActionService/GetAction"

RPC_SET_ACTION_402_PAYLOAD = {
    "header": {"timestamp": {"seconds": 0, "nanos": 0}},
    "command": {"action": ACTION_ID_SERVO_402, "ext_action": ACTION_NAME_SERVO_402},
}

RPC_STOP_MOTION_PLAYER_ENDPOINT = f"http://{A2_DEFAULT_IP}:{A2_MOTION_PLAYER_PORT}/rpc/aimdk.protocol.MotionPlayerService/SendMotionCommand"
RPC_STOP_MOTION_PLAYER_PAYLOAD = {"motion_id": "stop", "cmd_pause": True, "cmd_reset": False}
CMD_AIMA_STOP_MOTION_PLAYER = "aima em stop-app motion_player"


# -----------------------------------------------------------------------------
# 6. UNVERIFIED / NOT FOUND IN DOCUMENTATION (TODO CONSTANTS)
# -----------------------------------------------------------------------------
# [AIMDK: p. 49 only provides general 0..2000 rule; per-joint swing sign/offset not specified]
TODO_THUMB_SWING_SIGN_CALIBRATION = None

# [MANUAL: Sec 4.4, pp. 30-31 states hardware e-stop is a dedicated wireless unit; no ROS topic exists]
TODO_ROS2_ESTOP_TOPIC = None

# [AIMDK: p. 11 & SPEC line 140 state commands must be streamed >= 20 Hz; exact trip duration unstated]
TODO_HARDWARE_WATCHDOG_TIMEOUT_S = None

# [AIMDK: p. 51 states GetAction must be polled; previous action ID must be queried dynamically at runtime]
TODO_RESTORE_ACTION_PREVIOUS_ACTION_ID = None
