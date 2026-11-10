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
ARM_LOWER_14 = np.array([-2.91, -0.46, -2.91, -2.00, -2.88, -0.30, -0.20, -2.91, -1.5359, -2.91, 0.10, -2.88, -0.30, -0.20], dtype=np.float64)
ARM_UPPER_14 = np.array([2.91, 1.5359, 2.91, -0.10, 2.88, 0.30, 0.20, 2.91, 0.46, 2.91, 2.00, 2.88, 0.30, 0.20], dtype=np.float64)


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
HAND_SAFE_RAW_MAX = 800  # Conservative raw cap for initial real-robot tests [Item A.3]
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
