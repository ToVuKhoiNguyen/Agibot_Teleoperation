# AGIBOT A2 Ultra Teleoperation - Vendor Documentation Cross-Check Report

This document records the cross-check between AGIBOT official vendor documentation (`docs/vendor/aimdk_a2_dev.pdf`, `docs/vendor/agibot_a2_manual.pdf`) and our repository code (`teleop/robot/{aimdk_map,ros2_backend,safety_layer}.py`, `tools/aima_mode.py`, `tools/preflight.py`).

> **Safety Notice:** All findings regarding physical robot behavior are marked **unverified** on real hardware. Zero hardware connections or external network packets were transmitted during this cross-check.

---

## 1. Cross-Check Invariant Comparison Table

| Item | Vendor Value (Citing Doc Section) | Our Implementation Value | Match? |
| :--- | :--- | :--- | :--- |
| **14 Arm Joint Names & Order** | Table 7.1.9 & Sec 7.1.1 (p. 48, lines 4846-4849): 14 joints, first 7 Left Arm (`J1`..`J7`), last 7 Right Arm (`J1`..`J7`). URDF equivalents: `idx13_left_arm_joint1` .. `idx26_right_arm_joint7` | `ARM_JOINT_NAMES_AIMDK` (`Left Arm J1`..`Right Arm J7`) and `ARM_JOINT_NAMES_URDF` (`idx13`..`idx26`) in `teleop/robot/aimdk_map.py` | **MATCH** |
| **Wrist Pitch/Yaw Limits (Joints 6 & 7)** | Table 7.1.9 (pp. 50-51) lists physical $\pm 0.45$ rad (J6) & $\pm 0.35$ rad (J7). Safe envelope / yellow-region bound: $\pm 0.30$ rad (J6) & $\pm 0.20$ rad (J7). | `ARM_LOWER_14` / `ARM_UPPER_14` in `safety_layer.py`: indices 5 & 12 clamped to $[-0.30, 0.30]$; indices 6 & 13 clamped to $[-0.20, 0.20]$. | **MATCH** (Tightened) |
| **Arm Asymmetric Physical Limits** | Table 7.1.9 & Yellow region intersection: L2 $[-0.46, 1.5359]$, L4 $[-2.00, -0.10]$, R2 $[-1.5359, 0.46]$, R3 $[-2.91, 2.91]$, R4 $[0.10, 2.00]$, R7 $[-0.20, 0.20]$ | `ARM_LOWER_14` & `ARM_UPPER_14` in `safety_layer.py`: exactly enforce intersection bounds | **MATCH** (Tightened) |
| **Max Velocity Ceiling** | Table 7.1.9 (p. 50, "Upper Limb Joint Limits"): absolute max velocity $3.0\text{ rad/s}$ | `SafetyLayer.max_velocity` hard ceiling $3.0\text{ rad/s}$ (`min(val, 3.0)`); default safe test cap $1.0\text{ rad/s}$ | **MATCH** |
| **Max Acceleration Ceiling** | Table 7.1.9 (p. 50, "Upper Limb Joint Limits"): absolute max acceleration $6.28\text{ rad/s}^2$ ($2\pi$) | `SafetyLayer.max_accel` capped at $3.0\text{ rad/s}^2$ default (ceiling $6.28\text{ rad/s}^2$) | **MATCH** (Tightened) |
| **Publish Interval / Rate Guard** | Section 7.1 (p. 48 & FAQ p. 117): continuous joint commands supported $20\text{--}100\text{ Hz}$ (interval $\le 30\text{ ms}$, target $50\text{--}100\text{ Hz}$) | `Ros2Backend._publish_loop` runs at fixed $50\text{ Hz}$ ($20\text{ ms} \le 30\text{ ms}$). Watchdog trips if gap $> 200\text{ ms}$ or state stale. | **MATCH** |
| **Arm State Parsing Tolerance** | Section 7.1 (p. 50, `arm_joint_state`): first 5 joints provide position, velocity, effort; wrist joints 6 & 7 provide **position only** (velocity/effort may be empty or omitted) | `align_joint_state_positions` in `aimdk_map.py` parses 14 positions directly without requiring velocity/effort arrays of equal length | **MATCH** |
| **Hand Message `frame_id`** | Section 7.1 (p. 49, `hand_joint_command` & `hand_joint_state`): dexterous hand frame identifier required as `"AgiHand"` | `HAND_FRAME_ID = "AgiHand"` defined in `aimdk_map.py`; outgoing command sets `header.frame_id = "AgiHand"`; incoming state drops non-matching frames | **MATCH** |
| **Hand Position & Effort Range** | Section 7.1 (p. 49): position $[0..2000]$ ($0=\text{open}$, $2000=\text{closed}$); effort $[0..5700]$ ($5700 \approx 57\text{ N}$) | `HAND_POS_RAW_MIN=0`, `HAND_POS_RAW_MAX=2000`; `HAND_EFFORT_RAW_MIN=0`, `HAND_EFFORT_RAW_MAX=5700`; clamped in `ros2_backend.py` | **MATCH** |
| **SetAction Payload & Command** | Section 7.2 (pp. 58-60, lines 5860-5930, 5990-6009): `aimdk.protocol.McActionRequest` contains `action` (`402` / `McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO`) and `ext_action` name string | `RPC_SET_ACTION_402_PAYLOAD` in `aimdk_map.py` & `tools/aima_mode.py` sends `{"action": 402, "ext_action": "McAction_RL_LOCOMOTION_ARM_EXT_JOINT_SERVO"}` | **MATCH** |
| **RPC Host & Port Configuration** | Section 7.2 (p. 54, line 5474): Motion RPC service default `192.168.100.100:56322` | `A2_DEFAULT_IP = "192.168.100.100"`, `A2_MOTION_PORT = 56322`; configurable via `--ip`, `--port`, or `AGIBOT_RPC_HOST` / `AGIBOT_RPC_PORT` | **MATCH** |

---

## 2. Invariant Safety Enforcements Implemented

1. **Safety Envelope Joint Limits**:
   Clamped wrist and asymmetric arm joint limits strictly in the conservative tightening direction to prevent mechanical bind or collision:
   - Left Arm J6 / Right Arm J6 (pitch): $[-0.30, 0.30]\text{ rad}$
   - Left Arm J7 / Right Arm J7 (yaw): $[-0.20, 0.20]\text{ rad}$
   - Left Arm J2: $[-0.46, 1.5359]\text{ rad}$, Left Arm J4: $[-2.00, -0.10]\text{ rad}$
   - Right Arm J2: $[-1.5359, 0.46]\text{ rad}$, Right Arm J4: $[0.10, 2.00]\text{ rad}$

2. **Hand Telemetry & Command Frame Safety**:
   - `header.frame_id = "AgiHand"` enforced on all published hand control packets.
   - Hand telemetry subscriber rejects messages if `frame_id` is present and does not equal `"AgiHand"`.
   - Hand effort clamped to $[0..5700]$ with safe test default $1140$ ($20\%$).

3. **Tolerant State Parsing**:
   - Ensured `align_joint_state_positions` safely indexes 14 positions even when vendor state stream omits velocity or effort for wrist joints 6 and 7.

4. **Zero-Network Invariant**:
   - All tests run on loopback ROS 2 domain 77 with `ROS_LOCALHOST_ONLY=1`.
   - Zero packets were transmitted to robot IP `192.168.100.100` or ROS domain `232`.

