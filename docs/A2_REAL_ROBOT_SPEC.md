# AGIBOT A2 Ultra Real Robot Hardware & SDK Specification (Sim-to-Real)

> **Document Status**: Extracted strictly from official vendor manuals:
> - `[MANUAL]` = `/home/nguyen/a2_ws/docs/agibot_a2_manual.md` (V1.3 Product Manual, Dec 2025)
> - `[AIMDK]` = `/home/nguyen/a2_ws/docs/aimdk_a2_dev.md` (AimDK-A2 Dev Guide V1.0 / Version 2)

---

## 1. Control Interface
- **SDK / API Name**: Agibot Raise A2-AimDK (`aimdk`), built on top of the AIMA robotic platform `[AIMDK: Sec "Secondary Development Boundary Guidelines", p. 5; Sec "AimDK User Guide", p. 7]`.
- **Language**: Python / C++ (any language supporting HTTP POST and ROS 2 Humble) `[AIMDK: Sec "AimDK User Guide", pp. 7–9]`.
- **Transport**:
  - **High-level / State Machine / Planning**: HTTP JSON RPC requests `[AIMDK: pp. 7–8]`.
  - **Streaming Real-time Control / Feedback**: ROS 2 Topics (`sensor_msgs::msg::JointState`, custom `aimdk::protocol::*`), FastDDS transport `[AIMDK: pp. 8–10]`.
- **IP / Network & Ports**:
  - **x86 Host (Motion Control Board)**: Internal IP `192.168.100.100` (Motion Control HTTP port `56322`, Motion Player `56444`) `[AIMDK: pp. 8, 10, 46]`. External debug port uses DHCP `[AIMDK: p. 10]`.
  - **Orin Board (Perception / Secondary Dev)**: Static IP `192.168.2.50` (internal network `192.168.100.110`) `[AIMDK: p. 10; MANUAL: Sec 6.3.5.2.2, p. 136]`.
  - **VR Teleoperation Ports (Vendor Native)**: VR receiving local port `7033`, A2 robot receiving remote port `56640` `[MANUAL: Sec 6.3.5.2.4, p. 139]`.
  - **DDS Domain ID**: `export ROS_DOMAIN_ID=232`, `export ROS_LOCALHOST_ONLY=0`, `FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml` `[AIMDK: p. 9]`.
  - **QoS Profile**: `history: keep_last`, `depth: 10`, `reliability: best_effort` `[AIMDK: p. 9]`.
- **Authentication**:
  - API level: None (open HTTP/ROS 2 endpoints on robot local subnets) `[AIMDK: p. 8; NOT FOUND in auth headers]`.
  - Terminal SSH: username `agi`, default password `1` `[MANUAL: Sec 6.3.5.2.2, p. 136]`.
- **Installation / Deployment Boundaries**:
  - Program deployment on x86 is strictly prohibited (motion control stability) `[AIMDK: pp. 5, 10]`.
  - Secondary dev code must run in Docker or Python virtual environments on the Orin board or an external dev PC `[AIMDK: pp. 5, 10]`.

---

## 2. Control Modes & Switching Sequences
- **Available Control Modes**:
  - **Arms**:
    - *Joint Servo (Streaming)*: Position control only via `/motion/control/arm_joint_command` `[AIMDK: Sec "Channel Interface", p. 48]`. Torque/impedance control for arms is NOT supported in AimDK V1.0 `[AIMDK: p. 5, 48; MANUAL: p. 143]`.
    - *SE(3) Planning (Trajectory / Waypoint)*: RPC `pb:/aimdk.protocol.McMotionService/PlanningMove` sends target 6-DoF Cartesian pose (Flagship model only) `[AIMDK: pp. 52, 54]`.
  - **Hands**:
    - Coupled position + torque threshold control via `/motion/control/hand_joint_command` or RPC `SetHandCommand` `[AIMDK: pp. 49, 52]`.
- **Mode Switching Sequence**:
  - Mandatory state machine: SetAction is asynchronous; developer must poll `GetAction` (recommended 500ms–1s interval) until completion before commanding `[AIMDK: p. 51]`.
  - *Prerequisite*: Before entering any `JOINT_SERVO` mode, user must stop the onboard motion player (`aima em stop-app motion_player`) to prevent actuator conflicts and motor jitter `[AIMDK: pp. 12, 48]`.
  - *Arm Servo while Robot Stands*: `DEFAULT` -> `RL_JOINT_DEFAULT` -> Lower/stabilize robot on ground -> `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO` `[AIMDK: p. 47]`.
  - *Benchtop / Static Arm Servo*: `DEFAULT` -> `PASSIVE_UPPER_BODY_JOINT_SERVO` `[AIMDK: pp. 47, 58]`.
- **Recommended Command Rate**:
  - Joint Streaming Topics (`arm_joint_command`, `hand_joint_command`): **20 Hz to 100 Hz** `[AIMDK: pp. 11, 12, 48]`.
  - RPC Services (`PlanningMove`, `SetAction`, `SetHandCommand`): **<= 1 Hz** `[AIMDK: p. 51]`.

---

## 3. Joint Names & Mapping Table
- **Real Robot API Joint Ordering**:
  - `arm_joint_command`: 14 `double` values in a single array: first 7 for Left Arm, last 7 for Right Arm `[AIMDK: pp. 48, 67]`.
  - `hand_joint_command`: 12 named finger joints in `sensor_msgs::msg::JointState`: `[left_thumb_0, left_thumb_1, left_index, left_middle, left_ring, left_pinky, right_thumb_0, right_thumb_1, right_index, right_middle, right_ring, right_pinky]` `[AIMDK: p. 49]`.
  - `neck_joint_command`: 2 named joints: `[idx27_head_joint1, idx28_head_joint2]` `[AIMDK: pp. 48, 128]`.

### Arm Joint Mapping Comparison
| Arm Link | Real API (Index / Name) `[AIMDK: pp. 48, 50]` | Manual Diagram `[MANUAL: p. 20]` | Simulation URDF/MJCF Joint Name | Match? |
|---|---|---|---|---|
| Left J1 (Shoulder Pitch) | Index 0 (`Left Arm J1`) | J1 (Shoulder pitch) | `idx13_left_arm_joint1` | Direct index match |
| Left J2 (Shoulder Roll) | Index 1 (`Left Arm J2`) | J2 (Shoulder roll) | `idx14_left_arm_joint2` | Direct index match |
| Left J3 (Shoulder Yaw) | Index 2 (`Left Arm J3`) | J3 (Shoulder yaw) | `idx15_left_arm_joint3` | Direct index match |
| Left J4 (Elbow Pitch) | Index 3 (`Left Arm J4`) | J4 (Elbow pitch) | `idx16_left_arm_joint4` | Direct index match |
| Left J5 (Wrist Roll) | Index 4 (`Left Arm J5`) | J5 (Wrist roll) | `idx17_left_arm_joint5` | Direct index match |
| Left J6 (Wrist Pitch) | Index 5 (`Left Arm J6`) | J6 / Wrist J1 | `idx18_left_arm_joint6` | Direct index match |
| Left J7 (Wrist Yaw) | Index 6 (`Left Arm J7`) | J7 / Wrist J2 | `idx19_left_arm_joint7` | Direct index match |
| Right J1 (Shoulder Pitch) | Index 7 (`Right Arm J1`) | J1 (Shoulder pitch) | `idx20_right_arm_joint1` | Direct index match |
| Right J2 (Shoulder Roll) | Index 8 (`Right Arm J2`) | J2 (Shoulder roll) | `idx21_right_arm_joint2` | Direct index match |
| Right J3 (Shoulder Yaw) | Index 9 (`Right Arm J3`) | J3 (Shoulder yaw) | `idx22_right_arm_joint3` | Direct index match |
| Right J4 (Elbow Pitch) | Index 10 (`Right Arm J4`)| J4 (Elbow pitch) | `idx23_right_arm_joint4` | Direct index match |
| Right J5 (Wrist Roll) | Index 11 (`Right Arm J5`)| J5 (Wrist roll) | `idx24_right_arm_joint5` | Direct index match |
| Right J6 (Wrist Pitch) | Index 12 (`Right Arm J6`)| J6 / Wrist J1 | `idx25_right_arm_joint6` | Direct index match |
| Right J7 (Wrist Yaw) | Index 13 (`Right Arm J7`)| J7 / Wrist J2 | `idx26_right_arm_joint7` | Direct index match |

### Hand Joint Mapping Comparison
| Hand Actuator | Real API Joint Name `[AIMDK: p. 49]` | URDF/MJCF Active Joint Name | Coupled Mimic Joints in URDF | Discrepancy Note |
|---|---|---|---|---|
| Left Thumb Swing | `left_thumb_0` | `L_thumb_swing_joint` | None | Name prefix diff |
| Left Thumb Proximal | `left_thumb_1` | `L_thumb_1_joint` | `L_thumb_2_joint`, `L_thumb_3_joint` | Actuator drives 3 joints |
| Left Index Finger | `left_index` | `L_index_1_joint` | `L_index_2_joint` | Actuator drives 2 joints |
| Left Middle Finger | `left_middle` | `L_middle_1_joint` | `L_middle_2_joint` | Actuator drives 2 joints |
| Left Ring Finger | `left_ring` | `L_ring_1_joint` | `L_ring_2_joint` | Actuator drives 2 joints |
| Left Pinky Finger | `left_pinky` | `L_pinky_1_joint` | `L_pinky_2_joint` | Actuator drives 2 joints |
*(Right hand mirrors left hand with prefix `right_*` vs `R_*`).*

---

## 4. Joint Limits & Dynamic Range Discrepancies
*Comparison between AimDK Protocol `[AIMDK: pp. 50–51]`, Product Manual `[MANUAL: p. 20]`, and MuJoCo/URDF model:*

| Joint | AimDK Min..Max (rad) | AimDK Max Vel (rad/s) | AimDK Max Acc (rad/s²) | Product Manual Workspace | URDF / MJCF Limits (rad) | Discrepancy Alert |
|---|---|---|---|---|---|---|
| Left Arm J1 | -2.91 .. +2.91 | 3.00 | 6.28 | ±170° (-2.967 .. +2.967) | -2.967 .. +2.967 | AimDK software limit is tighter by ~3.2° |
| Left Arm J2 | -0.46 .. +1.60 | 3.00 | 6.28 | -30° ~ 95° (-0.524 .. +1.658) | -0.524 .. +1.658 | AimDK software limit is tighter |
| Left Arm J3 | -2.91 .. +2.91 | 3.00 | 6.28 | ±170° (-2.967 .. +2.967) | -2.967 .. +2.967 | AimDK software limit is tighter |
| Left Arm J4 | -2.00 .. -0.03 | 3.00 | 6.28 | -1° ~ 118° (-2.059 .. -0.017) | -2.094 .. 0.000 | AimDK software limits elbow buffer |
| Left Arm J5 | -2.94 .. +2.94 | 3.00 | 6.28 | ±170° (-2.967 .. +2.967) | -2.880 .. +2.880 | URDF limit is tighter than AimDK |
| Left Arm J6 | -0.45 .. +0.45 | 3.00 | 6.28 | ±45° (-0.785 .. +0.785) | -0.785 .. +0.785 | AimDK software limit is significantly tighter (±25.8° vs ±45°) |
| Left Arm J7 | -0.35 .. +0.35 | 3.00 | 6.28 | ±30° (-0.524 .. +0.524) | -0.524 .. +0.524 | AimDK software limit is tighter (±20.0° vs ±30°) |
| Right Arm J1 | -2.91 .. +2.91 | 3.00 | 6.28 | ±170° (-2.967 .. +2.967) | -2.967 .. +2.967 | AimDK tighter by ~3.2° |
| Right Arm J2 | -1.60 .. +0.46 | 3.00 | 6.28 | -95° ~ 30° (-1.658 .. +0.524) | -1.658 .. +0.524 | AimDK tighter |
| Right Arm J3 | -2.91 .. +2.91 | 3.00 | 6.28 | ±170° (-2.967 .. +2.967) | -2.967 .. +2.967 | AimDK tighter |
| Right Arm J4 | +0.03 .. +2.00 | 3.00 | 6.28 | 1° ~ 118° (+0.017 .. +2.059) | 0.000 .. +2.094 | AimDK software limits elbow buffer |
| Right Arm J5 | -2.94 .. +2.94 | 3.00 | 6.28 | ±170° (-2.967 .. +2.967) | -2.880 .. +2.880 | URDF tighter than AimDK |
| Right Arm J6 | -0.45 .. +0.45 | 3.00 | 6.28 | ±45° (-0.785 .. +0.785) | -0.785 .. +0.785 | AimDK software limit tighter (±25.8°) |
| Right Arm J7 | -0.35 .. +0.35 | 3.00 | 6.28 | ±30° (-0.524 .. +0.524) | -0.524 .. +0.524 | AimDK software limit tighter (±20.0°) |

> [!IMPORTANT]
> Real firmware enforces **Max Velocity = 3.00 rad/s** and **Max Acceleration = 6.28 rad/s²** across all arm joints. Wrist pitch/yaw software limits are strictly clipped to ±0.45 rad and ±0.35 rad. Real-world trajectory smoothing must clamp to these AimDK values rather than raw URDF limits.

---

## 5. Hand Control & Under-Actuation Architecture
- **Command Units**:
  - `position` field: Integer raw value in range **`0` (fully open) to `2000` (fully closed)** `[AIMDK: p. 49]`.
  - `effort` field: Torque threshold in range **`0` to `5700`** (5700 corresponds to ~57 N output force) `[AIMDK: p. 49]`.
- **Control Strategy**:
  - Dual threshold logic: motion stops as soon as either `position` or `effort` limit is reached `[AIMDK: p. 49]`.
- **Degrees of Freedom & Coupled Joints**:
  - **6 active actuators per hand** (12 total across robot) `[AIMDK: p. 49; MANUAL: p. 18]`.
  - Coupled / mimic joints: 4-bar linkage mechanism drives proximal, intermediate, and distal phalanges automatically in hardware `[AIMDK: p. 50]`. The user commands only the 6 active joints; mimic joints must NOT be commanded over the API.
- **Hand Feedback**:
  - Topic `/motion/control/hand_joint_state` publishes at **100 Hz** with fields `position` (0–2000) and `effort` (0–5700) `[AIMDK: p. 50]`. Negative effort values (down to -1000) are normal due to spring/linkage returns `[AIMDK: p. 50]`.

---

## 6. State Feedback & Telemetry
- **Available Topics & Rates**:
  - **Arm Joint State** (`/motion/control/arm_joint_state`): **100 Hz**. Fields: `position`, `velocity`, `effort` for joints 1–5; joints 6–7 report equivalent series `position` only `[AIMDK: p. 49]`.
  - **Hand Joint State** (`/motion/control/hand_joint_state`): **100 Hz**. Fields: `position` (0–2000), `effort` `[AIMDK: p. 50]`.
  - **Head Joint State** (`/motion/control/neck_joint_state`): **100 Hz**. Fields: `position`, `velocity`, `effort` `[AIMDK: pp. 49–50]`.
  - **Exception & Alarm Stream** (`/aima/hds/exception`): Event-driven / periodic exception list `[AIMDK: p. 78]`.
- **Timestamps**: ROS 2 message headers contain UNIX epoch seconds and nanoseconds (`aimdk::protocol::Timestamp`) `[AIMDK: pp. 31, 54, 78, 119]`. Hardware vs kernel timestamp origin is handled internally by AIMA `[AIMDK: p. 119]`.

---

## 7. Safety, E-Stop, Watchdogs & Fault Handling
- **Emergency Stop (Hardware & Software)**:
  - *Hardware E-Stop*: Dedicated wireless emergency stop button unit paired one-to-one by SN `[MANUAL: Sec 4.4, pp. 30–31]`. Pressing de-energizes all joints into a **damped collapse state**. Releasing (rotate clockwise) triggers automatic upper limb and head self-test `[MANUAL: p. 31]`.
  - *Software E-Stop*: Available via RPC `GetBmsState` / system status or AimMaster `[AIMDK: p. 10; MANUAL: p. 128]`.
- **Watchdog / Communication Timeout**:
  - RPC requests support `-H 'timeout: 1000'` (ms) header `[AIMDK: p. 8]`.
  - Streaming watchdog: NOT FOUND explicit timeout duration in seconds for ROS 2 channels in docs; commands must be streamed continuously at >= 20 Hz `[AIMDK: p. 11]`.
- **Collision & Overload Protection**:
  - In teleoperation mode, arms do **NOT** have active force control, and software collision detection is **disabled by default** `[MANUAL: Sec 6.3.6.2.3, p. 143]`.
  - Collision escape mode exists: `McAction_RL_LOCOMOTION_ARM_EXT_COLLISON_ESCAPE` (404) or `McAction_PASSIVE_UPPER_BODY_COLLISON_ESCAPE` (3) `[AIMDK: pp. 58–59]`.
- **Payload Limits**:
  - Single arm rated payload (including hand): **1.0 kg** `[MANUAL: Sec 3.4, p. 20]`.
  - Dexterous hand gripping payload (palm down): **< 3.0 kg** `[MANUAL: p. 20]`.
  - Dexterous hand hooking payload (palm up): **< 8.0 kg** `[MANUAL: p. 20]`.
  - Max arm end-effector linear speed: **1.0 m/s** `[MANUAL: p. 20]`.

---

## 8. Posture, Locomotion & Startup/Shutdown
- **Standing & Balance Modes**:
  - Traditional bent-leg walking is removed/deprecated; only reinforcement-learning straight-leg standing/locomotion (`RL_LOCOMOTION_*`) is supported in V1.0 `[AIMDK: p. 46]`.
- **Simultaneous Upper-Body & Locomotion**:
  - **Fully supported**: In mode `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO` (Action 402), the robot can accept continuous lower-limb velocity commands (`/motion/control/locomotion_velocity` at 20–100 Hz) while concurrently accepting arm joint streaming (`/motion/control/arm_joint_command` at 20–100 Hz) `[AIMDK: pp. 47, 48, 59]`.
- **Vendor Teleoperation Mode**:
  - Agibot provides a pre-packaged VR teleoperation pipeline using PICO 4 Ultra Enterprise headsets and the `TeleXperience` app running over UDP socket ports 7033 / 56640 `[MANUAL: Sec 6.3.3–6.3.5, pp. 129–139]`.
- **Safe Transition Sequence**:
  1. Boot robot and verify emergency stop device green link `[MANUAL: p. 31]`.
  2. Put robot on crane / transfer lift sling `[MANUAL: p. 140]`.
  3. Action `DEFAULT` -> `RL_JOINT_DEFAULT` (position standing).
  4. Lower robot to firm ground until feet contact floor securely `[MANUAL: p. 140]`.
  5. Action `RL_JOINT_DEFAULT` -> `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO` (Action 402).
  6. Stop `motion_player` process: `aima em stop-app motion_player` `[AIMDK: p. 12]`.
  7. Start arm and hand joint command streams `[AIMDK: pp. 12, 48]`.
  8. *Shutdown*: Switch back to `RL_JOINT_DEFAULT` before lifting or powering down to prevent leg spasms `[MANUAL: pp. 140, 143]`.

---

## 9. Relevant Ready-Made Examples in AimDK
Located in the AimDK repository under `examples/mc/` `[AIMDK: Sec "Motion Control Example Scripts", p. 67]`:
- `examples/mc/http/SetAction.py`: Sets Action state machine via RPC `[AIMDK: p. 67]`.
- `examples/mc/http/planning_move.py`: SE(3) Cartesian planning for arms `[AIMDK: pp. 12, 54, 67]`.
- `examples/mc/http/set_hand_command.py`: Controls hand gestures via RPC `[AIMDK: pp. 13, 67]`.
- `examples/mc/ros2/arm.py`: 14-DoF passthrough joint streaming (20–100 Hz) `[AIMDK: pp. 12, 48, 67]`.
- `examples/mc/ros2/hand.py`: 12-DoF dexterous hand streaming `[AIMDK: pp. 13, 49, 67]`.
- `examples/mc/ros2/walk.py`: Locomotion velocity streaming `[AIMDK: pp. 11, 48, 67]`.
- `examples/mc/ros2/joint_state.py`: Subscribes to 100 Hz arm/hand telemetry `[AIMDK: p. 67]`.

---

## 10. Contradictions & Ambiguities Between Docs
1. **Arm Workspace Limits**:
   - `[MANUAL: Sec 3.4, p. 20]` lists physical arm ranges: Elbow J4 `-1° to 118°` (-2.059 to -0.017 rad), Wrist Pitch J6 `±45°` (±0.785 rad), Wrist Yaw J7 `±30°` (±0.524 rad).
   - `[AIMDK: Sec "Upper Limb Joint Limits", pp. 50–51]` restricts software motion limits to: Elbow J4 `-2.00 to -0.03 rad`, Wrist Pitch J6 `±0.45 rad` (±25.8°), Wrist Yaw J7 `±0.35 rad` (±20.0°).
   - *Ambiguity*: AimDK cuts off ~42% of wrist pitch range and ~33% of wrist yaw range.
2. **Wrist Joint Velocity and Effort Feedback**:
   - `[AIMDK: Sec "Arm Joint State", p. 49]` states that for Wrist Pitch J6 and Wrist Yaw J7, only the `position` field is valid (series equivalent), while `velocity` and `effort` are invalid.
   - However, `[AIMDK: Protobuf Definition, p. 31]` marks velocity and effort as standard fields across all joints.
3. **Teleoperation Headset Dependency**:
   - `[MANUAL: Sec 6.3.3, p. 129]` specifies hardware requirement strictly as PICO 4 Ultra Enterprise.
   - `[AIMDK: p. 48]` shows standard ROS 2 topic `/motion/control/arm_joint_command`, meaning any WebXR device (including Meta Quest 3S via our teleop bridge) can command the robot directly over ROS 2 without PICO software.

---

## Gaps to Ask AGIBOT
1. **Watchdog Failsafe Behavior**: If ROS 2 command stream to `/motion/control/arm_joint_command` drops while in `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO`, does the controller hold the last received joint angles, damp, or freeze? What is the exact timeout (e.g., 100 ms, 500 ms)?
2. **Hand Calibration & Zeroing**: When sending raw position `0` to `2000`, does the robot auto-zero tendon encoders at power-on, or is there an explicit homing procedure required for the dexterous hands?
3. **Wrist Differential Mechanism**: The URDF models wrists as serial joints `idx18` and `idx19`, but physical wrists are parallel 2-DoF pushrod linkages (`left_wrist_rod_A/B`). Does the x86 controller's `/motion/control/arm_joint_command` accept serial equivalent angles, or motor pushrod actuator angles? (AimDK p. 49 mentions equivalent series angles for feedback, but does not explicitly specify command expectations).
4. **Collision Detection Enablement in Teleop**: Is there any API parameter to enable protective software self-collision stopping in `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO` mode without switching to `COLLISON_ESCAPE`?
5. **Exact Payload with Extended Reach**: Rated payload is 1 kg single arm / 3 kg grip. Is 1 kg permitted across the entire workspace envelope up to 0.61 m extension?
EOF
