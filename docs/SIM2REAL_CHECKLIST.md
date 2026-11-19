# AGIBOT A2 Ultra Sim-to-Real Transition Checklist

> **Tracking status for transitioning from MuJoCo simulation to physical AGIBOT A2 Ultra hardware.**
> Based on analysis of `agibot_a2_manual.md` and `aimdk_a2_dev.md` (see details in `A2_REAL_ROBOT_SPEC.md`).

## 1. Network & Communication Architecture
- [x] **SDK / Middleware Identified**: AimDK V1.0 over ROS 2 Humble (`ROS_DOMAIN_ID=232`, FastDDS) and HTTP JSON RPC (`56322`, `56444`).
- [x] **Host IP Configuration Known**: x86 Motion Control (`192.168.100.100`), Orin Secondary Dev (`192.168.2.50` / `192.168.100.110`).
- [x] **Deployment Boundaries Understood**: No custom binaries allowed on x86; deploy teleop ROS 2 bridge on Orin or external PC.
- [x] **Physical Network Ping Verification**: Connect Ethernet cable to Orin debug port and verify ping `192.168.2.50` (laptop reaches the Orin only; it has no route to the X86 `192.168.100.100`, so arm commands go laptop -> UDP -> Orin receiver -> ROS 2).

## 2. Joint & Actuator Mapping
- [x] **Arm Joint Ordering Verified**: 14 values (`[L0..L6, R0..R6]`) matching MuJoCo arm joint kinematic order.
- [x] **Joint Limit Discrepancies Cataloged**: Real robot enforces tighter limits (notably Wrist Pitch ±0.45 rad, Yaw ±0.35 rad) vs URDF (±0.785 rad, ±0.524 rad).
- [x] **Velocity & Acceleration Constraints Known**: Real robot firmware clamps to 3.00 rad/s and 6.28 rad/s².
- [x] **Hand Control Mapping Understood**: 6 active actuators per hand (12 total), raw integer positions `0` (open) to `2000` (closed), coupled mimic joints handled by hardware linkage.

## 3. State Machine & Mode Switching Sequence
- [x] **Prerequisite Identified**: Must stop onboard `motion_player` (`aima em stop-app motion_player`) before commanding joint servo.
- [x] **Locomotion + Arm Servo Mode Confirmed**: Mode `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO` (Action 402) supports simultaneous walking and arm teleoperation.
- [x] **Safe Startup/Shutdown Sequence Established**: Must be firmly grounded before enabling arm motion, and returned to `RL_JOINT_DEFAULT` before lifting.
- [x] **Live State Machine Switch Test**: Call `SetAction` and poll `GetAction` on physical robot (done by hand with `SetAction.py` 10 -> 12 = 402; the receiver checks `GetAction`; mode 205 is not offered in this robot's menu).

## 4. Telemetry & Teleoperation Bridge
- [x] **Telemetry Topics Mapped**: `/motion/control/arm_joint_state` (100 Hz), `/motion/control/hand_joint_state` (100 Hz).
- [x] **Command Topics Mapped**: `/motion/control/arm_joint_command` (20–100 Hz per vendor; our Orin pacer publishes 400 Hz, least shake measured 13/11), `/motion/control/hand_joint_command` (20–100 Hz).
- [ ] **Hardware Watchdog Clarification**: Awaiting AGIBOT response regarding exact timeout duration and drop behavior.
- [ ] **Tethered Gantry Safety Test**: Execute zero-velocity command test with robot hung on safety transfer lift before free-standing operation.
EOF
