# AGIBOT A2 Ultra - Known Unknowns & Hardware Safety Analysis

This document catalogs hardware ambiguities and unverified behaviors identified during reverse-engineering and specification review of the AGIBOT A2 Ultra.

---

## 1. Honest List of Known Unknowns

| Parameter / Subsystem | Documented in AimDK? | Adopted Worst-Case Default | Risk if Default Wrong |
| :--- | :---: | :--- | :--- |
| **Hardware Watchdog Timeout** | MISSING: No duration given (p. 11 states $\ge 20$ Hz) | Teleop streams at 50 Hz; latches fault if state older than 200 ms | Controller freezes or relaxes joints unexpectedly |
| **Physical E-Stop Firmware Behavior** | MISSING: Dedicated wireless unit | Assume firmware unpowers all joint motors (gravity collapse) | Overhead gantry harness must be tensioned at all times |
| **Thumb 0 Swing Sign Calibration** | MISSING: Unstated (p. 49 only gives 0..2000) | Since 2026-11-17 the thumb is held fixed in a grasp pose, swing 800 / flex 900 raw (`--thumb-pose`, laptop and receiver), also when not engaged; `--thumb-pose off` restores the 500 raw rest (`--thumb-rest`, since 2026-11-13); receiver flag `--no-thumb-swing` holds `thumb_0` at 0 | Inverse abduction causing tendon overstrain |
| **Wrist Differential Pushrod Model** | MISSING: Feedback is equivalent serial; command unstated | Mink IK outputs serial angles (`idx18`, `idx19`) directly | Wrists hit internal physical limits earlier than expected |
| **Payload Envelope at Full Reach** | MISSING: Rated 1 kg / 3 kg grip, reach unstated | Teleop enforces soft workspace limits ($X \le 0.9$ m) | Overcurrent trips or joint sagging at full extension |
| **3D Visual Mesh Assets License** | MISSING: Unverified | Bundled for research/academic teleoperation testing | Commercial redistribution restrictions |

---

## 2. Invariant Safety Mitigations Implemented

1. **Gantry Precondition**: A suspended gantry harness with counter-weight or tension must be attached to the robot torso prior to executing any motion commands.
2. **Two-Person Rule**: One operator dons the Meta Quest 3S headset; a second spotter holds the wireless hardware E-stop button with finger on the trigger.
3. **Five Hardware Interlocks**:
   - CLI acknowledgement (`--i-understand-hardware-risk`)
   - Environment authorization (`AGIBOT_ALLOW_REAL=1`)
   - Interactive typed passphrase (`GANTRY AND ESTOP READY`)
   - Live robot state verified streaming within $\le 1.0$ s
   - Explicit domain check (`ROS_DOMAIN_ID != 77`)
4. **Conservative First-Test Caps**:
   - Arm velocity ceiling: legacy ROS 2 backend (`teleop/robot/ros2_backend.py`) 0.3 rad/s. The current UDP + Orin pacer path follows `--speed` (default 6 = 2.0 rad/s, 6.28 rad/s²) and the per-joint Orin limits (config D: 1.2,1.5,1.2,1.5,2,2,2 rad/s, 6 rad/s²), never above the vendor 3 rad/s.
   - Hand position ceiling: raw 2000 (full travel; `--hand-raw-max` lowers it). Default thumb pose: grasp pose 800 / 900 raw (swing lowered 1200 -> 800 on 17/11: the thumb touched the index finger) (`--thumb-pose`; `--thumb-pose off` = 500 rest via `--thumb-rest`).
   - Hand effort ceiling: 20% torque limit (1140 units $\approx 11.4$ N force).
   - Fault latch: continuous hold position stream on $> 50$ mm deviation or $> 200$ ms delay.

