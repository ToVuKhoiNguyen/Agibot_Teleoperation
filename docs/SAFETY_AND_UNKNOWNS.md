# AGIBOT A2 Ultra - Known Unknowns & Hardware Safety Analysis

This document catalogs hardware ambiguities and unverified behaviors identified during reverse-engineering and specification review of the AGIBOT A2 Ultra.

---

## 1. Honest List of Known Unknowns

| Parameter / Subsystem | Documented in AimDK? | Adopted Worst-Case Default | Risk if Default Wrong |
| :--- | :---: | :--- | :--- |
| **Hardware Watchdog Timeout** | ❌ No duration given (p. 11 states $\ge 20$ Hz) | Teleop streams at 50 Hz; latches fault if state older than 200 ms | Controller freezes or relaxes joints unexpectedly |
| **Physical E-Stop Firmware Behavior** | ❌ Dedicated wireless unit | Assume firmware unpowers all joint motors (gravity collapse) | Overhead gantry harness must be tensioned at all times |
| **Thumb 0 Swing Sign Calibration** | ❌ Unstated (p. 49 only gives 0..2000) | `thumb_0` held neutral (0 raw) unless `--enable-thumb-swing` passed | Inverse abduction causing tendon overstrain |
| **Wrist Differential Pushrod Model** | ❌ Feedback is equivalent serial; command unstated | Mink IK outputs serial angles (`idx18`, `idx19`) directly | Wrists hit internal physical limits earlier than expected |
| **Payload Envelope at Full Reach** | ❌ Rated 1 kg / 3 kg grip, reach unstated | Teleop enforces soft workspace limits ($X \le 0.9$ m) | Overcurrent trips or joint sagging at full extension |
| **3D Visual Mesh Assets License** | ❌ Unverified | Bundled for research/academic teleoperation testing | Commercial redistribution restrictions |

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
   - Arm velocity ceiling: 0.3 rad/s (hard cap 1.0 rad/s unless `--i-accept-higher-speed`).
   - Hand position ceiling: raw 800 (40% maximum curl).
   - Hand effort ceiling: 20% torque limit (1140 units $\approx 11.4$ N force).
   - Fault latch: continuous hold position stream on $> 50$ mm deviation or $> 200$ ms delay.

