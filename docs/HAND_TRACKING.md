# WebXR Hand-Tracking Teleoperation Guide

## Overview
WebXR bare-hand tracking teleoperates the AgiBot A2 robot arms and dexterous hands using natural hand movements in VR without physical controllers. **Sim-only by default** (`sim`, `dryrun`, `ros2dry`).

## Tracking Modes
1. **Absolute 1:1 Mode** (`hand_follow_mode = "absolute"`, default for `?input=hands`):
   - Direct absolute shoulder-referenced kinematic following:
     $$p_{\text{target}} = P_{\text{robot\_shoulder}} + s \cdot M \cdot (p_{\text{palm}} - P_{\text{user\_shoulder}})$$
   - No clutching required: open palm, fist, and all finger poses move the arm continuously.
   - Head yaw is frozen/smoothed at calibration/recenter so turning the head does not deflect targets.
2. **Relative Mode** (`hand_follow_mode = "relative"`):
   - 1:1 following relative to latched anchor point $p_0 / R_0$. Fist and open palm both track.
3. **Gesture Clutch Mode** (`hand_follow_mode = "gesture"`):
   - Flat open palm still for 0.5s engages clutch; closed fist held 0.3s releases clutch.

## Calibration & Elbow Estimation
- **3s Pose Calibration**: Click "Calibrate Pose" in UI or in-scene HUD while holding arms straight forward with palms down. Records reference rotation $R_{\text{cal}}$ into `config/handcal.json` and latches torso reference frame.
- **Quick Recenter**: Click "Recenter" or hold both open palms facing robot still for 1.0 s.
- **Elbow Posture Hint** (`hand_elbow_hint = false` by default):
  *Note: Elbow pose is estimated mathematically from the shoulder-to-palm vector and a fixed down/out preference (it is NOT measured by VR sensors).*

## Safety Barriers
- **Tracking Loss**: Silence $> 150$ ms triggers smooth hold; regain smoothly ramps delta over 0.5 s (jump $< 2$ mm).
- **Jump Rejection**: Single-frame jumps $> 30$ cm are rejected.
- **Hardware Barrier**: Refused on real robot unless `--allow-handtracking-real` is passed.
