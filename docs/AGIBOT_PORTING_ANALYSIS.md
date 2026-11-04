# AGIBOT A2 Ultra XR Teleoperation Porting Analysis

## 1. Executive Summary
This document analyzes porting the WebXR teleoperation pipeline from Unitree's `xr_teleoperate` to create `agibot_xr_teleoperate` for the AGIBOT A2 Ultra humanoid robot in MuJoCo simulation. The system enables bilateral 7-DoF arm and 12-DoF dexterous hand control via Meta Quest 3S (WebXR/Vuer) in simulation without real hardware.

---

## 2. Reusable Modules from `xr_teleoperate`
* **TeleVuer / WebXR (`teleop/televuer`)**: WebXR session manager based on Vuer. Receives Quest 3S 6-DoF head pose, wrist poses, finger joint keypoints, and pinch states via browser WebSocket.
* **Signal Filtering (`WeightedMovingFilter`)**: Finite-impulse-response moving average filter with custom weight kernels, smoothing noisy human motion inputs before IK and actuator tracking.
* **State Management & Key Listeners**: Event transitions (`READY`, `START`, `PAUSED`, `ESTOP`), keyboard listener hooks (`r` for start/reset, `q` for e-stop).

---

## 3. Unitree-Specific Modules (To Drop / Replace)
| Unitree Module | Function in Original | Replacement in AGIBOT Port |
|---|---|---|
| `unitree_sdk2py` / DDS | Low-level DDS protocol to Unitree hardware/G1 sim | **Replaced** by MuJoCo native Python API (`data.ctrl`, `data.qpos`) |
| `robot_arm.py` | G1/H1 specific joint offsets and PD publishers | **Replaced** by `teleop/simulation/mujoco_env.py` |
| `robot_arm_ik.py` | G1/H1 Pinocchio + CasADi solver with G1 joint limits | **Replaced** by `agibot_a2_ik.py` tailored to A2 kinematic chains |
| `robot_hand_unitree/inspire/brainco.py` | Hand drivers for Unitree/Inspire/BrainCo hands | **Replaced** by `agibot_hand_retargeting.py` for A2 dexterous hands |
| `teleimager` / WebRTC | Video stream back to VR headset | **Dropped** (per requirement: no video stream back to headset) |

---

## 4. IK Library Selection
* **Choice**: **MuJoCo Damped Least Squares (DLS) & SQP Differential IK**.
* **Rationale**: Unitree's Pinocchio + CasADi pipeline requires C++ code generation, complex binary dependencies not available on standard pip for Python 3.12, and high initialization latency. MuJoCo's built-in analytical Jacobian routines (`mj_jac`, `mj_jacSite`) paired with DLS and null-space posture regularization run at > 500 Hz, achieve < 2 mm position error and < 1 deg orientation error, enforce exact joint limits, and require zero extra external dependencies.

---

## 5. AGIBOT A2 Ultra Model Architecture
* **Total Revolute Joints**: 52 active degrees of freedom.
  - **Arms**: 14 DoF (7 DoF left arm: `idx13`–`idx19`; 7 DoF right arm: `idx20`–`idx26`).
  - **Dexterous Hands**: 24 joints total (12 per hand).
    - **Active (6 DoF/hand)**: Thumb Swing, Thumb 1, Index 1, Middle 1, Ring 1, Pinky 1.
    - **Mimic/Coupled (6 DoF/hand)**:
      - Thumb 2 = 0.40 × Thumb 1; Thumb 3 = 0.60 × Thumb 1.
      - Index 2, Middle 2, Ring 2, Pinky 2 = 1.00 × respective Proximal joint (Joint 1).
  - **Legs**: 12 DoF (6 left leg: `idx01`–`idx06`; 6 right leg: `idx07`–`idx12`).
  - **Head**: 2 DoF (`idx27_head_joint1`, `idx28_head_joint2`).
* **Kinematic Root**: `base_link` (pelvis).

---

## 6. MuJoCo Simulation Strategy
* **Base Stabilization**: The A2 robot base is welded/fixed to the simulation world at `pos="0 0 0.93"`. This places the feet exactly 2 mm above the ground floor (`z = 0`), keeping the robot upright and stable for arm/hand teleoperation without falling or requiring whole-body locomotion controllers.
* **Mimic Joints**: Expressed via native MuJoCo `<equality><joint ... polycoef="0 mult 0 0 0"/></equality>` constraints.
* **Actuation & Dynamics**: Position actuators (`<position>`) with `implicitfast` Euler integrator (`dt = 0.002s`) and calibrated joint damping classes (`arm`: damping 5.0, kp 200; `hand`: damping 0.05, kp 20; `leg`: damping 30.0, kp 800).
* **End-Effector Tracking**: Sites (`left_arm_ee`, `right_arm_ee`) placed on wrist/palm frames for real-time Cartesian pose extraction and IK target tracking.

---

## 7. Meta Quest 3S Compatibility
* **WebXR / TeleVuer Pipeline**: Runs inside Meta Quest Browser via standard WebXR.
* **HTTPS / SSL**: Quest Browser mandates secure contexts (`https://` and `wss://`). Self-signed SSL certificates (`cert.pem`, `key.pem`) generated via OpenSSL with Quest local IP routing or USB reverse-tethering (`adb reverse tcp:8012 tcp:8012`).
* **Hand & Controller Support**: Native WebXR hand tracking provides 25 joints per hand; pinch gesture and trigger buttons map to finger grasp and emergency stop.
