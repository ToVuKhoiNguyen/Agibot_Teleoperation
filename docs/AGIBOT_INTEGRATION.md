# AGIBOT A2 Ultra Model Integration

## Robot Specification
The AGIBOT A2 Ultra is a full-size bipedal humanoid robot featuring 52 revolute degrees of freedom and a root pelvis link (`base_link`).

### Joint Allocation
* **Legs (12 DoF)**: 6 DoF per leg (`left_hip_roll/yaw/pitch`, `tarsus`, `toe_pitch/roll`).
* **Arms (14 DoF)**: 7 DoF per arm (`idx13`–`idx19` Left; `idx20`–`idx26` Right).
* **Dexterous Hands (24 DoF)**: 12 DoF per hand (6 active, 6 mimic).
* **Head (2 DoF)**: Yaw (`idx27`) and Pitch (`idx28`).

---

## Arm Kinematic Chains (7-DoF)
Kinematic chain from `base_link`:
`base_link` → `left_arm_link_base` → `left_arm_link01` … `left_arm_link07` → `left_hand` (wrist/palm frame).

| Joint Name (Left) | Joint Name (Right) | Type | Range (rad) | Velocity | Effort |
|---|---|---|---|---|---|
| `idx13_left_arm_joint1` | `idx20_right_arm_joint1` | Revolute | [-2.967, +2.967] | 12 rad/s | 100 Nm |
| `idx14_left_arm_joint2` | `idx21_right_arm_joint2` | Revolute | L: [-0.524, +1.658]<br/>R: [-1.658, +0.524] | 12 rad/s | 100 Nm |
| `idx15_left_arm_joint3` | `idx22_right_arm_joint3` | Revolute | [-2.967, +2.967] | 12 rad/s | 100 Nm |
| `idx16_left_arm_joint4` | `idx23_right_arm_joint4` | Revolute | L: [-2.094, 0.000]<br/>R: [0.000, +2.094] | 12 rad/s | 100 Nm |
| `idx17_left_arm_joint5` | `idx24_right_arm_joint5` | Revolute | [-2.880, +2.880] | 12 rad/s | 100 Nm |
| `idx18_left_arm_joint6` | `idx25_right_arm_joint6` | Revolute | [-0.785, +0.785] | 12 rad/s | 100 Nm |
| `idx19_left_arm_joint7` | `idx26_right_arm_joint7` | Revolute | [-0.524, +0.524] | 12 rad/s | 100 Nm |

---

## Dexterous Hands (6 Active DoF + 6 Mimic DoF per hand)
Each hand features 5 fingers with mechanical tendon coupling:

| Finger | Active Joint | Active Range | Mimic Joint | Multiplier | Mimic Range |
|---|---|---|---|---|---|
| **Thumb Swing** | `L/R_thumb_swing_joint` | [0.0, 2.286] | — | — | — |
| **Thumb Flexion** | `L/R_thumb_1_joint` | [0.0, 0.780] | `L/R_thumb_2_joint`<br/>`L/R_thumb_3_joint` | 0.40 × Thumb 1<br/>0.60 × Thumb 1 | [0.0, 0.914]<br/>[0.0, 1.372] |
| **Index** | `L/R_index_1_joint` | [0.0, 1.700] | `L/R_index_2_joint` | 1.00 × Index 1 | [0.0, 1.700] |
| **Middle** | `L/R_middle_1_joint` | [0.0, 1.700] | `L/R_middle_2_joint` | 1.00 × Middle 1 | [0.0, 1.700] |
| **Ring** | `L/R_ring_1_joint` | [0.0, 1.700] | `L/R_ring_2_joint` | 1.00 × Ring 1 | [0.0, 1.700] |
| **Pinky** | `L/R_pinky_1_joint` | [0.0, 1.700] | `L/R_pinky_2_joint` | 1.00 × Pinky 1 | [0.0, 1.700] |

In MuJoCo, mimic coupling is enforced dynamically via `<equality><joint joint1="..." joint2="..." polycoef="0 mult 0 0 0"/></equality>`.
