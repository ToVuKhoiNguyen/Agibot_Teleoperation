"""
teleop/retargeting/agibot_hand_retargeting.py
Retargets human hand tracking data (finger curls, WebXR 25-keypoint hand pose, pinch)
to the AGIBOT A2 Ultra dexterous hand (6 active joints + 6 coupled mimic joints).
Includes weighted moving window filtering and velocity slew rate clamping.
"""

import numpy as np
from teleop.utils.weighted_moving_filter import WeightedMovingFilter
from teleop.kinematics.filters import OneEuroFilter


class AgibotHandRetargeting:
    """
    Retargeter for AGIBOT A2 Ultra 6-DoF active dexterous hand.
    Active joints:
      0: thumb_swing_joint  [0.0, 2.286]
      1: thumb_1_joint      [0.0, 0.780]
      2: index_1_joint      [0.0, 1.700]
      3: middle_1_joint     [0.0, 1.700]
      4: ring_1_joint       [0.0, 1.700]
      5: pinky_1_joint      [0.0, 1.700]
    """

    def __init__(self, hand: str = "left", filter_weights=(0.5, 0.3, 0.2), max_vel_rad_s: float = 6.0):
        assert hand in ["left", "right"], f"Hand must be 'left' or 'right', got {hand}"
        self.hand = hand
        self.max_vel_rad_s = max_vel_rad_s

        # Joint limits from URDF
        self.limits = np.array([
            [0.0, 2.286],  # thumb_swing
            [0.0, 0.780],  # thumb_1
            [0.0, 1.700],  # index_1
            [0.0, 1.700],  # middle_1
            [0.0, 1.700],  # ring_1
            [0.0, 1.700],  # pinky_1
        ], dtype=np.float64)

        self.filter = WeightedMovingFilter(weights=filter_weights, data_size=6)
        self.one_euro_filter = OneEuroFilter(min_cutoff=1.5, beta=0.01, d_cutoff=1.0)
        self.last_cmd = np.zeros(6, dtype=np.float64)

    def reset(self):
        self.filter.reset()
        self.one_euro_filter.reset()
        self.last_cmd = np.zeros(6, dtype=np.float64)

    def retarget_curls(
        self,
        curls: np.ndarray,
        pinch_strength: float = 0.0,
        dt: float = 0.02
    ) -> np.ndarray:
        """
        Retargets normalized curl amounts (5 values for [thumb, index, middle, ring, pinky] in [0, 1])
        and pinch strength in [0, 1] to the 6 active joint angles.
        """
        curls = np.clip(np.asarray(curls, dtype=np.float64), 0.0, 1.0)
        c_thumb, c_index, c_middle, c_ring, c_pinky = curls[:5]

        # Thumb swing increases with pinch or thumb curl to achieve opposition
        thumb_swing = float(np.clip(1.2 * c_thumb + 0.8 * pinch_strength, 0.0, 2.286))
        thumb_1 = float(c_thumb * 0.780)

        # Fingers flexion
        index_1 = float(c_index * 1.700)
        middle_1 = float(c_middle * 1.700)
        ring_1 = float(c_ring * 1.700)
        pinky_1 = float(c_pinky * 1.700)

        raw_cmd = np.array([thumb_swing, thumb_1, index_1, middle_1, ring_1, pinky_1], dtype=np.float64)

        # Apply One Euro filter to finger inputs
        raw_cmd = self.one_euro_filter.filter(raw_cmd)

        # Apply moving average filter
        filtered_cmd = self.filter.add_data(raw_cmd)

        # Apply velocity slew rate limiting
        max_step = self.max_vel_rad_s * dt
        diff = filtered_cmd - self.last_cmd
        clamped_diff = np.clip(diff, -max_step, max_step)
        clamped_cmd = self.last_cmd + clamped_diff

        # Final joint limit enforcement
        final_cmd = np.clip(clamped_cmd, self.limits[:, 0], self.limits[:, 1])
        self.last_cmd = final_cmd.copy()
        return final_cmd

    def retarget_webxr_joints(self, joints_25: np.ndarray, dt: float = 0.02) -> np.ndarray:
        """
        Extracts finger flexion and thumb opposition from WebXR 25 hand joint 3D positions.
        WebXR layout:
          0: Wrist
          1-4: Thumb (metacarpal, proximal, distal, tip)
          5-9: Index (metacarpal, proximal, intermediate, distal, tip)
          10-14: Middle
          15-19: Ring
          20-24: Pinky
        """
        joints = np.asarray(joints_25, dtype=np.float64)
        if len(joints) < 25:
            # Fallback to zeros if incomplete
            return self.retarget_curls(np.zeros(5), dt=dt)

        # Calculate curl as distance reduction between fingertip and palm/wrist relative to full extension
        wrist = joints[0]
        middle_base = joints[10]
        palm_center = (wrist + middle_base) / 2.0

        tips = [joints[4], joints[9], joints[14], joints[19], joints[24]]
        bases = [joints[2], joints[6], joints[11], joints[16], joints[21]]

        # Finger curl is normalized ratio of tip-to-base distance
        curls = []
        for tip, base in zip(tips, bases):
            dist = np.linalg.norm(tip - base)
            # Full extension ~ 0.08m, full flexion ~ 0.025m
            curl_val = float(np.clip((0.08 - dist) / 0.055, 0.0, 1.0))
            curls.append(curl_val)

        # Pinch: distance between thumb tip (4) and index tip (9)
        pinch_dist = np.linalg.norm(joints[4] - joints[9])
        # In contact: < 0.025m, open: > 0.08m
        pinch = float(np.clip((0.075 - pinch_dist) / 0.05, 0.0, 1.0))

        return self.retarget_curls(np.array(curls), pinch_strength=pinch, dt=dt)

    @staticmethod
    def compute_mimic_joints(active_qpos: np.ndarray) -> dict:
        """
        Calculates all 6 coupled mimic finger joints from the 6 active joint values.
        """
        q = np.asarray(active_qpos, dtype=np.float64)
        return {
            "thumb_2": float(0.40 * q[1]),
            "thumb_3": float(0.60 * q[1]),
            "index_2": float(1.00 * q[2]),
            "middle_2": float(1.00 * q[3]),
            "ring_2": float(1.00 * q[4]),
            "pinky_2": float(1.00 * q[5]),
        }
