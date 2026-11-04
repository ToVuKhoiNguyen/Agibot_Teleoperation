"""
teleop/tracking/quest_tracking.py
Meta Quest 3S WebXR Tracking Interface.
Extracts 6-DoF head pose, bilateral wrist poses, and 25-keypoint hand tracking.
Includes tracking loss detection, timeout safety hold, and mock tracking generation.
"""

import time
import numpy as np


class QuestTrackingReceiver:
    """
    Manages Quest 3S WebXR tracking stream with safety tracking-loss monitoring.
    """

    def __init__(self, tv_wrapper=None, timeout_s: float = 0.5):
        self.tv_wrapper = tv_wrapper
        self.timeout_s = timeout_s

        self.last_valid_time_left = 0.0
        self.last_valid_time_right = 0.0
        self.last_valid_time_head = 0.0

        self.tracking_lost_left = True
        self.tracking_lost_right = True

        # Last raw matrices and controller flags for diagnostics
        self.last_tele_data = None
        self.last_raw_left_arm = None
        self.last_raw_right_arm = None
        self.last_head_pose = None
        self.has_controller_data = False
        self.has_hand_data = False

        # Controller buttons and analog states
        self.left_ctrl = {
            "grip": False,
            "trigger": 0.0,
            "squeeze": 0.0,
            "thumbstick": False,
            "a": False,
            "b": False
        }
        self.right_ctrl = {
            "grip": False,
            "trigger": 0.0,
            "squeeze": 0.0,
            "thumbstick": False,
            "a": False,
            "b": False
        }

        # Last known good poses (held during tracking loss)
        self.held_left_wrist_pos = np.array([0.30, 0.35, 1.15], dtype=np.float64)
        self.held_left_wrist_rot = np.eye(3, dtype=np.float64)
        self.held_right_wrist_pos = np.array([0.30, -0.35, 1.15], dtype=np.float64)
        self.held_right_wrist_rot = np.eye(3, dtype=np.float64)

        self.held_left_hand_curls = np.zeros(5, dtype=np.float64)
        self.held_right_hand_curls = np.zeros(5, dtype=np.float64)

    def update(self) -> dict:
        """
        Polls the latest tracking frame.
        Returns a structured dictionary with tracking state and safety flags.
        """
        now = time.time()

        if self.tv_wrapper is not None:
            try:
                tele_data = self.tv_wrapper.get_tele_data()
            except Exception as e:
                tele_data = None

            self.last_tele_data = tele_data
            if hasattr(self.tv_wrapper, "tvuer") and self.tv_wrapper.tvuer is not None:
                tvuer = self.tv_wrapper.tvuer
                self.last_raw_left_arm = tvuer.left_arm_pose
                self.last_raw_right_arm = tvuer.right_arm_pose
                self.last_head_pose = tvuer.head_pose

            if tele_data is not None:
                if hasattr(tele_data, "left_wrist_pose"):
                    # TeleData dataclass
                    is_ready = getattr(tele_data, "motion_data_ready", False)
                    left_mat = tele_data.left_wrist_pose
                    right_mat = tele_data.right_wrist_pose
                    left_hand_pts = tele_data.left_hand_pos
                    right_hand_pts = tele_data.right_hand_pos

                    # Detect if hand points or controller triggered motion
                    if left_hand_pts is not None and np.any(np.abs(left_hand_pts) > 1e-4):
                        self.has_hand_data = True

                    # Extract controller buttons & analog values if present
                    if hasattr(self.tv_wrapper, "tvuer") and self.tv_wrapper.tvuer is not None:
                        tvuer = self.tv_wrapper.tvuer
                        self.left_ctrl["squeeze"] = float(tvuer.left_ctrl_squeezeValue)
                        self.left_ctrl["grip"] = bool(tvuer.left_ctrl_squeeze or (self.left_ctrl["squeeze"] > 0.5))
                        self.left_ctrl["trigger"] = float(tvuer.left_ctrl_triggerValue)
                        self.left_ctrl["thumbstick"] = bool(tvuer.left_ctrl_thumbstick)
                        self.left_ctrl["a"] = bool(tvuer.left_ctrl_aButton)
                        self.left_ctrl["b"] = bool(tvuer.left_ctrl_bButton)

                        self.right_ctrl["squeeze"] = float(tvuer.right_ctrl_squeezeValue)
                        self.right_ctrl["grip"] = bool(tvuer.right_ctrl_squeeze or (self.right_ctrl["squeeze"] > 0.5))
                        self.right_ctrl["trigger"] = float(tvuer.right_ctrl_triggerValue)
                        self.right_ctrl["thumbstick"] = bool(tvuer.right_ctrl_thumbstick)
                        self.right_ctrl["a"] = bool(tvuer.right_ctrl_aButton)
                        self.right_ctrl["b"] = bool(tvuer.right_ctrl_bButton)

                        if tvuer.left_ctrl_trigger or tvuer.right_ctrl_trigger or self.left_ctrl["grip"] or self.right_ctrl["grip"]:
                            self.has_controller_data = True

                    # Extract translations & rotations from 4x4 SE(3) matrices
                    if left_mat is not None and isinstance(left_mat, np.ndarray) and left_mat.shape == (4, 4):
                        if np.all(np.isfinite(left_mat)):
                            pos_l = left_mat[:3, 3].copy()
                            # Check non-zero and updating / motion ready
                            if is_ready and np.linalg.norm(pos_l) > 1e-4:
                                self.held_left_wrist_pos = pos_l
                                self.held_left_wrist_rot = left_mat[:3, :3].copy()
                                self.last_valid_time_left = now
                                self.tracking_lost_left = False

                    if right_mat is not None and isinstance(right_mat, np.ndarray) and right_mat.shape == (4, 4):
                        if np.all(np.isfinite(right_mat)):
                            pos_r = right_mat[:3, 3].copy()
                            if is_ready and np.linalg.norm(pos_r) > 1e-4:
                                self.held_right_wrist_pos = pos_r
                                self.held_right_wrist_rot = right_mat[:3, :3].copy()
                                self.last_valid_time_right = now
                                self.tracking_lost_right = False
                elif isinstance(tele_data, dict):
                    # Fallback dict support
                    left_mat, l_valid = tele_data.get("left_arm", (None, False))
                    if l_valid and left_mat is not None:
                        self.held_left_wrist_pos = left_mat[:3, 3].copy()
                        self.held_left_wrist_rot = left_mat[:3, :3].copy()
                        self.last_valid_time_left = now
                        self.tracking_lost_left = False

                    right_mat, r_valid = tele_data.get("right_arm", (None, False))
                    if r_valid and right_mat is not None:
                        self.held_right_wrist_pos = right_mat[:3, 3].copy()
                        self.held_right_wrist_rot = right_mat[:3, :3].copy()
                        self.last_valid_time_right = now
                        self.tracking_lost_right = False

                    left_hand_pts = tele_data.get("left_hand_positions", None)
                    right_hand_pts = tele_data.get("right_hand_positions", None)
            else:
                left_hand_pts = None
                right_hand_pts = None
        else:
            left_hand_pts = None
            right_hand_pts = None

        # Check timeout for tracking loss
        if (now - self.last_valid_time_left) > self.timeout_s:
            self.tracking_lost_left = True
        if (now - self.last_valid_time_right) > self.timeout_s:
            self.tracking_lost_right = True

        return {
            "timestamp": now,
            "left": {
                "valid": not self.tracking_lost_left,
                "wrist_pos": self.held_left_wrist_pos.copy(),
                "wrist_rot": self.held_left_wrist_rot.copy(),
                "hand_joints": left_hand_pts,
                "held": self.tracking_lost_left,
                "ctrl": self.left_ctrl.copy()
            },
            "right": {
                "valid": not self.tracking_lost_right,
                "wrist_pos": self.held_right_wrist_pos.copy(),
                "wrist_rot": self.held_right_wrist_rot.copy(),
                "hand_joints": right_hand_pts,
                "held": self.tracking_lost_right,
                "ctrl": self.right_ctrl.copy()
            }
        }


class MockQuestStreamer:
    """
    Generates synthetic WebXR tracking packets for automated CI and simulation testing.
    Simulates smooth human operator head, wrist, and hand curl trajectories.
    """

    def __init__(self, frequency_hz: float = 60.0):
        self.freq = frequency_hz
        self.dt = 1.0 / frequency_hz
        self.step_idx = 0
        self.simulate_tracking_loss = False

    def get_frame(self) -> dict:
        t = self.step_idx * self.dt
        self.step_idx += 1

        # Smooth periodic arm motion relative to head in XR frame (Z-back, Y-up, X-right)
        # Forward reaching is negative Z in XR frame
        t_phase = 2.0 * np.pi * 0.5 * t

        # Left wrist reaches forward (-z) and left (-x)
        lx = -0.15 - 0.05 * np.cos(t_phase)
        ly = -0.15 + 0.05 * np.sin(t_phase)
        lz = -0.35 - 0.08 * np.sin(t_phase)

        # Right wrist reaches forward (-z) and right (+x)
        rx = 0.15 + 0.05 * np.cos(t_phase)
        ry = -0.15 + 0.05 * np.sin(t_phase)
        rz = -0.35 - 0.08 * np.sin(t_phase)

        # Curls [thumb, index, middle, ring, pinky]
        curl_wave = 0.5 * (1.0 - np.cos(2.0 * np.pi * 0.5 * t))
        curls = np.array([0.4 * curl_wave, curl_wave, curl_wave, curl_wave, curl_wave])

        # Tracking loss simulation toggle
        is_valid = not self.simulate_tracking_loss

        return {
            "timestamp": time.time(),
            "left": {
                "valid": is_valid,
                "wrist_pos": np.array([lx, ly, lz]),
                "wrist_rot": np.eye(3),
                "curls": curls.copy(),
                "pinch": float(0.8 * curl_wave)
            },
            "right": {
                "valid": is_valid,
                "wrist_pos": np.array([rx, ry, rz]),
                "wrist_rot": np.eye(3),
                "curls": curls.copy(),
                "pinch": float(0.8 * curl_wave)
            }
        }
