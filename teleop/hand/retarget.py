"""
teleop/hand/retarget.py
Retargets 25 WebXR hand tracking joint positions to AGIBOT A2 Ultra 6-DoF active dexterous hands.
Resolves XRHand joint names in any order, computes bone chain bending, separates thumb opposition,
provides 30s auto-calibration, and defaults fallback on bad/absent calibration.
"""

import json
from collections import deque
from pathlib import Path
from typing import Union, Dict, List
import numpy as np

from teleop.kinematics.signal_filters import OneEuroFilter

PROJECT_ROOT = Path(__file__).resolve().parents[2]
HANDCAL_FILE = PROJECT_ROOT / "config" / "handcal.json"

COMFORTABLE_LIMITS = np.array([1.60, 0.35, 1.30, 1.30, 1.30, 1.30], dtype=np.float64)

DEFAULT_CALIBRATION = {
    "open_angles": [0.15, 0.20, 0.20, 0.20, 0.20],
    "closed_angles": [2.80, 4.40, 4.50, 4.40, 4.30],
    "thumb_opp_open": 0.20,
    "thumb_opp_closed": 1.55,
}

XR_JOINT_ORDER = [
    "wrist",
    "thumb-metacarpal", "thumb-phalanx-proximal", "thumb-phalanx-distal", "thumb-tip",
    "index-finger-metacarpal", "index-finger-phalanx-proximal", "index-finger-phalanx-intermediate", "index-finger-phalanx-distal", "index-finger-tip",
    "middle-finger-metacarpal", "middle-finger-phalanx-proximal", "middle-finger-phalanx-intermediate", "middle-finger-phalanx-distal", "middle-finger-tip",
    "ring-finger-metacarpal", "ring-finger-phalanx-proximal", "ring-finger-phalanx-intermediate", "ring-finger-phalanx-distal", "ring-finger-tip",
    "pinky-finger-metacarpal", "pinky-finger-phalanx-proximal", "pinky-finger-phalanx-intermediate", "pinky-finger-phalanx-distal", "pinky-finger-tip",
]


def _vec_angle(v1: np.ndarray, v2: np.ndarray) -> float:
    n1 = float(np.linalg.norm(v1))
    n2 = float(np.linalg.norm(v2))
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    return float(np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)))


def resolve_joints_array(joints: Union[dict, list, np.ndarray]) -> np.ndarray:
    """Extracts a (25, 3) numpy array regardless of whether joints are array, dict, or named list."""
    if isinstance(joints, dict):
        j_arr = np.zeros((25, 3), dtype=np.float64)
        for idx, name in enumerate(XR_JOINT_ORDER):
            if name in joints:
                j_arr[idx] = np.asarray(joints[name], dtype=np.float64)
            elif str(idx) in joints:
                j_arr[idx] = np.asarray(joints[str(idx)], dtype=np.float64)
        return j_arr
    elif isinstance(joints, list) and len(joints) > 0 and isinstance(joints[0], dict) and "name" in joints[0]:
        j_arr = np.zeros((25, 3), dtype=np.float64)
        j_dict = {item["name"]: item.get("pos", [0, 0, 0]) for item in joints}
        for idx, name in enumerate(XR_JOINT_ORDER):
            if name in j_dict:
                j_arr[idx] = np.asarray(j_dict[name], dtype=np.float64)
        return j_arr
    return np.asarray(joints, dtype=np.float64)


class HandRetargeter:
    """Retargets 25-joint 3D hand tracking coordinates to dexterous hand joint commands."""

    def __init__(self, side: str = "right", cal_path: Path = HANDCAL_FILE):
        self.side = side.lower()
        self.cal_path = cal_path
        self.cal = self._load_calibration()

        self.prev_flexion = np.zeros(5, dtype=np.float64)
        self.prev_opp = 0.0
        self.prev_wrist_pos = None

        self.still_time = 0.0
        self.fist_time = 0.0
        self.overhead_time = 0.0
        self.calibration_bad = False
        self.bend_history = deque()
        self.flex_filters = [OneEuroFilter(min_cutoff=2.5, beta=0.08) for _ in range(5)]
        self.opp_filter = OneEuroFilter(min_cutoff=2.5, beta=0.08)

    def _load_calibration(self) -> dict:
        if self.cal_path.exists():
            try:
                with open(self.cal_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    return data.get(self.side, DEFAULT_CALIBRATION).copy()
            except Exception:
                pass
        return DEFAULT_CALIBRATION.copy()

    def update_auto_calibration(self, angles: np.ndarray, now_t: float):
        """Maintains 30s running min/max per finger and blends slowly into calibration range."""
        self.bend_history.append((now_t, angles.copy()))
        while self.bend_history and self.bend_history[0][0] < now_t - 30.0:
            self.bend_history.popleft()

        if len(self.bend_history) >= 20:
            arr = np.array([a for _, a in self.bend_history])
            run_min = np.min(arr, axis=0)
            run_max = np.max(arr, axis=0)
            blend = 0.005
            for i in range(5):
                if run_max[i] - run_min[i] >= np.radians(30.0):
                    self.cal["open_angles"][i] = float((1.0 - blend) * self.cal["open_angles"][i] + blend * run_min[i])
                    self.cal["closed_angles"][i] = float((1.0 - blend) * self.cal["closed_angles"][i] + blend * run_max[i])

    def compute_finger_angles(self, joints: Union[dict, list, np.ndarray]) -> tuple[np.ndarray, float, np.ndarray]:
        """Calculates 5 finger flexion angles (sum of bone chain bends), thumb opposition, and palm normal."""
        j = resolve_joints_array(joints)
        if len(j) < 25:
            return np.zeros(5), 0.0, np.array([0.0, 0.0, 1.0])

        angles = np.zeros(5, dtype=np.float64)
        # Thumb: chain 1->2 (metacarpal), 2->3 (proximal), 3->4 (distal)
        v_th_m = j[2] - j[1]
        v_th_p = j[3] - j[2]
        v_th_d = j[4] - j[3]
        angles[0] = _vec_angle(v_th_m, v_th_p) + _vec_angle(v_th_p, v_th_d)

        # 4 Fingers: MCP, PIP, DIP chains
        finger_bases = [5, 10, 15, 20]
        for idx, base in enumerate(finger_bases):
            v0 = j[base + 1] - j[base]      # MCP
            v1 = j[base + 2] - j[base + 1]  # PIP
            v2 = j[base + 3] - j[base + 2]  # DIP
            v3 = j[base + 4] - j[base + 3]  # tip
            angles[idx + 1] = _vec_angle(v0, v1) + _vec_angle(v1, v2) + _vec_angle(v2, v3)

        v_idx_meta = j[6] - j[5]
        thumb_opp = _vec_angle(v_th_m, v_idx_meta)

        v_len = j[5] - j[0]
        v_across = j[20] - j[5]
        cross = np.cross(v_len, v_across)
        if self.side == "left":
            cross = -cross
        palm_normal = cross / (float(np.linalg.norm(cross)) + 1e-8)

        return angles, float(thumb_opp), palm_normal

    def normalize_flexion(self, angles: np.ndarray, thumb_opp: float, now_t: float = 0.0) -> tuple[np.ndarray, float]:
        """Normalizes angles [rad] into [0..1] range, with fallback on bad/absent calibration."""
        if now_t > 0:
            self.update_auto_calibration(angles, now_t)

        op_a = np.array(self.cal.get("open_angles", DEFAULT_CALIBRATION["open_angles"]), dtype=np.float64)
        cl_a = np.array(self.cal.get("closed_angles", DEFAULT_CALIBRATION["closed_angles"]), dtype=np.float64)
        diff = cl_a - op_a

        cal_bad = False
        min_range = np.radians(30.0)
        for idx in range(5):
            if diff[idx] < min_range:
                cal_bad = True
                op_a[idx] = DEFAULT_CALIBRATION["open_angles"][idx]
                cl_a[idx] = DEFAULT_CALIBRATION["closed_angles"][idx]
                diff[idx] = cl_a[idx] - op_a[idx]
        self.calibration_bad = cal_bad

        norm_flex = np.clip((angles - op_a) / np.maximum(diff, 0.1), 0.0, 1.0)
        op_opp = float(self.cal.get("thumb_opp_open", DEFAULT_CALIBRATION["thumb_opp_open"]))
        cl_opp = float(self.cal.get("thumb_opp_closed", DEFAULT_CALIBRATION["thumb_opp_closed"]))
        norm_opp = float(np.clip((thumb_opp - op_opp) / max(cl_opp - op_opp, 0.2), 0.0, 1.0))

        return norm_flex, norm_opp

    def apply_filter_and_hysteresis(self, flexion: np.ndarray, opp: float, alpha: float = 0.60, deadband: float = 0.02, now_t: float = None, dt: float = 0.016) -> tuple[np.ndarray, float]:
        """Applies One-Euro filtering, small deadband (<=3%), and rate-limiting."""
        max_rate, max_df = 8.0, 8.0 * max(0.005, dt)
        filt_f = np.zeros(5, dtype=np.float64)
        for i in range(5):
            val_filt = float(self.flex_filters[i].filter(np.array([flexion[i]]), now_t)[0])
            diff = abs(val_filt - self.prev_flexion[i])
            target = self.prev_flexion[i] if diff < deadband else val_filt
            df = np.clip(target - self.prev_flexion[i], -max_df, max_df)
            filt_f[i] = np.clip(self.prev_flexion[i] + df, 0.0, 1.0)

        val_opp = float(self.opp_filter.filter(np.array([opp]), now_t)[0])
        diff_o = abs(val_opp - self.prev_opp)
        target_o = self.prev_opp if diff_o < deadband else val_opp
        df_o = np.clip(target_o - self.prev_opp, -max_df, max_df)
        filt_o = float(np.clip(self.prev_opp + df_o, 0.0, 1.0))

        self.prev_flexion, self.prev_opp = filt_f, filt_o
        return filt_f, filt_o

    def retarget_to_active_joints(self, flexion: np.ndarray, opp: float) -> np.ndarray:
        """Maps 5 flexions [0..1] and thumb opposition [0..1] to the 6 robot active joints."""
        q6 = np.zeros(6, dtype=np.float64)
        q6[0] = opp * COMFORTABLE_LIMITS[0]
        q6[1:6] = flexion * COMFORTABLE_LIMITS[1:6]
        return np.clip(q6, 0.0, COMFORTABLE_LIMITS)

    def detect_gestures(self, joints, wrist_pos: np.ndarray, flexion: np.ndarray, palm_normal: np.ndarray, dt: float, current_clutch: bool = False) -> dict:
        vel = 0.0
        if self.prev_wrist_pos is not None and dt > 1e-4:
            vel = float(np.linalg.norm(wrist_pos - self.prev_wrist_pos) / dt)
        self.prev_wrist_pos = wrist_pos.copy()

        is_open = bool(np.mean(flexion) < 0.25 and np.max(flexion) < 0.38)
        is_facing_robot = bool(palm_normal[2] < -0.20 or palm_normal[1] < -0.20)
        is_still = (vel < 0.08)

        self.still_time = (self.still_time + dt) if (is_open and is_facing_robot and is_still) else 0.0
        engage_triggered = (self.still_time >= 0.50 and not current_clutch)

        is_fist = bool(np.mean(flexion) >= 0.60 and np.min(flexion[1:4]) >= 0.50)
        self.fist_time = (self.fist_time + dt) if is_fist else 0.0
        release_triggered = (self.fist_time >= 0.30 and current_clutch)

        is_overhead = bool(wrist_pos[1] > 1.75)
        self.overhead_time = (self.overhead_time + dt) if is_overhead else 0.0

        return {
            "engage": engage_triggered,
            "release": release_triggered,
            "estop_candidate": bool(self.overhead_time >= 1.0),
            "is_open": is_open,
            "is_fist": is_fist,
            "is_overhead": is_overhead,
            "vel": vel,
        }
