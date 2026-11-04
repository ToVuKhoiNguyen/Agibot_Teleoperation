"""
teleop/tracking/webxr_vuer_handler.py
Integrates WebXR hands and motion controllers event handlers directly into WebPanelServer.
Receives Quest 3S poses, fingers, gestures, and controllers over WebSocket /ws_panel.
"""

import numpy as np

class WebXRReceiver:
    def __init__(self, use_controller: bool = False):
        self.use_controller = use_controller
        self.xr_active = False
        self.last_update_time = 0.0

        # Poses
        self.head_pose = np.eye(4, dtype=np.float64)
        self.left_wrist_pose = np.eye(4, dtype=np.float64)
        self.right_wrist_pose = np.eye(4, dtype=np.float64)
        self.left_hand_pts = np.zeros((25, 3), dtype=np.float64)
        self.right_hand_pts = np.zeros((25, 3), dtype=np.float64)

        # Controls & Gestures
        self.left_ctrl = {"squeeze": 0.0, "grip": False, "trigger": 0.0, "thumbstick": False, "a": False, "b": False}
        self.right_ctrl = {"squeeze": 0.0, "grip": False, "trigger": 0.0, "thumbstick": False, "a": False, "b": False}
        self.pinch_l = 0.0
        self.pinch_r = 0.0

    def handle_message(self, data: dict):
        msg_type = data.get("type", "")
        self.xr_active = True

        if msg_type in ("HAND_MOVE", "HANDS"):
            # Hand tracking packet from WebXR
            left_data = data.get("left")
            right_data = data.get("right")
            if left_data and len(left_data) >= 16:
                mat = np.array(left_data[:16], dtype=np.float64).reshape((4, 4), order="F")
                if np.all(np.isfinite(mat)) and np.linalg.norm(mat[:3, 3]) > 1e-4:
                    self.left_wrist_pose = mat
            if right_data and len(right_data) >= 16:
                mat = np.array(right_data[:16], dtype=np.float64).reshape((4, 4), order="F")
                if np.all(np.isfinite(mat)) and np.linalg.norm(mat[:3, 3]) > 1e-4:
                    self.right_wrist_pose = mat

            l_state = data.get("leftState", {})
            r_state = data.get("rightState", {})
            self.pinch_l = float(l_state.get("pinchValue", 1.0 if l_state.get("pinch") else 0.0))
            self.pinch_r = float(r_state.get("pinchValue", 1.0 if r_state.get("pinch") else 0.0))

        elif msg_type in ("CONTROLLER_MOVE", "MOTION_CONTROLLERS"):
            # Motion controller packet from WebXR
            left_data = data.get("left")
            right_data = data.get("right")
            if left_data and len(left_data) >= 16:
                mat = np.array(left_data[:16], dtype=np.float64).reshape((4, 4), order="F")
                if np.all(np.isfinite(mat)) and np.linalg.norm(mat[:3, 3]) > 1e-4:
                    self.left_wrist_pose = mat
            if right_data and len(right_data) >= 16:
                mat = np.array(right_data[:16], dtype=np.float64).reshape((4, 4), order="F")
                if np.all(np.isfinite(mat)) and np.linalg.norm(mat[:3, 3]) > 1e-4:
                    self.right_wrist_pose = mat

            l_st = data.get("leftState", {})
            r_st = data.get("rightState", {})
            self.left_ctrl["trigger"] = float(l_st.get("triggerValue", 1.0 if l_st.get("trigger") else 0.0))
            self.left_ctrl["squeeze"] = float(l_st.get("squeezeValue", 1.0 if l_st.get("squeeze") else 0.0))
            self.left_ctrl["grip"] = self.left_ctrl["squeeze"] > 0.4 or bool(l_st.get("squeeze", False))
            self.left_ctrl["thumbstick"] = bool(l_st.get("thumbstick", False))

            self.right_ctrl["trigger"] = float(r_st.get("triggerValue", 1.0 if r_st.get("trigger") else 0.0))
            self.right_ctrl["squeeze"] = float(r_st.get("squeezeValue", 1.0 if r_st.get("squeeze") else 0.0))
            self.right_ctrl["grip"] = self.right_ctrl["squeeze"] > 0.4 or bool(r_st.get("squeeze", False))
            self.right_ctrl["thumbstick"] = bool(r_st.get("thumbstick", False))

        elif msg_type in ("CAMERA_MOVE", "HEAD_MOVE"):
            head_data = data.get("camera", {}).get("matrix")
            if head_data and len(head_data) >= 16:
                self.head_pose = np.array(head_data[:16], dtype=np.float64).reshape((4, 4), order="F")
