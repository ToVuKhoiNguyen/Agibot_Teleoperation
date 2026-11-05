"""
teleop/app/cmd_system.py
Handles core system commands including E-stop, hold toggle, walk demo, and arm jogging.
"""

import time
import numpy as np
from scipy.spatial.transform import Rotation as R


def handle_system_cmd(app, name: str, cmd: dict) -> bool:
    """Processes system lifecycle commands. Returns True if command was handled."""
    if name == "estop":
        app.state_estop = True
        print("[ALERT] E-STOP ACTIVE")
        return True
    elif name == "clear_estop":
        app.state_estop = False
        print("[INFO] E-STOP CLEARED")
        return True
    elif name == "clear_fault":
        confirmed = bool(cmd.get("confirmed", False))
        if hasattr(app, "robot_safety") and app.robot_safety is not None:
            if app.robot_safety.clear_fault(confirm=confirmed):
                app.status_warning = ""
                print("[INFO] HARDWARE FAULT CLEARED")
            else:
                app.status_warning = "Clear fault requires explicit confirmation."
        else:
            app.status_warning = ""
        return True
    elif name == "hold_toggle":
        app.manual_held = not app.manual_held
        return True
    elif name in ("walk_toggle", "walk_demo"):
        app.walking_demo_active = not app.walking_demo_active
        if not app.walking_demo_active:
            if hasattr(app.env, "non_manip_qpos_indices"):
                app.env.data.qpos[app.env.non_manip_qpos_indices] = 0.0
        else:
            app.walk_start_time = time.time()
        return True

    if name in ("arm_jog", "jog_l", "jog_r"):
        if app.state_estop:
            app.status_warning = "Command ignored: E-STOP is active."
            return True

        arm = cmd.get("arm", "left" if name == "jog_l" else "right")
        axis_val = cmd.get("axis", 0)
        if isinstance(axis_val, str):
            axis_val = axis_val.lower()
            if axis_val in ("x", "0"):
                axis = 0
            elif axis_val in ("y", "1"):
                axis = 1
            elif axis_val in ("z", "2"):
                axis = 2
            elif axis_val in ("roll", "pitch", "yaw"):
                r_axis = 0 if axis_val == "roll" else (1 if axis_val == "pitch" else 2)
                direction = float(cmd.get("dir", 1.0))
                step_deg = float(cmd.get("step_deg", 5.0))
                rot_vec = np.zeros(3)
                rot_vec[r_axis] = np.radians(step_deg) * direction
                dR = R.from_rotvec(rot_vec).as_matrix()
                if arm == "left":
                    app.cur_target_rot_l = dR @ app.cur_target_rot_l
                    app._solve_arm_ik("left")
                else:
                    app.cur_target_rot_r = dR @ app.cur_target_rot_r
                    app._solve_arm_ik("right")
                return True
            else:
                axis = 0
        else:
            axis = int(axis_val)

        direction = float(cmd.get("dir", 1.0))
        if "step_cm" in cmd:
            step = float(cmd.get("step_cm", 2.0)) / 100.0 * direction
        else:
            step = float(cmd.get("step", 0.02)) * (direction if "dir" in cmd else 1.0)

        if arm == "left":
            new_t = app.cur_target_pos_l.copy()
            new_t[axis] += step
            new_t[0] = np.clip(new_t[0], -0.2, 0.9)
            new_t[1] = np.clip(new_t[1], -0.9, 0.9)
            new_t[2] = np.clip(new_t[2], 0.5, 1.8)
            app.cur_target_pos_l = new_t
            app.gizmo_active_l = True
            app._solve_arm_ik("left")
        else:
            new_t = app.cur_target_pos_r.copy()
            new_t[axis] += step
            new_t[0] = np.clip(new_t[0], -0.2, 0.9)
            new_t[1] = np.clip(new_t[1], -0.9, 0.9)
            new_t[2] = np.clip(new_t[2], 0.5, 1.8)
            app.cur_target_pos_r = new_t
            app.gizmo_active_r = True
            app._solve_arm_ik("right")
        return True

    return False

