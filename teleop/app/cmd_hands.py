"""
teleop/app/cmd_hands.py
Command dispatchers for hand postures, grasp presets, curls, direct joint jogs, and sweeps.
"""

import numpy as np
from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting


def handle_hand_cmd(app, name: str, cmd: dict) -> bool:
    """Processes hand-related control panel commands. Returns True if handled."""
    if name in ("hand_grasp", "hand_preset"):
        side = cmd.get("side", "both")
        grasp = cmd.get("grasp") or cmd.get("pose") or "cylinder"
        closure = float(cmd.get("closure", 1.0))
        if side in ("left", "both"):
            app.grasp_type_l = grasp
            app.hand_closure_l = closure
            app.target_curls_l, app.target_pinch_l = AgibotHandRetargeting.get_grasp_curls(grasp, closure)
            app.direct_hand_override_l = False
        if side in ("right", "both"):
            app.grasp_type_r = grasp
            app.hand_closure_r = closure
            app.target_curls_r, app.target_pinch_r = AgibotHandRetargeting.get_grasp_curls(grasp, closure)
            app.direct_hand_override_r = False
        app.hand_sweep_active = False
        return True

    elif name == "hand_curls":
        side = cmd.get("side", "left")
        curls = np.asarray(cmd.get("curls", [0, 0, 0, 0, 0]), dtype=np.float64)
        pinch = float(cmd.get("pinch", 0.0))
        if side in ("left", "both"):
            app.target_curls_l = curls.copy()
            app.target_pinch_l = pinch
            app.hand_closure_l = float(np.mean(curls))
            app.direct_hand_override_l = False
        if side in ("right", "both"):
            app.target_curls_r = curls.copy()
            app.target_pinch_r = pinch
            app.hand_closure_r = float(np.mean(curls))
            app.direct_hand_override_r = False
        app.hand_sweep_active = False
        return True

    elif name == "hand_joint":
        side = cmd.get("side", "left")
        jname = cmd.get("joint", "")
        angle = float(cmd.get("angle", 0.0))
        if side == "left" and jname in app.env.left_hand_active_joints:
            idx = app.env.left_hand_active_joints.index(jname)
            lo, hi = app.env.joint_limits[jname]
            if not getattr(app, "direct_hand_override_l", False):
                app.direct_hand_target_l = app.last_hand_cmd_left.copy()
            app.direct_hand_target_l[idx] = np.clip(angle, lo, hi)
            app.direct_hand_override_l = True
        elif side == "right" and jname in app.env.right_hand_active_joints:
            idx = app.env.right_hand_active_joints.index(jname)
            lo, hi = app.env.joint_limits[jname]
            if not getattr(app, "direct_hand_override_r", False):
                app.direct_hand_target_r = app.last_hand_cmd_right.copy()
            app.direct_hand_target_r[idx] = np.clip(angle, lo, hi)
            app.direct_hand_override_r = True
        app.hand_sweep_active = False
        return True

    elif name == "hand_sweep":
        mode = cmd.get("mode", "single_joints")
        side = cmd.get("side", "both")
        app.hand_sweep_active = True
        app.hand_sweep_mode = mode
        app.hand_sweep_side = side
        app.hand_sweep_time = 0.0
        app.hand_sweep_joint_idx = 0
        app.hand_sweep_saved_joints_l = app.last_hand_cmd_left.copy()
        app.hand_sweep_saved_joints_r = app.last_hand_cmd_right.copy()
        app.hand_sweep_info = f"Starting {mode} sweep on {side}"
        return True

    elif name == "hand_stop":
        app.hand_sweep_active = False
        app.direct_hand_override_l = False
        app.direct_hand_override_r = False
        app.hand_sweep_info = "Hand motion stopped"
        return True

    return False

