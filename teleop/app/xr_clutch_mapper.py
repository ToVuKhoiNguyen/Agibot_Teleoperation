"""
teleop/app/xr_clutch_mapper.py
4-state smart clutch state machine (LOCKED, TRANSLATE, ROTATE, 6D) and SE(3) target integration.
"""

import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from teleop.kinematics.reach_projection import SHOULDER_ORIGIN_LEFT, SHOULDER_ORIGIN_RIGHT, DEFAULT_MAX_REACH_M, project_to_reachable_workspace


def get_effective_axis_map(app) -> np.ndarray:
    rig_yaw = float(getattr(app, "xr_rig_yaw", 0.0))
    if abs(rig_yaw) < 1e-4:
        return app.xr_axis_map
    c, s = np.cos(-rig_yaw), np.sin(-rig_yaw)
    Ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)
    return app.xr_axis_map @ Ry


def get_effective_rot_axis_map(app) -> np.ndarray:
    rot_map = getattr(app, "xr_rot_axis_map", np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64))
    rig_yaw = float(getattr(app, "xr_rig_yaw", 0.0))
    if abs(rig_yaw) < 1e-4:
        return rot_map
    c, s = np.cos(-rig_yaw), np.sin(-rig_yaw)
    Ry = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)
    return rot_map @ Ry


AXLOCK_MIN_SPEED_M = 0.0004   # m per packet (~3 cm/s at 72 Hz): below this no axis is (re)selected
AXLOCK_ALPHA = 0.10           # velocity smoothing per packet (~140 ms at 72 Hz)
# Soft lock: snap to one axis only when the hand moves almost straight along it; a diagonal move is followed in 3D.
# (Hard lock, robot 16/11: every diagonal move was cut to one axis and the rest DROPPED, so the arm went somewhere
# else than the hand - 30 axis switches in 93 s.)
# 17/11 (operator: lock too tight for fine work) - looser defaults, same mechanism; all tunable by env:
#   AXLOCK_SNAP_DEG 20 -> 12, AXLOCK_RELEASE_DEG 30 -> 20: the lock engages only on a really straight move
#   AXLOCK_LEAK 0 -> 0.3: while locked, 30 % of the off-axis hand motion still passes (was dropped completely)
#   AXLOCK_SLOW_FREE_S 0.25: hand slower than the selection threshold for this long -> free 3D (fine work is slow;
#     before, a lock picked during a fast move stayed on through every slow correction afterwards). 0 = off.
#   Rotation: a slow rotation (below the selection speed) is now followed freely instead of being ignored.
AXLOCK_SNAP_COS = float(np.cos(np.radians(float(os.environ.get("AXLOCK_SNAP_DEG", "12")))))      # enter the lock
AXLOCK_RELEASE_COS = float(np.cos(np.radians(float(os.environ.get("AXLOCK_RELEASE_DEG", "20")))))  # leave it (hysteresis)
AXLOCK_LEAK = float(np.clip(float(os.environ.get("AXLOCK_LEAK", "0.3")), 0.0, 1.0))
AXLOCK_SLOW_FREE_S = max(0.0, float(os.environ.get("AXLOCK_SLOW_FREE_S", "0.25")))
AXLOCK_FREE = -1                                      # lock state value meaning "free 3D motion"


def _locked_part(vec: np.ndarray, axis: int) -> np.ndarray:
    """Locked on `axis`: that component in full, the others scaled by AXLOCK_LEAK."""
    out = AXLOCK_LEAK * np.asarray(vec, dtype=np.float64)
    out[axis] = vec[axis]
    return out


def _slow_release(app, key: str, sp: float, min_speed: float, cur, dt: float = 1.0 / 72.0):
    """A lock held while the hand stays slow for AXLOCK_SLOW_FREE_S -> free 3D (fine corrections are slow)."""
    if AXLOCK_SLOW_FREE_S <= 0.0 or cur is None or cur < 0: setattr(app, key, 0.0); return cur
    t = getattr(app, key, 0.0) + dt if sp <= min_speed else 0.0
    setattr(app, key, t)
    return AXLOCK_FREE if t >= AXLOCK_SLOW_FREE_S else cur


def _soft_lock_axis(vel: np.ndarray, cur, min_speed: float, undecided_free: bool = False):
    """-> new lock state: None (undecided), AXLOCK_FREE, or the locked axis 0/1/2.
    undecided_free (translation): while the hand is still too slow to pick an axis, follow it in free 3D instead of
    holding the target (a slow fine move was ignored until the smoothed speed reached the threshold: the target started
    56-400 ms after the hand; a 2 cm move at 2 cm/s did not move it at all)."""
    sp = float(np.linalg.norm(vel))
    if sp <= min_speed: return cur if (cur is not None or not undecided_free) else AXLOCK_FREE
    if cur is not None and cur >= 0 and abs(vel[cur]) >= AXLOCK_RELEASE_COS * sp: return cur
    best = int(np.argmax(np.abs(vel)))
    return best if abs(vel[best]) >= AXLOCK_SNAP_COS * sp else AXLOCK_FREE


def _lock_name(a):
    return "free 3D" if a == AXLOCK_FREE else "XYZ"[a]


SOFT_REACH_START = 0.80      # fraction of the max reach where the controller->robot ratio starts to shrink
SOFT_REACH_SUBSTEP_M = 0.004


def _soft_reach_step(app, s: str, side: str, pos: np.ndarray, inc: np.ndarray) -> np.ndarray:
    """Progressive scaling near the reach limit. Inside SOFT_REACH_START x reach the target moves 1:1. Further out,
    the OUTWARD (radial) part of a hand move is scaled by a gain that falls linearly to 0 at the reach sphere, so a
    hand stretched past the robot's reach still steers the arm instead of hitting a wall. Sideways (tangential)
    motion stays 1:1. Coming back, the compressed excursion is retraced at its average ratio (hand back at its
    start -> robot back at its start: no dead zone and no drift from hand jitter); past that point it is 1:1 again."""
    sh = SHOULDER_ORIGIN_LEFT if side == "left" else SHOULDER_ORIGIN_RIGHT
    r_max, r0 = DEFAULT_MAX_REACH_M, SOFT_REACH_START * DEFAULT_MAX_REACH_M
    h, x = getattr(app, f"xr_soft_h_{s}", 0.0), getattr(app, f"xr_soft_x_{s}", 0.0)   # hand / robot outward travel
    n = max(1, int(np.ceil(float(np.linalg.norm(inc)) / SOFT_REACH_SUBSTEP_M)))
    d, gain = inc / n, 1.0
    for _ in range(n):
        v = pos - sh
        r = float(np.linalg.norm(v))
        if r < 1e-6:
            pos = pos + d
            continue
        u = v / r
        dr = float(d @ u)
        tang = d - dr * u
        if dr > 0.0:
            gain = 1.0 if r <= r0 else float(np.clip((r_max - r) / (r_max - r0), 0.0, 1.0))
            if gain < 1.0 or h > 0.0:
                h, x = h + dr, x + gain * dr
            dr_out = gain * dr
        elif h > 1e-9:
            take = min(-dr, h)
            ratio = x / h
            dr_out = -(ratio * take) - (-dr - take)
            h, x = h - take, x - ratio * take
            if h <= 1e-9: h, x = 0.0, 0.0
            gain = ratio
        else:
            dr_out, gain = dr, 1.0
        pos = pos + tang + dr_out * u
    setattr(app, f"xr_soft_h_{s}", h); setattr(app, f"xr_soft_x_{s}", x); setattr(app, f"xr_soft_gain_{s}", gain)
    return pos


def _incremental_target(app, s: str, side: str, p_ctrl: np.ndarray, axis_map: np.ndarray, scale: float, lock: bool) -> np.ndarray:
    """Controller -> target, accumulated packet by packet.
    lock=True (soft axis lock): while the hand moves almost straight along one robot axis (AXLOCK_SNAP_DEG, released past
    AXLOCK_RELEASE_DEG) the target follows that axis fully and the off-axis wobble only at AXLOCK_LEAK; a diagonal or
    slow (fine) move is followed in free 3D. Incremental, so switching never jumps.
    lock=False: free 3D motion. Both pass through the progressive reach scaling (_soft_reach_step)."""
    prev = getattr(app, f"xr_axlock_prev_ctrl_{s}", None)
    acc = getattr(app, f"xr_axlock_pos_{s}", None)
    if prev is None or acc is None:
        setattr(app, f"xr_axlock_prev_ctrl_{s}", p_ctrl.copy())
        return np.array(getattr(app, f"cur_target_pos_{s}"), dtype=np.float64).copy()
    inc = scale * (axis_map @ (p_ctrl - prev))
    setattr(app, f"xr_axlock_prev_ctrl_{s}", p_ctrl.copy())
    if lock:
        vel = (1.0 - AXLOCK_ALPHA) * getattr(app, f"xr_axlock_vel_{s}") + AXLOCK_ALPHA * inc
        setattr(app, f"xr_axlock_vel_{s}", vel)
        prev_lock = getattr(app, f"xr_axlock_axis_{s}")
        cur = _soft_lock_axis(vel, prev_lock, AXLOCK_MIN_SPEED_M, undecided_free=True)
        cur = _slow_release(app, f"xr_axlock_slow_{s}", float(np.linalg.norm(vel)), AXLOCK_MIN_SPEED_M, cur)
        if cur != prev_lock:
            app._log_xr_event(f"{s.upper()} axis lock -> {_lock_name(cur)}")
            setattr(app, f"xr_axlock_axis_{s}", cur)
        step = np.zeros(3)
        pend = getattr(app, f"xr_axlock_pend_{s}", None)
        if cur is None:
            # not decided yet (first packets of a move): remember the motion instead of dropping it
            setattr(app, f"xr_axlock_pend_{s}", inc if pend is None else pend + inc)
        else:
            step = inc.copy() if cur == AXLOCK_FREE else _locked_part(inc, cur)
            if pend is not None:                  # give back what was moved before the decision
                step += pend if cur == AXLOCK_FREE else _locked_part(pend, cur)
                setattr(app, f"xr_axlock_pend_{s}", None)
    else:
        step = inc
    acc = _soft_reach_step(app, s, side, acc.copy(), step)
    acc[0] = np.clip(acc[0], -0.2, 0.9); acc[1] = np.clip(acc[1], -0.9, 0.9); acc[2] = np.clip(acc[2], 0.5, 1.8)
    setattr(app, f"xr_axlock_pos_{s}", acc)
    return acc.copy()


AXLOCK_ROT_MIN_RAD = 0.0025   # rad per packet (~10 deg/s at 72 Hz): below this no rotation axis is (re)selected


def _axis_locked_rot(app, s: str, r_ctrl: np.ndarray, rot_axis_map: np.ndarray, scale: float = 1.0) -> np.ndarray:
    """Soft rotation axis lock: while the controller turns almost purely about one robot axis (roll X, pitch Y or yaw Z)
    the target turns about that axis only; a mixed rotation is followed freely. Incremental, so switching never jumps."""
    prev = getattr(app, f"xr_axlock_prev_rot_{s}", None)
    acc = getattr(app, f"xr_axlock_rot_{s}", None)
    if prev is None or acc is None:
        setattr(app, f"xr_axlock_prev_rot_{s}", r_ctrl.copy())
        return np.array(getattr(app, f"cur_target_rot_{s}"), dtype=np.float64).copy()
    r_inc_xr = r_ctrl @ prev.T
    setattr(app, f"xr_axlock_prev_rot_{s}", r_ctrl.copy())
    w = scale * R.from_matrix(rot_axis_map @ r_inc_xr @ rot_axis_map.T).as_rotvec()
    vel = (1.0 - AXLOCK_ALPHA) * getattr(app, f"xr_axlock_wvel_{s}") + AXLOCK_ALPHA * w
    setattr(app, f"xr_axlock_wvel_{s}", vel)
    prev_lock = getattr(app, f"xr_axlock_raxis_{s}")
    cur = _soft_lock_axis(vel, prev_lock, AXLOCK_ROT_MIN_RAD, undecided_free=True)   # slow fine rotation: followed
    cur = _slow_release(app, f"xr_axlock_rslow_{s}", float(np.linalg.norm(vel)), AXLOCK_ROT_MIN_RAD, cur)
    if cur != prev_lock:
        app._log_xr_event(f"{s.upper()} rotation axis lock -> {_lock_name(cur)}")
        setattr(app, f"xr_axlock_raxis_{s}", cur)
    if cur is not None:
        rv = w.copy() if cur == AXLOCK_FREE else _locked_part(w, cur)
        acc = R.from_rotvec(rv).as_matrix() @ acc   # world-frame increment, same convention as the free mapping
        setattr(app, f"xr_axlock_rot_{s}", acc)
    return acc.copy()


def _process_arm_clutch(app, side: str, pkt: dict, rot_enabled: bool, axis_map: np.ndarray, rot_axis_map: np.ndarray):
    """Processes relative clutch state machine and SE(3) targets for a single arm."""
    s = side[0]  # 'l' or 'r'
    present = pkt is not None
    loss = present and (pkt.get("pos") is None or pkt.get("tracking") is False or pkt.get("tracking_loss") is True)

    if loss:
        clutch_state = getattr(app, f"xr_clutch_state_{s}")
        # Only release what the controller itself was driving: gizmo_active may belong to a mouse drag on the
        # panel, and clearing it on every idle/lost controller packet (72 Hz) stops the control loop from
        # solving IK between mouse updates (robot crawls, then jumps to the target on release).
        if clutch_state != "LOCKED" or getattr(app, f"xr_grip_active_{s}", False):
            setattr(app, f"xr_clutch_state_{s}", "LOCKED")
            setattr(app, f"xr_anchor_ctrl_pos_{s}", None)
            setattr(app, f"xr_anchor_ctrl_rot_{s}", None)
            setattr(app, f"gizmo_active_{s}", False)
            setattr(app, f"xr_grip_active_{s}", False)
            setattr(app, f"xr_reengage_required_{s}", True)
            setattr(app, f"xr_trig_down_{s}", False)
            setattr(app, f"xr_grip_down_{s}", False)
            app._log_xr_event(f"{side.upper()} tracking lost -> clutch released & target held")
        if not getattr(app, f"xr_tracking_lost_{s}", False):
            app.tracking_loss_events = getattr(app, "tracking_loss_events", 0) + 1
        setattr(app, f"xr_tracking_lost_{s}", True)
        return

    if not present or pkt.get("pos") is None:
        return

    # Trigger & Grip detection with hysteresis (Trigger: rise 0.5, fall 0.4; Grip: rise 0.5, fall 0.4 or bool)
    pkt_grip_raw = pkt.get("grip")
    pkt_trig_raw = pkt.get("trigger")
    t_val = float(pkt_trig_raw) if pkt_trig_raw is not None else getattr(app, f"xr_trig_val_{s}", 0.0)
    g_val = float(pkt_grip_raw) if pkt_grip_raw is not None else getattr(app, f"xr_grip_val_{s}", 0.0)
    cur_trig_down = getattr(app, f"xr_trig_down_{s}", False)
    cur_grip_down = getattr(app, f"xr_grip_down_{s}", False)

    # If pkt explicitly gives boolean grip/trigger, honor them as fallbacks
    pkt_grip = bool(pkt.get("grip", False))
    pkt_trig = bool(pkt.get("trigger_pressed", False) or (t_val >= 0.5))

    new_trig_down = (t_val >= 0.5 or pkt_trig) if not cur_trig_down else (t_val >= 0.4 or pkt_trig)
    new_grip_down = (g_val >= 0.5 or pkt_grip) if not cur_grip_down else (g_val >= 0.4 or pkt_grip)
    setattr(app, f"xr_trig_down_{s}", new_trig_down)
    setattr(app, f"xr_grip_down_{s}", new_grip_down)

    if getattr(app, f"xr_reengage_required_{s}", False):
        if not new_trig_down and not new_grip_down:
            setattr(app, f"xr_reengage_required_{s}", False)
            setattr(app, f"xr_tracking_lost_{s}", False)
            app._log_xr_event(f"{side.upper()} tracking restored & clutch re-armed")
        else:
            setattr(app, f"xr_clutch_state_{s}", "LOCKED")
            setattr(app, f"xr_anchor_ctrl_pos_{s}", None)
            setattr(app, f"xr_anchor_ctrl_rot_{s}", None)
            if getattr(app, f"xr_grip_active_{s}", False):
                setattr(app, f"gizmo_active_{s}", False)
            setattr(app, f"xr_grip_active_{s}", False)
            return

    p_ctrl = np.array(pkt["pos"], dtype=np.float64)
    q_ctrl = pkt.get("quat")
    setattr(app, f"xr_ctrl_pos_{s}", p_ctrl)   # raw controller position, for the DIAG trace
    setattr(app, f"xr_ctrl_quat_{s}", list(q_ctrl) if (q_ctrl is not None and len(q_ctrl) == 4) else None)

    # Evaluate 4-state clutch mode
    if new_grip_down and new_trig_down:
        next_state = "6D"
    elif new_trig_down and not new_grip_down:
        next_state = "TRANSLATE"
    elif new_grip_down and not new_trig_down:
        next_state = "ROTATE"
    else:
        next_state = "LOCKED"

    prev_state = getattr(app, f"xr_clutch_state_{s}")
    setattr(app, f"xr_clutch_state_{s}", next_state)

    # Independent channel re-anchoring when newly engaged or transitioning
    newly_translating = next_state in ("TRANSLATE", "6D") and prev_state not in ("TRANSLATE", "6D")
    newly_rotating = next_state in ("ROTATE", "6D") and prev_state not in ("ROTATE", "6D")

    # Read current EE pose from sim or current target as base anchor T0
    ee_curr = app.env.get_ee_poses()[side] if (hasattr(app, "env") and hasattr(app.env, "get_ee_poses")) else None
    curr_target_pos = getattr(app, f"cur_target_pos_{s}")
    curr_target_rot = getattr(app, f"cur_target_rot_{s}")

    if newly_translating:
        setattr(app, f"xr_anchor_ctrl_pos_{s}", p_ctrl.copy())
        # T0 = current commanded target pose (holds last commanded target without jump)
        # ...projected into the reachable workspace: anchoring on an out-of-reach target (e.g. a mouse/nudge target
        # past the arm's reach) would swallow the first part of every hand motion -> robot stands still.
        p_t0 = project_to_reachable_workspace(np.asarray(curr_target_pos, dtype=np.float64), side=side)[0].copy()
        setattr(app, f"cur_target_pos_{s}", p_t0.copy())
        setattr(app, f"xr_anchor_marker_pos_{s}", p_t0)
        setattr(app, f"gizmo_active_{s}", True)
        # axis-lock state (main.py --axis-lock)
        setattr(app, f"xr_axlock_prev_ctrl_{s}", p_ctrl.copy())
        setattr(app, f"xr_axlock_pos_{s}", p_t0.copy())
        setattr(app, f"xr_axlock_vel_{s}", np.zeros(3))
        setattr(app, f"xr_axlock_axis_{s}", None)
        setattr(app, f"xr_soft_h_{s}", 0.0); setattr(app, f"xr_soft_x_{s}", 0.0); setattr(app, f"xr_soft_gain_{s}", 1.0)
        setattr(app, f"xr_axlock_pend_{s}", None)
        setattr(app, f"xr_axlock_slow_{s}", 0.0)
        # trigger alone = translation only: wrist kept straight (ik_dispatcher._soft_orientation); with the grip
        # (6D) the orientation is commanded and held exactly
        setattr(app, f"soft_ori_{s}", next_state == "TRANSLATE")
        app._log_xr_event(f"{side.upper()} clutch translation re-anchored")

    if newly_rotating:
        setattr(app, f"soft_ori_{s}", False)      # an explicit rotation: hold it exactly from here on
        if q_ctrl is not None and len(q_ctrl) == 4:
            r_c0 = R.from_quat([q_ctrl[1], q_ctrl[2], q_ctrl[3], q_ctrl[0]]).as_matrix()
            setattr(app, f"xr_anchor_ctrl_rot_{s}", r_c0)
            # Anchor on the orientation the hand ACTUALLY has: an earlier target can be outside the
            # wrist limits, which would swallow the first part of every rotation (not 1:1).
            r_t0 = ee_curr["rot"].copy() if ee_curr is not None else curr_target_rot.copy()
            setattr(app, f"cur_target_rot_{s}", r_t0.copy())
            rot_follower = getattr(app, f"gizmo_rot_follower_{s}", None)
            if rot_follower is not None:
                rot_follower.reset(r_t0)
            setattr(app, f"xr_anchor_marker_rot_{s}", r_t0)
            # rotation axis-lock state (main.py --axis-lock)
            setattr(app, f"xr_axlock_prev_rot_{s}", r_c0.copy())
            setattr(app, f"xr_axlock_rot_{s}", r_t0.copy())
            setattr(app, f"xr_axlock_wvel_{s}", np.zeros(3))
            setattr(app, f"xr_axlock_raxis_{s}", None)
            setattr(app, f"xr_axlock_rslow_{s}", 0.0)
            setattr(app, f"gizmo_active_{s}", True)
            app._log_xr_event(f"{side.upper()} clutch rotation re-anchored")

    if next_state == "LOCKED":
        was_active = getattr(app, f"xr_anchor_ctrl_pos_{s}") is not None or getattr(app, f"xr_anchor_ctrl_rot_{s}") is not None
        setattr(app, f"xr_anchor_ctrl_pos_{s}", None)
        setattr(app, f"xr_anchor_ctrl_rot_{s}", None)
        if was_active:   # idle controller packets must not cancel a mouse gizmo drag on the same arm
            setattr(app, f"gizmo_active_{s}", False)
            from teleop.app.ik_dispatcher import freeze_arm_target, xr_coast_start
            if not xr_coast_start(app, side):     # still catching up: keep the straight-line lead until it arrives
                freeze_arm_target(app, side, from_xr=True)

    # While held: apply relative displacement
    anchor_ctrl_pos = getattr(app, f"xr_anchor_ctrl_pos_{s}")
    anchor_marker_pos = getattr(app, f"xr_anchor_marker_pos_{s}")
    if next_state in ("TRANSLATE", "6D") and anchor_ctrl_pos is not None and anchor_marker_pos is not None:
        delta_ctrl_xr = p_ctrl - anchor_ctrl_pos
        delta_rob = axis_map @ delta_ctrl_xr
        eff_scale = app.xr_scale * (app.xr_precision_scale_factor if getattr(app, f"precision_{s}") else 1.0)
        new_pos = _incremental_target(app, s, side, p_ctrl, axis_map, eff_scale, bool(getattr(app, "xr_axis_lock", False)))

        # Safety net: the progressive scaling keeps the target inside the reach sphere; if it still ends up outside
        # (sideways motion along the boundary), clamp it and continue from the clamped point.
        clamped_pos, is_clamped, d_beyond = project_to_reachable_workspace(new_pos, side=side)
        if is_clamped:
            new_pos = clamped_pos
            setattr(app, f"xr_axlock_pos_{s}", new_pos.copy())
        g_reach = float(getattr(app, f"xr_soft_gain_{s}", 1.0))
        setattr(app, f"reach_warning_{s}", f"Near reach limit: controller scale x{g_reach:.2f}" if g_reach < 0.5 else "")

        setattr(app, f"cur_target_pos_{s}", new_pos)
        setattr(app, f"gizmo_active_{s}", True)
        if hasattr(app, "target_tracer"):
            app.target_tracer.record_target_source(side, "controller clutch")

    anchor_ctrl_rot = getattr(app, f"xr_anchor_ctrl_rot_{s}")
    anchor_marker_rot = getattr(app, f"xr_anchor_marker_rot_{s}")
    if next_state in ("ROTATE", "6D") and rot_enabled and q_ctrl is not None and len(q_ctrl) == 4 and anchor_ctrl_rot is not None and anchor_marker_rot is not None:
        r_curr_ctrl = R.from_quat([q_ctrl[1], q_ctrl[2], q_ctrl[3], q_ctrl[0]]).as_matrix()
        if getattr(app, "xr_axis_lock", False):
            setattr(app, f"cur_target_rot_{s}", _axis_locked_rot(app, s, r_curr_ctrl, rot_axis_map))
        else:
            r_rel_xr = r_curr_ctrl @ anchor_ctrl_rot.T
            r_rel_rob = rot_axis_map @ r_rel_xr @ rot_axis_map.T
            setattr(app, f"cur_target_rot_{s}", r_rel_rob @ anchor_marker_rot)
        setattr(app, f"gizmo_active_{s}", True)
        if hasattr(app, "target_tracer"):
            app.target_tracer.record_target_source(side, "controller clutch")

    setattr(app, f"xr_grip_active_{s}", bool(next_state != "LOCKED"))


def update_xr_clutch(app, left_pkt: dict, right_pkt: dict, rot_enabled: bool):
    """Updates 4-state clutch machine and SE(3) targets for both arms."""
    axis_map = get_effective_axis_map(app)
    # Controllers: rotations use the SAME frame map as translations (operator "inside" the robot):
    # tip up -> robot hand tip up, roll right -> robot hand rolls right, yaw left -> yaw left.
    rot_axis_map = axis_map

    _process_arm_clutch(app, "left", left_pkt, rot_enabled, axis_map, rot_axis_map)
    _process_arm_clutch(app, "right", right_pkt, rot_enabled, axis_map, rot_axis_map)

    if app.xr_grip_active_l and app.xr_grip_active_r:
        app._solve_arm_ik("both")
    elif app.xr_grip_active_l:
        app._solve_arm_ik("left")
    elif app.xr_grip_active_r:
        app._solve_arm_ik("right")
