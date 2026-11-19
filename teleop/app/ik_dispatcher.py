"""
teleop/app/ik_dispatcher.py
Orchestrates dual-arm inverse kinematics solving, follower updates, and collision safety gating.
"""

import os
import time
import numpy as np
from teleop.kinematics.reach_projection import project_to_reachable_workspace, reach_message


GIZMO_FOLLOWER_OMEGA = 15.0   # rad/s: mouse gizmo / hand tracking smoothing (ramp lag 2/omega = 133 ms)
XR_FOLLOWER_HZ = 30.0         # Quest controllers: 30 Hz (ramp lag ~10 ms), same base as update_precision()


def _set_follower_bandwidth(app, c: str):
    """Controllers are already smooth 72-90 Hz poses: the 2.4 Hz mouse-gizmo smoothing only added ~100 ms lag."""
    pos_f = getattr(app, f"gizmo_follower_{c}", None)
    rot_f = getattr(app, f"gizmo_rot_follower_{c}", None)
    if getattr(app, f"xr_grip_active_{c}", False):
        hz = XR_FOLLOWER_HZ * (float(getattr(app, "xr_precision_follower_factor", 0.5)) if getattr(app, f"precision_{c}", False) else 1.0)
        omega = 2.0 * np.pi * hz
    else:
        omega = GIZMO_FOLLOWER_OMEGA
    if pos_f is not None: pos_f.omega = omega
    if rot_f is not None: rot_f.bandwidth_hz = omega / (2.0 * np.pi)




XR_LEAD_POS_M = float(os.environ.get("XR_LEAD_POS_M", "0.10"))     # IK goal never further than this from the actual hand
XR_LEAD_ROT_RAD = float(os.environ.get("XR_LEAD_ROT_RAD", "0.35"))  # ... nor rotated more than this (~20 deg)
# The arm must not move by itself: while a controller owns an arm and its Cartesian target is not changing and the
# IK solution already sits on it, the solution is HELD instead of being re-solved. Re-solving every packet let the
# posture terms (elbow-out spring, soft orientation, rest pose) keep turning the joints with the hand standing still
# (real robot 16/11: J1 0.85 rad in 1.6 s and J5 0.26 rad in 1 s after a release, hand within 3 mm of its goal).
# Posture still adapts whenever the hand moves. XR_HOLD_STILL=0 restores the former behaviour.
XR_HOLD_STILL = os.environ.get("XR_HOLD_STILL", "1") != "0"
XR_HOLD_POS_M = 0.001       # target moved less than this since the last accepted solve ...
XR_HOLD_ROT = 0.004         # ... and turned less than this (Frobenius norm of the rotation difference, ~0.15 deg)
XR_HOLD_ERR_M = 0.006       # ... and the held solution is within this of the target
XR_FAR_CRUISE = os.environ.get("XR_FAR_CRUISE", "1") != "0"        # 0: brake for every waypoint (behaviour before 16/11)
XR_COAST_MAX_S = 4.0        # after a grip release: longest time IK keeps stepping toward the released target
XR_COAST = os.environ.get("XR_COAST", "1") != "0"   # XR_COAST=0: old behaviour (release = jump to the final target)
# After the input stops (mouse released, grip released) IK keeps being stepped for the frozen target until the arm
# posture stops changing: the hand stays where it is while the elbow finishes swinging to its preferred side
# (mink_ik elbow-out / rest-pose bias). Without this the posture froze wherever the last input packet left it
# (measured 13/11: elbow 1.5 cm from the body although the same hand pose settles at 10 cm). IK_SETTLE_S=0 disables.
IK_SETTLE_MAX_S = float(os.environ.get("IK_SETTLE_S", "3.0"))
IK_SETTLE_EPS_RAD = 2e-4       # per tick; below this for IK_SETTLE_STILL ticks = settled
IK_SETTLE_STILL = 40


def _plan_ee(app, side: str, fallback: dict):
    """Hand pose of the limiter PLAN (what is being commanded right now). The twin in app.env can show the
    measured / plant-lagged arm instead, and a lead measured from that pose would throttle the plan itself."""
    lim = getattr(app, "mirror_limiter", None)
    try:
        import mujoco
        d = getattr(app, "_plan_fk_data", None)
        if d is None:
            d = app._plan_fk_data = mujoco.MjData(app.env.model)
        d.qpos[:] = app.env.data.qpos
        d.qpos[app.left_arm_qadr] = lim.q_plan[:7]
        d.qpos[app.right_arm_qadr] = lim.q_plan[7:]
        mujoco.mj_kinematics(app.env.model, d)
        sid = app.env.left_ee_site_id if side == "left" else app.env.right_ee_site_id
        return d.site_xpos[sid].copy(), d.site_xmat[sid].reshape(3, 3).copy()
    except Exception:
        return fallback["pos"], fallback["rot"]


def _fk_ee_pos(app, side: str, q7) -> np.ndarray:
    """Hand position of an arbitrary 7-joint posture of one arm (None if the twin model is not available)."""
    try:
        import mujoco
        d = getattr(app, "_hold_fk_data", None)
        if d is None:
            d = app._hold_fk_data = mujoco.MjData(app.env.model)
        d.qpos[:] = app.env.data.qpos
        d.qpos[app.left_arm_qadr if side == "left" else app.right_arm_qadr] = np.asarray(q7, dtype=np.float64)
        mujoco.mj_kinematics(app.env.model, d)
        return d.site_xpos[app.env.left_ee_site_id if side == "left" else app.env.right_ee_site_id].copy()
    except Exception:
        return None


def _xr_hold(app, c: str, side: str, pos, rot, last_pos, last_rot, q_last) -> bool:
    """True = keep the last IK solution of this arm (see XR_HOLD_STILL)."""
    if not XR_HOLD_STILL or last_pos is None or last_rot is None: return False
    if not (getattr(app, f"xr_grip_active_{c}", False) or getattr(app, f"xr_coast_{c}", False)): return False
    if float(np.linalg.norm(np.asarray(pos) - last_pos)) > XR_HOLD_POS_M: return False
    if float(np.linalg.norm(np.asarray(rot) - last_rot)) > XR_HOLD_ROT: return False
    ee = _fk_ee_pos(app, side, q_last)
    return ee is not None and float(np.linalg.norm(ee - np.asarray(pos))) <= XR_HOLD_ERR_M


def _xr_carrot(app, c: str, goal_pos: np.ndarray, goal_rot: np.ndarray, ee_pos: np.ndarray, ee_rot: np.ndarray):
    """While a Quest controller drives the arm, IK aims at a point at most a few cm ahead of the ACTUAL hand, on the
    straight line to the goal. When the robot cannot keep up, the hand then still travels along the commanded axis
    (hand up -> +z, hand forward -> +x, like the +-X/Y/Z nudge buttons) instead of cutting a joint-space chord that
    swings it sideways or behind the body. Inside the lead distance this changes nothing."""
    # Also after the grip is released, until the commanded hand has caught up with the released target ("coast",
    # see xr_coast_start): switching the lead off at release made the IK goal jump to the final target in one tick
    # (measured 6-21 cm, joints 0.2-0.5 rad) and back at the next squeeze, i.e. a twitch at every press / release.
    lim = getattr(app, "mirror_limiter", None)
    arm_i = 0 if c == "l" else 1
    if not (getattr(app, f"xr_grip_active_{c}", False) or getattr(app, f"xr_coast_{c}", False)):
        if hasattr(lim, "set_far"): lim.set_far(arm_i, False)
        return goal_pos, goal_rot
    # Only with the pass-through limiter (--limiter direct): the braking-curve law slows down near its target,
    # so a goal held a few cm ahead made it lag more (measured 455 -> 595 ms) for little straightness gain.
    if getattr(lim, "law", "brake") != "direct":
        return goal_pos, goal_rot
    d = np.asarray(goal_pos, dtype=np.float64) - ee_pos
    dist = float(np.linalg.norm(d))
    # The goal is further than the lead: the IK target is only a waypoint, the plan cruises through it (no braking)
    if hasattr(lim, "set_far"): lim.set_far(arm_i, XR_FAR_CRUISE and dist > XR_LEAD_POS_M)
    pos = ee_pos + d * (XR_LEAD_POS_M / dist) if dist > XR_LEAD_POS_M else goal_pos
    rot = goal_rot
    from scipy.spatial.transform import Rotation as _R
    rv = _R.from_matrix(np.asarray(goal_rot) @ np.asarray(ee_rot).T).as_rotvec()
    ang = float(np.linalg.norm(rv))
    if ang > XR_LEAD_ROT_RAD:
        rot = _R.from_rotvec(rv * (XR_LEAD_ROT_RAD / ang)).as_matrix() @ np.asarray(ee_rot)
    return pos, rot


def _within_lead(goal_pos, goal_rot, ee_pos, ee_rot) -> bool:
    if float(np.linalg.norm(np.asarray(goal_pos, dtype=np.float64) - ee_pos)) > XR_LEAD_POS_M: return False
    c = (np.trace(np.asarray(goal_rot) @ np.asarray(ee_rot).T) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0))) <= XR_LEAD_ROT_RAD


def xr_coast_start(app, side: str) -> bool:
    """Grip released while the commanded hand is still behind the target. True = IK keeps being stepped every tick
    with the same short lead as while the grip was held, so the hand goes on along the same straight line to the
    released target and the joint target never jumps. False = nothing to catch up (or another limiter law): the
    caller freezes the target at once, as before."""
    c = side[0]
    if not XR_COAST or getattr(getattr(app, "mirror_limiter", None), "law", "brake") != "direct" or getattr(app, "manual_held", False):
        return False
    ee_pos, ee_rot = _plan_ee(app, side, app.env.get_ee_poses()[side])
    goal_pos, _, _ = project_to_reachable_workspace(getattr(app, f"cur_target_pos_{c}"), side=side)
    if _within_lead(goal_pos, getattr(app, f"cur_target_rot_{c}"), ee_pos, ee_rot):
        return False
    setattr(app, f"xr_coast_{c}", True); setattr(app, f"xr_coast_t0_{c}", time.monotonic()); setattr(app, f"xr_coast_n_{c}", 0)
    return True


def xr_coast_update(app):
    """Once per control tick, after IK: ends a coast when the commanded hand is within the lead distance of the
    released target (IK is then already solving for the target itself, so the final freeze is only a refinement),
    when it takes too long (blocked), or when something else took the arm over."""
    for side in ("left", "right"):
        c = side[0]
        if not getattr(app, f"xr_coast_{c}", False): continue
        if getattr(app, f"xr_grip_active_{c}", False) or getattr(app, f"gizmo_active_{c}", False):
            setattr(app, f"xr_coast_{c}", False); continue        # squeezed again or mouse drag: normal tracking goes on
        if getattr(app, "sync_state", "ENGAGED") != "ENGAGED" or not getattr(app, f"arm_engaged_{c}", True):
            setattr(app, f"xr_coast_{c}", False); continue        # not driving the robot: the sync logic owns the pose
        ee_pos, ee_rot = _plan_ee(app, side, app.env.get_ee_poses()[side])
        goal_pos, _, _ = project_to_reachable_workspace(getattr(app, f"cur_target_pos_{c}"), side=side)
        # translation-only steering (soft orientation) does not command the hand orientation: only the position counts,
        # otherwise the coast ran its full 4 s with the hand already on the goal and the posture kept creeping
        goal_rot = ee_rot if (XR_HOLD_STILL and getattr(app, f"soft_ori_{c}", False)) else getattr(app, f"cur_target_rot_{c}")
        n = getattr(app, f"xr_coast_n_{c}", 0) + 1 if _within_lead(goal_pos, goal_rot, ee_pos, ee_rot) else 0
        setattr(app, f"xr_coast_n_{c}", n)
        if n >= 3 or time.monotonic() - getattr(app, f"xr_coast_t0_{c}", 0.0) > XR_COAST_MAX_S:
            freeze_arm_target(app, side, from_xr=True)            # clears the coast flag


NUDGE_GLIDE = os.environ.get("NUDGE_GLIDE", "1") != "0"    # NUDGE_GLIDE=0: XYZ / rotation steps jump as before


try:      # without gliding / settling a step is solved exactly once: then the solver must finish within that call
    import teleop.kinematics.mink_ik as _mink_ik
    _mink_ik.SOFT_ORI_FULL_PASS = not (NUDGE_GLIDE and IK_SETTLE_MAX_S > 0.0)
except Exception:
    pass


def ik_glide_start(app, c: str):
    """Keep stepping IK every control tick for this arm (same mechanism as the posture settling after an input
    stops) while its Cartesian goal glides to a stepped target. Ends by itself when the solution is still."""
    setattr(app, f"ik_settle_{c}", True); setattr(app, f"_ik_settle_until_{c}", time.monotonic() + max(IK_SETTLE_MAX_S, 3.0))
    setattr(app, f"_ik_settle_q_{c}", None); setattr(app, f"_ik_settle_n_{c}", 0)


def ik_settle_update(app):
    """Once per control tick, after IK: ends the posture-settling phase of an arm (see IK_SETTLE_MAX_S)."""
    now = time.monotonic()
    for side in ("left", "right"):
        c = side[0]
        if not getattr(app, f"ik_settle_{c}", False): continue
        if getattr(app, f"gizmo_active_{c}", False) or getattr(app, f"xr_coast_{c}", False):
            setattr(app, f"ik_settle_{c}", False); continue            # steered again: normal tracking
        q = np.asarray(getattr(app, f"last_ik_sol_{side}", getattr(app, f"last_arm_cmd_{side}")), dtype=np.float64)
        prev = getattr(app, f"_ik_settle_q_{c}", None)
        still = getattr(app, f"_ik_settle_n_{c}", 0) + 1 if (prev is not None and float(np.max(np.abs(q - prev))) < IK_SETTLE_EPS_RAD) else 0
        setattr(app, f"_ik_settle_q_{c}", q.copy()); setattr(app, f"_ik_settle_n_{c}", still)
        engaged = getattr(app, "sync_state", "ENGAGED") == "ENGAGED" and getattr(app, f"arm_engaged_{c}", True)
        if still >= IK_SETTLE_STILL or now > getattr(app, f"_ik_settle_until_{c}", 0.0) or not engaged:
            setattr(app, f"ik_settle_{c}", False)


SOFT_ORI_ENABLED = os.environ.get("SOFT_ORI", "1") != "0"   # SOFT_ORI=0: always hold the hand orientation rigidly
# Also while a controller TRANSLATES the hand (trigger only): the two wrist joints keep their angles and the hand
# turns with the forearm, instead of the wrist working all the time to hold the hand's world orientation while the
# arm moves (the hand rocks about the wrist on the real robot). XR_SOFT_ORI=0: controllers hold orientation rigidly.
XR_SOFT_ORI = os.environ.get("XR_SOFT_ORI", "1") != "0"


# Trigger-only motion (TRANSLATE) used to keep the two wrist joints exactly where the last rotation left them: on the
# real robot 17/11 the wrist command sat at a joint stop (J6 +-0.30 / J7 +-0.20) for 45-64 % of the trigger-only
# time, i.e. the hand stayed bent until the operator rotated it back by hand. Now the wrist returns to straight
# (in line with the forearm) while the hand travels: WRIST_RETURN_RAD_PER_M of wrist angle per metre of target
# travel, at most WRIST_RETURN_MAX_RAD_S. Tied to the travel, so a hand standing still never moves its wrist and a
# fast swing straightens it sooner. Grip (ROTATE / 6D) still sets any orientation. WRIST_RETURN=0 disables.
WRIST_RETURN = os.environ.get("WRIST_RETURN", "1") != "0"
WRIST_RETURN_RAD_PER_M = float(os.environ.get("WRIST_RETURN_RAD_PER_M", "3.0"))    # 0.30 rad gone after 10 cm
WRIST_RETURN_MAX_RAD_S = float(os.environ.get("WRIST_RETURN_MAX_RAD_S", "1.0"))


def _wrist_return_step(app, c: str, soft: bool, pos) -> float:
    """Wrist angle (rad) by which this tick's soft-orientation solve may straighten the wrist (0 = keep it)."""
    pos = np.asarray(pos, dtype=np.float64)
    last = getattr(app, f"_wrist_ret_pos_{c}", None)
    setattr(app, f"_wrist_ret_pos_{c}", pos.copy())
    if not (WRIST_RETURN and soft and last is not None and getattr(app, f"xr_clutch_state_{c}", "") == "TRANSLATE"):
        return 0.0
    return min(WRIST_RETURN_RAD_PER_M * float(np.linalg.norm(pos - last)), WRIST_RETURN_MAX_RAD_S * float(app.control_dt))


def _soft_orientation(app, c: str, ik) -> bool:
    """Translation-only mouse inputs (XYZ nudge, gizmo translate drag): let the hand turn with the forearm so the
    wrist stays straight (mink_ik SOFT_ORI_COST). Never for controllers / hand tracking, which command orientation."""
    # A squeezed controller: only in pure TRANSLATE (trigger alone). ROTATE / 6D command the orientation itself.
    xr_ok = (not getattr(app, f"xr_grip_active_{c}", False)) or (XR_SOFT_ORI and getattr(app, f"xr_clutch_state_{c}", "") == "TRANSLATE")
    soft = (bool(getattr(app, f"soft_ori_{c}", False)) and hasattr(ik, "wrist_task") and SOFT_ORI_ENABLED
            and xr_ok and getattr(app, "input_mode", "") != "hands")
    if soft and not getattr(app, f"_soft_ori_prev_{c}", False):
        # entering soft mode: hold the wrist as it is now (straight at HOME, or the bend an explicit rotation
        # left) - translating must not add wrist bend, and must not undo a rotation the user asked for either
        q_now = getattr(app, "last_ik_sol_left" if c == "l" else "last_ik_sol_right", None)
        if q_now is not None: ik.set_wrist_reference(np.asarray(q_now)[5:7])
    setattr(app, f"_soft_ori_prev_{c}", soft)
    return soft


def solve_arm_ik(app, arm="both", posture_hint_l: np.ndarray = None, posture_hint_r: np.ndarray = None):
    """Solves IK for requested arms, applies critically damped filtering and self-collision gating."""
    if getattr(app, "manual_held", False):
        return  # Toggle Hold: no input source (gizmo, controllers, hand tracking) may move the arms
    ee_poses = app.env.get_ee_poses()
    is_hands = getattr(app, "input_mode", "") == "hands"
    use_cart = os.environ.get("CART_FOLLOWER", "1") != "0"
    _set_follower_bandwidth(app, "l")
    _set_follower_bandwidth(app, "r")

    if arm in ("left", "both"):
        vel_l = getattr(app, "handtrack_vel_l", None) if is_hands else None
        if use_cart:
            follow_pos_l = app.gizmo_follower_l.update(app.cur_target_pos_l, app.control_dt, target_vel=vel_l)
            follow_rot_l = app.gizmo_rot_follower_l.update(app.cur_target_rot_l, app.control_dt)
        else:
            follow_pos_l = np.array(app.cur_target_pos_l, dtype=np.float64).copy()
            follow_rot_l = np.array(app.cur_target_rot_l, dtype=np.float64).copy()

        reach_pos_l, clamped_l, d_beyond_l = project_to_reachable_workspace(follow_pos_l, side="left")
        app.reach_warning_l = reach_message(d_beyond_l) if clamped_l else ""
        reach_pos_l, follow_rot_l = _xr_carrot(app, "l", reach_pos_l, follow_rot_l, *_plan_ee(app, "left", ee_poses["left"]))
        # the lead point lies on the straight line hand -> target, which can cut through the zone too close to the shoulder
        reach_pos_l = project_to_reachable_workspace(reach_pos_l, side="left")[0]

        slid_pos_l, _ = app.collision_checker.project_target_tangent(
            ee_poses["left"]["pos"], reach_pos_l, app.env.data
        )
        warm_q_l = getattr(app, "last_ik_sol_left", app.env.get_left_arm_qpos())
        xr_l = bool(getattr(app, "xr_grip_active_l", False))
        # Controllers and mouse: no hint -> the IK's weighted rest-pose bias (natural elbow-down posture, see
        # mink_ik.REST_WEIGHTS). Hand tracking keeps the current-posture hint.
        hint_l = posture_hint_l if posture_hint_l is not None else (warm_q_l if is_hands else None)
        kwargs_l = {"elbow_hint": hint_l} if (hint_l is not None and hasattr(app.ik_left, "set_elbow_hint")) else {}
        soft_l = _soft_orientation(app, "l", app.ik_left) and hint_l is None
        if soft_l: kwargs_l["soft_orientation"] = True
        wstep_l = _wrist_return_step(app, "l", soft_l, slid_pos_l)
        if wstep_l > 0.0: kwargs_l["wrist_return_step"] = wstep_l
        hold_l = _xr_hold(app, "l", "left", slid_pos_l, follow_rot_l, getattr(app, "_last_ee_target_l", None),
                         getattr(app, "_last_ee_rot_l", None), warm_q_l)
        if hold_l:      # hand on a target that is not moving: no posture re-solve, the joints stay where they are
            res_l = (np.asarray(warm_q_l, dtype=np.float64).copy(), True, getattr(app, "last_ik_pos_err_l", 0.0),
                     getattr(app, "last_ik_rot_err_l", 0.0), 0)
        else:
            res_l = app.ik_left.solve(
                target_pos=slid_pos_l, target_rot=follow_rot_l, init_qpos=warm_q_l,
                max_iters=15, pos_tol=0.005, rot_tol=0.040, **kwargs_l
            )
        q_sol_l = res_l[0]
        app.last_ik_residual_l = float(res_l[2])
        app.last_ik_pos_err_l = float(res_l[2])
        app.last_ik_rot_err_l = float(res_l[3])
        app.last_ik_relaxed_l = bool(getattr(app.ik_left, "last_ori_relaxed", False))

        if np.isnan(q_sol_l).any():
            q_sol_l = warm_q_l.copy()
        else:
            last_ee_l = getattr(app, "_last_ee_target_l", None)
            last_rot_l = getattr(app, "_last_ee_rot_l", None)
            ee_move_l = float(np.linalg.norm(slid_pos_l - last_ee_l)) if last_ee_l is not None else 1.0
            rot_move_l = float(np.linalg.norm(follow_rot_l - last_rot_l)) if last_rot_l is not None else 1.0
            jump_l = float(np.max(np.abs(q_sol_l - warm_q_l)))
            if jump_l > 0.15 and ee_move_l < 0.005 and rot_move_l < 0.05:
                q_sol_l = warm_q_l.copy()
            elif not hold_l:
                app._last_ee_target_l = np.array(slid_pos_l).copy()
                app._last_ee_rot_l = np.array(follow_rot_l).copy()
        app.last_ik_sol_left = q_sol_l.copy()
        cand_left = q_sol_l.copy()
    else:
        cand_left = app.last_arm_cmd_left.copy()

    if arm in ("right", "both"):
        vel_r = getattr(app, "handtrack_vel_r", None) if is_hands else None
        if use_cart:
            follow_pos_r = app.gizmo_follower_r.update(app.cur_target_pos_r, app.control_dt, target_vel=vel_r)
            follow_rot_r = app.gizmo_rot_follower_r.update(app.cur_target_rot_r, app.control_dt)
        else:
            follow_pos_r = np.array(app.cur_target_pos_r, dtype=np.float64).copy()
            follow_rot_r = np.array(app.cur_target_rot_r, dtype=np.float64).copy()

        reach_pos_r, clamped_r, d_beyond_r = project_to_reachable_workspace(follow_pos_r, side="right")
        app.reach_warning_r = reach_message(d_beyond_r) if clamped_r else ""
        reach_pos_r, follow_rot_r = _xr_carrot(app, "r", reach_pos_r, follow_rot_r, *_plan_ee(app, "right", ee_poses["right"]))
        # the lead point lies on the straight line hand -> target, which can cut through the zone too close to the shoulder
        reach_pos_r = project_to_reachable_workspace(reach_pos_r, side="right")[0]

        slid_pos_r, _ = app.collision_checker.project_target_tangent(
            ee_poses["right"]["pos"], reach_pos_r, app.env.data
        )
        warm_q_r = getattr(app, "last_ik_sol_right", app.env.get_right_arm_qpos())
        xr_r = bool(getattr(app, "xr_grip_active_r", False))
        # Controllers and mouse: no hint -> the IK's weighted rest-pose bias (natural elbow-down posture, see
        # mink_ik.REST_WEIGHTS). Hand tracking keeps the current-posture hint.
        hint_r = posture_hint_r if posture_hint_r is not None else (warm_q_r if is_hands else None)
        kwargs_r = {"elbow_hint": hint_r} if (hint_r is not None and hasattr(app.ik_right, "set_elbow_hint")) else {}
        soft_r = _soft_orientation(app, "r", app.ik_right) and hint_r is None
        if soft_r: kwargs_r["soft_orientation"] = True
        wstep_r = _wrist_return_step(app, "r", soft_r, slid_pos_r)
        if wstep_r > 0.0: kwargs_r["wrist_return_step"] = wstep_r
        hold_r = _xr_hold(app, "r", "right", slid_pos_r, follow_rot_r, getattr(app, "_last_ee_target_r", None),
                         getattr(app, "_last_ee_rot_r", None), warm_q_r)
        if hold_r:      # hand on a target that is not moving: no posture re-solve, the joints stay where they are
            res_r = (np.asarray(warm_q_r, dtype=np.float64).copy(), True, getattr(app, "last_ik_pos_err_r", 0.0),
                     getattr(app, "last_ik_rot_err_r", 0.0), 0)
        else:
            res_r = app.ik_right.solve(
                target_pos=slid_pos_r, target_rot=follow_rot_r, init_qpos=warm_q_r,
                max_iters=15, pos_tol=0.005, rot_tol=0.040, **kwargs_r
            )
        q_sol_r = res_r[0]
        app.last_ik_residual_r = float(res_r[2])
        app.last_ik_pos_err_r = float(res_r[2])
        app.last_ik_rot_err_r = float(res_r[3])
        app.last_ik_relaxed_r = bool(getattr(app.ik_right, "last_ori_relaxed", False))

        if np.isnan(q_sol_r).any():
            q_sol_r = warm_q_r.copy()
        else:
            last_ee_r = getattr(app, "_last_ee_target_r", None)
            last_rot_r = getattr(app, "_last_ee_rot_r", None)
            ee_move_r = float(np.linalg.norm(slid_pos_r - last_ee_r)) if last_ee_r is not None else 1.0
            rot_move_r = float(np.linalg.norm(follow_rot_r - last_rot_r)) if last_rot_r is not None else 1.0
            jump_r = float(np.max(np.abs(q_sol_r - warm_q_r)))
            if jump_r > 0.15 and ee_move_r < 0.005 and rot_move_r < 0.05:
                q_sol_r = warm_q_r.copy()
            elif not hold_r:
                app._last_ee_target_r = np.array(slid_pos_r).copy()
                app._last_ee_rot_r = np.array(follow_rot_r).copy()
        app.last_ik_sol_right = q_sol_r.copy()
        cand_right = q_sol_r.copy()
    else:
        cand_right = app.last_arm_cmd_right.copy()

    app.reach_warning = getattr(app, "reach_warning_l", "") or getattr(app, "reach_warning_r", "")
    if app.reach_warning:
        now_t = time.time()
        if now_t - getattr(app, "_last_reach_log_t", 0.0) >= 1.0:
            app._last_reach_log_t = now_t
            print(f"[REACH] {app.reach_warning}")

    # Gate 3: SafetyLayer self-collision verification & shadow follower evaluation
    if app.self_collision and app.collision_checker.enabled:
        curr_qpos = app.env.data.qpos.copy()

        if not hasattr(app, "shadow_cmd_q14") or app.shadow_cmd_q14 is None:
            app.shadow_cmd_q14 = np.concatenate([app.env.get_left_arm_qpos(), app.env.get_right_arm_qpos()])

        raw_q14 = np.concatenate([cand_left, cand_right])
        diff_q14 = raw_q14 - app.shadow_cmd_q14
        max_d = float(np.max(np.abs(diff_q14)))
        step_cap = getattr(app, "limiter_max_step", 0.01)
        scale = min(1.0, step_cap / max_d) if max_d > 1e-6 else 1.0
        post_limiter_q14 = app.shadow_cmd_q14 + diff_q14 * scale

        use_shadow = getattr(app, "real_follower_mode", False) or getattr(app, "backend", "") in ("real-udp", "real", "ros2")
        target_qpos = curr_qpos.copy()
        if use_shadow:
            target_qpos[app.left_arm_qadr] = post_limiter_q14[:7]
            target_qpos[app.right_arm_qadr] = post_limiter_q14[7:]
        else:
            target_qpos[app.left_arm_qadr] = cand_left
            target_qpos[app.right_arm_qadr] = cand_right

        _, _, min_dist = app.collision_checker.filter_step(curr_qpos, target_qpos)
        app.collision_blocked = False
        app.min_collision_distance_m = float(min_dist)

        if hasattr(app, "real_follower_instance") and app.real_follower_instance:
            app.real_follower_instance.step(raw_q14)
        app.last_arm_cmd_left = cand_left.copy()
        app.last_arm_cmd_right = cand_right.copy()
        app.shadow_cmd_q14 = raw_q14.copy()
    else:
        app.collision_blocked = False
        app.last_arm_cmd_left = cand_left
        app.last_arm_cmd_right = cand_right


def freeze_arm_target(app, arm: str = "both", from_xr: bool = False):
    """Latches final Cartesian target pose and solves IK once to high-precision convergence."""
    arms_to_freeze = ["left", "right"] if arm in ("both", "all") else [arm]
    held = getattr(app, "manual_held", False)  # Toggle Hold: keep the joint command exactly where it is

    for a in arms_to_freeze:
        arm_char = 'l' if a == 'left' else 'r'
        setattr(app, f"gizmo_active_{arm_char}", False)
        setattr(app, f"xr_coast_{arm_char}", False)
        ik_solver = getattr(app, f"ik_{a}", None)
        follower_pos = getattr(app, f"gizmo_follower_{arm_char}", None)
        follower_rot = getattr(app, f"gizmo_rot_follower_{arm_char}", None)
        cur_pos = getattr(app, f"cur_target_pos_{arm_char}")
        cur_rot = getattr(app, f"cur_target_rot_{arm_char}")

        if follower_pos: follower_pos.reset(cur_pos)
        if follower_rot: follower_rot.reset(cur_rot)
        # The released target was already recalled onto the reachable sphere: drop the stale warning.
        _, still_out, d_out = project_to_reachable_workspace(cur_pos, side=a)
        setattr(app, f"reach_warning_{arm_char}", reach_message(d_out) if still_out and abs(d_out) > 0.01 else "")
        app.reach_warning = getattr(app, "reach_warning_l", "") or getattr(app, "reach_warning_r", "")

        warm_q = getattr(app, f"last_ik_sol_{a}", getattr(app, f"last_arm_cmd_{a}"))
        proj_pos, _, _ = project_to_reachable_workspace(cur_pos, side=a)
        # Released controller with the IK solution already on the target: keep it. The final solve + posture settling
        # below would only re-arrange the joints (elbow, forearm roll) with the hand standing still = the arm moving
        # by itself after the operator has stopped (see XR_HOLD_STILL).
        xr_hold = False
        if from_xr and XR_HOLD_STILL:
            ee_hold = _fk_ee_pos(app, a, warm_q)
            xr_hold = ee_hold is not None and float(np.linalg.norm(ee_hold - proj_pos)) <= XR_HOLD_ERR_M
            if xr_hold: setattr(app, f"last_arm_cmd_{a}", np.asarray(warm_q, dtype=np.float64).copy())
        if ik_solver and not held and not xr_hold:
            res = ik_solver.solve(
                target_pos=proj_pos, target_rot=cur_rot, init_qpos=warm_q,
                max_iters=25, pos_tol=0.005, rot_tol=0.040,
                **({"soft_orientation": True} if _soft_orientation(app, arm_char, ik_solver) else {}),
            )
            q_conv = res[0]
            if not np.isnan(q_conv).any():
                setattr(app, f"last_ik_sol_{a}", q_conv.copy())
                setattr(app, f"last_arm_cmd_{a}", q_conv.copy())
                setattr(app, f"last_ik_pos_err_{arm_char}", float(res[2]))
                setattr(app, f"last_ik_rot_err_{arm_char}", float(res[3]))
                setattr(app, f"last_ik_relaxed_{arm_char}", bool(getattr(ik_solver, "last_ori_relaxed", False)))
            if IK_SETTLE_MAX_S > 0.0 and not (from_xr and XR_HOLD_STILL):   # let the posture finish settling on the frozen target (mouse)
                setattr(app, f"ik_settle_{arm_char}", True); setattr(app, f"_ik_settle_until_{arm_char}", time.monotonic() + IK_SETTLE_MAX_S)
                setattr(app, f"_ik_settle_q_{arm_char}", None); setattr(app, f"_ik_settle_n_{arm_char}", 0)

    app.status_warning = "TARGET_FROZEN"
    print(f"[FREEZE] Target frozen for {arm}: status=TARGET_FROZEN")


REATTACH_POS_M = 0.010      # gizmo further than this from the hand ...
REATTACH_ROT_RAD = 0.09     # ... or rotated more than ~5 deg ...
REATTACH_SOFT_ROT_RAD = 0.005  # ... (0.3 deg after a soft-orientation move: the target orientation follows the hand)
REATTACH_HOLD_S = 0.30      # ... while the arm has been at rest this long -> snap the gizmo back onto the hand
REATTACH_REST_RAD_S = 0.005


def reattach_idle_targets(app, now_t: float):
    """Keeps the gizmo attached to the hand: when nobody is steering and an arm has come to rest away from
    its Cartesian target (out of reach, armor block, IK could not converge, not engaged), the target is moved
    onto the actual hand pose and the browser gizmo is recalled there. The joint command is latched to the
    current plan, so this never moves the robot."""
    if now_t - getattr(app, "_reattach_last_t", 0.0) < 0.05:
        return
    app._reattach_last_t = now_t
    lim = getattr(app, "mirror_limiter", None)
    busy = (getattr(app, "gizmo_active_l", False) or getattr(app, "gizmo_active_r", False)
            or getattr(app, "homing_active", False) or bool(getattr(app, "active_sequence", None))
            or now_t < getattr(app, "preset_track_until", 0.0)
            # a connected headset is not "steering": only a squeezed controller clutch is (otherwise the gizmo is
            # never brought back onto the hand while the Quest session is open)
            or getattr(app, "xr_grip_active_l", False) or getattr(app, "xr_grip_active_r", False)
            or getattr(app, "xr_coast_l", False) or getattr(app, "xr_coast_r", False)
            or getattr(app, "ik_settle_l", False) or getattr(app, "ik_settle_r", False)
            or getattr(app, "state_estop", False) or getattr(app, "input_mode", "") == "hands")
    if lim is None or busy:
        app._reattach_rest_since = {}
        return
    rest = getattr(app, "_reattach_rest_since", None)
    if rest is None:
        rest = app._reattach_rest_since = {}
    ee = app.env.get_ee_poses()
    for a, c, sl in (("left", "l", slice(0, 7)), ("right", "r", slice(7, 14))):
        pos, rot = np.asarray(ee[a]["pos"], dtype=np.float64), np.asarray(ee[a]["rot"], dtype=np.float64)
        tgt_p, tgt_r = getattr(app, f"cur_target_pos_{c}"), getattr(app, f"cur_target_rot_{c}")
        pos_err = float(np.linalg.norm(tgt_p - pos))
        cos_a = float(np.clip((np.trace(tgt_r.T @ rot) - 1.0) * 0.5, -1.0, 1.0))
        moving = float(np.max(np.abs(lim.v_plan[sl]))) > REATTACH_REST_RAD_S
        # After a soft-orientation move (translation-only input) the orientation target was only a preference:
        # adopt the orientation the hand really took, so the next rotation command starts from the true pose.
        rot_tol = REATTACH_SOFT_ROT_RAD if getattr(app, f"soft_ori_{c}", False) else REATTACH_ROT_RAD
        if moving or (pos_err < REATTACH_POS_M and np.arccos(cos_a) < rot_tol):
            rest.pop(a, None)
            continue
        if now_t - rest.setdefault(a, now_t) < REATTACH_HOLD_S:
            continue
        rest.pop(a, None)
        q_now = lim.q_plan[sl].copy()
        setattr(app, f"cur_target_pos_{c}", pos.copy())
        setattr(app, f"cur_target_rot_{c}", rot.copy())
        setattr(app, f"last_arm_cmd_{a}", q_now.copy())
        setattr(app, f"last_ik_sol_{a}", q_now.copy())
        setattr(app, f"_last_ee_target_{c}", pos.copy())
        setattr(app, f"_last_ee_rot_{c}", rot.copy())
        for name, val in ((f"gizmo_follower_{c}", pos), (f"gizmo_rot_follower_{c}", rot)):
            f = getattr(app, name, None)
            if f: f.reset(val)
        setattr(app, f"reach_warning_{c}", "")
        app.reach_warning = getattr(app, "reach_warning_l", "") or getattr(app, "reach_warning_r", "")
        if hasattr(app, "shadow_cmd_q14") and app.shadow_cmd_q14 is not None:
            app.shadow_cmd_q14[sl] = q_now
        if hasattr(app, "server") and hasattr(app.server, "broadcast_event"):
            from scipy.spatial.transform import Rotation as _R
            app.server.broadcast_event({"type": "gizmo_recall", "arm": a, "pos": pos.tolist(),
                                        "rot": _R.from_matrix(rot).as_quat().tolist(), "duration_ms": 200})
        print(f"[REATTACH] {a} gizmo moved onto the hand (was {pos_err*100:.1f} cm away)")
