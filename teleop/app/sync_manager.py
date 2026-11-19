"""
teleop/app/sync_manager.py
Manages the SYNC -> READY -> ENGAGED -> FAULT state machine for real teleoperation,
source sanity gating, soft back-off at 0.035 rad, and mirror limiter dispatch.
"""

import os
import time
import numpy as np

FEEDBACK_LOST_S = 0.5  # ENGAGED -> SYNC when robot feedback is older than this



DIAG_PERIOD_S = 0.05


# Arm joint speed above which finger commands are held (0 disables): HAND_HOLD_MOVING=0.15 rad/s by default
HAND_HOLD_MOVING_RAD_S = float(os.environ.get("HAND_HOLD_MOVING", "0.15"))


def _log_diag(app):
    """20 Hz DIAG record in trace_laptop_*.jsonl: WHY an arm is not following (out of reach, armor block, soft
    collision slow-down, IK error, clutch state) plus the raw controller position and the Cartesian target."""
    tl = getattr(getattr(app, "robot_backend", None), "_trace_logger", None)
    if tl is None:
        return
    now = time.perf_counter()
    if now - getattr(app, "_diag_last_t", 0.0) < DIAG_PERIOD_S:
        return
    app._diag_last_t = now
    try:
        lim = getattr(app, "mirror_limiter", None)
        rec = {
            "sync_state": getattr(app, "sync_state", ""),
            "status": getattr(app, "status_warning", ""),
            "xr_scale": float(getattr(app, "xr_scale", 1.0)),
            "axis_lock": bool(getattr(app, "xr_axis_lock", False)),
            "armor_blocked": bool(getattr(lim, "armor_blocked", False)),
            "armor_pair": str(getattr(lim, "armor_pair", "")),
            "armor_min_dist_mm": round(float(getattr(lim, "armor_min_dist", 999.0)) * 1000.0, 1),
            "collision_gate": bool(getattr(app, "collision_blocked", False)),
        }
        for s in ("l", "r"):
            ctrl = getattr(app, f"xr_ctrl_pos_{s}", None)
            rec[s] = {
                "clutch": getattr(app, f"xr_clutch_state_{s}", ""),
                "coast": bool(getattr(app, f"xr_coast_{s}", False)),   # released, IK still stepping to the target
                "settle": bool(getattr(app, f"ik_settle_{s}", False)), # input stopped, posture still settling
                "ctrl_pos": [round(float(x), 4) for x in ctrl] if ctrl is not None else None,
                "target_pos": [round(float(x), 4) for x in getattr(app, f"cur_target_pos_{s}")],
                "reach": getattr(app, f"reach_warning_{s}", ""),
                "ik_pos_err_mm": round(float(getattr(app, f"last_ik_pos_err_{s}", 0.0)) * 1000.0, 1),
                "reach_gain": round(float(getattr(app, f"xr_soft_gain_{s}", 1.0)), 3),
                "lock_axis": getattr(app, f"xr_axlock_axis_{s}", None),
                "lock_rot_axis": getattr(app, f"xr_axlock_raxis_{s}", None),
            }
        tl.log_event("DIAG", **rec)
    except Exception:
        pass


def _eval_soft_collision(app, cur_plan: np.ndarray) -> float:
    """Evaluates proxy soft collision at <= 50 Hz, returns limiter speed scale."""
    if not hasattr(app, "proxy_collision") or app.proxy_collision is None:
        return 1.0
    now_t = time.time()
    if now_t - getattr(app.proxy_collision, "last_check_t", 0.0) >= 0.020:
        scale, d_m, dec = app.proxy_collision.evaluate_scale(cur_plan)
        app.proxy_collision.last_check_t = now_t
        app.proxy_min_distance_m = d_m
        app.proxy_collision_scale = scale
        app.proxy_collision_warning = f"Arm proximity: {d_m*1000:.0f} mm" if d_m <= 0.05 else ""
        if now_t - getattr(app, "_last_proxy_log_t", 0.0) >= 1.0:
            app._last_proxy_log_t = now_t
            print(f"[SOFT_COLLISION] min_dist: {d_m*1000:.1f} mm | scale: {scale:.2f} | dec: {dec}")
    return getattr(app, "proxy_collision_scale", 1.0)


def engage_arms(app, arm: str = "both") -> bool:
    """Engages arms from READY/ENGAGED: re-seeds safety + limiter at measured pose (no jump)."""
    if getattr(app, "sync_state", "READY") not in ("READY", "ENGAGED"):
        print(f"[SYNC] Cannot engage in state {getattr(app, 'sync_state', '?')}")
        return False
    if hasattr(app, "robot_safety") and app.robot_safety is not None:
        q_act, _, _ = app.robot_safety.read_state()
        if q_act is not None and len(q_act) == 14 and not np.any(np.isnan(q_act)):
            app.robot_safety._last_cmd_q14 = q_act.copy()
            app.robot_safety._last_cmd_vel = np.zeros(14, dtype=np.float64)
            app.robot_safety._deviation_start_t = None
            app.robot_safety.widen_to(q_act)          # robot stays where it stands (no pull into our joint bounds)
            if hasattr(app, "mirror_limiter"): app.mirror_limiter.reset(q_act)
    if arm in ("left", "both"): app.arm_engaged_l = True
    if arm in ("right", "both"): app.arm_engaged_r = True
    app.sync_state = "ENGAGED"
    print(f"[SYNC] Arms engaged: L={app.arm_engaged_l}, R={app.arm_engaged_r} (State: ENGAGED)")
    return True


FOLLOW_ERR_RAD, FOLLOW_ERR_S = 0.35, 1.0     # a joint this far from its command for this long is not being driven


def _check_joint_following(app, q_cmd: np.ndarray, q_act: np.ndarray) -> None:
    """17/11: an elbow the MC had stopped servoing stayed straight (-0.03) for whole sessions while we commanded
    0.03..2.0 - the real hand hung lower than the model and touched the leg, and nothing told the operator. Normal
    tracking lag is < 0.3 rad even at 2 rad/s, so a joint > FOLLOW_ERR_RAD off for > FOLLOW_ERR_S is not followed:
    show it on the panel (warning line) and in the terminal. Warning only - the operator decides to stop."""
    now = time.monotonic()
    err = np.abs(np.asarray(q_cmd, dtype=np.float64) - np.asarray(q_act, dtype=np.float64))
    since = getattr(app, "_follow_bad_since", None)
    if since is None: since = app._follow_bad_since = np.full(14, np.nan)
    bad = err > FOLLOW_ERR_RAD
    since[~bad] = np.nan
    since[bad & np.isnan(since)] = now
    stuck = [j for j in range(14) if bad[j] and now - since[j] >= FOLLOW_ERR_S]
    if not stuck:
        app.joint_follow_warning = ""
        return
    names = ", ".join(f"{'L' if j < 7 else 'R'} J{j % 7 + 1} (cmd {q_cmd[j]:+.2f}, robot {q_act[j]:+.2f})" for j in stuck)
    app.joint_follow_warning = f"JOINT NOT FOLLOWING: {names} - robot is not driving it: stop, SetAction 10 then 12 (power-cycle if it stays)"
    if now - getattr(app, "_follow_warn_t", 0.0) > 5.0:
        app._follow_warn_t = now
        print(f"[WARN] {app.joint_follow_warning}", flush=True)


def process_arm_sync_and_dispatch(app, is_real: bool, dt: float):
    """Processes state machine transitions, mirror limiter plan, and backend send."""
    _log_diag(app)   # state of the previous tick; here so SYNC/READY/FAULT early returns are logged too
    if getattr(app, "rec_active", False):
        from teleop.app.episode_recorder import log_tick
        log_tick(app)
    else:
        from teleop.app.episode_recorder import live_preview_tick
        live_preview_tick(app)
    if not hasattr(app, "mirror_limiter"):
        return

    if is_real and hasattr(app, "robot_safety") and app.robot_safety is not None:
        q_act, h_act, meta = app.robot_safety.read_state()
        age = meta.get("age_s", 999.0) if isinstance(meta, dict) else 0.0
        app.last_state_feedback_age_s = age

        if q_act is not None and len(q_act) == 14 and not np.any(np.isnan(q_act)):
            app.real_q14_measured = q_act.copy()
            app.max_sim_real_gap_rad = float(np.max(np.abs(app.mirror_limiter.q_plan - q_act)))
            if app.sync_state == "ENGAGED": _check_joint_following(app, app.mirror_limiter.q_plan, q_act)
        else:
            q_act = None

        rx_st = meta.get("rx_state", "WAITING") if isinstance(meta, dict) else "WAITING"
        app.orin_rx_state = rx_st

        # Check Orin robot node fault latch
        if meta.get("robot_fault", False) or rx_st.startswith("FAULT"):
            app.robot_safety.fault_latched = True
            reason = meta.get("fault_reason", "") or rx_st
            if "DUAL PUBLISHER" in reason:
                app.robot_safety.fault_reason = "DUAL PUBLISHER: another node is publishing on robot topics!"
            else:
                app.robot_safety.fault_reason = f"ROBOT NODE FAULT ({reason}): restart arm_teleop_rx.py"
            app.sync_state = "FAULT"
            return

        # 1. State: SYNC
        if app.sync_state == "SYNC":
            if q_act is not None and age < 0.200:
                app.last_arm_cmd_left = q_act[:7].copy()
                app.last_arm_cmd_right = q_act[7:].copy()
                app.mirror_limiter.reset(q_act)
                app.robot_safety.widen_to(q_act)
                app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)
                if h_act is not None and len(h_act) == 12:
                    app.last_hand_cmd_left = h_act[:6].copy()
                    app.last_hand_cmd_right = h_act[6:].copy()
                    app.current_safe_hand_l = h_act[:6].copy()
                    app.current_safe_hand_r = h_act[6:].copy()
                    app.env.set_hand_commands(app.last_hand_cmd_left, app.last_hand_cmd_right)
                ee_poses = app.env.get_ee_poses()
                app.cur_target_pos_l = ee_poses["left"]["pos"].copy()
                app.cur_target_rot_l = ee_poses["left"]["rot"].copy()
                app.cur_target_pos_r = ee_poses["right"]["pos"].copy()
                app.cur_target_rot_r = ee_poses["right"]["rot"].copy()
                app.gizmo_target_pos_l = app.cur_target_pos_l.copy()
                app.gizmo_target_pos_r = app.cur_target_pos_r.copy()
                if hasattr(app, "output_filter_left"): app.output_filter_left.reset(app.last_arm_cmd_left)
                if hasattr(app, "output_filter_right"): app.output_filter_right.reset(app.last_arm_cmd_right)
                if hasattr(app, "output_filter_left_gizmo"): app.output_filter_left_gizmo.reset(app.last_arm_cmd_left)
                if hasattr(app, "output_filter_right_gizmo"): app.output_filter_right_gizmo.reset(app.last_arm_cmd_right)
                app.last_ik_sol_left = app.last_arm_cmd_left.copy()
                app.last_ik_sol_right = app.last_arm_cmd_right.copy()
                if hasattr(app, "robot_safety") and app.robot_safety is not None:
                    app.robot_safety._last_cmd_q14 = q_act.copy()
                    app.robot_safety._last_cmd_vel = np.zeros(14, dtype=np.float64)
                    app.robot_safety._deviation_start_t = None
                app.sync_state = "READY"
                print("[SYNC] Robot state synchronized! Transitioned SYNC -> READY.")
            return

        # 2. State: READY -> auto-engage once feedback is fresh and Orin accepts us
        #    (no manual Engage button; disengage_arm API suppresses until engage_arm)
        if app.sync_state == "READY":
            if (not getattr(app, "auto_engage_suppressed", False)
                    and q_act is not None and age < 0.200
                    and rx_st in ("RX_OK", "NOT_ENGAGED", "WAITING", "REJECT:stale_target")):
                # REJECT:stale_target = the receiver still holds an engaged session but no fresh packets: exactly
                # the state after main.py restarts (or stalls) while arm_teleop_rx.py keeps running. The laptop only
                # transmits once ENGAGED, so waiting for RX_OK here would deadlock; engaging re-seeds the limiter at
                # the measured pose, so the first packets equal where the arm already is.
                engage_arms(app, "both")
            return

        # 3. State: FAULT
        if app.sync_state == "FAULT" or getattr(app.robot_safety, "fault_latched", False):
            app.sync_state = "FAULT"
            return

        # 3b. Feedback lost while ENGAGED (receiver stopped / network down): drop to SYNC and stop engaging.
        #     Otherwise the open-loop plan keeps following the operator and drifts away from the real arm
        #     (seen: 0.6 rad lead after 35 s) - the arm would then jump there as soon as the receiver comes back.
        if q_act is None or age > FEEDBACK_LOST_S:
            app.sync_state = "SYNC"
            app.arm_engaged_l = False
            app.arm_engaged_r = False
            app.status_warning = f"Robot feedback lost ({age*1000:.0f} ms): re-syncing to measured pose"
            print(f"[SYNC] Feedback lost ({age*1000:.0f} ms) -> ENGAGED to SYNC (arms disengaged until fresh feedback).")
            return

        # 4. State: ENGAGED - must never show ENGAGED while rx_state != RX_OK
        if rx_st not in ("RX_OK", "WAITING"):
            # stale_target only means our packets have not arrived yet (we just (re)engaged, or the laptop stalled):
            # keep transmitting so the receiver recovers, instead of dropping to READY and stopping the stream.
            if (rx_st.startswith("REJECT") and rx_st != "REJECT:stale_target") or rx_st == "HOLDING":
                app.sync_state = "READY"
                app.status_warning = f"Orin state: {rx_st}"
                return
        raw_target_q14 = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
        last_plan = app.mirror_limiter.q_plan

        # Source sanity check (NaN / inf protection; step bounding handled by mirror_limiter)
        if np.any(np.isnan(raw_target_q14)) or np.any(np.isinf(raw_target_q14)):
            raw_target_q14 = last_plan.copy()

        # Deviation is log-only (NEVER slowdowns or scales command rate)
        lead = float(np.max(np.abs(last_plan - q_act))) if q_act is not None else 0.0
        if getattr(app, "status_warning", "").startswith("Tracking deviation"):
            app.status_warning = ""

        # Update digital twin hand commands gated by ENGAGED
        eng_l = bool(getattr(app, "arm_engaged_l", False))
        eng_r = bool(getattr(app, "arm_engaged_r", False))
        h_left = app.last_hand_cmd_left if eng_l else (h_act[:6] if h_act is not None else np.zeros(6))
        h_right = app.last_hand_cmd_right if eng_r else (h_act[6:] if h_act is not None else np.zeros(6))
        if HAND_HOLD_MOVING_RAD_S > 0.0 and hasattr(app, "mirror_limiter"):
            # fingers keep their last command while that arm is moving: a finger that starts or stops moving
            # mid-swing adds its own shake to the hand; the pending open / close is applied once the arm is slow
            for c, sl, eng in (("l", slice(0, 7), eng_l), ("r", slice(7, 14), eng_r)):
                held = getattr(app, f"_hand_hold_{c}", None)
                moving = float(np.max(np.abs(app.mirror_limiter.v_plan[sl]))) > HAND_HOLD_MOVING_RAD_S
                cur = h_left if c == "l" else h_right
                if eng and moving and held is not None: cur = held
                else: setattr(app, f"_hand_hold_{c}", np.array(cur, dtype=np.float64).copy())
                if c == "l": h_left = cur
                else: h_right = cur
        app.env.set_hand_commands(h_left, h_right)

        # Static offset calibration during hold phase (>= 2 s stationary after ENGAGED)
        if not getattr(app, "_static_offset_calibrated", False):
            if not hasattr(app, "_hold_samples"):
                app._hold_samples = []
            dq_sent = np.max(np.abs(getattr(app.mirror_limiter, "v_plan", np.zeros(14))))
            if dq_sent < 0.002 and q_act is not None:
                app._hold_samples.append(q_act - last_plan)
                if len(app._hold_samples) >= int(2.0 / dt):
                    raw_offset = np.median(app._hold_samples, axis=0)
                    clamped_offset = np.clip(raw_offset, -0.03, 0.03)
                    if hasattr(app.robot_safety, "dev_checker"):
                        app.robot_safety.dev_checker.static_offset = clamped_offset.copy()
                    app._static_offset_calibrated = True
                    off_str = " ".join(f"{x:+.4f}" for x in clamped_offset)
                    print(f"[OFFSET] Static offset calibrated ({len(app._hold_samples)} samples, clamped <= 0.03): {off_str}")
            else:
                app._hold_samples = []

        # Evaluate proxy soft collision on PLAN at <= 50 Hz
        cur_plan = app.mirror_limiter.q_plan if hasattr(app, "mirror_limiter") else raw_target_q14
        soft_scale = _eval_soft_collision(app, cur_plan)
        app.robot_safety.proxy_scale = soft_scale

        # Deliver raw target to backend; limiter runs at 100 Hz inside UdpBackend
        cur_q = app.mirror_limiter.q_plan if hasattr(app, "mirror_limiter") else app.last_arm_cmd_left
        app.env.set_arm_commands(cur_q[:7], cur_q[7:])
        h12 = np.concatenate([h_left, h_right])
        app.robot_safety.context["input_mode"] = getattr(app, "input_mode", "mouse")
        app.robot_safety.context["collision_gate"] = bool(getattr(app, "collision_blocked", False))
        ok = app.robot_safety.send_arm(raw_target_q14, dt=dt)
        if hasattr(app.robot_safety, "send_hand"):
            app.robot_safety.send_hand(h12)
    elif getattr(app, "real_follower_mode", False):
        # Sim mode with real follower enabled (--sim-follower real): self-tracking via sim qpos
        raw_target_q14 = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
        meas_q14 = np.concatenate([app.env.get_left_arm_qpos(), app.env.get_right_arm_qpos()])
        soft_scale = _eval_soft_collision(app, app.mirror_limiter.q_plan)
        q_plan = app.mirror_limiter.step(raw_target_q14, measured_q14=meas_q14, dt=dt, speed_scale=soft_scale)
        app.last_arm_cmd_left = q_plan[:7].copy()
        app.last_arm_cmd_right = q_plan[7:].copy()
        app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)
        app.sim_plant_q = q_plan.copy()
        app.sim_cmd_q = q_plan.copy()
        app.env.set_hand_commands(app.last_hand_cmd_left, app.last_hand_cmd_right)
    else:
        # Default sim mode: step through the SAME mirror_limiter as real-udp
        if hasattr(app, "mirror_limiter") and app.mirror_limiter is not None:
            raw_target_q14 = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
            soft_scale = _eval_soft_collision(app, app.mirror_limiter.q_plan)
            q_plan = app.mirror_limiter.step(raw_target_q14, dt=dt, speed_scale=soft_scale)
            if getattr(app, "sim_plant_active", False) and getattr(app, "sim_plant", None) is not None:
                q_act = app.sim_plant.step(q_plan, dt=dt)
                if hasattr(app, "armor_collision") and app.armor_collision is not None:
                    d_plant, _, _ = app.armor_collision.compute_min_distance(q_act)
                    app.sim_plant_min_dist = d_plant
            else:
                q_act = q_plan
                app.sim_plant_min_dist = 999.0
            app.env.set_arm_commands(q_act[:7], q_act[7:])
            app.sim_plant_q = q_act.copy()
            app.sim_cmd_q = q_plan.copy()
        else:
            app.env.set_arm_commands(app.last_arm_cmd_left, app.last_arm_cmd_right)
            q_direct = np.concatenate([app.last_arm_cmd_left, app.last_arm_cmd_right])
            app.sim_plant_q = q_direct.copy()
            app.sim_cmd_q = q_direct.copy()
        app.env.set_hand_commands(app.last_hand_cmd_left, app.last_hand_cmd_right)

    # Sync armor status and log
    if hasattr(app, "mirror_limiter") and getattr(app.mirror_limiter, "armor_blocked", False):
        d_mm = app.mirror_limiter.armor_min_dist * 1000.0
        pair = app.mirror_limiter.armor_pair
        app.armor_warning = f"Collision armor: BLOCKED {pair} ({d_mm:.0f} mm)"
        now_t = time.time()
        if now_t - getattr(app, "_last_armor_log_t", 0.0) >= 1.0:
            app._last_armor_log_t = now_t
            print(f"[ARMOR] BLOCKED {pair} ({d_mm:.1f} mm)")
    else:
        app.armor_warning = ""
