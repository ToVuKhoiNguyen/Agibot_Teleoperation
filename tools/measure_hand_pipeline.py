#!/usr/bin/env python3
"""
tools/measure_hand_pipeline.py
Measures hand-tracking teleop performance against the synthetic noise model:
- 2 mm position noise, 0.6 deg rotation noise, 5% packet jitter
- Static still phase (2 s), 1 Hz sinusoid (5 s), 3 Hz sinusoid (5 s)
Reports sampling, noise, smoothness, latency, gains, and IK stability.
"""
import sys, time, math
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.kinematics.mink_ik import AgibotMinkIK
from teleop.hand.synth_hand import make_synth_hand
from teleop.app.handtrack_cmd import handle_xr_handtrack_input, _ensure_handtrack_state
from teleop.kinematics.followers import CriticallyDampedFollower, QuaternionSlerpFollower
from teleop.utils.collision_checker import CollisionChecker


class BenchmarkApp:
    def __init__(self):
        scene_xml = str(PROJECT_ROOT / "assets" / "agibot_a2_ultra" / "mjcf" / "scene.xml")
        self.env = AgibotSimEnv(scene_xml)
        self.backend, self.allow_handtracking_real, self.input_mode = "sim", False, "hands"
        self.xr_rig_yaw, self.xr_axis_map = 0.0, np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        self.cur_target_pos_l = np.array([0.235, 0.504, 1.250])
        self.cur_target_pos_r = np.array([0.235, -0.504, 1.250])
        self.cur_target_rot_l, self.cur_target_rot_r = np.eye(3), np.eye(3)
        self.gizmo_active_l = self.gizmo_active_r = False
        self.current_safe_hand_l = self.current_safe_hand_r = np.zeros(6)
        self.last_hand_cmd_left = self.last_hand_cmd_right = np.zeros(6)
        self.last_arm_cmd_left, self.last_arm_cmd_right = self.env.get_left_arm_qpos().copy(), self.env.get_right_arm_qpos().copy()
        self.last_ik_sol_left, self.last_ik_sol_right = self.last_arm_cmd_left.copy(), self.last_arm_cmd_right.copy()
        self.hand_closure_l = self.hand_closure_r = 0.0
        self.status_warning, self.state_estop = "", False
        self.xr_event_log = []
        self.hand_follow_mode = "absolute"
        self.cal_head_pos = np.array([0.0, 1.5, 0.0], dtype=np.float64)
        self.cal_head_yaw = self.last_head_yaw = 0.0
        self.last_head_pos = np.array([0.0, 1.5, 0.0], dtype=np.float64)
        self.fingers_follow_always = self.head_follow = True
        self.self_collision, self.control_dt = False, 0.01

        self.ik_left = AgibotMinkIK(scene_xml, arm="left")
        self.ik_right = AgibotMinkIK(scene_xml, arm="right")
        self.collision_checker = CollisionChecker(self.env.model)

        self.gizmo_follower_l = CriticallyDampedFollower(natural_freq_hz=15.0, dt=0.01)
        self.gizmo_follower_r = CriticallyDampedFollower(natural_freq_hz=15.0, dt=0.01)
        self.gizmo_rot_follower_l = QuaternionSlerpFollower(bandwidth_hz=15.0, dt=0.01)
        self.gizmo_rot_follower_r = QuaternionSlerpFollower(bandwidth_hz=15.0, dt=0.01)

        _ensure_handtrack_state(self)
        # Calibrate orientation to natural forward arm rest pose
        ee_rot = self.env.get_ee_poses()["right"]["rot"]
        self.handtrack_rcal_r = self.xr_axis_map.T @ ee_rot

    def _solve_arm_ik(self, arm="right"):
        from teleop.app.ik_dispatcher import solve_arm_ik
        solve_arm_ik(self, arm=arm)

    def _log_xr_event(self, msg):
        self.xr_event_log.append(msg)


def run_benchmark():
    np.random.seed(42)
    app = BenchmarkApp()

    dt = 1.0 / 72.0  # 72 Hz XR frame rate
    t_total = 12.0
    num_frames = int(t_total / dt)
    t_arr = np.linspace(0, t_total, num_frames)

    raw_pos_hist, actual_pos_hist = [], []
    actual_rot_hist, finger_flex_hist, dq_hist, intervals = [], [], [], []

    base_p = np.array([0.20, 1.20, -0.40])
    prev_q_arm = app.env.get_right_arm_qpos().copy()

    for idx, t in enumerate(t_arr):
        if t < 2.0:
            pure_p = base_p.copy()
            pure_r = R.identity()
            pure_curl = 0.0
        elif t < 7.0:
            pure_p = base_p + np.array([0.0, 0.08 * math.sin(2.0 * math.pi * 1.0 * (t - 2.0)), 0.0])
            ang = math.radians(12.0) * math.sin(2.0 * math.pi * 1.0 * (t - 2.0))
            pure_r = R.from_euler("y", ang)
            pure_curl = 0.5 * (1.0 + math.sin(2.0 * math.pi * 1.0 * (t - 2.0)))
        else:
            pure_p = base_p + np.array([0.0, 0.03 * math.sin(2.0 * math.pi * 3.0 * (t - 7.0)), 0.0])
            ang = math.radians(8.0) * math.sin(2.0 * math.pi * 3.0 * (t - 7.0))
            pure_r = R.from_euler("y", ang)
            pure_curl = 0.5 * (1.0 + math.sin(2.0 * math.pi * 3.0 * (t - 7.0)))

        # Add synthetic noise: 2 mm pos noise, 0.6 deg rot noise
        noisy_p = pure_p + np.random.normal(0.0, 0.002, 3)
        ax = np.random.normal(0.0, 1.0, 3)
        ax /= np.linalg.norm(ax)
        noisy_r = pure_r * R.from_rotvec(ax * np.random.normal(0.0, math.radians(0.6)))
        q_xyzw = noisy_r.as_quat()
        q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]

        jitter = np.random.exponential(0.001) if np.random.rand() < 0.05 else 0.0
        if idx > 0: intervals.append(dt + jitter)

        pkt = make_synth_hand(noisy_p, [pure_curl] * 5, quat=q_wxyz)
        cmd = {"cmd": "xr_handtrack", "t": t, "head": {"pos": [0.0, 1.5, 0.0], "yaw": 0.0, "pitch": 0.0}, "right": pkt}
        handle_xr_handtrack_input(app, cmd)
        app.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=app.last_arm_cmd_left, hold_right_qpos=app.last_arm_cmd_right)

        ee_pos = app.env.get_ee_poses()["right"]["pos"].copy()
        ee_rot = app.env.get_ee_poses()["right"]["rot"].copy()
        cur_q = app.env.get_right_arm_qpos().copy()
        dq = np.max(np.abs(cur_q - prev_q_arm))
        prev_q_arm = cur_q.copy()

        raw_pos_hist.append(noisy_p.copy())
        actual_pos_hist.append(ee_pos)
        actual_rot_hist.append(ee_rot)
        finger_flex_hist.append(app.hand_closure_r)
        dq_hist.append(dq)

    # 1. SAMPLING ANALYSIS
    intervals = np.array(intervals)
    p95_interval = float(np.percentile(intervals, 95) * 1000.0)
    gaps_30 = int(np.sum(intervals > 0.030))
    rate_hz = 1.0 / np.mean(intervals)

    # 2. NOISE ANALYSIS (Still steady-state: t 0.8s to 2.0 s)
    still_s, still_e = int(0.8 / dt), int(2.0 / dt)
    raw_p_still = np.array(raw_pos_hist[still_s:still_e])
    act_p_still = np.array(actual_pos_hist[still_s:still_e])
    pos_jitter_raw_rms = float(np.mean(np.std(raw_p_still, axis=0)) * 1000.0)
    pos_jitter_act_rms = float(np.mean(np.std(act_p_still, axis=0)) * 1000.0)

    r_still = [R.from_matrix(m) for m in actual_rot_hist[still_s:still_e]]
    angles_deg = [np.rad2deg((r_still[0].inv() * r).magnitude()) for r in r_still]
    rot_jitter_act_deg = float(np.std(angles_deg))
    finger_jitter_pct = float(np.std(finger_flex_hist[still_s:still_e]) * 100.0)

    # 3. SMOOTHNESS (Jerk RMS & high-freq power)
    act_p = np.array(actual_pos_hist)
    jerk_act = np.diff(np.diff(np.diff(act_p, axis=0) / dt, axis=0) / dt, axis=0) / dt
    jerk_act_rms = float(np.sqrt(np.mean(jerk_act ** 2)))
    raw_p_arr = np.array(raw_pos_hist)
    jerk_raw = np.diff(np.diff(np.diff(raw_p_arr, axis=0) / dt, axis=0) / dt, axis=0) / dt
    jerk_raw_rms = float(np.sqrt(np.mean(jerk_raw ** 2)))
    jerk_ratio = (jerk_act_rms / jerk_raw_rms) if jerk_raw_rms > 0 else 1.0

    # High frequency power (> 8 Hz) on static noise phase
    fft_raw = np.abs(np.fft.rfft(raw_p_still[:, 1])) ** 2
    fft_act = np.abs(np.fft.rfft(act_p_still[:, 2])) ** 2
    freqs = np.fft.rfftfreq(len(raw_p_still), d=dt)
    hf_mask = freqs > 8.0
    hf_power_ratio = float(np.sum(fft_act[hf_mask]) / np.sum(fft_raw[hf_mask])) if np.sum(fft_raw[hf_mask]) > 0 else 1.0

    # 4. LATENCY & GAINS
    f1_start, f1_end = int(2.5 / dt), int(6.5 / dt)
    sig_raw = raw_p_arr[f1_start:f1_end, 1] - np.mean(raw_p_arr[f1_start:f1_end, 1])
    sig_act = act_p[f1_start:f1_end, 2] - np.mean(act_p[f1_start:f1_end, 2])
    corr = np.correlate(sig_act, sig_raw, mode="full")
    shift = np.argmax(corr) - (len(sig_raw) - 1)
    e2e_lag_ms = float(shift * dt * 1000.0)
    gain_1hz = float(np.std(sig_act) / np.std(sig_raw))

    f3_start, f3_end = int(7.5 / dt), int(11.5 / dt)
    sig3_raw = raw_p_arr[f3_start:f3_end, 1] - np.mean(raw_p_arr[f3_start:f3_end, 1])
    sig3_act = act_p[f3_start:f3_end, 2] - np.mean(act_p[f3_start:f3_end, 2])
    gain_3hz = float(np.std(sig3_act) / np.std(sig3_raw))

    # 5. IK STABILITY
    max_dq = float(np.max(dq_hist[10:])) # ignore initial settle
    branch_flips = int(np.sum(np.array(dq_hist[10:]) > 0.40))

    # 6. DROPOUT TEST (Bridge 2 lost frames during steady tracking)
    for _ in range(5):
        handle_xr_handtrack_input(app, {"cmd": "xr_handtrack", "t": t_total + 0.05, "head": {"pos": [0.0, 1.5, 0.0]}, "right": make_synth_hand(base_p, [0.0]*5)})
        app.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=app.last_arm_cmd_left, hold_right_qpos=app.last_arm_cmd_right)
    pkt_drop = make_synth_hand(base_p, [0.0]*5)
    pkt_drop["tracking"] = False
    handle_xr_handtrack_input(app, {"cmd": "xr_handtrack", "t": t_total + dt + 0.1, "head": {"pos": [0.0, 1.5, 0.0]}, "right": pkt_drop})
    app.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=app.last_arm_cmd_left, hold_right_qpos=app.last_arm_cmd_right)
    p_drop1 = app.env.get_ee_poses()["right"]["pos"].copy()
    handle_xr_handtrack_input(app, {"cmd": "xr_handtrack", "t": t_total + 2*dt + 0.1, "head": {"pos": [0.0, 1.5, 0.0]}, "right": pkt_drop})
    app.env.step(num_substeps=2, hold_arms=True, hold_left_qpos=app.last_arm_cmd_left, hold_right_qpos=app.last_arm_cmd_right)
    p_drop2 = app.env.get_ee_poses()["right"]["pos"].copy()
    drop_dev_mm = float(np.linalg.norm(p_drop2 - p_drop1) * 1000.0)

    print("=" * 80)
    print("HAND-TRACKING PIPELINE BENCHMARK (AFTER TUNED FILTERS + FOLLOWER)")
    print("=" * 80)
    print(f"1) Sampling: XR Rate={rate_hz:.1f} Hz | p95 Interval={p95_interval:.1f} ms | Gaps >30ms={gaps_30}")
    print(f"2) Noise: Raw Pos Jitter={pos_jitter_raw_rms:.2f} mm | Act Pos Jitter={pos_jitter_act_rms:.2f} mm (target <=1.5mm)")
    print(f"   Rot Jitter={rot_jitter_act_deg:.2f}° (target <=0.5°) | Finger Jitter={finger_jitter_pct:.2f}% (target <=2%)")
    print(f"3) Smoothness: Jerk Raw={jerk_raw_rms:.1f} m/s^3 | Jerk Act={jerk_act_rms:.1f} m/s^3 | Ratio={jerk_ratio*100.0:.1f}% (target <=40%)")
    print(f"   >8Hz HF Power Ratio={hf_power_ratio*100.0:.1f}% (target <=10%)")
    print(f"4) Latency & Gains: Lag={e2e_lag_ms:.1f} ms (target <=50ms) | 1Hz Gain={gain_1hz:.3f} | 3Hz Gain={gain_3hz:.3f} (target >=0.85)")
    print(f"5) IK Stability: Max |dq|={max_dq:.3f} rad/tick (target <=0.08) | Branch Flips={branch_flips}")
    print(f"6) Dropout Bridging: 2-frame deviation={drop_dev_mm:.2f} mm (target <2.0mm)")
    print("=" * 80)

    return {
        "rate_hz": rate_hz, "p95_interval": p95_interval, "pos_jitter_act_rms": pos_jitter_act_rms,
        "rot_jitter_act_deg": rot_jitter_act_deg, "finger_jitter_pct": finger_jitter_pct,
        "jerk_ratio": jerk_ratio, "hf_power_ratio": hf_power_ratio, "e2e_lag_ms": e2e_lag_ms,
        "gain_1hz": gain_1hz, "gain_3hz": gain_3hz, "max_dq": max_dq, "branch_flips": branch_flips,
        "drop_dev_mm": drop_dev_mm
    }


if __name__ == "__main__":
    run_benchmark()
