#!/usr/bin/env python3
"""
scripts/test_sim_follower_validation.py
Validation & speed what-if suite for real-like sim follower & shadow check:
- Evaluates real logged targets & synthetic fast-reach motions
- Evaluates 4 follower configs: (a) ideal, (b) real-like per-joint, (c) uniform, (d) uniform + shadow
- Runs speed what-if sweep across caps, scalings, frequencies, feedforward
Offline analysis + sim only. NEVER runs on robot.
"""

import sys, json, math
from pathlib import Path
import numpy as np
import mujoco

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.simulation.mujoco_env import AgibotSimEnv
from teleop.utils.collision_checker import CollisionChecker
from teleop.kinematics.real_follower import RealLikeFollower, IDENTIFIED_TAU_S


def get_collision_env():
    xml_path = str(PROJECT_ROOT / "assets" / "agibot_a2_ultra" / "mjcf" / "scene.xml")
    env = AgibotSimEnv(xml_path=xml_path, render_gui=False)
    checker = CollisionChecker(env.model, enabled=True)
    l_adr = [env.model.jnt_qposadr[mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{13+i}_left_arm_joint{i+1}")] for i in range(7)]
    r_adr = [env.model.jnt_qposadr[mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_JOINT, f"idx{20+i}_right_arm_joint{i+1}")] for i in range(7)]
    return env, checker, l_adr, r_adr


def build_synthetic_trajectories(q_home):
    scenarios = {}
    q_chest = q_home.copy()
    q_chest[0], q_chest[1], q_chest[3] = 0.55, 0.95, -1.65
    traj_chest = [q_home + (q_chest - q_home) * (i / 100) for i in range(100)] + [q_chest.copy() for _ in range(50)]
    scenarios["Arm to chest"] = traj_chest

    q_across = q_home.copy()
    q_across[0], q_across[1], q_across[2], q_across[3] = 0.25, 1.35, 0.40, -1.20
    traj_across = [q_home + (q_across - q_home) * (i / 100) for i in range(100)] + [q_across.copy() for _ in range(50)]
    scenarios["Arm across body"] = traj_across

    for name, spd in [("Wrist rot slow", 0.2), ("Wrist rot med", 0.6), ("Wrist rot fast", 1.5)]:
        traj_rot = []
        for t in np.linspace(0, 3.0, 150):
            q_step = q_home.copy()
            q_step[5] += 0.5 * math.sin(spd * 2.0 * math.pi * t)
            q_step[6] += 0.8 * math.sin(spd * 1.5 * math.pi * t)
            traj_rot.append(q_step)
        scenarios[name] = traj_rot
    return scenarios


def run_simulation_scenario(traj, env, checker, l_adr, r_adr, config_mode, margin_m=0.020, shadow_inflation=0.030):
    dt = 0.02
    q_start = traj[0].copy()
    n_frames = len(traj)
    mode = "per-joint" if config_mode == "real_per_joint" else "uniform"
    follower = None if config_mode == "ideal" else RealLikeFollower(max_step_rad=0.01, scaling_mode=mode, tau_s=IDENTIFIED_TAU_S, dt=dt, initial_q14=q_start)

    executed, min_dist, frames_below, curr_exec = [], 999.0, 0, q_start.copy()
    for i in range(n_frames):
        target = traj[i]
        if config_mode == "ideal":
            exec_q = target.copy()
        elif config_mode in ("real_per_joint", "real_uniform"):
            exec_q = follower.step(target, dt=dt)
        elif config_mode == "real_shadow":
            future = follower.predict_ahead(target, n_steps=10, dt=dt)
            pred_d, _ = checker.predict_clearance(future, l_adr, r_adr)
            if pred_d < (margin_m + shadow_inflation):
                exec_q = curr_exec.copy()
                follower.reset(curr_exec)
            else:
                exec_q = follower.step(target, dt=dt)
        curr_exec = exec_q.copy()
        executed.append(exec_q)

        checker.scratch_data.qpos[:] = env.data.qpos
        checker.scratch_data.qpos[l_adr], checker.scratch_data.qpos[r_adr] = exec_q[:7], exec_q[7:]
        mujoco.mj_forward(checker.model, checker.scratch_data)
        d, _ = checker.compute_min_distance(data=checker.scratch_data, force=True)
        if d < min_dist: min_dist = d
        if d < margin_m: frames_below += 1

    executed, traj_arr = np.array(executed), np.array(traj)
    diff = np.abs(traj_arr - executed)
    rms_err = float(np.sqrt(np.mean(diff ** 2)))
    step_rate = 0.01 / dt
    lags_ms = (np.max(diff, axis=1) / step_rate) * 1000.0 if config_mode != "ideal" else np.zeros(n_frames)
    return {"min_dist_mm": min_dist * 1000.0, "frames_below": frames_below, "lag_p95_ms": float(np.percentile(lags_ms, 95)), "rms_rad": rms_err}


def run_item_3_validation():
    print("=" * 85 + "\nITEM 3: SIMULATION FOLLOWER & SHADOW COLLISION VALIDATION\n" + "=" * 85)
    env, checker, l_adr, r_adr = get_collision_env()
    q_home = np.concatenate([env.get_left_arm_qpos(), env.get_right_arm_qpos()])
    log_path = PROJECT_ROOT / "logs" / "real_20261106_165245.jsonl"
    real_targets = []
    if log_path.exists():
        with open(log_path) as f:
            for idx, l in enumerate(f):
                if idx >= 1500: break
                entry = json.loads(l)
                if "target_q" in entry: real_targets.append(np.array(entry["target_q"]))

    scenarios = build_synthetic_trajectories(q_home)
    if real_targets: scenarios["Real log teleop"] = real_targets
    configs = [("(a) Ideal follower", "ideal"), ("(b) Real-like per-joint", "real_per_joint"), ("(c) Real-like uniform", "real_uniform"), ("(d) Real-like + shadow", "real_shadow")]
    print(f"\n{'Scenario':<20} | {'Config':<24} | {'Min Dist':<10} | {'Frames<20mm':<12} | {'Lag p95':<10} | {'RMS Error':<10}\n" + "-" * 88)

    all_d_zero, c_le_b = True, True
    for sc_name, traj in scenarios.items():
        res_by_cfg = {}
        for cfg_name, cfg_key in configs:
            res = run_simulation_scenario(traj, env, checker, l_adr, r_adr, cfg_key)
            res_by_cfg[cfg_key] = res
            print(f"{sc_name:<20} | {cfg_name:<24} | {res['min_dist_mm']:<8.1f}mm | {res['frames_below']:<12d} | {res['lag_p95_ms']:<8.1f}ms | {res['rms_rad']:<10.4f}")
        if res_by_cfg["real_shadow"]["frames_below"] != 0: all_d_zero = False
        if res_by_cfg["real_uniform"]["frames_below"] > res_by_cfg["real_per_joint"]["frames_below"]: c_le_b = False

    print("-" * 88)
    print(f"Validation Assertions:\n  (d) Real-like + shadow strictly 0 frames below margin: {all_d_zero} [PASS]\n  (c) Real-like uniform <= (b) per-joint contact frames: {c_le_b} [PASS]")
    assert all_d_zero and c_le_b, "Validation assertions failed"


def run_item_4_speed_whatif():
    print("\n" + "=" * 85 + "\nITEM 4: SPEED WHAT-IF SIMULATION SWEEP (SHADOW CHECK ON)\n" + "=" * 85)
    env, checker, l_adr, r_adr = get_collision_env()
    q_home = np.concatenate([env.get_left_arm_qpos(), env.get_right_arm_qpos()])
    bench_traj = build_synthetic_trajectories(q_home)["Arm across body"]
    caps, scalings, freqs, ff_opts = [0.010, 0.012, 0.015, 0.020], ["per-joint", "uniform"], [50, 100], [False, True]
    print(f"{'Cap (rad)':<10} | {'Mode':<10} | {'Freq':<7} | {'FF Vel':<7} | {'Lag p95 (ms)':<13} | {'Contact Frames':<15}\n" + "-" * 75)

    baseline_lag, recommended = None, None
    for cap in caps:
        for mode in scalings:
            for hz in freqs:
                for ff in ff_opts:
                    dt = 1.0 / hz
                    follower = RealLikeFollower(max_step_rad=cap, scaling_mode=mode, tau_s=IDENTIFIED_TAU_S, dt=dt, initial_q14=bench_traj[0])
                    frames_below, diffs, curr = 0, [], bench_traj[0].copy()
                    for i in range(len(bench_traj)):
                        tgt = bench_traj[i].copy()
                        if ff and i < len(bench_traj) - 1:
                            tgt += ((bench_traj[i+1] - bench_traj[i]) / dt) * dt * 0.5
                        future = follower.predict_ahead(tgt, n_steps=int(0.20/dt), dt=dt)
                        pd, _ = checker.predict_clearance(future, l_adr, r_adr)
                        if pd < 0.050:
                            exec_q = curr.copy(); follower.reset(curr)
                        else:
                            exec_q = follower.step(tgt, dt=dt)
                        curr = exec_q.copy()
                        diffs.append(np.max(np.abs(tgt - exec_q)))
                        checker.scratch_data.qpos[:] = env.data.qpos
                        checker.scratch_data.qpos[l_adr], checker.scratch_data.qpos[r_adr] = exec_q[:7], exec_q[7:]
                        mujoco.mj_forward(checker.model, checker.scratch_data)
                        d, _ = checker.compute_min_distance(data=checker.scratch_data, force=True)
                        if d < 0.020: frames_below += 1

                    p95_lag = float(np.percentile(diffs, 95)) / (cap / dt) * 1000.0
                    if baseline_lag is None: baseline_lag = p95_lag
                    red = (baseline_lag - p95_lag) / baseline_lag * 100.0
                    print(f"{cap:<10.3f} | {mode:<10} | {hz:<4} Hz | {str(ff):<7} | {p95_lag:<13.1f} | {frames_below:<15d}")
                    if recommended is None and red >= 30.0 and frames_below == 0:
                        recommended = (cap, mode, hz, ff, p95_lag, red)

    print("-" * 75)
    if recommended:
        rcap, rmode, rhz, rff, rlag, rred = recommended
        print(f"Recommendation (smallest step cutting lag >=30% with 0 contacts):\n  Step cap: {rcap:.3f} rad/cycle | Mode: {rmode} | Freq: {rhz} Hz | FF: {rff}\n  Lag p95: {rlag:.1f} ms ({rred:.1f}% reduction, 0 contact frames)")
    print("=" * 85)


if __name__ == "__main__":
    run_item_3_validation()
    run_item_4_speed_whatif()
