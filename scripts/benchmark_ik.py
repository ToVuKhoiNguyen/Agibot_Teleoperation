#!/usr/bin/env python3
"""
scripts/benchmark_ik.py
Comprehensive Benchmark comparing DLS vs Mink Inverse Kinematics on AGIBOT A2 Ultra:
Evaluates:
  1. Smooth Circle
  2. Fast Line
  3. Noisy Hand Trajectory (3 mm Gaussian jitter)
  4. Near-Limit Path (workspace boundary)
  5. Near-Singular Path (stretched reach)
Metrics computed:
  - Mean Position Error (mm)
  - Mean Orientation Error (deg)
  - Max Joint Velocity (rad/s)
  - Max Joint Acceleration (rad/s2)
  - Jerk RMS (rad/s3)
  - Number of Elbow Flips (>0.3 rad jump in 1 step)
  - Solve Time p50 / p95 (ms)
  - Constraint Violations count
Saves results to:
  - scripts/benchmark_results.csv
  - scripts/benchmark_comparison.png
"""

import os
import sys
import time
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from teleop.kinematics.agibot_a2_ik import AgibotA2IK
from teleop.kinematics.mink_ik import AgibotMinkIK, REAL_ROBOT_LIMITS
from teleop.kinematics.filters import InputFilterPipeline, JointTrajectoryFilter

SCENE_XML = str(PROJECT_ROOT / "assets" / "agibot_a2_ultra" / "mjcf" / "scene.xml")


def generate_trajectories(home_pos, home_rot, num_steps=300, dt=0.02):
    trajectories = {}

    # 1. Smooth Circle (Y-Z plane, radius 8 cm, 0.5 Hz)
    circle = []
    for i in range(num_steps):
        t = i * dt
        dy = 0.08 * np.sin(2 * np.pi * 0.5 * t)
        dz = 0.08 * np.cos(2 * np.pi * 0.5 * t)
        p = home_pos + np.array([0.0, dy, dz])
        circle.append((p, home_rot))
    trajectories["Smooth Circle"] = circle

    # 2. Fast Line (Forward X line, 15 cm in 1 sec, peak speed 0.3 m/s)
    fast_line = []
    for i in range(num_steps):
        t = i * dt
        dx = 0.15 * min(1.0, t / 1.0)
        p = home_pos + np.array([dx, 0.0, 0.0])
        fast_line.append((p, home_rot))
    trajectories["Fast Line"] = fast_line

    # 3. Noisy Hand Trajectory (Circle + 3 mm standard deviation Gaussian jitter)
    np.random.seed(42)
    noisy_circle = []
    for i in range(num_steps):
        t = i * dt
        dy = 0.06 * np.sin(2 * np.pi * 0.5 * t)
        dz = 0.06 * np.cos(2 * np.pi * 0.5 * t)
        noise = np.random.normal(0.0, 0.003, 3) # 3 mm jitter
        p = home_pos + np.array([0.0, dy, dz]) + noise
        noisy_circle.append((p, home_rot))
    trajectories["Noisy Hand (3mm Jitter)"] = noisy_circle

    # 4. Near-Limit Path (wrist pitch / yaw limit approach)
    near_limit = []
    for i in range(num_steps):
        t = i * dt
        p = home_pos + np.array([0.10 * np.sin(np.pi * t), 0.05 * np.cos(np.pi * t), -0.05])
        # Pitch tilt near real robot 0.45 rad limit
        angle = 0.40 * np.sin(np.pi * t)
        rot = home_rot @ np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])
        near_limit.append((p, rot))
    trajectories["Near-Limit Path"] = near_limit

    # 5. Near-Singular Path (stretched forward reach to X = 0.65 m)
    near_singular = []
    for i in range(num_steps):
        frac = i / float(num_steps)
        p = home_pos + np.array([0.42 * frac, 0.0, 0.0])
        near_singular.append((p, home_rot))
    trajectories["Near-Singular Path"] = near_singular

    return trajectories


def run_benchmark():
    print("=" * 85)
    print("AGIBOT A2 ULTRA INVERSE KINEMATICS BENCHMARK: DLS vs MINK (QP)")
    print("=" * 85)

    dls_solver = AgibotA2IK(SCENE_XML, arm="left")
    mink_solver = AgibotMinkIK(SCENE_XML, arm="left", use_real_robot_profile=True)

    home_pos, home_rot = dls_solver.forward_kinematics(dls_solver.nominal_qpos)
    dt = 0.02
    trajectories = generate_trajectories(home_pos, home_rot, num_steps=250, dt=dt)

    # Real robot joint bounds for left arm
    joint_names = [f"idx{i}_left_arm_joint{i-12}" for i in range(13, 20)]
    bounds = np.array([REAL_ROBOT_LIMITS[j] for j in joint_names])

    results = []

    for traj_name, points in trajectories.items():
        for solver_name, solver, use_filters in [("DLS", dls_solver, False), ("Mink (QP)", mink_solver, True)]:
            q_history = []
            pos_errors = []
            rot_errors = []
            solve_times = []
            violations = 0
            elbow_flips = 0

            # State & filters
            q_cur = solver.nominal_qpos.copy()
            if use_filters:
                in_filter = InputFilterPipeline()
                out_filter = JointTrajectoryFilter(num_joints=7, max_vel=3.0, max_acc=6.28)
                out_filter.reset(q_cur)

            for p_tgt, r_tgt in points:
                if use_filters:
                    p_tgt, r_tgt = in_filter.filter(p_tgt, r_tgt)

                t0 = time.perf_counter()
                if solver_name == "DLS":
                    q_sol, succ, pe, re, _ = solver.solve(p_tgt, r_tgt, init_qpos=q_cur, max_iters=25)
                else:
                    q_sol, succ, pe, re, _ = solver.solve(p_tgt, r_tgt, init_qpos=q_cur, max_iters=5)
                t_solve = (time.perf_counter() - t0) * 1000.0
                solve_times.append(t_solve)

                if use_filters:
                    q_sol = out_filter.filter(q_sol)

                # Check constraint violations against real robot profile
                for j_idx in range(7):
                    if q_sol[j_idx] < bounds[j_idx, 0] - 1e-3 or q_sol[j_idx] > bounds[j_idx, 1] + 1e-3:
                        violations += 1

                # Check elbow flips (joint 4 / index 3 jump > 0.3 rad in 1 step)
                if len(q_history) > 0:
                    elbow_jump = abs(q_sol[3] - q_history[-1][3])
                    if elbow_jump > 0.30:
                        elbow_flips += 1

                q_history.append(q_sol.copy())
                q_cur = q_sol.copy()
                pos_errors.append(pe * 1000.0) # mm
                rot_errors.append(re)          # deg

            q_arr = np.array(q_history)
            # Velocity, Acceleration, Jerk
            vel = np.diff(q_arr, axis=0) / dt
            acc = np.diff(vel, axis=0) / dt
            jerk = np.diff(acc, axis=0) / dt

            max_v = float(np.max(np.abs(vel)))
            max_a = float(np.max(np.abs(acc)))
            jerk_rms = float(np.sqrt(np.mean(jerk ** 2)))
            p50 = float(np.percentile(solve_times, 50))
            p95 = float(np.percentile(solve_times, 95))
            mean_pe = float(np.mean(pos_errors))
            mean_re = float(np.mean(rot_errors))

            results.append({
                "trajectory": traj_name,
                "solver": solver_name,
                "mean_pos_err_mm": mean_pe,
                "mean_rot_err_deg": mean_re,
                "max_vel_rad_s": max_v,
                "max_acc_rad_s2": max_a,
                "jerk_rms": jerk_rms,
                "elbow_flips": elbow_flips,
                "solve_p50_ms": p50,
                "solve_p95_ms": p95,
                "violations": violations
            })

    # Output formatted markdown table
    print("\n" + "=" * 115)
    print(f"{'Trajectory':<22} | {'Solver':<9} | {'Pos Err (mm)':<12} | {'Max Vel':<8} | {'Jerk RMS':<9} | {'Elbow Flips':<11} | {'p95 (ms)':<8} | {'Violations'}")
    print("-" * 115)
    for r in results:
        print(f"{r['trajectory']:<22} | {r['solver']:<9} | {r['mean_pos_err_mm']:<12.2f} | {r['max_vel_rad_s']:<8.2f} | {r['jerk_rms']:<9.1f} | {r['elbow_flips']:<11} | {r['solve_p95_ms']:<8.2f} | {r['violations']}")
    print("=" * 115)

    # Save CSV
    csv_path = PROJECT_ROOT / "scripts" / "benchmark_results.csv"
    with open(csv_path, "w") as f:
        f.write("Trajectory,Solver,Mean_Pos_Err_mm,Mean_Rot_Err_deg,Max_Vel_rad_s,Max_Acc_rad_s2,Jerk_RMS,Elbow_Flips,Solve_p50_ms,Solve_p95_ms,Violations\n")
        for r in results:
            f.write(f"{r['trajectory']},{r['solver']},{r['mean_pos_err_mm']:.3f},{r['mean_rot_err_deg']:.3f},{r['max_vel_rad_s']:.3f},{r['max_acc_rad_s2']:.3f},{r['jerk_rms']:.3f},{r['elbow_flips']},{r['solve_p50_ms']:.3f},{r['solve_p95_ms']:.3f},{r['violations']}\n")
    print(f"\n[INFO] Saved benchmark metrics to CSV: {csv_path}")

    # Plot PNG comparison
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        trajs = list(trajectories.keys())
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        # 1. Jerk RMS
        ax1 = axes[0, 0]
        dls_jerk = [r["jerk_rms"] for r in results if r["solver"] == "DLS"]
        mink_jerk = [r["jerk_rms"] for r in results if r["solver"] == "Mink (QP)"]
        x = np.arange(len(trajs))
        width = 0.35
        ax1.bar(x - width/2, dls_jerk, width, label="DLS", color="#e57373")
        ax1.bar(x + width/2, mink_jerk, width, label="Mink (QP)", color="#4db6ac")
        ax1.set_ylabel("Jerk RMS (rad/s³)")
        ax1.set_title("Jerk Smoothness (Lower is Better)")
        ax1.set_xticks(x)
        ax1.set_xticklabels([t.replace(" ", "\n") for t in trajs], fontsize=8)
        ax1.legend()
        ax1.grid(True, linestyle="--", alpha=0.5)

        # 2. Position Error
        ax2 = axes[0, 1]
        dls_pos = [r["mean_pos_err_mm"] for r in results if r["solver"] == "DLS"]
        mink_pos = [r["mean_pos_err_mm"] for r in results if r["solver"] == "Mink (QP)"]
        ax2.bar(x - width/2, dls_pos, width, label="DLS", color="#e57373")
        ax2.bar(x + width/2, mink_pos, width, label="Mink (QP)", color="#4db6ac")
        ax2.set_ylabel("Mean Position Error (mm)")
        ax2.set_title("Tracking Accuracy (< 5 mm Reachable)")
        ax2.set_xticks(x)
        ax2.set_xticklabels([t.replace(" ", "\n") for t in trajs], fontsize=8)
        ax2.legend()
        ax2.grid(True, linestyle="--", alpha=0.5)

        # 3. Solve Time p95
        ax3 = axes[1, 0]
        dls_p95 = [r["solve_p95_ms"] for r in results if r["solver"] == "DLS"]
        mink_p95 = [r["solve_p95_ms"] for r in results if r["solver"] == "Mink (QP)"]
        ax3.bar(x - width/2, dls_p95, width, label="DLS", color="#e57373")
        ax3.bar(x + width/2, mink_p95, width, label="Mink (QP)", color="#4db6ac")
        ax3.set_ylabel("Solve Time p95 (ms)")
        ax3.set_title("Computational Latency (p95 < 5 ms)")
        ax3.set_xticks(x)
        ax3.set_xticklabels([t.replace(" ", "\n") for t in trajs], fontsize=8)
        ax3.legend()
        ax3.grid(True, linestyle="--", alpha=0.5)

        # 4. Limit Violations
        ax4 = axes[1, 1]
        dls_viol = [r["violations"] for r in results if r["solver"] == "DLS"]
        mink_viol = [r["violations"] for r in results if r["solver"] == "Mink (QP)"]
        ax4.bar(x - width/2, dls_viol, width, label="DLS", color="#e57373")
        ax4.bar(x + width/2, mink_viol, width, label="Mink (QP)", color="#4db6ac")
        ax4.set_ylabel("Violations Count")
        ax4.set_title("Real Robot Hardware Limit Violations (Goal: 0)")
        ax4.set_xticks(x)
        ax4.set_xticklabels([t.replace(" ", "\n") for t in trajs], fontsize=8)
        ax4.legend()
        ax4.grid(True, linestyle="--", alpha=0.5)

        plt.tight_layout()
        plot_path = PROJECT_ROOT / "scripts" / "benchmark_comparison.png"
        plt.savefig(plot_path, dpi=180)
        print(f"[INFO] Saved comparison plot to: {plot_path}")
    except Exception as e:
        print(f"[WARN] Matplotlib plotting skipped: {e}")

    # Pass verification check
    noisy_dls = next(r for r in results if r["trajectory"] == "Noisy Hand (3mm Jitter)" and r["solver"] == "DLS")
    noisy_mink = next(r for r in results if r["trajectory"] == "Noisy Hand (3mm Jitter)" and r["solver"] == "Mink (QP)")
    jerk_reduction_ratio = noisy_dls["jerk_rms"] / max(1e-3, noisy_mink["jerk_rms"])
    print(f"\n[VERIFICATION] Jerk RMS Reduction Ratio on Noisy Trajectory: {jerk_reduction_ratio:.2f}x (Required: >= 3x)")

    total_mink_violations = sum(r["violations"] for r in results if r["solver"] == "Mink (QP)")
    print(f"[VERIFICATION] Total Mink Constraint Violations: {total_mink_violations} (Required: 0)")

    total_mink_elbow_flips = sum(r["elbow_flips"] for r in results if r["solver"] == "Mink (QP)")
    print(f"[VERIFICATION] Total Mink Elbow Flips: {total_mink_elbow_flips} (Required: 0)")

    mink_p95_max = max(r["solve_p95_ms"] for r in results if r["solver"] == "Mink (QP)")
    print(f"[VERIFICATION] Max Mink Solve Time p95: {mink_p95_max:.2f} ms (Required: < 5 ms)")

    assert total_mink_violations == 0, "Mink had constraint violations!"
    assert total_mink_elbow_flips == 0, "Mink had elbow flips!"
    assert jerk_reduction_ratio >= 3.0, f"Jerk reduction {jerk_reduction_ratio:.2f}x was less than 3x!"
    assert mink_p95_max < 5.0, f"Mink p95 {mink_p95_max:.2f} ms was >= 5 ms!"

    print("\n[PASS] ALL BENCHMARK CRITERIA MET AND VERIFIED!")
    return results


if __name__ == "__main__":
    run_benchmark()

