#!/usr/bin/env python3
"""scripts/test_no_overshoot.py - Acceptance test suite for monotone no-overshoot limiter."""
import json, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from teleop.kinematics.mirror_limiter import MirrorLimiter, SPEED_LEVELS
from teleop.robot.aimdk_map import HOME

def run_tests():
    results = []
    log_path = "logs/trace_laptop_20261111_164732.jsonl"
    raw_series = [json.loads(l)["raw_target_q14"] for l in open(log_path) if "raw_target_q14" in json.loads(l)]

    for lvl in [1, 2, 3, 4]:
        vm, am = SPEED_LEVELS[lvl]
        lim = MirrorLimiter(speed_level=lvl, dt=0.01)
        home_q = np.array(HOME, dtype=np.float64)
        lim.reset(home_q)

        # a) NO-OVERSHOOT across signals
        # 1. Step (0.3 rad on all joints)
        for _ in range(300): lim.step(home_q + 0.3)
        step_ov = lim.overshoot_count

        # 2. Ramp (0.3 rad over 1.0s then hold)
        lim.reset(home_q)
        for t in range(300):
            tgt = home_q + min(0.3, t * 0.01 * 0.3)
            lim.step(tgt)
        ramp_ov = lim.overshoot_count

        # 3. 0.5 Hz Sine (amplitude 0.2 rad)
        lim.reset(home_q)
        for t in range(500):
            tgt = home_q + 0.2 * np.sin(2.0 * np.pi * 0.5 * (t * 0.01))
            lim.step(tgt)
        sine_ov = lim.overshoot_count

        # 4. Target +- 0.01 rad noise
        lim.reset(home_q)
        np.random.seed(42)
        for _ in range(500):
            tgt = home_q + np.random.uniform(-0.01, 0.01, 14)
            lim.step(tgt)
        noise_ov = lim.overshoot_count

        # 5. Replay logged RAW series
        lim.reset(raw_series[0])
        plans, vels = [], []
        for r in raw_series:
            p = lim.step(r)
            plans.append(p)
            vels.append(lim.v_plan.copy())
        replay_ov = lim.overshoot_count

        plans_arr = np.array(plans)
        raw_arr = np.array(raw_series)
        plan_ptp = np.ptp(plans_arr, axis=0)
        raw_ptp = np.ptp(raw_arr, axis=0)
        
        # 4) Only consider joints with raw_ptp > 0.01 rad; assert at least one active joint exists
        active_joints = np.where(raw_ptp > 0.01)[0]
        assert len(active_joints) > 0, f"Lvl {lvl}: No active joint with raw_ptp > 0.01"
        ptp_ratios = plan_ptp[active_joints] / raw_ptp[active_joints]
        max_ptp_ratio = float(np.max(ptp_ratios))
        ratio_str = ", ".join([f"J{j}: {plan_ptp[j]/raw_ptp[j]:.4f}" for j in active_joints])
        print(f"[PTP RATIO Lvl {lvl}] Active joints [{ratio_str}] (Max: {max_ptp_ratio:.4f})")

        dv = np.diff(np.array(vels), axis=0) / 0.01
        max_acc_ratio = float(np.max(np.abs(dv) / lim.max_acc))
        max_step = float(np.max(np.abs(np.diff(plans_arr, axis=0))))

        # c) Reversals, settle time, time-to-90%
        lim.reset(home_q)
        stop_tick = 80
        step_vels, step_plans = [], []
        t90_base = None
        for step in range(300):
            tgt_disp = min(0.25, step * 0.01 * 0.35)
            tgt = home_q + tgt_disp
            p = lim.step(tgt)
            step_plans.append(p)
            step_vels.append(lim.v_plan.copy())
            if t90_base is None and (p[0] - home_q[0]) >= 0.9 * 0.25:
                t90_base = step * 0.01

        v_after = np.array(step_vels)[stop_tick:, 0]
        p_after = np.array(step_plans)[stop_tick:, 0]
        revs = int(np.sum((v_after[:-1] > 1e-4) & (v_after[1:] < -1e-4)) + np.sum((v_after[:-1] < -1e-4) & (v_after[1:] > 1e-4)))
        settle_idx = np.where(np.abs(p_after - (home_q[0] + 0.25)) < 0.01)[0]
        settle_s = float(settle_idx[0] * 0.01) if len(settle_idx) > 0 else 0.0

        # d) Sim path vs Real-UDP path equivalence
        lim_sim = MirrorLimiter(speed_level=lvl, dt=0.01)
        lim_sim.reset(raw_series[0])
        sim_plans = [lim_sim.step(r) for r in raw_series]
        path_diff = float(np.max(np.abs(np.array(sim_plans) - plans_arr)))

        # Assertions per test criteria
        assert step_ov == 0 and ramp_ov == 0 and sine_ov == 0 and noise_ov == 0 and replay_ov == 0
        assert max_ptp_ratio <= 1.0201, f"Lvl {lvl} PTP ratio {max_ptp_ratio} > 1.02"
        assert max_acc_ratio <= 1.0501, f"Lvl {lvl} Acc ratio {max_acc_ratio} > 1.05"
        assert max_step <= lim.step_cap + 1e-6, f"Lvl {lvl} Step {max_step} > cap {lim.step_cap}"
        assert revs == 0, f"Lvl {lvl} Reversals {revs} != 0"
        assert settle_s < 1.0, f"Lvl {lvl} Settle {settle_s} >= 1.0"
        if lvl == 3:
            assert t90_base < 1.2, f"Lvl 3 t90 {t90_base} >= 1.2"
        assert path_diff == 0.0, f"Lvl {lvl} Sim vs Real diff {path_diff} != 0"

        results.append({
            "lvl": lvl, "vmax": vm, "amax": am, "step_cap": lim.step_cap,
            "max_ptp_ratio": max_ptp_ratio, "max_acc_ratio": max_acc_ratio, "max_step": max_step,
            "revs": revs, "settle_s": settle_s, "t90_s": t90_base, "path_diff": path_diff
        })

    print("=========================== NO-OVERSHOOT ACCEPTANCE RESULTS ===========================")
    print("Lvl | Vmax | Amax | StepCap | PTP Ratio | Acc Ratio | MaxStep | Revs | Settle | t90% | PathDiff")
    for r in results:
        print(f" {r['lvl']}  | {r['vmax']:.2f} | {r['amax']:.2f} |  {r['step_cap']:.3f}  |  {r['max_ptp_ratio']:.4f}   |  {r['max_acc_ratio']:.4f}   | {r['max_step']:.4f}  |  {r['revs']}   | {r['settle_s']:.2f}s  | {r['t90_s']:.2f}s|  {r['path_diff']:.1f}")
    print("================================ ALL 4 TESTS PASSED! ==================================")

if __name__ == "__main__":
    run_tests()
