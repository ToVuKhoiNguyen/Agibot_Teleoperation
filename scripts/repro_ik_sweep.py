"""
scripts/repro_ik_sweep.py
Runs the scripted reproduction sweep from HOME for left and right arms in +z, +x, +y, -z directions,
both continuous and with release after each step.
"""

import sys
import numpy as np
from main import UnifiedTeleopApp
from teleop.app.cmd_system import handle_system_cmd
from teleop.app.cmd_targets import handle_target_cmd
from teleop.app.ik_dispatcher import freeze_arm_target

SHOULDER_ORIGIN_LEFT = np.array([-0.03779, 0.19100, 1.27210], dtype=np.float64)
SHOULDER_ORIGIN_RIGHT = np.array([-0.03779, -0.19100, 1.27210], dtype=np.float64)
DEFAULT_MAX_REACH_M = 0.500

def project_to_reachable_workspace(target_pos, side="left", max_reach=DEFAULT_MAX_REACH_M):
    shldr = SHOULDER_ORIGIN_LEFT if side.lower() == "left" else SHOULDER_ORIGIN_RIGHT
    pos = np.asarray(target_pos, dtype=np.float64)
    v = pos - shldr
    dist = float(np.linalg.norm(v))
    if dist > max_reach and dist > 1e-6:
        proj_pos = shldr + v * (max_reach / dist)
        return proj_pos, True, dist - max_reach
    return pos.copy(), False, 0.0


def run_sweep(port=8091):
    app = UnifiedTeleopApp(backend="sim", render_viewer=False, no_adb=True, no_quest_launch=True, port=port)

    arms = ["left", "right"]
    directions = [
        ("z", 1.0, "+z"),
        ("x", 1.0, "+x"),
        ("y", 1.0, "+y"),
        ("z", -1.0, "-z"),
    ]
    modes = ["no_release", "with_release"]

    results = {}

    for arm in arms:
        results[arm] = {}
        for axis_str, dir_val, dir_name in directions:
            results[arm][dir_name] = {}
            for mode in modes:
                # Reset to HOME
                handle_target_cmd(app, "preset", {"cmd": "preset", "pose": "reset"})
                # Run steps
                max_err_raw = 0.0
                max_err_proj = 0.0
                step_gt_10 = None
                step_gt_50 = None
                step_logs = []

                for step_idx in range(1, 26): # 25 steps * 2cm = 0.50m
                    handle_system_cmd(app, "arm_jog", {
                        "cmd": "arm_jog",
                        "arm": arm,
                        "axis": axis_str,
                        "dir": dir_val,
                        "step_cm": 2.0
                    })
                    if mode == "with_release":
                        handle_target_cmd(app, "gizmo_end", {"cmd": "gizmo_end", "arm": arm})

                    cur_tgt = getattr(app, f"cur_target_pos_{arm[0]}").copy()
                    proj_tgt, is_c, _ = project_to_reachable_workspace(cur_tgt, side=arm)

                    q_sol = getattr(app, f"last_ik_sol_{arm}").copy()
                    ik_s = getattr(app, f"ik_{arm}")
                    for idx, val in zip(ik_s.qpos_indices, q_sol):
                        ik_s.conf.data.qpos[idx] = val
                    ik_s.conf.update()
                    fk_pos = np.array(ik_s.conf.get_transform_frame_to_world(ik_s.site_name, "site").translation(), dtype=np.float64)

                    err_raw = float(np.linalg.norm(fk_pos - cur_tgt)) * 1000.0 # mm
                    err_proj = float(np.linalg.norm(fk_pos - proj_tgt)) * 1000.0 # mm

                    if err_proj > max_err_proj:
                        max_err_proj = err_proj
                    if err_raw > max_err_raw:
                        max_err_raw = err_raw

                    if err_proj > 10.0 and step_gt_10 is None:
                        step_gt_10 = step_idx
                    if err_proj > 50.0 and step_gt_50 is None:
                        step_gt_50 = step_idx

                    ik_res = getattr(app, f"last_ik_residual_{arm[0]}", 0.0) * 1000.0

                    step_logs.append({
                        "step": step_idx,
                        "raw_tgt": cur_tgt,
                        "proj_tgt": proj_tgt,
                        "fk_pos": fk_pos,
                        "err_raw": err_raw,
                        "err_proj": err_proj,
                        "ik_residual": ik_res,
                    })

                results[arm][dir_name][mode] = {
                    "max_err_raw": max_err_raw,
                    "max_err_proj": max_err_proj,
                    "first_gt_10": step_gt_10,
                    "first_gt_50": step_gt_50,
                    "final_err_raw": step_logs[-1]["err_raw"],
                    "final_err_proj": step_logs[-1]["err_proj"],
                }

    return results


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8091
    res = run_sweep(port=port)
    import json
    # Print formatted table
    print(f"\n{'Arm':<6} | {'Dir':<4} | {'Mode':<12} | {'MaxRaw(mm)':<10} | {'MaxProj(mm)':<11} | {'>10mm Step':<10} | {'>50mm Step':<10}")
    print("-" * 75)
    for arm in res:
        for d in res[arm]:
            for m in res[arm][d]:
                r = res[arm][d][m]
                s10 = str(r["first_gt_10"]) if r["first_gt_10"] is not None else "-"
                s50 = str(r["first_gt_50"]) if r["first_gt_50"] is not None else "-"
                print(f"{arm:<6} | {d:<4} | {m:<12} | {r['max_err_raw']:<10.1f} | {r['max_err_proj']:<11.1f} | {s10:<10} | {s50:<10}")
