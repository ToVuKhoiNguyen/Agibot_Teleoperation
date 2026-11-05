#!/usr/bin/env python3
"""
scripts/test_hand_collision.py
Comprehensive numerical validation of hand collision avoidance:
1. Tests both left and right hands across all 5 grasp types (cylinder, pinch, tripod, point, thumbs_up).
2. Sweeps closure 0.0 -> 1.0 in 50 steps per grasp type.
3. Tests per-finger and per-joint sweeps.
4. Asserts min signed distance >= 0.0 mm over all intra-hand pairs.
5. Asserts closure 1.0 produces a visibly closed hand (fingertip-to-palm distance < 3 cm).
6. Verifies that candidate penetrating commands are safely backed off by filter_hand_step.
"""

import os
import sys
import numpy as np
import mujoco

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from teleop.retargeting.agibot_hand_retargeting import AgibotHandRetargeting
from teleop.utils.collision_checker import CollisionChecker


def main():
    print("=" * 70)
    print("TEST: HAND SELF-COLLISION AND SAFE RETARGETING VALIDATION")
    print("=" * 70)

    scene_xml = os.path.join(PROJECT_ROOT, "assets", "agibot_a2_ultra", "mjcf", "scene.xml")
    model = mujoco.MjModel.from_xml_path(scene_xml)
    data = mujoco.MjData(model)
    checker = CollisionChecker(model, enabled=True)

    fromto = np.zeros(6, dtype=np.float64)
    grasps = ["cylinder", "pinch", "tripod", "point", "thumbs_up"]
    steps = 50

    all_min_dists = {}

    for side in ("left", "right"):
        print(f"\n--- Testing {side.upper()} Hand Grasps (50 steps per grasp) ---")
        palm_geom = [
            gid for gid in range(model.ngeom)
            if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[gid]) == f"{side}_arm_link07"
            and model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH
            and (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, model.geom_dataid[gid]) or "") == "R_paw"
        ][0]

        tip_bodies = [f"{'L' if side == 'left' else 'R'}_{f}" for f in ("thumb_3", "index_2", "middle_2", "ring_2", "pinky_2")]
        tip_geoms = [
            gid for gid in range(model.ngeom)
            if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[gid]) in tip_bodies
            and ("hull" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, model.geom_dataid[gid]) or "") or model.geom_group[gid] == 0)
        ]

        for grasp in grasps:
            min_dist_grasp = 999.0
            worst_pair = None

            for s_idx in range(steps + 1):
                closure = float(s_idx) / float(steps)
                target_cmd = AgibotHandRetargeting.get_grasp_targets(grasp, closure)

                # SafetyLayer FK validation & line search gate
                safe_cmd, blocked, min_d = checker.filter_hand_step(side, np.zeros(6), target_cmd, margin=0.001)

                is_safe, dist, pair = checker.check_hand_collision(side, safe_cmd, margin=0.0)
                assert dist >= -1e-6, f"[{side.upper()} {grasp} cl={closure:.2f}] Penetration detected: {dist*1000:.2f} mm in {pair}"

                if dist < min_dist_grasp:
                    min_dist_grasp = dist
                    worst_pair = pair

            all_min_dists[f"{side}_{grasp}"] = min_dist_grasp
            print(f"  • {grasp:10s}: min dist = {min_dist_grasp*1000:6.2f} mm (closest pair: {worst_pair[2]}) [PASS]")

            # Check closure 1.0 produces visibly closed hand (fingertip-to-palm distance < 3 cm)
            full_cmd = AgibotHandRetargeting.get_grasp_targets(grasp, 1.0)
            safe_full_cmd, _, _ = checker.filter_hand_step(side, np.zeros(6), full_cmd, margin=0.001)
            checker.check_hand_collision(side, safe_full_cmd, margin=0.0)

            # Measure fingertip distances to palm
            curled_tips_under_3cm = 0
            for tg in tip_geoms:
                d_palm = mujoco.mj_geomDistance(model, checker.scratch_data, palm_geom, tg, 0.5, fromto)
                if d_palm < 0.03:  # < 3 cm
                    curled_tips_under_3cm += 1

            if grasp in ("cylinder", "fist", "power"):
                # All 4 curling fingers must be < 3 cm from palm
                assert curled_tips_under_3cm >= 4, f"[{side} {grasp}] Expected >= 4 fingertips < 3 cm, got {curled_tips_under_3cm}"
            elif grasp in ("point", "thumbs_up"):
                # At least 3 fingers curled < 3 cm
                assert curled_tips_under_3cm >= 3, f"[{side} {grasp}] Expected >= 3 fingertips < 3 cm, got {curled_tips_under_3cm}"

        print(f"\n--- Testing {side.upper()} Per-Joint & Per-Finger Sweeps ---")
        for j_idx in range(6):
            lim = AgibotHandRetargeting.COMFORTABLE_LIMITS[j_idx]
            for val in np.linspace(0.0, lim, 20):
                cmd = np.zeros(6)
                cmd[j_idx] = val
                safe_cmd, blocked, min_d = checker.filter_hand_step(side, np.zeros(6), cmd, margin=0.001)
                is_safe, dist, pair = checker.check_hand_collision(side, safe_cmd, margin=0.0)
                assert dist >= 0.0, f"Per-joint sweep failed at joint {j_idx}={val:.2f} rad: {dist*1000:.2f} mm"
        print(f"  • All 6 active joint sweeps PASSED with min_dist >= 0.0 mm")

    print("\n--- Testing Safety Gate on Deliberately Penetrating Commands ---")
    deep_penetrating_cmd = np.array([1.6, 0.78, 1.7, 1.7, 1.7, 1.7])
    for side in ("left", "right"):
        safe_cmd, blocked, safe_d = checker.filter_hand_step(side, np.zeros(6), deep_penetrating_cmd, margin=0.001)
        assert blocked is True, f"Expected blocked=True for penetrating command on {side}"
        assert safe_d >= 0.001, f"Expected safe_d >= 1.0 mm, got {safe_d*1000:.2f} mm"
        print(f"  • {side.upper()} penetrating command backed off to safe closure: min_dist = {safe_d*1000:.2f} mm [PASS]")

    overall_min = min(all_min_dists.values())
    print("\n" + "=" * 70)
    print(f"RESULTS: ALL HAND COLLISION CHECKS PASSED (Overall Min Dist: {overall_min*1000:.2f} mm)")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
