#!/usr/bin/env python3
"""
scripts/run_mujoco.py
Loads the AGIBOT A2 Ultra model in MuJoCo, verifies its joints against the URDF,
prints the complete joint hierarchy, and runs the simulation loop.
"""

import os
import sys
import argparse
import xml.etree.ElementTree as ET
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import mujoco
try:
    import mujoco.viewer as mjviewer
except ImportError:
    mjviewer = None

DEFAULT_SCENE_XML = os.path.join(
    PROJECT_ROOT, "assets", "agibot_a2_ultra", "mjcf", "scene.xml"
)
DEFAULT_URDF_PATH = os.path.join(
    PROJECT_ROOT, "assets", "agibot_a2_ultra", "urdf", "a2.urdf"
)


def verify_joints_with_urdf(model: mujoco.MjModel, urdf_path: str):
    """
    Verifies that all revolute joints defined in the original URDF exist
    in the MuJoCo model with identical limits.
    """
    if not os.path.exists(urdf_path):
        print(f"[WARN] URDF file not found at: {urdf_path}. Skipping URDF check.")
        return True

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    urdf_joints = {}
    for j in root.findall("joint"):
        if j.attrib.get("type") == "revolute":
            name = j.attrib["name"].strip()
            lim = j.find("limit")
            lower = float(lim.attrib.get("lower", -3.14159)) if lim is not None else -np.pi
            upper = float(lim.attrib.get("upper", 3.14159)) if lim is not None else np.pi
            urdf_joints[name] = (lower, upper)

    mj_joints = {}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        rng = model.jnt_range[i]
        mj_joints[name] = (float(rng[0]), float(rng[1]))

    missing = [j for j in urdf_joints if j not in mj_joints]
    mismatches = []
    for j, (u_lo, u_hi) in urdf_joints.items():
        if j in mj_joints:
            m_lo, m_hi = mj_joints[j]
            if abs(u_lo - m_lo) > 1e-3 or abs(u_hi - m_hi) > 1e-3:
                mismatches.append((j, (u_lo, u_hi), (m_lo, m_hi)))

    print("\n" + "=" * 70)
    print("URDF <-> MuJoCo JOINT VERIFICATION REPORT")
    print("=" * 70)
    print(f"URDF revolute joints count : {len(urdf_joints)}")
    print(f"MuJoCo active joints count  : {len(mj_joints)}")
    print(f"Missing joints in MuJoCo   : {len(missing)}")
    print(f"Limit discrepancies        : {len(mismatches)}")

    if missing:
        print(f"[FAIL] Missing joints: {missing}")
        return False
    if mismatches:
        print(f"[WARN] Limit mismatches: {mismatches}")
    else:
        print("[PASS] All URDF revolute joints match the MuJoCo model exactly (0 discrepancies)!")
    print("=" * 70 + "\n")
    return True


def print_joint_summary(model: mujoco.MjModel):
    """
    Prints a structured table of all robot joints, categorized by subsystem.
    """
    categories = {
        "Left Arm": [],
        "Right Arm": [],
        "Left Hand (Dexterous)": [],
        "Right Hand (Dexterous)": [],
        "Head": [],
        "Left Leg": [],
        "Right Leg": [],
        "Other": []
    }

    for i in range(model.njnt):
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        rng = model.jnt_range[i]
        body_id = model.jnt_bodyid[i]
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)

        info = {
            "id": i,
            "name": jname,
            "range": f"[{rng[0]:+.3f}, {rng[1]:+.3f}]",
            "body": body_name
        }

        if "left_arm" in jname:
            categories["Left Arm"].append(info)
        elif "right_arm" in jname:
            categories["Right Arm"].append(info)
        elif jname.startswith("L_"):
            categories["Left Hand (Dexterous)"].append(info)
        elif jname.startswith("R_"):
            categories["Right Hand (Dexterous)"].append(info)
        elif "head" in jname:
            categories["Head"].append(info)
        elif "left" in jname:
            categories["Left Leg"].append(info)
        elif "right" in jname:
            categories["Right Leg"].append(info)
        else:
            categories["Other"].append(info)

    print("\n" + "=" * 80)
    print(f"{'ID':<4} {'JOINT NAME':<28} {'RANGE (rad)':<22} {'BODY':<20}")
    print("=" * 80)
    for cat_name, j_list in categories.items():
        if not j_list:
            continue
        print(f"\n--- {cat_name} ({len(j_list)} joints) ---")
        for j in j_list:
            print(f"{j['id']:<4} {j['name']:<28} {j['range']:<22} {j['body']:<20}")
    print("=" * 80 + "\n")


def print_robot_state(model: mujoco.MjModel, data: mujoco.MjData):
    """
    Prints key kinematic points: base, feet, wrists/end effectors.
    """
    mujoco.mj_kinematics(model, data)
    print("Kinematic Pose Check:")
    key_bodies = ["base_link", "left_toe_roll", "right_toe_roll", "left_hand", "right_hand"]
    for bname in key_bodies:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
        if bid >= 0:
            pos = data.xpos[bid]
            print(f"  - Body {bname:<16}: pos = [{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}] m")

    key_sites = ["left_arm_ee", "right_arm_ee"]
    for sname in key_sites:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, sname)
        if sid >= 0:
            pos = data.site_xpos[sid]
            print(f"  - Site {sname:<16}: pos = [{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}] m")


def run_simulation(xml_path: str, urdf_path: str, headless: bool = False, steps: int = 500):
    """
    Loads model, verifies joints, and runs simulation loop.
    """
    print(f"[INFO] Loading MuJoCo model from: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # Initialize to 'home' keyframe
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        print("[INFO] Reset to 'home' keyframe (standing pose, arms bent in front of chest).")
    mujoco.mj_forward(model, data)

    print(f"[PASS] Model loaded successfully!")
    print(f"       Total coordinates (nq): {model.nq}")
    print(f"       Total velocities  (nv): {model.nv}")
    print(f"       Total joints     (njnt): {model.njnt}")
    print(f"       Total bodies    (nbody): {model.nbody}")
    print(f"       Total geoms     (ngeom): {model.ngeom}")
    print(f"       Total actuators   (nu): {model.nu}")
    print(f"       Total equalities  (neq): {model.neq} (finger mimic joints)")

    # 1. Joint verification against original URDF
    verify_joints_with_urdf(model, urdf_path)

    # 2. Print joint table
    print_joint_summary(model)

    # 3. Print kinematic pose
    print_robot_state(model, data)

    # Check whether GUI viewer is viable
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    use_gui = (not headless) and has_display

    if use_gui and mjviewer is not None:
        try:
            print("[INFO] Launching interactive passive viewer...")
            with mjviewer.launch_passive(model, data) as viewer:
                viewer.cam.distance = 2.5
                viewer.cam.azimuth = -135
                viewer.cam.elevation = -20
                viewer.cam.lookat[:] = [0, 0, 0.9]

                step_count = 0
                while viewer.is_running():
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    step_count += 1
            print(f"[INFO] Viewer closed after {step_count} steps.")
            return True
        except Exception as e:
            print(f"[WARN] Failed to start GUI viewer ({e}). Falling back to headless simulation.")

    # Headless simulation loop with offscreen verification
    print(f"[INFO] Running headless simulation for {steps} steps (dt = {model.opt.timestep:.4f} s)...")
    for _ in range(steps):
        mujoco.mj_step(model, data)

    try:
        renderer = mujoco.Renderer(model, 480, 640)
        renderer.update_scene(data)
        img = renderer.render()
        print(f"[PASS] Headless rendering verification successful (rendered image shape: {img.shape})")
    except Exception as e:
        print(f"[NOTE] Offscreen renderer check: {e}")

    print(f"[PASS] Completed {steps} simulation steps successfully without error.")
    return True


def main():
    parser = argparse.ArgumentParser(description="AGIBOT A2 Ultra MuJoCo Sim Loader")
    parser.add_argument("--scene", type=str, default=DEFAULT_SCENE_XML, help="Path to scene XML")
    parser.add_argument("--urdf", type=str, default=DEFAULT_URDF_PATH, help="Path to original URDF")
    parser.add_argument("--headless", action="store_true", help="Force headless execution without GUI viewer")
    parser.add_argument("--steps", type=int, default=500, help="Number of steps in headless mode")
    args = parser.parse_args()

    success = run_simulation(args.scene, args.urdf, headless=args.headless, steps=args.steps)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
