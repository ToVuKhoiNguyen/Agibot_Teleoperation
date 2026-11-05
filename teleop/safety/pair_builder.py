"""
teleop/safety/pair_builder.py
Discovers collision geometry hulls and builds pair lists:
- Body/torso/arm/hand collision pairs
- Intra-hand finger-finger and finger-palm collision pairs
"""

import mujoco


def discover_collision_geoms(model: mujoco.MjModel) -> dict:
    """Discovers visual/hull collision geoms for torso, head, arms, and hands."""
    torso_geoms = []
    head_geoms = []
    left_arm_geoms = []
    left_hand_geoms = []
    right_arm_geoms = []
    right_hand_geoms = []

    l_hand_prefixes = ("L_thumb", "L_index", "L_middle", "L_ring", "L_pinky")
    r_hand_prefixes = ("R_thumb", "R_index", "R_middle", "R_ring", "R_pinky")

    for gid in range(model.ngeom):
        bid = model.geom_bodyid[gid]
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
        mid = model.geom_dataid[gid]
        mname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid) if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH else ""

        is_hull = (model.geom_group[gid] == 0) or ("hull" in mname) or ("hull" in gname)
        if not is_hull:
            continue

        if bname == "base_link":
            torso_geoms.append(gid)
        elif "head" in bname:
            head_geoms.append(gid)
        elif bname in ("left_arm_link03", "left_arm_link04", "left_arm_link05", "left_arm_link06", "left_arm_link07"):
            left_arm_geoms.append(gid)
        elif bname in ("right_arm_link03", "right_arm_link04", "right_arm_link05", "right_arm_link06", "right_arm_link07"):
            right_arm_geoms.append(gid)
        elif any(bname.startswith(p) for p in l_hand_prefixes):
            left_hand_geoms.append(gid)
        elif any(bname.startswith(p) for p in r_hand_prefixes):
            right_hand_geoms.append(gid)

    return {
        "torso_geoms": torso_geoms,
        "head_geoms": head_geoms,
        "left_arm_geoms": left_arm_geoms,
        "left_hand_geoms": left_hand_geoms,
        "right_arm_geoms": right_arm_geoms,
        "right_hand_geoms": right_hand_geoms,
    }


def build_collision_pairs(
    torso_geoms: list,
    head_geoms: list,
    left_arm_geoms: list,
    left_hand_geoms: list,
    right_arm_geoms: list,
    right_hand_geoms: list,
) -> list:
    """Constructs list of (geom1, geom2, margin_category)."""
    pairs = []

    # 1. Torso vs Left Arm & Hand
    for tg in torso_geoms:
        for ag in left_arm_geoms:
            pairs.append((tg, ag, "torso_arm"))
        for hg in left_hand_geoms:
            pairs.append((tg, hg, "torso_hand"))

    # 2. Torso vs Right Arm & Hand
    for tg in torso_geoms:
        for ag in right_arm_geoms:
            pairs.append((tg, ag, "torso_arm"))
        for hg in right_hand_geoms:
            pairs.append((tg, hg, "torso_hand"))

    # 3. Head vs Left Arm & Hand
    for hg in head_geoms:
        for ag in left_arm_geoms:
            pairs.append((hg, ag, "head_arm"))
        for lh in left_hand_geoms:
            pairs.append((hg, lh, "head_hand"))

    # 4. Head vs Right Arm & Hand
    for hg in head_geoms:
        for ag in right_arm_geoms:
            pairs.append((hg, ag, "head_arm"))
        for rh in right_hand_geoms:
            pairs.append((hg, rh, "head_hand"))

    # 5. Left Arm/Hand vs Right Arm/Hand
    for lg in left_arm_geoms:
        for rg in right_arm_geoms:
            pairs.append((lg, rg, "arm_arm"))
        for rh in right_hand_geoms:
            pairs.append((lg, rh, "arm_arm"))

    for lh in left_hand_geoms:
        for rg in right_arm_geoms:
            pairs.append((lh, rg, "arm_arm"))
        for rh in right_hand_geoms:
            pairs.append((lh, rh, "hand_hand"))

    return pairs


def build_hand_collision_pairs(model: mujoco.MjModel) -> dict:
    """Constructs intra-hand collision pairs for finger-finger and finger-palm pairs."""
    hand_pairs = {"left": [], "right": []}
    excluded = {
        ("thumb_swing", "thumb_1"),
        ("thumb_1", "thumb_2"),
        ("thumb_2", "thumb_3"),
        ("thumb_swing", "thumb_2"),
        ("thumb_swing", "thumb_3"),
        ("index_1", "index_2"),
        ("middle_1", "middle_2"),
        ("ring_1", "ring_2"),
        ("pinky_1", "pinky_2"),
        ("thumb_swing", "palm"),
        ("thumb_1", "palm"),
        ("index_1", "middle_1"),
        ("middle_1", "ring_1"),
        ("ring_1", "pinky_1"),
    }

    for side in ("left", "right"):
        p = "L_" if side == "left" else "R_"
        palm_bname = "left_arm_link07" if side == "left" else "right_arm_link07"
        body_geoms = {}
        for gid in range(model.ngeom):
            bid = model.geom_bodyid[gid]
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""
            mid = model.geom_dataid[gid]
            mname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mid) if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_MESH else ""
            if mname == "R_paw" and bname == palm_bname:
                body_geoms["palm"] = gid
            elif ("hull" in mname or model.geom_group[gid] == 0) and bname.startswith(p):
                key = bname[len(p):]
                body_geoms[key] = gid

        pairs = []
        keys = list(body_geoms.keys())
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                k1, k2 = keys[i], keys[j]
                if (k1, k2) in excluded or (k2, k1) in excluded:
                    continue
                if "palm" in (k1, k2) and any(b in (k1, k2) for b in ["index_1", "middle_1", "ring_1", "pinky_1"]):
                    continue
                pairs.append((body_geoms[k1], body_geoms[k2], f"{k1}_{k2}"))
        hand_pairs[side] = pairs

    return hand_pairs
