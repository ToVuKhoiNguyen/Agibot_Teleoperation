"""
teleop/hand/synth_hand.py
Synthesizes 25-joint WebXR hand tracking frames for automated headless tests.
Produces complete kinematic bone chains for thumb and all four fingers.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R


def make_synth_hand(wrist_pos=None, flexions=None, opp=0.0, quat=None, facing_robot=True):
    """Generates synthetic 25-joint WebXR hand data matching user specifications."""
    p = np.array(wrist_pos if wrist_pos is not None else [0.2, 1.2, -0.4], dtype=np.float64)
    fl = np.array(flexions if flexions is not None else [0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)
    q = quat if quat is not None else [1.0, 0.0, 0.0, 0.0]
    j = np.zeros((25, 3), dtype=np.float64)
    j[0] = p

    # Thumb: 1: CMC, 2: MCP, 3: IP, 4: Tip
    th_ang = 0.08 + fl[0] * 1.35
    j[1] = p + np.array([-0.02, 0.01, -0.03])
    j[2] = p + np.array([-0.03, 0.02, -0.05])
    v_prox = j[2] - j[1]
    v_norm = v_prox / (np.linalg.norm(v_prox) + 1e-8)
    perp = np.array([1.0, 1.0, 0.0])
    perp = perp - np.dot(perp, v_norm) * v_norm
    perp = perp / (np.linalg.norm(perp) + 1e-8)
    v2 = R.from_rotvec(th_ang * perp).apply(v_norm) * 0.02
    v3 = R.from_rotvec(2.0 * th_ang * perp).apply(v_norm) * 0.015
    j[3] = j[2] + v2
    j[4] = j[3] + v3

    # Fingers 5..24: Index, Middle, Ring, Pinky
    bases = [5, 10, 15, 20]
    for idx, base in enumerate(bases):
        f = fl[idx + 1]
        theta = 0.05 + f * 1.45
        x_off = 0.02 * (idx - 1.5)
        j[base] = p + np.array([x_off, 0.0, -0.03])
        v0 = np.array([0.0, 0.0, -0.03])
        v1 = np.array([0.0, -0.03 * np.sin(theta), -0.03 * np.cos(theta)])
        v2 = np.array([0.0, -0.02 * np.sin(2.0 * theta), -0.02 * np.cos(2.0 * theta)])
        v3 = np.array([0.0, -0.015 * np.sin(3.0 * theta), -0.015 * np.cos(3.0 * theta)])
        j[base + 1] = j[base] + v0
        j[base + 2] = j[base + 1] + v1
        j[base + 3] = j[base + 2] + v2
        j[base + 4] = j[base + 3] + v3

    return {
        "tracking": True,
        "confidence": 1.0,
        "wrist": {"pos": p.tolist(), "quat": q},
        "joints": j.tolist(),
    }
