"""
tests/test_sim_fidelity.py
Fidelity checks for the in-process sim plant (teleop/simulation/real_plant.py, config/sim_plant.json).

a) Replay: logged laptop SENT stream -> plant -> compare with logged ACT, stepping the plant with the
   log's REAL timestamps (50 Hz and 100 Hz logs both valid).
b) Rate invariance: same input at 100 Hz and at 200 Hz (control loop rate) gives the same output.
c) Step response through MirrorLimiter + plant at speeds 1..4: command monotone (no hunting),
   ACT settles (post-arrival ptp < 3 mrad), reach projection clamps, fast flick stays finite.
d) Model limitations summary.

For the SIL (fake MC) plant use scripts/test_sil.py instead.
"""

import os
import sys
import json
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from teleop.simulation.real_plant import RealPlantSimulator  # noqa: E402
from teleop.kinematics.mirror_limiter import MirrorLimiter  # noqa: E402
from teleop.kinematics.reach_projection import project_to_reachable_workspace  # noqa: E402

PLANT = "config/sim_plant.json"
# Real robot home (scripts/orin_teleop_core.HOME) - within LIMS (left J3 in [-2.0, -0.10])
HOME = np.array([0.0, 1.2, 0.0, -0.5, 1.5, 0.0, 0.0, 0.0, -1.2, 0.0, 0.5, 1.5, 0.0, 0.0])
REPLAY_LOGS = ["logs/real_20261110_181925.jsonl", "logs/real_20261111_183746.jsonl"]
RMS_OK_RAD = 0.010


def load(path):
    t, s, a = [], [], []
    with open(path) as f:
        for line in f:
            try:
                d = json.loads(line)
            except Exception:
                continue
            if "target_q" in d and "measured_q" in d and any(abs(x) > 0 for x in d["measured_q"]):
                t.append(d["t_laptop"]); s.append(d["target_q"]); a.append(d["measured_q"])
    return np.array(t), np.array(s), np.array(a)


def test_replay_fidelity():
    print("[1/3] Replay logged SENT through plant (real timestamps) vs logged ACT")
    plant = RealPlantSimulator(config_path=PLANT)
    assert not plant.is_ideal, f"plant not loaded: {plant.summary_str}"
    print(f"      {plant.summary_str}")
    joints = plant.plant_data["joints"]
    all_ok = True
    for path in REPLAY_LOGS:
        if not os.path.exists(path):
            print(f"      skip {path} (missing)"); continue
        t, sent, act = load(path)
        dts = np.diff(t)
        plant.reset(act[0])
        sim = [plant.step(sent[0], dt=0.0)]
        for k in range(1, len(t)):
            sim.append(plant.step(sent[k - 1], dt=dts[k - 1]))  # ZOH of previous command over the real interval
        sim = np.array(sim)
        err = sim - act
        rms, peak = np.sqrt(np.mean(err ** 2, axis=0)), np.max(np.abs(err), axis=0)
        print(f"      {path}: {len(t)} samples @ {np.median(dts)*1000:.0f} ms")
        print(f"      {'J':<4}{'RMS mrad':>10}{'peak mrad':>11}  conf")
        for j in range(14):
            conf = joints[str(j)].get("confidence", "?")
            bad = conf == "OK" and rms[j] > RMS_OK_RAD
            all_ok &= not bad
            print(f"      J{j:02d}{rms[j]*1000:10.2f}{peak[j]*1000:11.1f}  {conf}{'  FAIL' if bad else ''}")
    assert all_ok, f"replay RMS above {RMS_OK_RAD*1000:.0f} mrad on an OK joint"


def test_rate_invariance():
    print("\n[2/3] Rate invariance: 100 Hz vs 200 Hz stepping")
    u = HOME.copy(); u[1] += 0.2; u[11] -= 0.15
    out = {}
    for dt in (0.010, 0.005):
        p = RealPlantSimulator(config_path=PLANT); p.reset(HOME)
        ys = [p.step(u, dt=dt) for _ in range(int(round(1.5 / dt)))]
        r = int(round(0.010 / dt))
        out[dt] = np.array(ys)[r - 1:: r]  # sample right after each native 10 ms tick
    d = float(np.max(np.abs(out[0.010] - out[0.005])))
    print(f"      max |y100 - y200| = {d*1000:.4f} mrad")
    assert d < 1e-6, "plant output depends on caller rate"


def test_scripted_scenarios():
    print("\n[3/3] MirrorLimiter + plant step scenarios, speeds 1..4")
    print(f"      {'spd':<4}{'scenario':<22}{'path/net':>9}{'ACT tail ptp mrad':>19}  status")
    for spd in (1, 2, 3, 4):
        lim = MirrorLimiter(speed_level=spd, initial_q14=HOME.copy())
        plant = RealPlantSimulator(config_path=PLANT)
        lim.reset(HOME); plant.reset(HOME)
        tgt = HOME.copy(); tgt[1] += 0.20; tgt[3] -= 0.15
        sents, acts = [], []
        for _ in range(int(3.0 / 0.010)):
            u = lim.step(tgt, dt=0.010)
            sents.append(u.copy()); acts.append(plant.step(u, dt=0.010))
        sents, acts = np.array(sents), np.array(acts)
        path = np.sum(np.abs(np.diff(sents, axis=0)), axis=0)
        net = np.abs(sents[-1] - sents[0])
        mv = net > 0.01
        ratio = float(np.max(path[mv] / net[mv]))
        tail = float(np.max(np.ptp(acts[-80:], axis=0)))
        ok = ratio <= 1.01 and tail < 0.003 and np.max(np.abs(sents[-1] - tgt)) < 3e-3  # limiter arrival deadband ~2 mrad
        print(f"      {spd:<4}{'step & settle':<22}{ratio:9.3f}{tail*1000:19.3f}  {'PASS' if ok else 'FAIL'}")
        assert ok, f"speed {spd}: step not monotone / not settled"

        proj, clamped, beyond = project_to_reachable_workspace(np.array([0.80, 0.25, 1.25]), side="left")
        assert clamped and beyond > 0.0
        print(f"      {spd:<4}{'+x reach clamp':<22}{'':>9}{'':>19}  PASS ({beyond*100:.1f} cm beyond)")

        flick = HOME.copy(); flick[0] += 0.35
        for _ in range(80):
            y = plant.step(lim.step(flick, dt=0.010), dt=0.010)
        assert np.all(np.isfinite(y))
        print(f"      {spd:<4}{'fast flick':<22}{'':>9}{'':>19}  PASS")


def print_plant_limitations():
    print("\n--- SIM PLANT LIMITATIONS ---")
    print("1. Per-joint linear model (delay + 1st/2nd order + saturation); no coupling, gravity or payload.")
    print("2. End-to-end scope: network/playout/Orin guard are lumped in; for those as real code use SIL.")
    print("3. No contact with the environment; armor self-collision is checked separately.")
    print("4. Hand is IDEAL until hand_joint_state logs are identified (scripts/plant_excitation.py).")


if __name__ == "__main__":
    test_replay_fidelity()
    test_rate_invariance()
    test_scripted_scenarios()
    print_plant_limitations()
    print("\nALL SIM PLANT FIDELITY TESTS PASSED.")
