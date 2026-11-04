# AGIBOT A2 Ultra Teleoperation Cleanup & Audit Report

**Date**: 2026-11-04  
**Audit Target**: `agibot_xr_teleoperate`  
**Trash Location**: `/home/nguyen/a2_ws/_trash/20261104/`  

---

## 1. Summary of Actions
- **Files moved to `_trash/20261104`**: 19 files (123.79 KB).
- **Protected directories & files intact**: `docs/agibot_a2_manual.*`, `aimdk_a2_dev.*`, `assets/agibot_a2_ultra/{urdf,meshes}`, `LICENSE`, `venv`.
- **System Verification**: All acceptance tests (`test_web_sync.py`, `test_panel_api.py`, `test_arm_ik.py`, `test_hand_control.py`, `test_controller_axes.py`) and MuJoCo simulation start / `/health` probe **PASSED** (100%).
- **Bytecode Recompilation**: `python -m compileall -q teleop scripts main.py` succeeded with exit code 0.

---

## 2. Moved to `_trash` (HIGH Confidence)

| Relative Path | Size | Evidence / Reason | Confidence | Action Taken |
| :--- | :--- | :--- | :--- | :--- |
| `agibot_xr_teleoperate/config/cert.pem` | 1,265 B | Duplicate of `certs/cert.pem` (MD5: `facf441b94dda4e9f5907cb59bb3202b`). Active configuration points to `certs/cert.pem`. | HIGH | MOVED TO TRASH |
| `agibot_xr_teleoperate/config/key.pem` | 1,704 B | Duplicate of `certs/key.pem` (MD5: `4d0c1580309578b2ccd7ae4849bf92f0`). Active configuration points to `certs/key.pem`. | HIGH | MOVED TO TRASH |
| `agibot_xr_teleoperate/teleop/**/__pycache__/*.pyc` (17 files) | 123.78 KB | Generated Python bytecode cache files. | HIGH | MOVED TO TRASH |

---

## 3. Items Retained for REVIEW (MEDIUM / LOW Confidence)

| Path | Size | Evidence / Reason | Confidence | Recommended Action |
| :--- | :--- | :--- | :--- | :--- |
| `agibot_xr_teleoperate/scripts/run_teleoperation.py` | 51.5 KB | Superseded by consolidated `main.py` entrypoint. Retained for developer reference or direct standalone CLI invocation. | MEDIUM | REVIEW (Keep or archive after user sign-off) |
| `agibot_xr_teleoperate/scripts/decimate_meshes.py` | 5.6 KB | Mesh simplification utility for converting visual meshes to web meshes (`assets/agibot_a2_ultra/web_meshes`). Not part of runtime. | LOW | KEEP (Tooling asset) |
| `agibot_xr_teleoperate/agibot_*_retargeting.py` & `agibot_a2_ik.py` (3 files) | ~600 B total | Top-level backward-compatibility shims importing from `teleop.*`. Used by `scripts/test_controller_axes.py`. | LOW | KEEP (Compatibility shims) |
| `agibot_xr_teleoperate/assets/agibot_a2_ultra/web_meshes/{base_link,left_hand,right_hand}.stl` | ~12.2 MB total | Meshes present in URDF (`a2.urdf`) and web mesh set, not directly instantiated in MJCF XML. | LOW | KEEP (URDF visual assets) |
| `requirements-ros.txt` | 2.5 KB | Optional ROS 2 workspace package inventory for sim2real deployment. | LOW | KEEP (Reference documentation) |
| `docs/{ARCHITECTURE,AGIBOT_PORTING_ANALYSIS,QUEST3S_SETUP,MUJOCO_SETUP}.md` | ~11 KB total | Engineering documentation not directly linked from README.md table of contents. | LOW | KEEP (Add links in README) |

---

## 4. Verification Results
- **MuJoCo Simulation + HTTPS Server (`main.py`)**: Starts cleanly, serves HTTPS/WSS on port 8012, `/health` endpoint returns `HTTP 200 OK` (`{"status": "OK", "sim": "RUNNING"}`).
- **3D Web Panel Sync (`scripts/test_web_sync.py`)**: 4/4 tests passed (link poses streamed over WebSocket match MuJoCo xpos within 0.053 mm; dragging target marker drove MuJoCo robot via IK; E-Stop confirmed).
- **Web Control Panel API (`scripts/test_panel_api.py`)**: 6/6 tests passed (Jog X, Jog Z, Hand Close, Wave Right Hand, Reset Home, Emergency Stop).
- **Kinematics & Control Acceptance Tests**:
  - `test_arm_ik.py`: 100/100 targets solved (>770 Hz, mean position error < 2.8 mm).
  - `test_hand_control.py`: Mimic joint coupling and scripted bilateral motion verified.
  - `test_controller_axes.py`: 6/6 cardinal axes verified.
- **Bytecode Compilation**: `python -m compileall -q teleop scripts main.py` exit code 0.

---

## 5. Exact Restore Command
If any moved files need to be restored to their original location:
```bash
cp -r /home/nguyen/a2_ws/_trash/20261104/agibot_xr_teleoperate/* /home/nguyen/a2_ws/agibot_xr_teleoperate/
```

