# AGIBOT A2 Ultra Teleoperation - Agent Git Workflow

This document outlines the standard operational workflows for AI assistants and contributors maintaining this repository.

## Commands

### 1. `push [note]`
Triggered when the user says `push` or `push [note]`:
1. Run `git status --short`.
2. Inspect changes and **STOP immediately** to ask the user if any file >5 MB (not previously approved), `*.pem`, `*.key`, `.env`, `venv/`, `_trash/`, or `logs/` would be committed.
3. Unless the user specifies `"push nhanh"`, run acceptance test suite:
   ```bash
   python scripts/test_web_sync.py && python scripts/test_panel_api.py
   ```
4. Run `git add -A`.
5. Create a commit using Conventional Commits syntax (`feat:`, `fix:`, `docs:`, `chore:`, `refactor:`) with the user's note if provided.
6. Push to GitHub: `git push origin main`.
7. Report outcome in $\le$ 3 lines: commit hash, changed files count, and remote commit URL.

### 2. `tag vX.Y.Z [message]`
Triggered when the user says `tag vX.Y.Z [message]`:
1. Check working directory status; commit any uncommitted changes first.
2. Create an annotated git tag:
   ```bash
   git tag -a vX.Y.Z -m "<message>"
   ```
3. Push tag to remote:
   ```bash
   git push origin vX.Y.Z
   ```
4. If the user specifies `"release"`, also generate GitHub release notes:
   ```bash
   gh release create vX.Y.Z --generate-notes
   ```
5. Report outcome in $\le$ 3 lines: tag name, release URL.

## Invariant Safety Rules
- **NEVER** use `--force` or force push (`-f`).
- **NEVER** rewrite published history (`rebase -i`, `reset --hard` on remote branches).
- **NEVER** commit or log secrets, tokens, passwords, private keys, or internal vendor robot credentials.
- **NEVER** push unless explicitly requested by the user.

## Code Map
- `main.py`: Launcher entrypoint and facade (`UnifiedTeleopApp`).
- `teleop/app/`: Core app modules (`control_loop`, `init_subsystems`, `init_state`, `homing_controller`, `ik_dispatcher`, `hand_sweep`, `telemetry_builder`, `cmd_system`, `cmd_hands`, `cmd_targets`, `xr_input_handler`, `xr_clutch_mapper`).
- `teleop/web/`: Web server (`server/web_server`, `manifest_builder`, `panel_routes`, `ws_broadcaster`, `html_builder`), frontend JS (`static/js/`), CSS (`static/css/`), partials (`static/partials/`).
- `teleop/simulation/`: MuJoCo env facade (`mujoco_env.py` -> `env_discovery`, `env_poses`), web panel facade (`web_panel.py`).
- `teleop/kinematics/`: IK solvers (`mink_ik.py`, `agibot_a2_ik.py`), filters facade (`filters.py` -> `signal_filters`, `followers`).
- `teleop/utils/`: Collision checker facade (`collision_checker.py` -> `pair_builder`, `arm_safety`, `hand_safety`), `startup_banner.py`, `ssl_certs.py`, `port_check.py`.
- `teleop/retargeting/`: Arm & hand retargeters (`agibot_arm_retargeting.py`, `agibot_hand_retargeting.py`).
- `teleop/tracking/`: WebXR and Quest tracking receivers.
