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

## Code Style Rules
- **NO icons or emoji** in code, UI text, log/print messages, comments, or docs: no emoji (warning sign, cross, check mark, scissors, trash can, gamepad, magnifier...) and no pictograph glyphs used as button labels (play/pause triangles, refresh arrow, close cross, skip/trim arrows). Use plain words instead (`Chạy`, `Dừng`, `Cắt`, `Xoá đoạn`, `Đóng`, `WARNING`, `OK`, `MISSING`).
- Allowed: letters (including Vietnamese diacritics), math and unit symbols (`°`, `±`, `²`, `×`, `≤`, `≥`, `≈`), typographic dashes and arrows used in prose (`–`, `—`, `→`), and box-drawing / arrow characters inside ASCII architecture diagrams (`docs/SIL.md`, `sim_command.md`).
- Vendored third-party code (`teleop/xr/televuer/`) is not rewritten.
- Check before commit: `grep -rnP "[\x{2700}-\x{27BF}\x{26A0}\x{25B6}\x{25C0}\x{27F3}\x{21E4}\x{21E5}\x{FE0F}\x{1F000}-\x{1FFFF}]" teleop scripts tests tools main.py README.md docs` must print nothing outside the diagram files above.

## Code Map
- `main.py`: Launcher entrypoint and facade (`UnifiedTeleopApp`).
- `teleop/app/`: Core app modules (`control_loop`, `init_subsystems`, `init_state`, `homing_controller`, `ik_dispatcher`, `hand_sweep`, `telemetry_builder`, `cmd_system`, `cmd_hands`, `cmd_targets`, `xr_input_handler`, `xr_clutch_mapper`).
- `teleop/web/`: Web server (`server/web_server`, `manifest_builder`, `panel_routes`, `ws_broadcaster`, `html_builder`), frontend JS (`static/js/`), CSS (`static/css/`), partials (`static/partials/`).
- `teleop/simulation/`: MuJoCo env facade (`mujoco_env.py` -> `env_discovery`, `env_poses`), web panel facade (`web_panel.py`), pick-into-bin practice scene (`task_scene.py`, used by `main.py --sim --scene pick_bin`; MJCF `assets/agibot_a2_ultra/mjcf/scene_pick_bin.xml`, web view `teleop/web/static/js/task_scene.js`).
- `teleop/kinematics/`: IK solvers (`mink_ik.py`, `agibot_a2_ik.py`), joint limiter (`mirror_limiter.py`), reach projection (`reach_projection.py`), filters facade (`filters.py` -> `signal_filters`, `followers`).
- `teleop/utils/`: Collision checker facade (`collision_checker.py` -> `teleop/safety/`: `pair_builder`, `arm_safety`, `hand_safety`), `startup_banner.py`, `ssl_certs.py`, `port_check.py`.
- `teleop/retargeting/`: Arm & hand retargeters (`agibot_arm_retargeting.py`, `agibot_hand_retargeting.py`).
- `teleop/tracking/`: WebXR and Quest tracking receivers.
- `scripts/`: only files the runbooks use (`real_robot_command.md`, `sim_command.md`): Orin receiver (`arm_teleop_rx.py`, `orin_teleop_core.py`, `orin_trace_logger.py`, `async_writer.py`), `test_sil.py`, `plant_excitation.py`, `classify_arrival.py`, and the acceptance tests `test_web_sync.py`, `test_panel_api.py`. Keep these paths stable.
- `tests/`: regression tests (`bash tests/test_all.sh`). `tools/`: measurement, log analysis, plant identification, ROS 2 utilities (`joint_limit_sweep.py` on the Orin + `joint_sweep_report.py`: per-joint shake vs speed -> JOINT_VMAX / JOINT_AMAX; `view_episode.py`: preview.mp4 + plots.png per recorded episode; DATA_COLLECTION.md (repo root) = full data-collection guide (operator, Vietnamese); `data_status.py`: one-screen inventory of recordings/ and lerobot_datasets/ (which raw episode went into which dataset); `dataset_cache.py`: decodes a LeRobot episode for the /dataset page (teleop/web/server/dataset_routes.py + static/dataset.html + js/dataset_viewer.js; live view of the episode being recorded: `LIVE` bus in episode_recorder.py + teleop/web/server/live_routes.py (/ws/live) + js/dataset_live.js, test tests/test_live_stream.py); `export_lerobot.py` trims idle lead-in / tail, `--append` = incremental; teleop/app/episode_recorder.py `AutoExporter` runs it after every stopped episode, env A2_AUTO_EXPORT=0 disables).
- `sil/`: A2SIM / software-in-the-loop (fake Orin + MuJoCo fake MC), see `docs/SIL.md`.
