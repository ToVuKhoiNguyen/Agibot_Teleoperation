# Refactoring Plan: Modular Architecture (<=250 Lines/File)

## 1. Completed Modules (Steps 1–3)
- `teleop/kinematics/filters.py` (344L) -> `signal_filters.py` (170L), `followers.py` (184L), facade (28L).
- `teleop/simulation/mujoco_env.py` (450L) -> `env_discovery.py` (146L), `env_poses.py` (107L), facade (245L).
- `teleop/utils/collision_checker.py` (458L) -> `pair_builder.py` (159L), `hand_safety.py` (126L), `arm_safety.py` (184L), facade (226L).

## 2. Re-Plan Step 4: `teleop/simulation/web_panel.py` (2,577 lines -> sum: 2,523 lines, -2.1%)
| Original Range | Target Module | Lines | Responsibility |
|---|---|---|---|
| L35-40, 225-568, 2221-2225 | `teleop/web/static/index.html` | 355 | Static HTML DOM layout & UI cards |
| L41-224 | `teleop/web/static/css/panel.css` | 184 | Responsive styles & control badges |
| L569-785 | `teleop/web/static/js/scene.js` | 217 | Three.js scene, camera, lights, floor |
| L786-1010 | `teleop/web/static/js/gizmo.js` | 225 | Dual TransformControls, markers, hotkeys |
| L1011-1219 | `teleop/web/static/js/raycaster.js` | 209 | Mouse raycasting, mesh cache & unpacking |
| L1220-1404 | `teleop/web/static/js/robot_mesh.js` | 185 | Manifest link hierarchy & STL loading |
| L1405-1487 | `teleop/web/static/js/render_stream.js` | 83 | Zero-delay pose lerp/slerp & loop |
| L1488-1640 | `teleop/web/static/js/webxr_session.js` | 153 | Immersive VR session & floor alignment |
| L1641-1770 | `teleop/web/static/js/webxr_controllers.js` | 130 | Controller pose/button input pump |
| L1771-1950 | `teleop/web/static/js/telemetry_stream.js` | 180 | WebSocket client & status packet decode |
| L1951-2075 | `teleop/web/static/js/telemetry_ui.js` | 125 | Hand status & Quest telemetry panels |
| L2076-2220 | `teleop/web/static/js/panel_ui.js` | 145 | Jog buttons, sliders, sweeps, init |
| L2294-2358 | `teleop/web/server/manifest_builder.py` | 65 | Visual geom manifest & bundle caching |
| L2359-2480 | `teleop/web/server/panel_routes.py` | 122 | aiohttp HTTP route handlers (/health, /api) |
| L2423-2468, 2481-2545 | `teleop/web/server/ws_broadcaster.py` | 110 | WebSocket client manager & pose stream |
| L1-34, 2226-2293, 2546-2577 | `teleop/simulation/web_panel.py` | 135 | Server facade, app lifecycle, static serving |

## 3. Re-Plan Step 5: `main.py` (1,821 lines -> sum: 1,820 lines, -0.05%)
| Original Range | Target Module | Lines | Responsibility |
|---|---|---|---|
| L1-92 | `teleop/utils/startup_banner.py` | 92 | Banner CLI printing, IP discovery |
| L93-238 | `teleop/app/init_subsystems.py` | 146 | Subsystem instantiations in `__init__` |
| L239-424 | `teleop/app/init_state.py` | 186 | Session variables, limits, Quest config |
| L425-564 | `teleop/app/homing_controller.py` | 140 | Signals, smooth homing slerp machine |
| L565-625 | `teleop/app/hand_sweep.py` | 61 | Single & all-finger sweep test state |
| L626-717 | `teleop/app/ik_dispatcher.py` | 92 | `_solve_arm_ik` pipeline & follower |
| L718-800 | `teleop/app/cmd_system.py` | 83 | E-stop, hold, walk demo, arm jog |
| L801-960 | `teleop/app/xr_input_handler.py` | 160 | Buttons, sticks, grasp & closure pump |
| L961-1112 | `teleop/app/xr_clutch_mapper.py` | 152 | 4-state clutch machine & 6D target calc |
| L1113-1182 | `teleop/app/cmd_hands.py` | 70 | Grasp presets, manual joints, sweeps |
| L1183-1355 | `teleop/app/cmd_targets.py` | 173 | Wrist targets, presets, motion seqs |
| L1356-1398 | `teleop/app/motion_demos.py` | 43 | Sequence player & walk demo stepper |
| L1561-1760 | `teleop/app/telemetry_builder.py` | 200 | 20 Hz status JSON packet assembler |
| L1399-1560, 1761-1821 | `main.py` | 222 | 200 Hz run loop, close(), CLI entrypoint |
