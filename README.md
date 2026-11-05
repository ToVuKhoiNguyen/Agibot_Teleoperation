# AGIBOT A2 Ultra - XR Teleoperation & Digital Twin System

Low-latency bi-manual teleoperation and real-time digital twin system for the AGIBOT A2 Ultra humanoid robot with 6-DoF active dexterous hands.

---

## 1. Safety Statement & Credits
- **Safety Invariant**: **SIMULATION (MuJoCo) ONLY BY DEFAULT**. Physical hardware execution is locked behind 5 distinct software gates. Nothing here has been verified on real hardware; all safety defaults assume worst-case vendor hardware behavior.
- **Credit & License**: Architecture derived from Unitree `xr_teleoperate` (Apache 2.0).
- **Asset Status**: Visual 3D mesh assets in [`assets/agibot_a2_ultra/`](file:///home/nguyen/a2_ws/assets/agibot_a2_ultra/) have **UNVERIFIED** license status (vendor-extracted for research).

---

## 2. Installation & Quick Start
### Prerequisites
- **OS**: Ubuntu 24.04 LTS (or 22.04 LTS).
- **ROS 2**: ROS 2 Jazzy Jalisco (`/opt/ros/jazzy`).
- **Python**: 3.12 virtualenv (`--system-site-packages` to access `rclpy`).

```bash
cd ~/a2_ws
source /opt/ros/jazzy/setup.bash
python3 -m venv venv --system-site-packages
source venv/bin/activate
pip install -r requirements.txt
```

### Launch Simulation & Web Panel
```bash
python main.py
```
Open **`https://localhost:8012/`** (accept self-signed SSL cert).
For complete automated and manual verification procedures, see [`docs/TESTING.md`](file:///home/nguyen/a2_ws/docs/TESTING.md).

### Quest 3S USB Setup
1. Enable USB debugging on Quest 3S and connect via USB-C cable.
2. Reverse port: `adb reverse tcp:8012 tcp:8012`.
3. Open Quest Browser to `https://localhost:8012/` and click the floating **ENTER VR** button.

### Controller Mapping Table
| Physical Control | Action / Mode | Teleoperation Effect |
| :--- | :--- | :--- |
| **Front Trigger (Hold)** | Translation Mode | Wrist tracks 3D position $(x, y, z)$; orientation locked |
| **Side Grip (Hold)** | Rotation Mode | Wrist tracks 3D orientation $(R)$; position locked |
| **Both Held** | 6D Mode | Full 6-DoF wrist teleoperation (position + orientation) |
| **Joystick Y (Push/Pull)**| Finger Flexion | Velocity-driven hand close/open; holds on release |
| **Joystick Click** | Panic Open | Immediately snaps finger flexion to 0 (open) |
| **Button A (Hold 1.5s)** | Home Reset | Smooth minimum-jerk return to home prepare posture |
| **Button B (Press)** | Precision Toggle | Scales movement to $\times 0.30$ fine manipulation |
| **Buttons B + Y (Hold 0.5s)**| Emergency Stop | Software E-STOP latch; clamps all arms and clutches |
| **Both Sticks Click (0.5s)** | View Mode Toggle | Right stick controls scene orbit yaw & distance |

---

## 3. Code Map
- [`main.py`](file:///home/nguyen/a2_ws/main.py): Application entrypoint and CLI facade (`UnifiedTeleopApp`).
- [`teleop/app/`](file:///home/nguyen/a2_ws/teleop/app/): Control loop (200 Hz), subsystem init, homing, IK dispatch, XR input.
- [`teleop/robot/`](file:///home/nguyen/a2_ws/teleop/robot/): `RobotInterface`, `SafetyLayer`, AimDK DDS map, `Ros2Backend`, dryrun backends.
- [`teleop/kinematics/`](file:///home/nguyen/a2_ws/teleop/kinematics/): Mink QP and DLS inverse kinematics solvers, signal filters.
- [`teleop/retargeting/`](file:///home/nguyen/a2_ws/teleop/retargeting/): Dual-arm and dexterous hand retargeters, grasp tables.
- [`teleop/simulation/`](file:///home/nguyen/a2_ws/teleop/simulation/): MuJoCo physics wrapper and visual manifest generator.
- [`teleop/web/`](file:///home/nguyen/a2_ws/teleop/web/): HTTPS/WSS server, Three.js 3D viewport, Quest WebXR client.
- [`scripts/`](file:///home/nguyen/a2_ws/scripts/): Sign check, home comparison, preflight, mode switcher, and tests.

---

## 4. Execution Modes
| Mode | Flag | Hardware Motion? | Description |
| :--- | :--- | :---: | :--- |
| **sim** (default) | `--backend sim` | ❌ No | Fully interactive MuJoCo physics simulation (200 Hz) |
| **dryrun** | `--backend dryrun` | ❌ No | Command generator without physics; prints 1 Hz summary |
| **ros2dry** | `--backend ros2dry` | ❌ No | 2 Hz console table (`sim | would-publish | unit`) |
| **replay** | `--backend replay` | ❌ No | Replays recorded joint/hand trajectories |
| **real** | `--backend real` | ⚠️ **YES** | Live DDS pass-through over AimDK ROS 2 topics (50 Hz) |

---

## 5. Real Robot First Contact Runbook
Complete protocol documented in [`docs/FIRST_CONTACT_RUNBOOK.md`](file:///home/nguyen/a2_ws/docs/FIRST_CONTACT_RUNBOOK.md).

**Prerequisites**: Suspended gantry harness tensioned, physical wireless E-stop held by a **second person**, 2.0 m clear perimeter, spotter stationed, robot battery charged, network subnet verified (`192.168.100.x`), `ROS_DOMAIN_ID=232`.

| Step | Script / Command | Target / Go Threshold | Abort Condition |
| :--- | :--- | :--- | :--- |
| **S0** | `python scripts/preflight.py` | 6/6 `[PASS]` offline checks | Any `[FAIL]` |
| **S1** | `python scripts/ros2_listen.py` | Arm/Hand states $\ge 90$ Hz (read-only) | No data within 3 s |
| **S2** | `python scripts/ros2_compare_home.py` | Arm $|\Delta| \le 0.1$ rad, hand raw $< 150$ | Any $|\Delta| > 0.1$ rad |
| **S3** | `python scripts/aima_mode.py --execute`| Switch controller to Action 402 (Servo) | RPC error / $\ne 402$ |
| **S4** | `python scripts/ros2_sign_check.py --arm left` | Joint-by-joint $+0.05$ rad jog at 0.1 rad/s | Opposite read-back |
| **S5** | `./scripts/run_real.sh --arms left --no-hand` | Left arm Quest teleop at 0.3 rad/s | Tracking loss / Fault |
| **S6** | `./scripts/run_real.sh --arms right --no-hand`| Right arm Quest teleop at 0.3 rad/s | Tracking loss / Fault |
| **S7** | `./scripts/run_real.sh --arms both --hand-raw-max 800` | Hands active, capped at 800 raw (40%) | Hand raw $> 800$ |
| **S8** | `python scripts/aima_mode.py --restore` | Stop teleop, restore stance, click E-stop | Power sequence |

---

## 6. Troubleshooting
| Symptom | Cause | Solution |
| :--- | :--- | :--- |
| **No state topics** | Robot not in servo mode or wrong subnet | Ping `192.168.100.100`; execute S3 mode switch |
| **Domain mismatch** | `ROS_DOMAIN_ID` does not match robot | Set `export ROS_DOMAIN_ID=232` (or vendor ID) |
| **FastDDS Discovery** | Network multicast blocked between Jazzy/Humble | Set `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` |
| **Tracking loss** | Quest lost line of sight; watchdog triggered | Clutches auto-release after 350 ms; regain tracking |
| **FAULT latch** | State $> 200$ ms stale or position deviation $> 0.05$ rad | Hold active; click "Clear Fault" on web UI to confirm |
| **Enter VR missing** | Non-HTTPS connection or unsupported browser | Access via `https://localhost:8012/` on Quest Browser |
| **Import rclpy error** | Python venv isolated from ROS 2 system packages | Recreate venv with `--system-site-packages` |

---

## 7. Known Unknowns (Honest List)
Details documented in [`docs/SAFETY_AND_UNKNOWNS.md`](file:///home/nguyen/a2_ws/docs/SAFETY_AND_UNKNOWNS.md):
- **Hardware Watchdog Timeout**: Exact trip duration unstated in vendor docs; teleop streams at 50 Hz and latches hold if state $> 200$ ms old.
- **Physical E-Stop Firmware Behavior**: Dedicate wireless unit; assume motors unpower and robot collapses $\to$ gantry harness required.
- **Thumb 0 Swing Sign**: AimDK unstated; `thumb_0` held neutral (0 raw) unless `--enable-thumb-swing` explicitly passed.
- **Wrist Pushrod Kinematics**: Serial IK output assumed equivalent to pushrods.
- **Payload Envelope**: Rated 1 kg single / 3 kg grip; reach limits unstated.
- **Mesh License**: 3D visual assets status is unverified.
- **Real Hardware Status**: Nothing has been executed or verified on real hardware yet.

---

## 8. Release Hygiene & Git Operations
- **Pre-Push Checks**: Verify no files $> 5$ MB, no private credentials (`*.key`, `*.pem`, `.env`), no vendor internal IP leaks, mesh license noted.
- **Git Commit & Tag**: Follow Conventional Commits (`feat:`, `fix:`, `docs:`) per [`AGENTS.md`](file:///home/nguyen/a2_ws/AGENTS.md). Annotated tags: `git tag -a vX.Y.Z -m "<msg>"`.
