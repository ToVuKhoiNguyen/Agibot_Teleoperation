# AGIBOT A2 Ultra - Real Robot First Contact Runbook

This runbook specifies the strict, ordered protocol for the very first live hardware test.
**Mandatory Safety Invariant**: A dedicated second operator MUST hold the physical wireless E-stop at all times.

---

## 1. Prerequisites (Go / No-Go Checklist)

| Item | Requirement | Go / No-Go Threshold |
| :--- | :--- | :--- |
| **Gantry** | Overhead gantry/harness attached and weighted | Robot cannot collapse if unpowered |
| **E-Stop** | Wireless hardware E-STOP in 2nd person's hand | Tested click before power-on |
| **Perimeter**| 2.0 m perimeter clear of all bystanders | Zero persons in workspace |
| **Battery** | Robot battery charged (> 50%) and x86 on | Voltage nominal |
| **Network** | PC connected to robot network (`192.168.100.x`) | `ping -c 3 192.168.100.100` (< 2 ms) |
| **Domain** | `export ROS_DOMAIN_ID=232` (or robot DDS domain) | Must NOT be test domain 77 |

---

## 2. Ordered Test Steps (S0 to S8)

### S0: Pre-Flight Offline System Check
Verify software stack integrity without emitting any robot network packets.
```bash
export ROS_DOMAIN_ID=232
python scripts/preflight.py
```
- **Go Criteria**: 6/6 `[PASS]` (Python, rclpy, domain, port 8012, safety locks, hand caps).
- **Abort Condition**: Any `[FAIL]`. Resolve before proceeding.

### S1: Live Read-Only Telemetry Verification
Verify real robot states are streaming on production domain without publishing.
```bash
python scripts/ros2_listen.py
```
- **Go Criteria**: 14 arm joint states received at $\ge 90$ Hz; 12 hand states received at $\ge 90$ Hz.
- **Abort Condition**: No data received within 3.0 s, or rate $< 50$ Hz.

### S2: Live Home Posture Comparison
Compare actual robot suspended posture against simulation home posture.
```bash
python scripts/ros2_compare_home.py
```
- **Go Criteria**: All 14 arm joints match sim home within $|\Delta| \le 0.10$ rad; all hand joints raw $< 150$.
- **Abort Condition**: Any joint $|\Delta| > 0.10$ rad. Adjust robot posture on gantry.

### S3: Switch Controller Mode to Action 402 (Servo)
Preview RPC payloads, then transition motion controller to `RL_LOCOMOTION_ARM_EXT_JOINT_SERVO`.
```bash
# Dry preview first:
python scripts/aima_mode.py
# Execute switch:
python scripts/aima_mode.py --execute
```
- **Go Criteria**: HTTP 200, `action=402`, `motion_player` stopped.
- **Restore / Rollback**: `python scripts/aima_mode.py --restore` (re-enters Stand Action 400).
- **Abort Condition**: HTTP error or status $\ne 402$.

### S4: Joint-by-Joint Active Sign Check (Left Arm)
Execute controlled $+0.05$ rad jog over 2.0 s per joint (velocity $\le 0.025$ rad/s $\le 0.1$ rad/s cap).
```bash
export AGIBOT_ALLOW_REAL=1
python scripts/ros2_sign_check.py --arm left --i-understand-hardware-risk
```
- Type confirmation phrase `GANTRY AND ESTOP READY`.
- Observe motion. If joint moves in expected direction, enter `y`. Press `q` to abort.
- **Go Criteria**: 7/7 joints report `[PASS]`, logged in [`docs/SIGN_CHECK.json`](file:///home/nguyen/a2_ws/docs/SIGN_CHECK.json).
- **Abort Condition**: Read-back mismatch ($> 0.015$ rad opposite direction) aborts immediately.

### S5: Quest Teleoperation - Left Arm Only (No Hand)
Launch conservative teleoperation with hands completely disabled.
```bash
./scripts/run_real.sh --arms left --no-hand --real-max-vel 0.3
```
- Fasten Quest headset, open `https://localhost:8012/`, enter VR.
- Trigger clutch: gentle translation only (wrist $\pm 5$ cm). Grip clutch: orientation only ($\pm 10^\circ$).
- **Go Criteria**: Smooth response, tracking latency $< 30$ ms, zero safety trips.
- **Abort Condition**: Any jerk, tracking loss $> 350$ ms, or FAULT latch.

### S6: Quest Teleoperation - Right Arm Only (No Hand)
Repeat isolated teleoperation for the right arm.
```bash
./scripts/run_real.sh --arms right --no-hand --real-max-vel 0.3
```
- **Go Criteria**: Right arm tracks smoothly at 0.3 rad/s cap without collision warnings.

### S7: Dexterous Hand Verification (Capped at 800 Raw)
Enable hands with strict conservative safety caps (raw max 800 = 40% curl, thumb swing neutral).
```bash
./scripts/run_real.sh --arms both --hand-raw-max 800 --real-max-vel 0.3
```
- Test joystick forward curl $\to$ fingers flex smoothly up to raw 800.
- Thumbstick click $\to$ panic-open restores fingers to 0 raw immediately.
- **Go Criteria**: Raw values never exceed 800; thumb swing stays neutral (0).

### S8: Formal Shutdown Order
1. Release all controller clutches in Quest.
2. In terminal, press `Ctrl+C` to cleanly disconnect [`Ros2Backend`](file:///home/nguyen/a2_ws/teleop/robot/ros2_backend.py) (publishers hold measured pose).
3. Restore A2 default stance: `python scripts/aima_mode.py --restore`.
4. Trigger physical wireless E-STOP button.
5. Power off x86 controller and robot battery breaker.

