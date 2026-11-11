# Teleoperation Oscillation & Stepping Diagnosis Report
**Date:** 2026-11-11  
**Git Branch:** `fix/smooth-stream`  
**Dataset Analyzed:** `logs/real_20261111_134136.jsonl` (13,924 records @ 100 Hz), `logs/real_20261111_132533.jsonl` (3,205 records @ 50 Hz).

---

## A) Oscillation Characterization (Worst 3-Second Window)
Analyzed worst 3-second window (ticks 300..600, $t=3.00\text{ s}$ to $6.00\text{ s}$ in `real_20261111_134136.jsonl`):

| Joint | SENT ptp (rad) | SENT dom_f (Hz) | ACT ptp (rad) | ACT dom_f (Hz) | Phase Lag (ms) | ERR ptp (rad) |
|---|---|---|---|---|---|---|
| J1 (J0) | 0.2575 | 0.33 | 0.2712 | 0.67 | +0.0 | 0.1175 |
| J2 (J1) | 0.0617 | 0.33 | 0.0599 | 0.33 | +0.0 | 0.0119 |
| J3 (J2) | 0.1341 | 0.33 | 0.1286 | 0.33 | +0.0 | 0.0644 |
| J4 (J3) | 0.0930 | 0.33 | 0.0877 | 0.33 | +0.0 | 0.0371 |
| J5 (J4) | 0.1905 | 0.67 | 0.1820 | 0.67 | +30.0 | 0.0985 |
| J6 (J5) | 0.0586 | 0.33 | 0.0578 | 0.33 | +0.0 | 0.0284 |
| J7 (J6) | 0.0000 | 0.33 | 0.0057 | 5.67 | +270.0 | 0.0057 |
| J8..J14 (Right) | 0.0000 | 0.00 | $\le 0.0004$ | - | - | $\le 0.0004$ |

- **Where does it oscillate?**
  - **CONFIRMED:** During motion, low-frequency oscillation ($0.33\text{ Hz}$ to $0.67\text{ Hz}$) is present in SENT ($ptp$ up to $0.2575\text{ rad}$) and tracked by ACT.
  - **CONFIRMED:** At rest/steady state ($t > 18\text{ s}$, ticks 2000..10000), SENT std is identically $0.000000\text{ rad}$ across all 14 joints. ACT std is $\le 0.0001\text{ rad}$. There is ZERO residual oscillation at rest.
  - **CONFIRMED:** High-frequency jitter (>5 Hz) is 52% to 69% of velocity power in ACT, while only 6% to 14% in SENT.

---

## B) Command Path Audit
Grep audit of command path stages from user input to UDP transmission:
1. Web Gizmo / Quest Controller input $\to$ `teleop/app/control_loop.py:85-89` calls `app._solve_arm_ik()` in `teleop/app/ik_dispatcher.py:31,70`. IK seeds from `last_ik_sol_left` (internal warm-start cache), NOT measured state. Zero branch flipping when target is still: SENT std = $0.000000$.
2. Sync manager: `teleop/app/sync_manager.py:85,97` calls `app.mirror_limiter.step(raw_target_q14, dt=dt)`. `measured_q14` is passed purely for `self.lead_val` calculation (log-only, lines 118-121). Command path is pure open-loop.
3. Safety layer: `teleop/robot/safety_layer.py:153-228` checks kinematic limits and formats trace.
4. UDP transmit: `teleop/robot/udp_backend.py:133-180` runs thread at 100 Hz sending `self._last_arm_cmd`.

- **State dependence on act / arrival:** NONE in active command calculation.
- **Still target behavior:** Exact zero change (no limit-cycle, std = 0.0).

---

## C) Timing & Streaming Architecture
- **Laptop sender rate:** Mean $\Delta t = 10.000\text{ ms}$ ($100.0\text{ Hz}$), std = $0.576\text{ ms}$, min = $0.463\text{ ms}$, max = $18.014\text{ ms}$.
- **Packet delivery:** Monotonic seq ($+1$ every packet), 0 skipped, 0 duplicated, 0 reordered.
- **Orin receiver state:** `scripts/arm_teleop_rx.py:38,66,121-134` runs at 100 Hz with 30 ms playout interpolation buffer.
- **Root Cause of "small steps / stepping":**
  - **CONFIRMED:** The 200 Hz control loop (`control_dt = 0.005` in `teleop/app/control_loop.py:16,133`) executes `mirror_limiter.step()` at 200 Hz with discrete target steps from Gizmo/IK, updating `_target_tx_q`. Meanwhile `UdpBackend` asynchronously samples at 100 Hz (`teleop/robot/udp_backend.py:143`). This asynchronous rate beating (200 Hz limiter output sampled by 100 Hz UDP sender) created inter-tick velocity jumps and jerk spikes ($a_{sent}$ up to $42.7\text{ rad/s}^2$).

---

## D) Other Publishers
- **Topics:** `/motion/control/arm_joint_command` and `/motion/control/hand_joint_command`.
- **Node detected in logs:** `example_ros2_protobuf_publisher_node` (log indicates warning matching on discovery).
- **Receiver defense:** `scripts/arm_teleop_rx.py:207-216,250-258` runs 1.5 s preflight topic rate check; refuses to publish arm/hand if rate > 0 msg/s. Runtime check latches `DUAL PUBLISHER` and halts if rate exceeds our own by > 10% within 300 ms.

---

## E) Plant Dynamics & Ringing Response
Analyzed clean step settle at tick 953 ($t=9.53\text{ s}$):
- **J1 (J0):** $ptp = 0.00152\text{ rad}$, 0 zero-crossings, overdamped decay ($< 0.3\text{ s}$).
- **J4 (J3):** $ptp = 0.00496\text{ rad}$, overdamped.
- **J5 (J4 elbow):** $ptp = 0.01221\text{ rad}$, overdamped settling within $0.4\text{ s}$.
- **J6/J7 (J5/J6 wrist parallel):** $ptp = 0.00484\text{ rad}$ on J5, $0.00208\text{ rad}$ on J6, ringing at $1.03\text{ Hz}$ to $5.67\text{ Hz}$ with 3–4 zero-crossings, settling within $0.5\text{ s}$.
- **CONFIRMED:** The physical robot has underdamped wrist resonance ($f \approx 5.67\text{ Hz}$) that rings mechanically when commanded with rapid acceleration changes.

---

## F) Hand / Arm Coupling
- Hand packets (`pkt["hand"]`) are packed into the same UDP packet in `teleop/robot/udp_backend.py:176`.
- In `scripts/arm_teleop_rx.py:175-188`, arm command is published first, followed by hand command on `/motion/control/hand_joint_command`.
- **SUSPECTED:** Hand command publication does not alter arm timing, but electrical/CAN-bus contention on the Orin robot interface during simultaneous arm+hand streaming may increase MC servo loop latency.

---

## G) Summary Table: Suspects Ranked by Likelihood

| Rank | Suspect | Evidence | Status | Minimal Fix Idea |
|---|---|---|---|---|
| 1 | Rate beating between 200 Hz control loop and 100 Hz UDP thread | $a_{sent}$ peaks at $42.7\text{ rad/s}^2$, $j_{sent} > 7000\text{ rad/s}^3$ (`real_20261111_134136.jsonl`) | CONFIRMED | Lock limiter and UDP sender to a single synchronized 100 Hz clock tick. |
| 2 | High-frequency mechanical wrist resonance ($5.7\text{ Hz}$) triggered by jerk | ACT high-frequency power is 52-97%, ACT J6 rings at $5.67\text{ Hz}$ | CONFIRMED | Soften wrist joint (J5/J6) acceleration/jerk limits in mirror limiter. |
| 3 | Target staircase from 20-30 Hz web/VR inputs | Step lengths of raw input are 30-50 ms between distinct poses | CONFIRMED | Smooth raw 3D pose input with critically-damped follower before IK. |
| 4 | Second publisher (`example_ros2_protobuf`) transmitting on DDS | ROS topic discovery warnings in Orin DDS log | SUSPECTED | Ensure preflight 1.5s rate check blocks startup if any message is seen. |
| 5 | CAN/DDS bus contention from hand command traffic | Hand joint commands published at same tick as arm commands | SUSPECTED | Stagger or decimate hand command publication rate (e.g. 50 Hz). |

