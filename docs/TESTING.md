# AGIBOT A2 Ultra Teleoperation Testing Guide

## 1. Automated Acceptance Tests
Run the complete acceptance suite (isolated domain 77, localhost only):
```bash
./scripts/test_all.sh
```
All 7 test suites must report `PASS`.

## 2. Interactive Simulation & Web Panel Testing
1. Launch app: `python main.py`
2. Open `https://localhost:8012/` in browser:
   - **Gizmo Translation/Rotation:** Drag wrist gizmos (W/E/R). Confirm robot follows within <2 mm.
   - **Preset Motions:** Click 'Home', 'Arms Fwd', 'Wave'. Check visual sync.
   - **Hand Closure:** Drag 'Hand Close' slider. Confirm 5 fingers close without mesh interpenetration.
   - **Emergency Stop:** Click 'EMERGENCY STOP'. All motion freezes immediately.

## 3. Meta Quest 3S VR Teleoperation Testing
Connect headset via ADB (`adb reverse tcp:8012 tcp:8012`) and open `https://localhost:8012/`:
1. **ENTER VR Button:** Click large green button (>=160x56 px). Verify front-facing initial view.
2. **Translation Only:** Press and hold Index Trigger (front). Move controller; position follows, rotation locked.
3. **Rotation Only:** Press and hold Side Grip (squeeze). Rotate controller; wrist rotates, position locked.
4. **Full 6D Clutch:** Hold both Trigger + Grip. Verify full SE(3) tracking with zero transition jump.
5. **Finger Flexion & Hold:** Push Thumbstick Y forward to close hand (>=60% range). Hold for 10 s; verify stable grip.
6. **Pinch vs Close:** Cycle grasp with Thumbstick X to PINCH (only thumb + index flex).
7. **Panic Open & Home:** Click thumbstick for instant open; hold A/X 1.5 s for smooth homing.
8. **View Mode:** Hold both thumbstick clicks for 0.5 s. Use Right Stick X (orbit) & Y (distance).
9. **Tracking Loss:** Cover controller tracking sensors with hand. Verify clutch releases, robot holds, and HUD shows `TRACKING LOST`. Re-clutch requires releasing both buttons.

## 4. Virtual Dry-Run Check
Test locked backend without hardware:
```bash
python main.py --backend dryrun
```
Verify terminal prints live 50 Hz ASCII command tables with joint values and rates.

## 5. Real Robot Testing (Runbook S0–S8)
*Requires operator authorization and hardware risk unlock.*
Follow [FIRST_CONTACT_RUNBOOK.md](file:///home/nguyen/a2_ws/docs/FIRST_CONTACT_RUNBOOK.md):
- **S0–S2:** Physical e-stop tethered, domain 77 loopback test, zero robot network sockets (`ss -tan`).
- **S3–S4:** Preflight script (`python scripts/preflight.py`) and home position diff check (`python scripts/ros2_compare_home.py`).
- **S5–S6:** Single-joint direction check (`python scripts/ros2_sign_check.py`) with 0.05 rad delta cap.
- **S7–S8:** 5-gate unlocked run (`./scripts/run_real.sh`) with 0.3 rad/s speed cap and 800 raw hand limit.

## 6. Generating Session Reports
After testing, bundle session metrics, test results, and git revision:
```bash
python scripts/make_report.py
```
Output is saved to `reports/report_<timestamp>.md`.

