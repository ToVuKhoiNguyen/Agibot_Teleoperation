# MuJoCo Simulation Setup

## Model Structure
* **`assets/agibot_a2_ultra/mjcf/scene.xml`**: Top-level simulation scene containing ground plane, lighting, skybox, visual properties, and inclusion of the robot model.
* **`assets/agibot_a2_ultra/mjcf/a2_ultra.xml`**: Converted MJCF model containing:
  - 52 revolute joints with verified limits matching `a2.urdf`.
  - 142 geoms (72 visual, 69 collision).
  - 12 mimic finger equality constraints.
  - 40 position actuators (`<position>`).
  - End-effector sites (`left_arm_ee`, `right_arm_ee`).
* **`assets/agibot_a2_ultra/meshes/`**: High-resolution STL visual meshes (`meshes/visual/`) and convex hull collision meshes (`meshes/collision/`).

## Upright Base Fixation
In the original URDF, `base_link` is attached directly without a floating base joint. In MuJoCo:
* The root `base_link` body is positioned at `pos="0 0 0.93"`.
* With `z = 0.93 m`, both feet (`left_toe_roll`, `right_toe_roll`) contact the floor plane at `z = 0.002 m`.
* Fixation/welding prevents the robot from toppling over under gravity, eliminating the requirement for complex bipedal locomotion balancing while keeping arm and hand teleoperation 100% stable.

## Dynamics & Actuator Tuning
* **Integrator**: `implicitfast` Euler implicit integrator with `timestep = 0.002 s` (500 Hz).
* **Actuator Classes**:
  - `arm`: `kp = 200`, `damping = 5.0`, `armature = 0.05`, `forcerange = [-100, 100] Nm`.
  - `hand`: `kp = 20`, `damping = 0.05`, `armature = 0.001`, `forcerange = [-10, 10] Nm`.
  - `leg`: `kp = 800`, `damping = 30.0`, `armature = 0.1`, `forcerange = [-200, 200] Nm`.

## Running the Simulation

### 1. Interactive GUI Viewer
If a graphical display is available (`$DISPLAY` or `$WAYLAND_DISPLAY` set):
```bash
python scripts/run_mujoco.py
```

### 2. Headless Simulation Mode
For SSH/terminal environments without a display:
```bash
python scripts/run_mujoco.py --headless --steps 500
```
The script automatically verifies all 52 joints against the URDF, prints the joint table, and confirms physics stability.
