# ROS 2 (Humble) Environment Setup for AGIBOT A2 Ultra

The laptop has ROS 2 **Humble** on the system Python 3.10 (`/opt/ros/humble`). The teleop venv is Python 3.12 (`uv`), so `import rclpy` does **not** work inside `venv/` and is not needed there: `main.py` talks to the robot over UDP (the Orin receiver publishes the ROS 2 topics). The laptop has no route to the X86 `192.168.100.100`, so it cannot send arm commands over ROS 2 directly.

ROS 2 on the laptop is only needed for SIL / A2SIM (`docs/SIL.md`) and a few tests in `tests/`:

1. Source ROS 2 Humble and use the system Python 3.10 (not the venv) for ROS-based scripts:
   ```bash
   source /opt/ros/humble/setup.bash
   python3 -c "import rclpy"
   ```
2. Test safety variables (never connect to robot domain 232 during tests):
   ```bash
   export ROS_DOMAIN_ID=77
   export ROS_LOCALHOST_ONLY=1
   ```
