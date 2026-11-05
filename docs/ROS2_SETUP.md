# ROS 2 (Jazzy) Environment Setup for AGIBOT A2 Ultra

If `python -c "import rclpy"` fails in the virtual environment:

1. Ensure ROS 2 Jazzy environment is sourced:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```
2. Enable access to system ROS packages from venv:
   ```bash
   echo "/opt/ros/jazzy/lib/python3.12/site-packages" > ~/a2_ws/venv/lib/python3.12/site-packages/ros_jazzy.pth
   ```
3. Test safety variables (never connect to robot domain 232 during tests):
   ```bash
   export ROS_DOMAIN_ID=77
   export ROS_LOCALHOST_ONLY=1
   ```

