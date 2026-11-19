#!/usr/bin/env bash
# sil/a2sim/robot_power_on.sh - "Power on" the simulated A2 inside the a2sim namespace (= the robot).
#   Fresh-boot state: action McAction_RL_LOCOMOTION_DEFAULT (not 402), motion_player enabled, arms at HOME.
#   MuJoCo physics body + identified MC response (config/mc_plant.json), ROS 2 on domain 232 (isolated in netns),
#   aimdk RPC on 192.168.100.100:56322/56444, MuJoCo window = the "real" robot (A2SIM_VIEWER=0 to disable).
# Asks for your sudo password (entering a network namespace needs root); the robot then runs as your user.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ip netns list 2>/dev/null | grep -qw a2sim || { echo "a2sim network not set up: sudo -E bash sil/a2sim/setup.sh" >&2; exit 1; }
RUN_DIR="$REPO/logs/sil/$(date +%Y%m%d_%H%M%S)_a2sim"
mkdir -p "$RUN_DIR"
VIEW=(--viewer); [[ "${A2SIM_VIEWER:-1}" == "0" ]] && VIEW=()
echo "[a2sim] powering on simulated A2 (logs: $RUN_DIR)"
exec sudo ip netns exec a2sim sudo -u "$USER" \
  env HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin" DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" \
      WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" PYTHONUNBUFFERED=1 \
  bash -c 'set +u; source /opt/ros/humble/setup.bash; set -u
           export ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=0
           export FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml
           exec /usr/bin/python3 -u "$0/sil/fake_mc_node.py" --a2sim --physics mujoco --boot real \
                --plant "$0/config/mc_plant.json" --log-dir "$1" "${@:2}"' \
  "$REPO" "$RUN_DIR" "${VIEW[@]}" ${A2SIM_MC_ARGS:-}
