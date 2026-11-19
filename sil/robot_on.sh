#!/usr/bin/env bash
# sil/robot_on.sh - "Power on" the simulated robot (fake MC), in the state a freshly booted A2 is in:
#   action McAction_RL_LOCOMOTION_DEFAULT (not 402), motion_player enabled.
# Then follow real_robot_command.md in sil/ssh_orin.sh (the simulated "ssh agi@192.168.2.50").
# Env: SIL_DOMAIN (default 88), SIL_PLANT (default config/mc_plant.json), SIL_IDEAL=1, SIL_MC_ARGS
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOMAIN="${SIL_DOMAIN:-88}"
[[ "$DOMAIN" == "232" ]] && { echo "FATAL: 232 is the real robot's ROS domain" >&2; exit 2; }
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID="$DOMAIN" ROS_LOCALHOST_ONLY=1 PYTHONUNBUFFERED=1
unset FASTRTPS_DEFAULT_PROFILES_FILE || true
RUN_DIR="$REPO/logs/sil/$(date +%Y%m%d_%H%M%S)_manual"
mkdir -p "$RUN_DIR"
ARGS=(--boot real --plant "${SIL_PLANT:-$REPO/config/mc_plant.json}" --log-dir "$RUN_DIR")
[[ "${SIL_IDEAL:-0}" == "1" ]] && ARGS+=(--ideal)
# shellcheck disable=SC2206
ARGS+=(${SIL_MC_ARGS:-})
echo "[robot_on] simulated A2 powering on (ROS_DOMAIN_ID=$DOMAIN, localhost only). Logs: $RUN_DIR"
exec /usr/bin/python3 -u "$REPO/sil/fake_mc_node.py" "${ARGS[@]}"
