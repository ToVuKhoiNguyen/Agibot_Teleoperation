#!/usr/bin/env bash
# tools/armpy_run_plot.sh - one command: push arm.py to the Orin, run it, Ctrl-C to stop, pull its trace, plot cmd vs state.
# arm.py writes logs/trace_armpy_<ts>.jsonl itself (CMD it publishes + STATE it reads), same format as trace_recorder.
#
# Usage (from ~/a2_ws):
#   tools/armpy_run_plot.sh              # copy arm.py, run on the robot (Ctrl-C to stop), pull trace, plot
#   tools/armpy_run_plot.sh --plot-only  # do not run: pull the newest trace_armpy_* from the robot and plot it
#   JOINTS=all tools/armpy_run_plot.sh   # passed to plot_cmd_state.py --joints (default 0,3,7,10 = L/R J1, J4)
#   ARM_ARGS="--hz 50" tools/armpy_run_plot.sh   # extra arm.py arguments (publish rate: default 100, 50 = old behaviour, 200/400 to compare)
#
# Precondition (by hand, as in real_robot_command.md): motion player disabled, arm_teleop_rx NOT running.
# One SSH connection is shared (ControlMaster), so the password is asked once.
set -uo pipefail
ROBOT="${ROBOT:-agi@192.168.2.50}"
MC="${MC:-/agibot/data/home/agi/Desktop/mc}"
JOINTS="${JOINTS:-0,3,7,10}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARM_PY="$HERE/agibot_a2_aimdk-dev2.0/examples/mc/arm.py"
PY="$HERE/venv/bin/python"; [[ -x "$PY" ]] || PY=python3
CTL="/tmp/armpy_ssh_%r@%h:%p"
SSH_OPTS=(-o ControlMaster=auto -o "ControlPath=$CTL" -o ControlPersist=120)
ssh_() { ssh "${SSH_OPTS[@]}" "$@"; }
scp_() { scp "${SSH_OPTS[@]}" "$@"; }

ROS_ENV='source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=0 FASTRTPS_DEFAULT_PROFILES_FILE=/agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml'

if [[ "${1:-}" != "--plot-only" ]]; then
  echo "[armpy] copy arm.py -> $ROBOT:$MC/"
  scp_ "$ARM_PY" "$ROBOT:$MC/arm.py" || { echo "[armpy] scp failed"; exit 1; }
  if ssh_ "$ROBOT" "pgrep -f '[a]rm_teleop_rx|[a]rm\.py' >/dev/null"; then
    echo "[armpy] ABORT: arm_teleop_rx / arm.py already running on the robot (two arm publishers)."; exit 1
  fi
  echo "[armpy] running arm.py on the robot - press Ctrl-C to stop"
  ssh_ -t "$ROBOT" "$ROS_ENV; cd $MC && exec python3 -u arm.py ${ARM_ARGS:-}"   # Ctrl-C goes to the remote arm.py (tty), it saves its trace
fi

REMOTE=$(ssh_ "$ROBOT" "ls -t $MC/logs/trace_armpy_*.jsonl $MC/trace_armpy_*.jsonl 2>/dev/null | head -1")
[[ -n "$REMOTE" ]] || { echo "[armpy] no trace_armpy_*.jsonl on the robot"; exit 1; }
mkdir -p "$HERE/logs"
LOCAL="$HERE/logs/$(basename "$REMOTE")"
echo "[armpy] pull $REMOTE"
scp_ "$ROBOT:$REMOTE" "$LOCAL" || exit 1
ssh "${SSH_OPTS[@]}" -O exit "$ROBOT" 2>/dev/null

"$PY" "$HERE/tools/plot_cmd_state.py" "$LOCAL" --joints "$JOINTS" || exit 1
PNG="${LOCAL%.jsonl}_cmd_state.png"
command -v xdg-open >/dev/null && xdg-open "$PNG" >/dev/null 2>&1 &
echo "[armpy] plot: $PNG"
