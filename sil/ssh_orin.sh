#!/usr/bin/env bash
# sil/ssh_orin.sh - Simulated `ssh agi@192.168.2.50`: an interactive shell laid out like the Orin so the
# commands of real_robot_command.md can be typed as-is.
#
#   ~ = /agibot/data/home/agi (simulated under sil/orin_fs/; real absolute paths work if you create
#       the optional symlink:  sudo ln -s ~/a2_ws/sil/orin_fs/agibot /agibot )
#   ~/Desktop/mc            vendor SetAction.py; receiver files arrive here via BUOC 0 (cp = scp)
#   ~/Desktop/motion_player vendor disable/enable_motion_player.sh (curl, unmodified)
#   aima em load-env        no-op stand-in
#   HTTP to 192.168.100.100 (RPC) is routed to the fake MC through http_proxy, so vendor scripts and
#   the receiver's GetAction run unmodified.
#   ROS: ROS_DOMAIN_ID=88 / ROS_LOCALHOST_ONLY=1 are read-only here; `export ROS_DOMAIN_ID=232`,
#   `ROS_LOCALHOST_ONLY=0` and the Orin-only FASTRTPS profile are intercepted (they could reach the real robot).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$REPO/sil/orin_fs"
AGI_HOME="$ROOT/agibot/data/home/agi"
VENDOR="$REPO/agibot_a2_aimdk-dev2.0/examples"
DOMAIN="${SIL_DOMAIN:-88}"
RPC_PORT="${SIL_RPC_PORT:-56322}"

mkdir -p "$AGI_HOME/Desktop/mc/logs" "$AGI_HOME/Desktop/motion_player"
for f in SetAction.py; do [[ -f "$AGI_HOME/Desktop/mc/$f" ]] || cp "$VENDOR/mc/$f" "$AGI_HOME/Desktop/mc/"; done
for f in disable_motion_player.sh enable_motion_player.sh; do
  [[ -f "$AGI_HOME/Desktop/motion_player/$f" ]] || cp "$VENDOR/motion_player/$f" "$AGI_HOME/Desktop/motion_player/"
done

RC="$(mktemp)"
cat > "$RC" <<RCEOF
# Orin has no venv: python3 must be the system 3.10 (rclpy, requests), so drop any activated venv/conda
export PATH="$REPO/sil/orin_bin:/usr/bin:/bin:\$(echo "\$PATH" | tr ':' '\n' | grep -v -e '/venv/bin' -e 'conda' -e '^/usr/bin\$' -e '^/bin\$' | paste -sd:)"
unset VIRTUAL_ENV CONDA_PREFIX PYTHONHOME
export PYTHONUSERBASE="$HOME/.local"          # keep the laptop's user site-packages (numpy) under the fake HOME
export HOME="$AGI_HOME"
export http_proxy="http://127.0.0.1:$RPC_PORT" HTTP_PROXY="http://127.0.0.1:$RPC_PORT"
unset https_proxy HTTPS_PROXY all_proxy ALL_PROXY
export no_proxy="127.0.0.1,localhost" NO_PROXY="127.0.0.1,localhost"
unset FASTRTPS_DEFAULT_PROFILES_FILE
declare -rx ROS_DOMAIN_ID=$DOMAIN
declare -rx ROS_LOCALHOST_ONLY=1
export() {
  local a out=()
  for a in "\$@"; do
    case "\$a" in
      ROS_DOMAIN_ID=*)        echo "[SIL] \$a ignored: ROS_DOMAIN_ID stays $DOMAIN (232 = real robot)"; continue ;;
      ROS_LOCALHOST_ONLY=*)   echo "[SIL] \$a ignored: ROS_LOCALHOST_ONLY stays 1 (isolated from robot network)"; continue ;;
      FASTRTPS_DEFAULT_PROFILES_FILE=*)
        if [[ ! -f "\${a#*=}" ]]; then echo "[SIL] FASTRTPS profile ignored (Orin-only file, not on the laptop)"; continue; fi ;;
    esac
    out+=("\$a")
  done
  (( \${#out[@]} )) && builtin export "\${out[@]}"
  return 0
}
PS1='\[\e[1;33m\][SIM]\[\e[0m\] agi@orin:\w\$ '
cd "\$HOME"
echo "=================================================================="
echo " SIMULATED ORIN (SIL) - not the real robot. ROS_DOMAIN_ID=$DOMAIN, localhost only"
echo " ~ = /agibot/data/home/agi   ->  cd ~/Desktop/mc"
[[ -d /agibot/data/home/agi/Desktop/mc ]] && echo " /agibot symlink present: absolute paths from real_robot_command.md work" \
  || echo " (optional) sudo ln -s $ROOT/agibot /agibot   to make /agibot/... paths work verbatim"
echo " Receiver: python3 arm_teleop_rx.py --allow-ip 127.0.0.1   (laptop is 127.0.0.1 in SIL)"
echo " exit = logout"
echo "=================================================================="
RCEOF
exec bash --rcfile "$RC" -i
