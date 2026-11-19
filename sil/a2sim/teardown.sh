#!/usr/bin/env bash
# sil/a2sim/teardown.sh - Remove the simulated robot network (do this BEFORE connecting to the real robot).
#   sudo bash sil/a2sim/teardown.sh           stop robot + sshd, delete netns/veth, remove ~/.ssh/config block
#   sudo bash sil/a2sim/teardown.sh --purge   also delete user agi, /agibot, /etc/a2sim, ~/.ssh/known_hosts_a2sim
set -uo pipefail
[[ $EUID -eq 0 ]] || { echo "Run with: sudo bash $0 [--purge]" >&2; exit 1; }
REAL_USER="${SUDO_USER:-}"; REAL_HOME="$(getent passwd "${REAL_USER:-root}" | cut -d: -f6)"
if ip netns list | grep -qw a2sim; then
  for p in $(ip netns pids a2sim); do kill "$p" 2>/dev/null; done
  sleep 1
  for p in $(ip netns pids a2sim); do kill -9 "$p" 2>/dev/null; done
  ip netns del a2sim
  echo "[a2sim] robot, sshd and namespace removed (veth + 192.168.2.75 gone)"
fi
rm -f /run/a2sim-sshd.pid
if [[ -n "$REAL_USER" && -f "$REAL_HOME/.ssh/config" ]]; then
  sed -i '/^# >>> a2sim/,/^# <<< a2sim/d' "$REAL_HOME/.ssh/config"
  sed -i '1{/^$/d}' "$REAL_HOME/.ssh/config"
  echo "[a2sim] ~/.ssh/config block removed"
fi
if [[ "${1:-}" == "--purge" ]]; then
  id agi >/dev/null 2>&1 && userdel agi && echo "[a2sim] user agi deleted"
  rm -rf /agibot /etc/a2sim && echo "[a2sim] /agibot and /etc/a2sim deleted"
  [[ -n "$REAL_USER" ]] && rm -f "$REAL_HOME/.ssh/known_hosts_a2sim"
  echo "[a2sim] openssh-server package left installed (system ssh service stays disabled)"
fi
ip -o -4 addr show | grep -E '192\.168\.(2|100)\.' || echo "[a2sim] no 192.168.2.x left on this laptop - ready for the real robot"
