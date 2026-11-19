#!/usr/bin/env bash
# sil/a2sim/status.sh - Is the simulated robot network up, and is anything conflicting with the real robot?
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ok() { echo -e "  \e[32mOK\e[0m   $*"; }; bad() { echo -e "  \e[31m--\e[0m   $*"; }
echo "a2sim status"
ip netns list 2>/dev/null | grep -qw a2sim && ok "netns a2sim present" || bad "netns a2sim missing (sudo -E bash sil/a2sim/setup.sh)"
ip -o -4 addr show a2sim-host 2>/dev/null | grep -q 192.168.2.75 && ok "laptop side 192.168.2.75 on a2sim-host" || bad "laptop side IP missing"
others=$(ip -o -4 addr show | awk '{print $2, $4}' | grep -v '^a2sim-host ' | grep -E ' 192\.168\.(2|100)\.')
[[ -z "$others" ]] && ok "no real robot network on other interfaces" || bad "REAL robot network also present: $others  -> teardown the sim!"
timeout 1 bash -c '</dev/tcp/192.168.2.50/22' 2>/dev/null && ok "ssh agi@192.168.2.50 reachable" || bad "sshd on 192.168.2.50 not reachable"
id agi >/dev/null 2>&1 && ok "user agi exists" || bad "user agi missing"
if timeout 1 bash -c '</dev/tcp/192.168.2.50/22' 2>/dev/null; then :; fi
st=$(SIL_CTL_HOST=192.168.2.50 python3 "$REPO/sil/sil_ctl.py" status 2>/dev/null)
if [[ -n "$st" ]]; then
  ok "simulated robot powered on: $(echo "$st" | python3 -c 'import sys,json; s=json.load(sys.stdin); print(s["action"], "| motion_player", "ON" if s["motion_player"] else "off", "| state msgs", s["n_state"], "| arm cmds", s["n_arm_cmd"])')"
else
  bad "simulated robot not powered on (bash sil/a2sim/robot_power_on.sh)"
fi
