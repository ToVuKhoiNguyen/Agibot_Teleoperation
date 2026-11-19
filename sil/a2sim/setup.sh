#!/usr/bin/env bash
# sil/a2sim/setup.sh - Build the isolated "robot network" so real_robot_command.md runs VERBATIM against the
# MuJoCo-simulated A2.   Run:  sudo -E bash sil/a2sim/setup.sh      (-E keeps the proxy vars for apt/pip)
#
# What it creates (all reversible with sil/a2sim/teardown.sh [--purge]):
#   network namespace "a2sim" = the robot. veth pair:
#       laptop side  a2sim-host  192.168.2.75/24   (same IP as the real laptop)
#       robot side   a2sim-orin  192.168.2.50/24   (same IP as the real Orin)   + lo alias 192.168.100.100 (MC RPC)
#     The namespace has NO default route -> nothing inside can reach any other network / the real robot.
#   openssh-server package; the system ssh service is DISABLED (laptop does not expose port 22). A separate
#     sshd runs only inside the namespace on 192.168.2.50:22, user agi only.
#   Linux user "agi" (password "1", like the Orin), home /agibot/data/home/agi, Orin-like tree:
#     ~/Desktop/mc (vendor SetAction.py), ~/Desktop/motion_player (vendor scripts), `aima` stub,
#     /agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml (SIM substitute: FastDDS defaults).
#   ~/.ssh/config block (marked a2sim) so the simulated Orin's host key goes to ~/.ssh/known_hosts_a2sim
#     and never collides with the real robot's key in ~/.ssh/known_hosts.
# Refuses to run while the laptop is on a 192.168.2.x / 192.168.100.x network (real robot connected).
set -euo pipefail

[[ $EUID -eq 0 ]] || { echo "Run with: sudo -E bash $0" >&2; exit 1; }
REAL_USER="${SUDO_USER:-}"
[[ -n "$REAL_USER" && "$REAL_USER" != "root" ]] || { echo "Run via sudo from your normal user account" >&2; exit 1; }
REAL_HOME="$(getent passwd "$REAL_USER" | cut -d: -f6)"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENDOR="$REPO/agibot_a2_aimdk-dev2.0/examples"
NS=a2sim; HIF=a2sim-host; NIF=a2sim-orin
LAPTOP_IP=192.168.2.75; ORIN_IP=192.168.2.50; RPC_IP=192.168.100.100
AGI_HOME=/agibot/data/home/agi
say() { echo -e "\e[1;36m[a2sim]\e[0m $*"; }

# ---------------------------------------------------------------- 0. never coexist with the real robot network
if ip -o -4 addr show | awk '{print $2, $4}' | grep -v "^$HIF " | grep -qE ' 192\.168\.(2|100)\.'; then
  echo "FATAL: this laptop already has a 192.168.2.x/192.168.100.x address (real robot network?)." >&2
  ip -o -4 addr show | awk '{print "   ", $2, $4}' | grep -E '192\.168\.(2|100)\.' >&2
  echo "       Disconnect from the robot network first. The simulation reuses the robot's IPs." >&2
  exit 2
fi

# ---------------------------------------------------------------- 1. openssh-server (system service kept OFF)
if [[ ! -x /usr/sbin/sshd ]]; then
  say "installing openssh-server"
  APT_PROXY=()
  [[ -n "${http_proxy:-}" ]] && APT_PROXY=(-o "Acquire::http::Proxy=$http_proxy")
  [[ -n "${https_proxy:-}" ]] && APT_PROXY+=(-o "Acquire::https::Proxy=$https_proxy")
  DEBIAN_FRONTEND=noninteractive apt-get "${APT_PROXY[@]}" install -y openssh-server
fi
systemctl disable --now ssh.service ssh.socket >/dev/null 2>&1 || true
say "system sshd disabled (laptop does not listen on port 22)"

# ---------------------------------------------------------------- 2. user agi + Orin-like filesystem
if [[ -L /agibot ]]; then rm -f /agibot; say "removed old /agibot symlink"; fi
mkdir -p "$AGI_HOME/Desktop/mc/logs" "$AGI_HOME/Desktop/motion_player" /agibot/sim/bin /agibot/software/v0/entry/bin/cfg
if ! id agi >/dev/null 2>&1; then
  useradd -M -d "$AGI_HOME" -s /bin/bash agi
  say "created user agi"
fi
echo 'agi:1' | chpasswd
[[ -f "$AGI_HOME/Desktop/mc/SetAction.py" ]] || cp "$VENDOR/mc/SetAction.py" "$AGI_HOME/Desktop/mc/"
for f in disable_motion_player.sh enable_motion_player.sh; do
  [[ -f "$AGI_HOME/Desktop/motion_player/$f" ]] || cp "$VENDOR/motion_player/$f" "$AGI_HOME/Desktop/motion_player/"
done
install -m 0755 "$REPO/sil/orin_bin/aima" /agibot/sim/bin/aima
cat > /agibot/software/v0/entry/bin/cfg/ros_dds_configuration.xml <<'XML'
<?xml version="1.0" encoding="UTF-8" ?>
<!-- A2SIM substitute for the Orin's vendor DDS profile (not available off-robot): UDPv4 only, no shared
     memory, so ROS nodes run by different Linux users (simulated robot vs. agi's receiver) can talk. -->
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>a2sim_udpv4</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="a2sim_default_participant" is_default_profile="true">
    <rtps>
      <name>a2sim</name>
      <userTransports><transport_id>a2sim_udpv4</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
XML
if ! grep -q "a2sim" "$AGI_HOME/.bashrc" 2>/dev/null; then
  [[ -f "$AGI_HOME/.bashrc" ]] || cp /etc/skel/.bashrc "$AGI_HOME/.bashrc"
  cat >> "$AGI_HOME/.bashrc" <<'RC'

# --- a2sim: simulated Orin (sil/a2sim) ---
export PATH="/agibot/sim/bin:$PATH"
echo -e "\e[1;33m[A2SIM] Simulated Orin (MuJoCo robot) - NOT the real robot\e[0m"
RC
fi
# Standard Ubuntu login files like the Orin: ssh starts a LOGIN shell, which reads ~/.profile, which sources ~/.bashrc.
[[ -f "$AGI_HOME/.profile" ]] || cp /etc/skel/.profile "$AGI_HOME/.profile"
# The real Orin has no corporate proxy, but pam_env loads /etc/environment (http_proxy=...) into every ssh
# login here. Unset it at the very top of .bashrc (before the non-interactive return) so SetAction.py,
# curl and the receiver's GetAction reach 192.168.100.100 directly, exactly like on the robot.
if ! grep -q ">>> a2sim-noproxy" "$AGI_HOME/.bashrc"; then
  { printf '# >>> a2sim-noproxy: the real Orin has no corporate proxy\nunset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY ftp_proxy FTP_PROXY all_proxy ALL_PROXY no_proxy NO_PROXY\n# <<< a2sim-noproxy\n\n'; cat "$AGI_HOME/.bashrc"; } > "$AGI_HOME/.bashrc.a2sim.tmp"
  mv "$AGI_HOME/.bashrc.a2sim.tmp" "$AGI_HOME/.bashrc"
fi
chown -R agi:agi "$AGI_HOME"
chmod 755 /agibot /agibot/data /agibot/data/home
say "user agi / password 1, home $AGI_HOME"

# ---------------------------------------------------------------- 3. network namespace = robot
if ! ip netns list | grep -qw "$NS"; then
  ip netns add "$NS"
  ip link add "$HIF" type veth peer name "$NIF"
  ip link set "$NIF" netns "$NS"
  ip addr add "$LAPTOP_IP/24" dev "$HIF"
  ip link set "$HIF" up
  ip netns exec "$NS" ip link set lo up
  ip netns exec "$NS" ip addr add "$ORIN_IP/24" dev "$NIF"
  ip netns exec "$NS" ip link set "$NIF" up
  ip netns exec "$NS" ip addr add "$RPC_IP/32" dev lo
  ip netns exec "$NS" ip route add 224.0.0.0/4 dev "$NIF"   # DDS multicast discovery (no default route on purpose)
  say "netns $NS: laptop $LAPTOP_IP <-> robot $ORIN_IP (+ RPC $RPC_IP)"
else
  say "netns $NS already exists"
fi

# ---------------------------------------------------------------- 4. sshd only inside the namespace
mkdir -p /etc/a2sim /run/sshd
[[ -f /etc/a2sim/ssh_host_ed25519_key ]] || ssh-keygen -q -t ed25519 -N '' -f /etc/a2sim/ssh_host_ed25519_key
cat > /etc/a2sim/sshd_config <<CFG
# a2sim: simulated Orin sshd, listens ONLY inside netns $NS on $ORIN_IP
ListenAddress $ORIN_IP
Port 22
HostKey /etc/a2sim/ssh_host_ed25519_key
PidFile /run/a2sim-sshd.pid
AllowUsers agi
PermitRootLogin no
PasswordAuthentication yes
KbdInteractiveAuthentication no
UsePAM yes
X11Forwarding no
PrintMotd no
Subsystem sftp /usr/lib/openssh/sftp-server
CFG
if [[ -f /run/a2sim-sshd.pid ]] && kill -0 "$(cat /run/a2sim-sshd.pid)" 2>/dev/null; then
  kill "$(cat /run/a2sim-sshd.pid)"; sleep 0.5
fi
ip netns exec "$NS" /usr/sbin/sshd -f /etc/a2sim/sshd_config
say "sshd: agi@$ORIN_IP (inside $NS only)"

# ---------------------------------------------------------------- 5. keep the real robot's host key untouched
SSH_CFG="$REAL_HOME/.ssh/config"
mkdir -p "$REAL_HOME/.ssh"; touch "$SSH_CFG"
if ! grep -q ">>> a2sim" "$SSH_CFG"; then
  { printf '# >>> a2sim (simulated Orin; removed by sil/a2sim/teardown.sh)\nHost %s\n    UserKnownHostsFile ~/.ssh/known_hosts_a2sim\n    StrictHostKeyChecking accept-new\n# <<< a2sim\n\n' "$ORIN_IP"; cat "$SSH_CFG"; } > "$SSH_CFG.a2sim.tmp"
  mv "$SSH_CFG.a2sim.tmp" "$SSH_CFG"
fi
chown "$REAL_USER:" "$REAL_HOME/.ssh" "$SSH_CFG"; chmod 700 "$REAL_HOME/.ssh"; chmod 600 "$SSH_CFG"

# ---------------------------------------------------------------- 6. robot software deps (MuJoCo for python3.10)
if ! sudo -u "$REAL_USER" /usr/bin/python3 -c "import mujoco" 2>/dev/null; then
  say "installing mujoco for /usr/bin/python3 (user $REAL_USER)"
  sudo -u "$REAL_USER" -E /usr/bin/python3 -m pip install --user "mujoco==3.17.0"
fi

# ---------------------------------------------------------------- 7. firewall hint
if command -v ufw >/dev/null && ufw status 2>/dev/null | grep -q "Status: active"; then
  say "ufw is active: allowing traffic on $HIF"
  ufw allow in on "$HIF" >/dev/null
fi

cat <<EOF

$(say "READY.")  Next (see sim_command.md):
   Terminal R:  bash sil/a2sim/robot_power_on.sh          (power on the simulated robot)
   then follow real_robot_command.md exactly:  scp ... agi@$ORIN_IP:...,  python3 main.py,  ssh agi@$ORIN_IP (password 1)
   Status: bash sil/a2sim/status.sh      Remove before using the real robot: sudo bash sil/a2sim/teardown.sh
EOF
