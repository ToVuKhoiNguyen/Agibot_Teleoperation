#!/usr/bin/env bash
# sil/sil_up.sh - Bring up the Software-In-the-Loop fake robot on this laptop (loopback).
#
#   fake_mc_node.py   (ROS 2 Humble, system python3.10)  <- MC + motors (identified plant)
#   arm_teleop_rx.py  (UNMODIFIED, via run_fake_orin.py)  <- the exact Orin receiver
#
# Then, in another terminal, run the laptop exactly as for the real robot, only pointed at loopback:
#   ROBOT_IP=127.0.0.1 venv/bin/python main.py --speed 3
#
# Env:
#   SPEED=1..4           receiver speed level (default 3, must match main.py --speed)
#   SIL_DOMAIN=88        isolated ROS_DOMAIN_ID (232 = real robot, refused)
#   SIL_PLANT=path       plant json (default config/mc_plant.json); SIL_IDEAL=1 for pass-through
#   SIL_NETEM="delay 3ms 1ms loss 0.5%"   optional tc-netem on lo (needs sudo; removed on exit)
#   SIL_RX_ARGS="..."    extra args for arm_teleop_rx.py (e.g. "--dry", "--no-thumb-swing")
#   SIL_MC_ARGS="..."    extra args for fake_mc_node.py (e.g. "--action 10")
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SPEED="${SPEED:-3}"
DOMAIN="${SIL_DOMAIN:-88}"
PLANT="${SIL_PLANT:-$REPO/config/mc_plant.json}"
PY=/usr/bin/python3

if [[ "$DOMAIN" == "232" ]]; then
  echo "FATAL: SIL_DOMAIN=232 is the real robot's ROS domain. Refusing." >&2; exit 2
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="$DOMAIN" ROS_LOCALHOST_ONLY=1
unset FASTRTPS_DEFAULT_PROFILES_FILE || true
export PYTHONUNBUFFERED=1
# corporate http(s)_proxy must never see loopback traffic (receiver's GetAction RPC uses urllib)
export SIL_CTL_HOST=127.0.0.1  # loopback SIL, never the a2sim robot
export no_proxy="127.0.0.1,localhost${no_proxy:+,$no_proxy}" NO_PROXY="127.0.0.1,localhost${NO_PROXY:+,$NO_PROXY}"

RUN_DIR="$REPO/logs/sil/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR/logs"
echo "[sil] run dir: $RUN_DIR  (ROS_DOMAIN_ID=$DOMAIN, localhost only, speed=$SPEED)"

PIDS=()
NETEM_ON=0
cleanup() {
  for p in "${PIDS[@]:-}"; do kill -INT "$p" 2>/dev/null || true; done
  sleep 0.5
  for p in "${PIDS[@]:-}"; do kill -9 "$p" 2>/dev/null || true; done
  if [[ "$NETEM_ON" == "1" ]]; then sudo tc qdisc del dev lo root 2>/dev/null || true; echo "[sil] netem removed"; fi
}
trap cleanup EXIT INT TERM

if [[ -n "${SIL_NETEM:-}" ]]; then
  echo "[sil] applying netem on lo: $SIL_NETEM (affects ALL loopback traffic until exit)"
  sudo tc qdisc replace dev lo root netem $SIL_NETEM && NETEM_ON=1
fi

MC_ARGS=(--plant "$PLANT" --log-dir "$RUN_DIR")
[[ "${SIL_IDEAL:-0}" == "1" ]] && MC_ARGS+=(--ideal)
# shellcheck disable=SC2206
MC_ARGS+=(${SIL_MC_ARGS:-})
"$PY" -u "$REPO/sil/fake_mc_node.py" "${MC_ARGS[@]}" > >(sed -u 's/^/[mc] /') 2>&1 &
PIDS+=($!)

MC_PID=$!
# wait until the fake MC publishes state (receiver pre-flight needs a quiet command topic, not state)
export REPO_DIR="$REPO"
for _ in $(seq 1 50); do
  if "$PY" - <<'PYEOF' 2>/dev/null; then break; fi
import sys, os
sys.path.insert(0, os.path.join(os.environ.get("REPO_DIR", "."), "sil"))
from sil_ctl import ctl
st = ctl({"cmd": "status"}, timeout=0.3)
sys.exit(0 if st.get("n_state", 0) > 0 else 1)
PYEOF
  kill -0 "$MC_PID" 2>/dev/null || { echo "FATAL: fake MC exited during startup" >&2; exit 1; }
  sleep 0.2
done

# shellcheck disable=SC2206
RX_ARGS=(--speed "$SPEED" ${SIL_RX_ARGS:-})
( cd "$RUN_DIR" && exec "$PY" -u "$REPO/sil/run_fake_orin.py" "${RX_ARGS[@]}" ) > >(sed -u 's/^/[orin] /') 2>&1 &
PIDS+=($!)

cat <<EOF

[sil] Fake robot starting. In another terminal run the laptop side:
      ROBOT_IP=127.0.0.1 venv/bin/python main.py --speed $SPEED
      (fault injection: venv/bin/python sil/sil_ctl.py status | freeze on | dual on | action 10 | burst 3 | push 3 0.05)
[sil] Ctrl+C to stop.
EOF
wait
