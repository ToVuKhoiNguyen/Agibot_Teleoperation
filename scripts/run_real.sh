#!/usr/bin/env bash
# scripts/run_real.sh
# Production launcher for AGIBOT A2 Ultra real robot hardware teleoperation.
# Enforces pre-flight safety checklist, domain checks, and local process isolation.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_PYTHON="${PROJECT_ROOT}/venv/bin/python"

if [ ! -f "${VENV_PYTHON}" ]; then
    echo "[FATAL] Virtualenv python not found at ${VENV_PYTHON}. Run setup first." >&2
    exit 1
fi

echo "================================================================================"
echo "          AGIBOT A2 ULTRA - REAL ROBOT TELEOPERATION LAUNCHER"
echo "================================================================================"
echo "PRE-FLIGHT SAFETY CHECKLIST (ALL 4 REQUIRED BEFORE COMMENCING):"
echo "  [ ] 1. Overhead gantry / safety harness is physically attached and tensioned."
echo "  [ ] 2. Wireless hardware E-STOP is held by a dedicated SECOND SPOTTER."
echo "  [ ] 3. Robot workspace is completely clear of personnel (2 m safety perimeter)."
echo "  [ ] 4. Robot battery charged, x86 controller powered, emergency stop functional."
echo "--------------------------------------------------------------------------------"

# Check ROS_DOMAIN_ID
DOMAIN_ID="${ROS_DOMAIN_ID:-}"
if [ -z "${DOMAIN_ID}" ]; then
    echo "[FATAL] ROS_DOMAIN_ID is unset in environment." >&2
    echo "        Export the robot DDS domain ID first (e.g., export ROS_DOMAIN_ID=232)." >&2
    exit 1
fi

if [ "${DOMAIN_ID}" = "77" ]; then
    echo "[WARNING] ROS_DOMAIN_ID=77 is the simulated loopback test domain." >&2
    echo "          For real hardware execution, use the physical robot domain (default: 232)." >&2
    read -r -p "Are you testing on test domain 77 loopback? (y/N): " CONFIRM_DOMAIN
    if [ "${CONFIRM_DOMAIN,,}" != "y" ]; then
        echo "[ABORTED] Update ROS_DOMAIN_ID and re-run."
        exit 1
    fi
fi

echo ""
echo "Type exact phrase 'I CONFIRM PREFLIGHT CHECKLIST COMPLETE' to proceed:"
read -r -p "> " USER_CONFIRMATION

if [ "${USER_CONFIRMATION}" != "I CONFIRM PREFLIGHT CHECKLIST COMPLETE" ]; then
    echo "[ABORTED] Confirmation mismatch. Real hardware execution cancelled."
    exit 1
fi

echo ""
echo "[INFO] Authorizing hardware backend for this session process..."
# AGIBOT_ALLOW_REAL is exported ONLY for the executed child process
export AGIBOT_ALLOW_REAL=1

cd "${PROJECT_ROOT}"

# Default real flags: conservative velocity 0.3 rad/s, capped hand raw 800, thumb swing held neutral
exec "${VENV_PYTHON}" main.py \
    --backend real \
    --i-understand-hardware-risk \
    --real-max-vel 0.3 \
    --hand-raw-max 800 \
    "$@"

