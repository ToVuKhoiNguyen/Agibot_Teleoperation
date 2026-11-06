#!/usr/bin/env bash
# scripts/run.sh
# Unified launcher for AGIBOT A2 Ultra Teleoperation system.
# Wraps existing main.py flags and diagnostic scripts.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# Resolve Python executable
if [ -f "$PROJECT_ROOT/venv/bin/python" ]; then
    PYTHON_BIN="$PROJECT_ROOT/venv/bin/python"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
else
    PYTHON_BIN="python"
fi

print_url_and_adb() {
    local url="$1"
    echo "================================================================================"
    echo "  WEB URL     : $url"
    echo "  QUEST ADB   : adb reverse tcp:8012 tcp:8012"
    echo "  HEALTH CHECK: https://localhost:8012/health"
    echo "================================================================================"
}

print_help() {
    cat << 'EOF'
AGIBOT A2 Ultra Teleoperation - Unified Launcher
Usage: ./scripts/run.sh <subcommand> [options]

Subcommands:
| Command      | Description                                      | Hardware Motion |
|--------------|--------------------------------------------------|-----------------|
| sim          | MuJoCo simulation with 3D mouse gizmos           | NO (Simulation) |
| controllers  | Quest 3S WebXR dual controller teleop in sim     | NO (Simulation) |
| hands        | Quest 3S WebXR 1:1 hand-tracking teleop in sim   | NO (Simulation) |
| dryrun       | Pure-virtual dry-run backend (CLI telemetry)     | NO (Virtual)    |
| ros2dry      | ROS 2 publishers with zero robot actuation       | NO (Sim/Dry)    |
| replay <f>   | Replay joint trajectory from recorded file       | NO (Replay)     |
| real         | Real robot teleoperation via scripts/run_real.sh | YES (DANGER!)   |
| test         | Run complete 8-suite automated test suite        | NO (Localhost)  |
| preflight    | Run pre-flight safety & connectivity checks      | NO (Diagnostic) |
| report       | Generate test & safety acceptance report         | NO (Read-only)  |
| listen       | Monitor ROS 2 robot joint state topics           | NO (Read-only)  |
| compare-home | Compare current robot joint poses against home   | NO (Read-only)  |
| sign-check   | Verify joint direction signs against URDF model  | NO (Read-only)  |
| help         | Display this subcommand reference table          | NO (Help)       |

Note: Default without arguments displays this help and exits.
EOF
}

CMD="${1:-}"
if [ -z "$CMD" ]; then
    print_help
    exit 1
fi
shift || true

case "$CMD" in
    sim)
        print_url_and_adb "https://localhost:8012/"
        exec "$PYTHON_BIN" main.py --backend sim --input controllers "$@"
        ;;
    controllers)
        print_url_and_adb "https://localhost:8012/?input=controllers"
        exec "$PYTHON_BIN" main.py --backend sim --controller --input controllers "$@"
        ;;
    hands)
        print_url_and_adb "https://localhost:8012/?input=hands"
        exec "$PYTHON_BIN" main.py --backend sim --input hands "$@"
        ;;
    dryrun)
        print_url_and_adb "https://localhost:8012/"
        exec "$PYTHON_BIN" main.py --backend dryrun "$@"
        ;;
    ros2dry)
        print_url_and_adb "https://localhost:8012/"
        exec "$PYTHON_BIN" main.py --backend ros2dry "$@"
        ;;
    replay)
        if [ -z "${1:-}" ]; then
            echo "[ERROR] 'replay' requires a trajectory file path. Usage: ./scripts/run.sh replay <file>" >&2
            exit 1
        fi
        print_url_and_adb "https://localhost:8012/"
        exec "$PYTHON_BIN" main.py --backend replay "$@"
        ;;
    real)
        echo "[INFO] Delegating to scripts/run_real.sh with pre-flight checklist & typed confirmation."
        exec bash "$SCRIPT_DIR/run_real.sh" "$@"
        ;;
    test)
        exec bash "$SCRIPT_DIR/test_all.sh" "$@"
        ;;
    preflight)
        exec "$PYTHON_BIN" "$SCRIPT_DIR/preflight.py" "$@"
        ;;
    report)
        exec "$PYTHON_BIN" "$SCRIPT_DIR/make_report.py" "$@"
        ;;
    listen)
        exec "$PYTHON_BIN" "$SCRIPT_DIR/ros2_listen.py" "$@"
        ;;
    compare-home)
        exec "$PYTHON_BIN" "$SCRIPT_DIR/ros2_compare_home.py" "$@"
        ;;
    sign-check)
        exec "$PYTHON_BIN" "$SCRIPT_DIR/ros2_sign_check.py" "$@"
        ;;
    help|--help|-h)
        print_help
        exit 0
        ;;
    *)
        echo "[ERROR] Unknown subcommand: '$CMD'" >&2
        echo "" >&2
        print_help >&2
        exit 1
        ;;
esac
