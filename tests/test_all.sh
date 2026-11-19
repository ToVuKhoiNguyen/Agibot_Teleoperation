#!/usr/bin/env bash
# tests/test_all.sh - Unified End-to-End Teleoperation Acceptance Test Runner
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$PROJECT_ROOT"

# Enforce loopback domain 77 isolation (strict safety invariant)
export ROS_DOMAIN_ID=77
export ROS_LOCALHOST_ONLY=1

PYTHON="${PROJECT_ROOT}/venv/bin/python"
if [ ! -f "$PYTHON" ]; then
  PYTHON="python3"
fi

TESTS=(
  "tests/test_mapping.py"
  "tests/test_ros2_backend.py"
  "tests/test_robot_safety.py"
  "tests/test_real_suite.py"
  "scripts/test_web_sync.py"
  "scripts/test_panel_api.py"
  "tests/test_xr_controllers.py"
  "tests/test_handtracking.py"
  "tests/test_xr_inputsources.py"
  "tests/test_fault_diagnosis.py"
  "tests/test_vendor_crosscheck.py"
  "tests/test_real_lag_contact.py"
  "tests/test_sim_follower_validation.py"
  "tests/test_latency_ladder.py"
  "tests/test_pre_live_harness.py"
  "tests/test_gizmo_regression.py"
)

TOTAL=${#TESTS[@]}
PASSED=0
FAILED=0

declare -A RESULTS
declare -A TIMES

echo "================================================================================"
echo "AGIBOT A2 ULTRA TELEOPERATION ACCEPTANCE TEST SUITE"
echo "Isolated ROS 2 Domain: $ROS_DOMAIN_ID (Localhost Only: $ROS_LOCALHOST_ONLY)"
echo "Python Executable: $PYTHON"
echo "================================================================================"

for test in "${TESTS[@]}"; do
  test_name=$(basename "$test")
  pkill -9 -f "python.*main.py" 2>/dev/null || true
  fuser -k 8012/tcp >/dev/null 2>&1 || true
  fuser -k 8013/tcp >/dev/null 2>&1 || true
  fuser -k 8035/tcp >/dev/null 2>&1 || true
  sleep 2.0
  echo -e "\n>>> RUNNING: $test_name ..."
  t_start=$(date +%s%N)
  
  if "$PYTHON" "$test"; then
    RESULTS["$test_name"]="PASS"
    PASSED=$((PASSED + 1))
  else
    RESULTS["$test_name"]="FAIL"
    FAILED=$((FAILED + 1))
  fi
  
  t_end=$(date +%s%N)
  elapsed_ms=$(( (t_end - t_start) / 1000000 ))
  elapsed_s=$(awk -v ms="$elapsed_ms" 'BEGIN { printf "%.2f", ms / 1000 }')
  TIMES["$test_name"]="${elapsed_s}s"
  sleep 1
done

echo ""
echo "================================================================================"
echo "                           TEST EXECUTION SUMMARY                               "
echo "================================================================================"
printf "| %-30s | %-10s | %-12s |\n" "Test Suite Script" "Status" "Duration"
echo "+--------------------------------+------------+--------------+"

for test in "${TESTS[@]}"; do
  test_name=$(basename "$test")
  status="${RESULTS[$test_name]}"
  dur="${TIMES[$test_name]}"
  printf "| %-30s | %-10s | %-12s |\n" "$test_name" "$status" "$dur"
done

echo "+--------------------------------+------------+--------------+"
echo "Summary: $PASSED / $TOTAL passed, failed: $FAILED"
echo "================================================================================"

if [ "$FAILED" -eq 0 ]; then
  echo "[SUCCESS] All teleoperation tests passed successfully!"
  exit 0
else
  echo "[ERROR] One or more test suites failed!"
  exit 1
fi
