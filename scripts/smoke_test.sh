#!/usr/bin/env bash
# Gate 0 smoke test: bring up PX4 SITL + Gazebo headless, take off over the
# ArUco marker, and pass when the detector publishes marker poses.
#
# Usage: scripts/smoke_test.sh [timeout_s]
# Needs: built workspace (scripts/bootstrap_ws.sh) and PX4 (scripts/setup_px4.sh).
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TIMEOUT="${1:-240}"
LOG_DIR="${DTP_LOG_DIR:-$REPO_ROOT/results/smoke}"
mkdir -p "$LOG_DIR"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/drone_ws/install/setup.bash"

ros2 launch dtp_bringup sim.launch.py scenario:=smoke_aruco headless:=true \
  >"$LOG_DIR/sim.log" 2>&1 &
SIM_PID=$!
cleanup() {
  kill -INT "$SIM_PID" 2>/dev/null || true
  sleep 5
  kill -9 "$SIM_PID" 2>/dev/null || true
  pkill -f px4 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -f MicroXRCEAgent 2>/dev/null || true
}
trap cleanup EXIT

set +e
# PYTHONNOUSERSITE: ignore the NumPy 2 that PX4's setup pip-installs into
# ~/.local (see sim.launch.py). timeout_s is a double parameter, hence ".0".
PYTHONNOUSERSITE=1 timeout "$TIMEOUT" ros2 run dtp_test smoke_flight \
  --ros-args -p timeout_s:="$((TIMEOUT - 20)).0" \
  2>&1 | tee "$LOG_DIR/smoke_flight.log"
RC=${PIPESTATUS[0]}
set -e

if [ "$RC" -eq 0 ]; then
  echo "SMOKE TEST PASSED"
else
  # Collect diagnostics while the sim is still up (CI logs may be the only
  # place they can be read).
  echo "SMOKE TEST FAILED (exit $RC)"
  PX4_BIN="$(eval echo "${PX4_DIR:-$HOME/PX4-Autopilot}")/build/px4_sitl_default/bin"
  if [ -x "$PX4_BIN/px4-commander" ]; then
    echo "=== PX4 pre-flight checks ==="
    (cd "$PX4_BIN/.." && timeout 10 "$PX4_BIN/px4-commander" check 2>&1) || true
    sleep 2
    grep -v '^\[microxrce_agent' "$LOG_DIR/sim.log" | tail -n 30 || true
  fi
  echo "=== Gazebo topics ==="
  timeout 10 gz topic -l 2>&1 | head -60 || true
  echo "=== ROS topics ==="
  timeout 10 ros2 topic list 2>&1 | grep -v '^/fmu/' || true
  echo "=== PX4 / Gazebo / bridge warnings and errors ==="
  grep -v '^\[microxrce_agent' "$LOG_DIR/sim.log" \
    | grep -Ei 'warn|error|fail|preflight|arming|camera|render|ogre|egl|gl ' \
    | grep -v 'successfully created' | tail -n 120 || true
  echo "=== Last sim log lines (agent filtered) ==="
  grep -v '^\[microxrce_agent' "$LOG_DIR/sim.log" | tail -n 60 || true
fi
exit "$RC"
