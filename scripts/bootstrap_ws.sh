#!/usr/bin/env bash
# Import pinned third-party sources, install ROS dependencies, and build the workspace.
# Safe to re-run.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$REPO_ROOT/drone_ws"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

mkdir -p "$WS/src/third_party"
vcs import --skip-existing "$WS/src" < "$REPO_ROOT/dtp.repos"

if command -v rosdep >/dev/null; then
  rosdep update --rosdistro jazzy >/dev/null || true
  rosdep install --from-paths "$WS/src" --ignore-src -y --rosdistro jazzy \
    --skip-keys "px4_msgs px4_ros2_cpp px4_ros2_py" || true
fi

cd "$WS"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
echo "Workspace built. Run: source $WS/install/setup.bash"
