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

# The PX4 ROS 2 interface library is not used until Phase 1 (landing mode),
# and px4_ros2_cpp currently fails to configure in an isolated colcon install
# (ament_index_get_resource cannot find px4_msgs' rosidl_interfaces). Skip it
# by default; set DTP_BUILD_INTERFACE_LIB=1 to build it.
SKIP=()
if [[ "${DTP_BUILD_INTERFACE_LIB:-0}" != "1" ]]; then
  SKIP=(--packages-skip px4_ros2_cpp px4_ros2_py)
fi

cd "$WS"
colcon build --symlink-install "${SKIP[@]}" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
echo "Workspace built. Run: source $WS/install/setup.bash"
