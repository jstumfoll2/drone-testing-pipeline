#!/usr/bin/env bash
# Clone PX4-Autopilot at the pinned release, install its simulation toolchain
# (this also installs Gazebo Harmonic), and build the SITL target.
#
# Usage: scripts/setup_px4.sh [install_dir]
#   PX4_VERSION   release tag to check out (default: v1.17.0)
#   PX4_NO_BUILD  set to 1 to skip `make px4_sitl`
set -euo pipefail

PX4_VERSION="${PX4_VERSION:-v1.17.0}"
PX4_DIR="${1:-${PX4_DIR:-$HOME/PX4-Autopilot}}"

if [[ ! -d "$PX4_DIR/.git" ]]; then
  git clone --branch "$PX4_VERSION" --depth 1 --recurse-submodules --shallow-submodules \
    https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
else
  echo "PX4 already present at $PX4_DIR; checking out $PX4_VERSION"
  git -C "$PX4_DIR" fetch --depth 1 origin "refs/tags/$PX4_VERSION:refs/tags/$PX4_VERSION"
  git -C "$PX4_DIR" checkout "$PX4_VERSION"
  git -C "$PX4_DIR" submodule update --init --recursive --depth 1
fi

# --no-nuttx: skip the ARM firmware toolchain; we only need SITL until hardware arrives.
bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx

if [[ "${PX4_NO_BUILD:-0}" != "1" ]]; then
  # Build once so the first `make px4_sitl gz_*` launch is fast.
  (cd "$PX4_DIR" && DONT_RUN=1 make px4_sitl)
fi

echo "PX4 $PX4_VERSION ready at $PX4_DIR"
