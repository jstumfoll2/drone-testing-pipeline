# Pinned versions

| Component | Version | Where it is pinned |
| --- | --- | --- |
| Ubuntu | 24.04 LTS | `.devcontainer/Dockerfile` base image |
| ROS 2 | Jazzy | `.devcontainer/Dockerfile` (`ros:jazzy`) |
| Gazebo | Harmonic (via `ros-jazzy-ros-gz`, the default Gazebo pairing for Jazzy) | `.devcontainer/Dockerfile` |
| PX4-Autopilot | `v1.17.0` | `scripts/setup_px4.sh` (`PX4_VERSION`) |
| px4_msgs | `release/1.17` | `dtp.repos` |
| px4-ros2-interface-lib | `release/1.17` | `dtp.repos` |
| Micro XRCE-DDS Agent | `v2.4.3` | `dtp.repos` |

## Upgrading PX4

px4_msgs and px4-ros2-interface-lib must define the same messages as the
PX4 firmware. When moving to a new PX4 release:

1. Change `PX4_VERSION` in `scripts/setup_px4.sh`.
2. Change both `release/1.xx` branches in `dtp.repos`.
3. Rebuild the dev container and the workspace, then run the simulation
   smoke test (`scripts/smoke_test.sh`).

The interface library checks message compatibility when a mode registers and
will refuse to start on a mismatch.

## Why not ROS 2 Lyrical / Ubuntu 26.04 yet

PX4's CI and release target is Ubuntu 24.04 and it does not yet support 26.04.
JetPack 7 (the onboard Jetson OS) is also Ubuntu 24.04. Revisit when both move.
