# Drone testing pipeline

A software recreation of the staged drone testing framework from
Jiang et al., *"A Step-by-Step Guide to Creating a Robust Autonomous Drone
Testing Pipeline"* (arXiv 2506.11400, 2025), built on current tooling and designed to run end to end
in simulation until hardware is available.

The paper takes a marker-based landing system through four stages:
software in the loop, hardware in the loop, controlled real-world tests and
in-field tests. This repo keeps that structure but swaps each hardware stage
for a simulated stand-in, behind one topic contract, so the same autonomy
nodes and test scenarios move to real hardware later without changes.

| Stage | Here (now) | Later (hardware) |
|---|---|---|
| 1. SIL | PX4 SITL + Gazebo Harmonic | same |
| 2. HIL | SITL in a CPU/memory-capped container, `tc netem` link effects, PX4 failure injection | Jetson companion + Pixhawk |
| 3. Controlled real world | Gazebo worlds with wind, lighting and camera noise | Netted test area, mocap or RTK |
| 4. In field | Scenario suites with randomised conditions | Field trials |

## Stack

ROS 2 Jazzy on Ubuntu 24.04, Gazebo Harmonic, PX4 v1.17.0, Micro XRCE-DDS
Agent v2.4.3. See [docs/VERSIONS.md](docs/VERSIONS.md) for every pin and how
to upgrade.

## Layout

```
.devcontainer/          Dev container (ROS 2 Jazzy desktop + PX4 SITL)
dtp.repos               Pinned third-party sources (px4_msgs, interface lib, XRCE agent)
scripts/
  setup_px4.sh          Clone and build PX4 SITL at the pinned version
  bootstrap_ws.sh       Import sources, rosdep, colcon build
  smoke_test.sh         Gate 0: take off over the marker and see it
drone_ws/src/
  dtp_perception/       ArUco marker detector (ROS-free core + rclpy node)
  dtp_test/             Scenarios, landing metrics, smoke flight runner
  dtp_bringup/          Launch files (sim backend today; hil and real next)
docs/TOPICS.md          The topic contract every backend must satisfy
```

## Quick start

Open the repo in the dev container (VS Code: *Reopen in Container*). The
image builds PX4 SITL and the post-create step builds the workspace. Then:

```bash
source drone_ws/install/setup.bash
ros2 launch dtp_bringup sim.launch.py            # Gazebo GUI, aruco world
scripts/smoke_test.sh                             # headless Gate 0 check
```

Without the container, on Ubuntu 24.04 with ROS 2 Jazzy installed:

```bash
scripts/setup_px4.sh            # ~/PX4-Autopilot by default
scripts/bootstrap_ws.sh
```

## Tests

The detector core and the metrics/scenario code have no ROS dependency, so
they run anywhere:

```bash
pip install opencv-contrib-python-headless numpy pyyaml pytest
(cd drone_ws/src/dtp_perception && pytest -q test)
(cd drone_ws/src/dtp_test && pytest -q test)
```

CI runs those on every push, a full `colcon build` and `colcon test` in a
`ros:jazzy` container, and the PX4 + Gazebo smoke test nightly or on demand.

## Roadmap

Phase 0 (this) is the scaffold and Gate 0 smoke test. Next: the landing
controller (MLS-V1) and scored landing scenarios, then the simulated HIL
backend, then condition sweeps for stages 3 and 4.

## License

Apache-2.0
