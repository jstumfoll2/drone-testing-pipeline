# Development and CI image: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + PX4 SITL.
# Build:  docker build -t dtp-dev -f .devcontainer/Dockerfile .
FROM osrf/ros:jazzy-desktop

ARG PX4_VERSION=v1.17.0
ARG PX4_DIR=/opt/PX4-Autopilot
ARG BUILD_PX4=1
ENV DEBIAN_FRONTEND=noninteractive \
    PX4_VERSION=${PX4_VERSION} \
    PX4_DIR=${PX4_DIR}

# Base tools. PX4's ubuntu.sh calls sudo, so make sure it exists.
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo git curl wget ca-certificates lsb-release gnupg \
      python3-pip python3-vcstool python3-pytest python3-yaml \
      python3-opencv ros-jazzy-cv-bridge ros-jazzy-launch-testing-ament-cmake \
      ccache \
    && rm -rf /var/lib/apt/lists/*

# PX4 source + simulation toolchain (installs Gazebo Harmonic from the OSRF repo).
COPY scripts/setup_px4.sh /tmp/setup_px4.sh
RUN if [ "$BUILD_PX4" = "1" ]; then \
      PX4_NO_BUILD=0 bash /tmp/setup_px4.sh "$PX4_DIR"; \
    else \
      PX4_NO_BUILD=1 bash /tmp/setup_px4.sh "$PX4_DIR"; \
    fi \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 <-> Gazebo Harmonic bridge (needs the OSRF repo added by ubuntu.sh above).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gzharmonic \
    && rm -rf /var/lib/apt/lists/*

# The base image ships a non-root `ubuntu` user (uid 1000); give it PX4 and sudo.
RUN chown -R ubuntu:ubuntu "$PX4_DIR" \
    && echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc \
    && echo '[ -f /workspaces/drone-testing-pipeline/drone_ws/install/setup.bash ] && source /workspaces/drone-testing-pipeline/drone_ws/install/setup.bash' >> /home/ubuntu/.bashrc

USER ubuntu
WORKDIR /workspaces/drone-testing-pipeline
