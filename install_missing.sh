#!/usr/bin/env bash
# Installs the system/ROS packages this workspace needs to build.
#
# Safe to run standalone (e.g. `./install_missing.sh`) or sourced by other
# scripts before a colcon build. Requires ROS_DISTRO to be set (sourcing
# /opt/ros/<distro>/setup.bash does this).

set -eo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is not set; source /opt/ros/<distro>/setup.bash first." >&2
  exit 1
fi

sudo apt-get update -y

# rosdep resolves everything declared in this workspace's package.xml files
# (e.g. nav2_msgs), so prefer it when available/initialized.
if command -v rosdep >/dev/null 2>&1; then
  if [[ ! -d /etc/ros/rosdep/sources.list.d ]] || [[ -z "$(ls -A /etc/ros/rosdep/sources.list.d 2>/dev/null)" ]]; then
    sudo rosdep init || true
  fi
  rosdep update || true
  PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  rosdep install --from-paths "${PROJECT_ROOT}" --ignore-src -y --rosdistro "${ROS_DISTRO}" || true
fi

# A few build-time system libs aren't declared in any package.xml (e.g. Qt,
# needed to build the rviz plugin), so install them directly too.
sudo apt-get install -y \
  "ros-${ROS_DISTRO}-nav2-msgs" \
  qtbase5-dev \
  qtdeclarative5-dev \
  "ros-${ROS_DISTRO}-rviz2" \
  "ros-${ROS_DISTRO}-rviz-common" \
  "ros-${ROS_DISTRO}-navigation2" \
  "ros-${ROS_DISTRO}-nav2-bringup" \
  "ros-${ROS_DISTRO}-nav2-minimal-tb*"
