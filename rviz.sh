#!/bin/bash
# set -u dropped: /opt/ros/jazzy/setup.bash and install/setup.bash reference
# unset variables internally and abort under nounset.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ros2 launch bme_ros2_navigation ... below needs this workspace's overlay to
# find the package; rviz2 itself would launch fine without it (standard
# message types only), silently leaving the VXCH displays subscribed to
# nothing, so source explicitly rather than depend on the calling shell.
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/install/setup.bash"

# shellcheck source=/dev/null
[[ -f "${SCRIPT_DIR}/experiment.conf" ]] && source "${SCRIPT_DIR}/experiment.conf"

HAAR_LEVELS="${HAAR_LEVELS:-4}"
ROBOT_NAMES="${ROBOT_NAMES:-robot1,robot2,robot3}"

export ROS_DOMAIN_ID=42
export ROS_STATIC_PEERS="192.168.100.108;192.168.100.135;192.168.100.114"

# Decodes each robot's own raw VXCH band stream locally, so rviz can show the
# progressive codec feed itself (sharpening band-by-band) rather than only an
# already-reconstructed map. See viz_vxch_decode.launch.py.
ros2 launch bme_ros2_navigation viz_vxch_decode.launch.py \
  robot_names:="${ROBOT_NAMES}" haar_levels:="${HAAR_LEVELS}" &
DECODE_PID=$!
trap 'kill "${DECODE_PID}" 2>/dev/null' EXIT

rviz2 -d ./minimal.rviz
