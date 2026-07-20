#!/usr/bin/env bash
set -euo pipefail

# Barrier-starts exploration on every robot in ROBOT_HOSTS at (close to) the same time,
# instead of each robot racing its own fixed startup_delay_s timer against Nav2's own
# (variable-length) lifecycle bring-up -- see config.yaml's `autostart: false` comment.
#
# Run this from the laptop, after ./launch_multi_robot.sh has kicked off every robot (each
# robot's own launch_real_hardware.sh must still be running, with
# frontier_exploration_ros2/config.yaml's autostart:false + control_service_enabled:true).
# The laptop can reach every robot's ROS graph directly for this -- hw_namespaced_stack.launch.py
# statically peers each robot with the laptop's IP even though robots don't discover each other.
#
# No robot gets ACTION_START until every robot in ROBOT_HOSTS has a confirmed-active Nav2
# stack (navigate_to_pose action server up) and a live frontier_explorer control service.
#
# Usage: ./sync_start_exploration.sh [--timeout <seconds>]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"

TIMEOUT_S=180
if [[ "${1:-}" == "--timeout" ]]; then
  TIMEOUT_S="${2:?--timeout requires a value in seconds}"
fi

# shellcheck source=/dev/null
source "${PROJECT_ROOT}/experiment.conf"
if [[ -z "${ROBOT_HOSTS:-}" ]]; then
  echo "ROBOT_HOSTS is not set in experiment.conf -- cannot determine which robots to start." >&2
  exit 1
fi

source /opt/ros/jazzy/setup.bash
if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${PROJECT_ROOT}/install/setup.bash"
fi

ROBOT_IDS=()
IFS=',' read -ra _robot_hosts <<< "${ROBOT_HOSTS}"
for _host in "${_robot_hosts[@]}"; do
  _host="$(echo "${_host}" | xargs)"
  [[ -z "${_host}" ]] && continue
  ROBOT_IDS+=("${_host%%@*}")
done

if [[ ${#ROBOT_IDS[@]} -eq 0 ]]; then
  echo "No robots parsed from ROBOT_HOSTS='${ROBOT_HOSTS}'." >&2
  exit 1
fi

echo "Waiting for Nav2 + frontier_explorer to be ready on: ${ROBOT_IDS[*]} (timeout ${TIMEOUT_S}s each)"

wait_for_robot_ready() {
  local robot_id="$1"
  local action_name="/${robot_id}/navigate_to_pose"
  local service_name="/${robot_id}/control_exploration"
  local waited=0
  local nav2_ready=false
  local service_ready=false

  while (( waited < TIMEOUT_S )); do
    if [[ "${nav2_ready}" == false ]] && ros2 action list 2>/dev/null | grep -qx "${action_name}"; then
      nav2_ready=true
    fi
    if [[ "${service_ready}" == false ]] && ros2 service list 2>/dev/null | grep -qx "${service_name}"; then
      service_ready=true
    fi
    if [[ "${nav2_ready}" == true && "${service_ready}" == true ]]; then
      echo "  ${robot_id}: ready (${waited}s)"
      return 0
    fi
    sleep 2
    waited=$(( waited + 2 ))
  done

  echo "  ${robot_id}: TIMED OUT after ${TIMEOUT_S}s (nav2=${nav2_ready} control_service=${service_ready})" >&2
  return 1
}

READY_PIDS=()
for robot_id in "${ROBOT_IDS[@]}"; do
  wait_for_robot_ready "${robot_id}" &
  READY_PIDS+=("$!")
done

FAILED=0
for pid in "${READY_PIDS[@]}"; do
  wait "${pid}" || FAILED=1
done

if [[ "${FAILED}" -eq 1 ]]; then
  echo "Not all robots came up in time -- exploration NOT started on any robot. Check the robot(s) reported above." >&2
  exit 1
fi

echo "All ${#ROBOT_IDS[@]} robots ready. Starting exploration on all of them..."

START_PIDS=()
for robot_id in "${ROBOT_IDS[@]}"; do
  (
    ros2 service call "/${robot_id}/control_exploration" \
      frontier_exploration_ros2/srv/ControlExploration "{action: 1}"
  ) > "/tmp/sync_start_${robot_id}.log" 2>&1 &
  START_PIDS+=("$!")
done
for pid in "${START_PIDS[@]}"; do
  wait "${pid}" || true
done

for robot_id in "${ROBOT_IDS[@]}"; do
  echo "--- ${robot_id} ---"
  cat "/tmp/sync_start_${robot_id}.log"
  rm -f "/tmp/sync_start_${robot_id}.log"
done

echo
echo "Exploration start requested on: ${ROBOT_IDS[*]}"
