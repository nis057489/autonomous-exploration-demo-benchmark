#!/usr/bin/env bash
set -euo pipefail

# Derives ROS_STATIC_PEERS from experiment.conf's ROBOT_HOSTS (rather than
# hardcoding IPs here) so this list can't drift out of sync with whichever
# robots are actually active -- a hardcoded copy previously kept pointing at
# a retired robot's stale IP after the fleet's addresses changed.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/experiment.conf"
if [[ -z "${ROBOT_HOSTS:-}" ]]; then
  echo "ROBOT_HOSTS is not set in experiment.conf -- cannot determine robot IPs." >&2
  exit 1
fi

peers=()
IFS=',' read -ra _robot_hosts <<< "${ROBOT_HOSTS}"
for _host in "${_robot_hosts[@]}"; do
  _host="$(echo "${_host}" | xargs)"
  [[ -z "${_host}" ]] && continue
  IFS='@' read -r _name _ip _x _y _yaw <<< "${_host}"
  peers+=("${_ip}")
done
ROS_STATIC_PEERS="$(IFS=';'; echo "${peers[*]}")"

ROS_DOMAIN_ID=42 ROS_STATIC_PEERS="${ROS_STATIC_PEERS}" rviz2 -d "${SCRIPT_DIR}/real.rviz"
