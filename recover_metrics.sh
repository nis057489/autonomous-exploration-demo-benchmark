#!/usr/bin/env bash
set -euo pipefail

# Pulls down experiment_runs/ (rosbag + ROS node logs + CPU load samples)
# recorded by launch_real_hardware.sh on each robot when RECORD_METRICS=true
# in experiment.conf. See experiment.conf's RECORD_METRICS comment for what
# each run directory contains.
#
# Usage: ./recover_metrics.sh [--clean]
#   --clean  Delete each robot's experiment_runs/ after a successful pull.
#            Off by default -- these are the only copy of the data until
#            pulled, and robot disk is limited but not usually tight enough
#            to require this after every run.

SSH_USER="ubuntu"
SSH_KEY="${HOME}/.ssh/lenovo_laptop"
SSH_OPTS=(-o ConnectTimeout=10 -i "${SSH_KEY}")
REPO_DIR='$HOME/autonomous-exploration-demo-benchmark'
LOCAL_OUT_DIR="./experiment_runs"

CLEAN=0
if [[ "${1:-}" == "--clean" ]]; then
  CLEAN=1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/experiment.conf"
if [[ -z "${ROBOT_HOSTS:-}" ]]; then
  echo "ROBOT_HOSTS is not set in experiment.conf -- cannot determine which robots to pull from." >&2
  exit 1
fi

mkdir -p "${LOCAL_OUT_DIR}"

IFS=',' read -ra _robot_hosts <<< "${ROBOT_HOSTS}"
for _host in "${_robot_hosts[@]}"; do
  _host="$(echo "${_host}" | xargs)"
  [[ -z "${_host}" ]] && continue
  IFS='@' read -r robot_id ip _x _y _yaw <<< "${_host}"

  echo "==> Pulling experiment_runs from ${robot_id} (${ip})"

  # rsync's user@host:path form does NOT expand $HOME the way a plain
  # `ssh host "command referencing \$REPO_DIR"` does elsewhere in this repo
  # (that trick relies on the remote login shell interpreting the command
  # string) -- rsync takes the path after the colon literally, so resolve it
  # to a real absolute path over ssh first.
  remote_repo_dir="$(ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "echo ${REPO_DIR}" 2>/dev/null)"
  if [[ -z "${remote_repo_dir}" ]]; then
    echo "  (could not reach ${robot_id} -- skipping)"
    continue
  fi

  if ! ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "[ -d ${remote_repo_dir}/experiment_runs ]" 2>/dev/null; then
    echo "  (no experiment_runs on ${robot_id} -- skipping)"
    continue
  fi

  mkdir -p "${LOCAL_OUT_DIR}/${robot_id}"
  rsync -az -e "ssh ${SSH_OPTS[*]}" \
    "${SSH_USER}@${ip}:${remote_repo_dir}/experiment_runs/" "${LOCAL_OUT_DIR}/${robot_id}/"

  if [[ "${CLEAN}" -eq 1 ]]; then
    echo "  Cleaning remote copy on ${robot_id}"
    ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "rm -rf ${remote_repo_dir}/experiment_runs"
  fi
done

echo
echo "All runs pulled into ${LOCAL_OUT_DIR}/<robot_id>/<timestamp>_<transport>_<robot_id>/"
echo "Each run directory contains:"
echo "  bag/            -- rosbag with /explore/traversed_path (path length),"
echo "                     /nav_map (composite map, for viewing), and the actual"
echo "                     map traffic sent/received: /map + /incoming/<peer>/map"
echo "                     on baseline runs, vxch/map + incoming/<peer> band_*/"
echo "                     manifest on vxch runs. Use \`ros2 bag info\` on either"
echo "                     for exact per-topic byte totals (bandwidth)."
echo "  ros_logs/       -- ROS node logs, incl. ddil_proxy_node stats (rough running"
echo "                     bytes-sent counter, not authoritative -- use the bag) and"
echo "                     occupancy_grid_vxch_node stats (encoded size before DDIL"
echo "                     throttling, vxch runs only)"
echo "  cpu_load.log    -- periodic /proc/loadavg samples"
