#!/usr/bin/env bash
set -euo pipefail

# Pulls each robot's recorded bags (see the bag-recording block in
# hw_namespaced_stack.launch.py) from ~/bags/ on the robot into ./bags/ on
# this laptop, via rsync over ssh. Bag folder names already embed
# {robot}_{map_transport}_{timestamp}, so robots merge into one flat local
# directory without colliding.
#
# Non-destructive by default. Pass --delete-remote to remove each robot's
# bags after a successful transfer (frees SD card space) -- only use this
# once you've confirmed the pulled copy is good.
#
# Usage: ./retrieve_bags.sh [--delete-remote]

DELETE_REMOTE=false
if [[ "${1:-}" == "--delete-remote" ]]; then
  DELETE_REMOTE=true
fi

SSH_USER="ubuntu"
SSH_KEY="${HOME}/.ssh/lenovo_laptop"
SSH_OPTS=(-o ConnectTimeout=10 -i "${SSH_KEY}")
# Single-quoted so $HOME expands on the remote side, not here.
REMOTE_BAGS_DIR='$HOME/bags/'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCAL_BAGS_DIR="${SCRIPT_DIR}/bags"

# Robot list/IPs come from experiment.conf's ROBOT_HOSTS -- the same single
# source of truth launch_multi_robot.sh and hw_namespaced_stack.launch.py use,
# so this always matches whichever robots you actually ran.
source "${SCRIPT_DIR}/experiment.conf"
if [[ -z "${ROBOT_HOSTS:-}" ]]; then
  echo "ROBOT_HOSTS is not set in experiment.conf -- cannot determine which robots to pull from." >&2
  exit 1
fi

mkdir -p "${LOCAL_BAGS_DIR}"

IFS=',' read -ra _robot_hosts <<< "${ROBOT_HOSTS}"
fail=0
for _host in "${_robot_hosts[@]}"; do
  _host="$(echo "${_host}" | xargs)"
  [[ -z "${_host}" ]] && continue
  IFS='@' read -r robot_id ip _x _y _yaw <<< "${_host}"

  echo "==> Checking ${robot_id} (${ip}) for bags..."
  if ! ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "[ -d ${REMOTE_BAGS_DIR} ]" 2>/dev/null; then
    echo "   no bags directory on ${robot_id} (nothing recorded yet, or unreachable) -- skipping"
    continue
  fi

  echo "==> Pulling ${robot_id}'s bags into ${LOCAL_BAGS_DIR}/"
  if rsync -avz --progress -e "ssh ${SSH_OPTS[*]}" \
      "${SSH_USER}@${ip}:${REMOTE_BAGS_DIR}" "${LOCAL_BAGS_DIR}/"; then
    if [[ "${DELETE_REMOTE}" == true ]]; then
      echo "==> Removing bags from ${robot_id} after successful transfer"
      ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "rm -rf ${REMOTE_BAGS_DIR}"
    fi
  else
    echo "!! Failed to pull bags from ${robot_id} -- see rsync output above." >&2
    fail=1
  fi
done

if [[ ${fail} -ne 0 ]]; then
  echo "One or more robots failed to transfer." >&2
  exit 1
fi

echo
echo "All available bags retrieved into ${LOCAL_BAGS_DIR}/"
ls -1 "${LOCAL_BAGS_DIR}"
