#!/usr/bin/env bash
set -euo pipefail

# Launches launch_real_hardware.sh on all 3 robots over SSH, from a laptop
# that is not itself one of the robots. All robots are synced and launched
# concurrently (one background subshell per robot) rather than one at a
# time, so a slow git pull or ssh round-trip on one robot doesn't delay the
# others. Each robot runs its command inside a detached tmux session so it
# keeps running independently of this SSH connection and can be stopped
# cleanly with stop_multi_robot.sh.
#
# See MULTI_ROBOT.md for the underlying per-robot commands.
#
# Usage: ./launch_multi_robot.sh [--rebuild]
#   --rebuild  Force a colcon rebuild of the vxch/nav/explore packages on each
#              robot before launching. launch_real_hardware.sh's automatic
#              build check only looks at whether install/ artifacts exist, not
#              whether the source changed -- so a git-pulled source change to
#              an already-built package (e.g. voxelcodec_ros) silently keeps
#              running the stale binary unless this is passed.

REBUILD_FLAG=""
if [[ "${1:-}" == "--rebuild" ]]; then
  REBUILD_FLAG="--rebuild"
fi

SSH_USER="ubuntu"
SSH_KEY="${HOME}/.ssh/lenovo_laptop"
SSH_OPTS=(-o ConnectTimeout=10 -i "${SSH_KEY}")
REPO_DIR='$HOME/autonomous-exploration-demo-benchmark'
TMUX_SESSION="exploration"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Robot id/ip/offsets come from experiment.conf's ROBOT_HOSTS -- the same
# single source of truth hw_namespaced_stack.launch.py uses for peer offsets
# in team_map_fusion. A hardcoded second copy here previously drifted out of
# sync with experiment.conf (different x/y per robot), which made each
# robot place its own map at one offset but its peers' shared map data at a
# different, mismatched offset -- the direct cause of inconsistent map
# alignment between robots in RViz.
source "${SCRIPT_DIR}/experiment.conf"
if [[ -z "${ROBOT_HOSTS:-}" ]]; then
  echo "ROBOT_HOSTS is not set in experiment.conf -- cannot determine which robots to launch." >&2
  exit 1
fi

# robot-id ip offset-x offset-y offset-yaw
ROBOTS=()
IFS=',' read -ra _robot_hosts <<< "${ROBOT_HOSTS}"
for _host in "${_robot_hosts[@]}"; do
  _host="$(echo "${_host}" | xargs)"
  [[ -z "${_host}" ]] && continue
  IFS='@' read -r _name _ip _x _y _yaw <<< "${_host}"
  if [[ -z "${_name}" || -z "${_ip}" || -z "${_x}" || -z "${_y}" || -z "${_yaw}" ]]; then
    echo "Malformed ROBOT_HOSTS entry '${_host}' (expected name@ip@x@y@yaw)." >&2
    exit 1
  fi
  ROBOTS+=("${_name} ${_ip} ${_x} ${_y} ${_yaw}")
done

# Status dir for background jobs to report their exit code back to this
# shell -- $! / wait only tell us the exit status of the sed in each
# robot's pipeline (the last command in the pipe), not the ssh commands
# feeding it, so each subshell records its own real status to a file here.
STATUS_DIR="$(mktemp -d)"
trap 'rm -rf "${STATUS_DIR}"' EXIT

pids=()
for entry in "${ROBOTS[@]}"; do
  read -r robot_id ip offset_x offset_y offset_yaw <<< "$entry"

  (
    # Disabled so a failed ssh step doesn't kill this subshell before it
    # records its status below (the pipeline as a whole, and thus this
    # subshell under -e, would otherwise abort on the first failure).
    set +e
    {
      echo "==> Syncing repo on ${robot_id} on ${ip}"
      ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" \
        "cd ${REPO_DIR} && git pull && git submodule sync --recursive && git submodule update --init --recursive"

      # Push the local experiment.conf as-is, bypassing git entirely, *after*
      # the git pull above so it isn't blocked by "local changes would be
      # overwritten by merge". This way DDIL params (MAP_TRANSPORT,
      # BANDWIDTH_KBPS, ...) always match what's on this laptop even if the
      # change isn't committed/pushed yet, or the robot's checkout is on a
      # different branch than git pull would fast-forward.
      echo "==> Pushing experiment.conf to ${robot_id} on ${ip}"
      ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "cat > ${REPO_DIR}/experiment.conf" < experiment.conf

      echo "==> Launching ${robot_id} on ${ip}"

      # Command exactly as you'd type it at an interactive SSH prompt.
      launch_line="cd ${REPO_DIR} && ./launch_real_hardware.sh --robot-id ${robot_id} --robot-offset-x ${offset_x} --robot-offset-y ${offset_y} --robot-offset-yaw ${offset_yaw} --local-bringup ${REBUILD_FLAG}"

      # tmux runs its panes as login shells by default, same as a fresh
      # interactive SSH session, and send-keys types the line in as if you'd
      # typed it yourself (Enter key included) rather than running it via a
      # non-interactive `ssh host command`.
      remote_cmd="tmux kill-session -t ${TMUX_SESSION} >/dev/null 2>&1; tmux new-session -d -s ${TMUX_SESSION}; tmux send-keys -t ${TMUX_SESSION} '${launch_line}' Enter"

      ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "${remote_cmd}"
    } 2>&1 | sed -u "s/^/[${robot_id}] /"
    status=${PIPESTATUS[0]}
    echo "${status}" > "${STATUS_DIR}/${robot_id}"
    exit "${status}"
  ) &
  pids+=($!)
done

# Wait for every robot's subshell regardless of individual failures, then
# check their recorded statuses -- a plain `wait` with -e would bail out on
# the first failing job and leave the others' ssh sessions/output dangling.
for pid in "${pids[@]}"; do
  wait "${pid}" || true
done

fail=0
for entry in "${ROBOTS[@]}"; do
  read -r robot_id _ <<< "$entry"
  st="$(cat "${STATUS_DIR}/${robot_id}" 2>/dev/null || echo 1)"
  if [[ "${st}" != "0" ]]; then
    echo "!! ${robot_id} failed (exit ${st}) -- see [${robot_id}] output above." >&2
    fail=1
  fi
done

if [[ ${fail} -ne 0 ]]; then
  echo "One or more robots failed to sync/launch." >&2
  exit 1
fi

echo
echo "All robots launching in detached tmux session '${TMUX_SESSION}'."
echo "Attach to watch a robot's output, e.g.:"
echo "  ssh -i ${SSH_KEY} ${SSH_USER}@192.168.100.108 -t tmux attach -t ${TMUX_SESSION}"
echo "(Ctrl-b d to detach without stopping it.)"
echo
echo "Stop everything with: ./stop_multi_robot.sh"
