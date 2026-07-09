#!/usr/bin/env bash
set -euo pipefail

# Stops the tmux sessions started by launch_multi_robot.sh on all 3 robots.
#
# Default: sends Ctrl-C into each session so launch_real_hardware.sh (and
# the ROS nodes it started) shut down cleanly.
# --force: kills the tmux session outright instead.

SSH_USER="ubuntu"
SSH_KEY="${HOME}/.ssh/lenovo_laptop"
SSH_OPTS=(-o ConnectTimeout=10 -i "${SSH_KEY}")
TMUX_SESSION="exploration"
IPS=(192.168.100.108 192.168.100.135 192.168.100.114)

FORCE=0
if [[ "${1:-}" == "--force" ]]; then
  FORCE=1
fi

for ip in "${IPS[@]}"; do
  echo "==> Stopping session on ${ip}"
  if [[ "${FORCE}" -eq 1 ]]; then
    ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "tmux kill-session -t ${TMUX_SESSION}" \
      || echo "  (no session running on ${ip})"
  else
    ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "tmux send-keys -t ${TMUX_SESSION} C-c" \
      || echo "  (no session running on ${ip})"
  fi
done

if [[ "${FORCE}" -eq 0 ]]; then
  echo
  echo "Sent Ctrl-C to each robot's launch (graceful shutdown)."
  echo "Run './stop_multi_robot.sh --force' to kill the tmux sessions outright."
fi
