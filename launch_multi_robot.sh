#!/usr/bin/env bash
set -euo pipefail

# Launches launch_real_hardware.sh on all 3 robots over SSH, from a laptop
# that is not itself one of the robots. Each robot runs its command inside
# a detached tmux session so it keeps running independently of this SSH
# connection and can be stopped cleanly with stop_multi_robot.sh.
#
# See MULTI_ROBOT.md for the underlying per-robot commands.

SSH_USER="ubuntu"
SSH_KEY="${HOME}/.ssh/lenovo_laptop"
SSH_OPTS=(-o ConnectTimeout=10 -i "${SSH_KEY}")
REPO_DIR='$HOME/autonomous-exploration-demo-benchmark'
TMUX_SESSION="exploration"

# robot-id ip offset-x offset-y offset-yaw
ROBOTS=(
  "robot1 192.168.100.108 0.0 0.0 0.0"
  "robot2 192.168.100.135 5.7 1.5 0.0"
  "robot3 192.168.100.114 0.0 -3.6 0.0"
)

for entry in "${ROBOTS[@]}"; do
  read -r robot_id ip offset_x offset_y offset_yaw <<< "$entry"
  echo "==> Launching ${robot_id} on ${ip}"

  # Command exactly as you'd type it at an interactive SSH prompt.
  launch_line="cd ${REPO_DIR} && git pull && git submodule sync --recursive && git submodule update --init --recursive && ./launch_real_hardware.sh --robot-id ${robot_id} --robot-offset-x ${offset_x} --robot-offset-y ${offset_y} --robot-offset-yaw ${offset_yaw} --local-bringup"

  # tmux runs its panes as login shells by default, same as a fresh
  # interactive SSH session, and send-keys types the line in as if you'd
  # typed it yourself (Enter key included) rather than running it via a
  # non-interactive `ssh host command`.
  remote_cmd="tmux kill-session -t ${TMUX_SESSION} >/dev/null 2>&1; tmux new-session -d -s ${TMUX_SESSION}; tmux send-keys -t ${TMUX_SESSION} '${launch_line}' Enter"

  ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "${remote_cmd}"
done

echo
echo "All robots launching in detached tmux session '${TMUX_SESSION}'."
echo "Attach to watch a robot's output, e.g.:"
echo "  ssh -i ${SSH_KEY} ${SSH_USER}@192.168.100.108 -t tmux attach -t ${TMUX_SESSION}"
echo "(Ctrl-b d to detach without stopping it.)"
echo
echo "Stop everything with: ./stop_multi_robot.sh"
