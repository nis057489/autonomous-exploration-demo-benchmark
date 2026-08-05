#!/usr/bin/env bash
set -euo pipefail

# Provisions all robots with the GitHub deploy key used to pull this repo,
# so launch_multi_robot.sh's git-pull-then-launch flow doesn't need
# credentials typed in over SSH each time. Also clones the repo (checking
# out $GIT_BRANCH) and runs install_missing.sh on each robot.
#
# Copies ~/.ssh/github (+ .pub) to each robot and adds a Host github.com
# block to the robot's ~/.ssh/config pointing git operations at that key.
# Safe to re-run: the config block is marked and replaced, not duplicated,
# and the clone step is skipped (fetch + checkout instead) if the repo
# directory already exists.

SSH_USER="ubuntu"
SSH_KEY="${HOME}/.ssh/lenovo_laptop"
SSH_OPTS=(-o ConnectTimeout=10 -i "${SSH_KEY}")
GITHUB_USER="nis057489"
GIT_REPO="git@github.com:nis057489/autonomous-exploration-demo-benchmark.git"
GIT_BRANCH="vxch_3"
REPO_DIR='$HOME/autonomous-exploration-demo-benchmark'

LOCAL_GITHUB_KEY="${HOME}/.ssh/github"
LOCAL_GITHUB_PUB="${HOME}/.ssh/github.pub"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f "${LOCAL_GITHUB_KEY}" || ! -f "${LOCAL_GITHUB_PUB}" ]]; then
  echo "Expected ${LOCAL_GITHUB_KEY} and ${LOCAL_GITHUB_KEY}.pub to exist locally." >&2
  exit 1
fi

# Robot IPs come from experiment.conf's ROBOT_HOSTS -- same single source of
# truth launch_multi_robot.sh uses, so this stays in sync as the fleet changes.
source "${SCRIPT_DIR}/experiment.conf"
if [[ -z "${ROBOT_HOSTS:-}" ]]; then
  echo "ROBOT_HOSTS is not set in experiment.conf -- cannot determine which robots to provision." >&2
  exit 1
fi

IPS=()
IFS=',' read -ra _robot_hosts <<< "${ROBOT_HOSTS}"
for _host in "${_robot_hosts[@]}"; do
  _host="$(echo "${_host}" | xargs)"
  [[ -z "${_host}" ]] && continue
  IFS='@' read -r _name _ip _rest <<< "${_host}"
  IPS+=("${_ip}")
done

REMOTE_SETUP_CMD=$(cat <<EOF
set -euo pipefail
mkdir -p ~/.ssh
chmod 700 ~/.ssh
chmod 600 ~/.ssh/github
chmod 644 ~/.ssh/github.pub
touch ~/.ssh/config
chmod 600 ~/.ssh/config
sed -i '/# BEGIN provision_robots.sh github block/,/# END provision_robots.sh github block/d' ~/.ssh/config
cat >> ~/.ssh/config <<'CFG'
# BEGIN provision_robots.sh github block
Host github.com
    HostName github.com
    User ${GITHUB_USER}
    IdentityFile ~/.ssh/github
    IdentitiesOnly yes
# END provision_robots.sh github block
CFG
ssh-keyscan -H github.com >> ~/.ssh/known_hosts 2>/dev/null
if [ -d "${REPO_DIR}" ]; then
  echo "  (repo already present at ${REPO_DIR}, skipping clone)"
  git -C "${REPO_DIR}" fetch origin
else
  git clone "${GIT_REPO}" "${REPO_DIR}"
fi
git -C "${REPO_DIR}" checkout "${GIT_BRANCH}"
source /opt/ros/jazzy/setup.bash
"${REPO_DIR}/install_missing.sh"
EOF
)

for ip in "${IPS[@]}"; do
  echo "==> Provisioning ${ip}"
  scp "${SSH_OPTS[@]}" "${LOCAL_GITHUB_KEY}" "${LOCAL_GITHUB_PUB}" "${SSH_USER}@${ip}:~/.ssh/"
  ssh "${SSH_OPTS[@]}" "${SSH_USER}@${ip}" "${REMOTE_SETUP_CMD}"
done

echo
echo "Done. Verify on a robot with: ssh -T git@github.com"
