#!/usr/bin/env bash
# Launch the autonomous exploration stack on a real TurtleBot3 (ROS 2 Jazzy).
#
# Usage:
#   ./launch_real_hardware.sh [--local-bringup] [--model burger|waffle|waffle_pi] [--num-robots <n>]
#
# Flags:
#   --local-bringup   Also launch turtlebot3_bringup on this machine (useful when the
#                     workstation IS the robot, or when testing via USB-to-OpenCR).
#                     Omit this flag when the robot's Raspberry Pi is running its own
#                     bringup and you are only launching the navigation/exploration side.
#   --model <name>    TurtleBot3 model (burger | waffle | waffle_pi). Defaults to the
#                     TURTLEBOT3_MODEL env var, then falls back to waffle_pi.
#   --num-robots <n>  Number of robots (default 1). When >1 the script launches the
#                     multi-robot VXCH experiment stack instead of the single-robot path.
#                     Each robot must already be running turtlebot3_bringup independently.
#
# Experiment parameters are read from experiment.conf (project root) or env vars:
#   MAP_TRANSPORT, BANDWIDTH_KBPS, LOSS_PCT, DELAY_MS, HAAR_LEVELS
#
# What this script does NOT do (run these yourself, on the robot or on another terminal):
#   - turtlebot3_bringup robot.launch.py  (unless --local-bringup is given, single-robot only)
#   - RViz (open it manually, or pass rviz:=true to the launch file)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"

# Load experiment parameters from experiment.conf if present.
# Individual env vars always take precedence over conf file values.
if [[ -f "${PROJECT_ROOT}/experiment.conf" ]]; then
  # shellcheck source=/dev/null
  source "${PROJECT_ROOT}/experiment.conf"
fi

# Experiment parameters (set by experiment.conf or env).
MAP_TRANSPORT="${MAP_TRANSPORT:-baseline}"
BANDWIDTH_KBPS="${BANDWIDTH_KBPS:-0}"
LOSS_PCT="${LOSS_PCT:-0.0}"
DELAY_MS="${DELAY_MS:-0}"
HAAR_LEVELS="${HAAR_LEVELS:-4}"
RANDOM_SEED="${RANDOM_SEED:--1}"
ROBOT_STARTUP_DELAY_S="${ROBOT_STARTUP_DELAY_S:-0.0}"

# ── Parse arguments ──────────────────────────────────────────────────────────

LOCAL_BRINGUP=true
TB3_MODEL="${TURTLEBOT3_MODEL:-}"
NUM_ROBOTS="${NUM_ROBOTS:-1}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --local-bringup)
      LOCAL_BRINGUP=true
      shift
      ;;
    --model)
      TB3_MODEL="$2"
      shift 2
      ;;
    --num-robots)
      NUM_ROBOTS="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      echo "Usage: $0 [--local-bringup] [--model burger|waffle|waffle_pi] [--num-robots <n>]" >&2
      exit 1
      ;;
  esac
done

if ! [[ "${NUM_ROBOTS}" =~ ^[0-9]+$ ]] || (( NUM_ROBOTS < 1 )); then
  echo "NUM_ROBOTS must be a positive integer (got '${NUM_ROBOTS}')." >&2
  exit 1
fi

if [[ -z "${TB3_MODEL}" ]]; then
  TB3_MODEL="waffle_pi"
  echo "TURTLEBOT3_MODEL not set and --model not given; defaulting to '${TB3_MODEL}'."
fi

export TURTLEBOT3_MODEL="${TB3_MODEL}"

# ── ROS setup ────────────────────────────────────────────────────────────────

source /opt/ros/jazzy/setup.bash
if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  source "${PROJECT_ROOT}/install/setup.bash"
fi
set -u

# ── Validate required packages ───────────────────────────────────────────────

check_pkg() {
  if ! ros2 pkg prefix "$1" >/dev/null 2>&1; then
    echo "Required package '$1' not found. Make sure it is installed and the workspace is sourced." >&2
    exit 1
  fi
}

ensure_frontier_exploration_ros2() {
  if ros2 pkg prefix frontier_exploration_ros2 >/dev/null 2>&1; then
    return 0
  fi

  echo "Package 'frontier_exploration_ros2' not found. Installing from source..."

  local src_dir="${PROJECT_ROOT}/src/frontier_exploration_ros2"
  mkdir -p "${PROJECT_ROOT}/src"

  if [[ -d "${src_dir}/.git" ]]; then
    echo "Source directory exists; pulling latest..."
    git -C "${src_dir}" pull --ff-only
  else
    git clone https://github.com/nis057489/frontier_exploration_ros2.git "${src_dir}"
  fi

  echo "Building frontier_exploration_ros2 (this may take a minute)..."
  (
    cd "${PROJECT_ROOT}"
    colcon build --packages-select frontier_exploration_ros2 --symlink-install
  )

  # Re-source workspace so the newly built package is visible.
  # shellcheck source=/dev/null
  source "${PROJECT_ROOT}/install/setup.bash"

  if ! ros2 pkg prefix frontier_exploration_ros2 >/dev/null 2>&1; then
    echo "Build succeeded but 'frontier_exploration_ros2' is still not found — check build output above." >&2
    exit 1
  fi

  echo "frontier_exploration_ros2 installed successfully."
}

check_pkg slam_toolbox
check_pkg nav2_bringup
ensure_frontier_exploration_ros2

if [[ "${LOCAL_BRINGUP}" == true ]]; then
  check_pkg turtlebot3_bringup
fi

# ── Nav2 process cleanup (same as launch.sh) ─────────────────────────────────

declare -a NAV2_PATTERNS=(
  "nav2_lifecycle_manager/lifecycle_manager"
  "nav2_controller/controller_server"
  "nav2_planner/planner_server"
  "nav2_bt_navigator/bt_navigator"
  "nav2_waypoint_follower/waypoint_follower"
  "nav2_behaviors/behavior_server"
  "nav2_map_server/map_server"
  "nav2_amcl/amcl"
  "nav2_velocity_smoother/velocity_smoother"
  "nav2_collision_monitor/collision_monitor"
  "nav2_smoother/smoother_server"
)

cleanup_existing_nav2() {
  local pattern
  local found=false
  for pattern in "${NAV2_PATTERNS[@]}"; do
    if pgrep -f "${pattern}" >/dev/null 2>&1; then
      found=true
      break
    fi
  done
  [[ "${found}" == false ]] && return

  echo "Existing Nav2 processes detected. Cleaning up..."
  for pattern in "${NAV2_PATTERNS[@]}"; do
    pkill -TERM -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
  for pattern in "${NAV2_PATTERNS[@]}"; do
    pkill -KILL -f "${pattern}" 2>/dev/null || true
  done
}

cleanup() {
  trap - EXIT INT TERM
  [[ -n "${BRINGUP_PID:-}"  ]] && kill "${BRINGUP_PID}"  2>/dev/null || true
  [[ -n "${NAV_PID:-}"      ]] && kill "${NAV_PID}"      2>/dev/null || true
  [[ -n "${TRACKER_PID:-}"  ]] && kill "${TRACKER_PID}"  2>/dev/null || true
  [[ -n "${EXPLORE_PID:-}"  ]] && kill "${EXPLORE_PID}"  2>/dev/null || true
}
trap cleanup EXIT INT TERM

cleanup_existing_nav2

# ── Config file paths ─────────────────────────────────────────────────────────

# Reuse the existing sim nav params — they are already tuned to TB3 velocity limits.
# If you have a robot-specific nav params file, point NAVIGATION_PARAMS at it instead.
NAVIGATION_PARAMS="${PROJECT_ROOT}/simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation/config/navigation_hw.yaml"
SLAM_PARAMS="${PROJECT_ROOT}/simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation/config/slam_toolbox_mapping_hw.yaml"

TRACKER_PARAMS="${PROJECT_ROOT}/install/rviz_autonomous_exploration_benchmark/share/rviz_autonomous_exploration_benchmark/config/frontier_path_tracker.yaml"
if [[ ! -f "${TRACKER_PARAMS}" ]]; then
  TRACKER_PARAMS="${PROJECT_ROOT}/rviz/src/frontier_path_tracker.yaml"
fi

# ── Multi-robot path ─────────────────────────────────────────────────────────

if (( NUM_ROBOTS > 1 )); then
  echo "Multi-robot mode: ${NUM_ROBOTS} robots, map_transport=${MAP_TRANSPORT}, bandwidth_kbps=${BANDWIDTH_KBPS}, loss_pct=${LOSS_PCT}, delay_ms=${DELAY_MS}."
  echo "Assumes each robot is already running turtlebot3_bringup + SLAM + Nav2 independently."

  ros2 launch bme_ros2_navigation multi_robot_vxch_experiment.launch.py \
    num_robots:="${NUM_ROBOTS}" \
    use_sim_time:=false \
    rviz:=false \
    map_transport:="${MAP_TRANSPORT}" \
    bandwidth_kbps:="${BANDWIDTH_KBPS}" \
    loss_pct:="${LOSS_PCT}" \
    delay_ms:="${DELAY_MS}" \
    haar_levels:="${HAAR_LEVELS}" \
    rng_seed:="${RANDOM_SEED}" \
    robot_startup_delay_s:="${ROBOT_STARTUP_DELAY_S}"

  exit $?
fi

# ── 1) Optional: local TurtleBot3 bringup ────────────────────────────────────

BRINGUP_PID=""
if [[ "${LOCAL_BRINGUP}" == true ]]; then
  echo "Starting turtlebot3_bringup (model=${TB3_MODEL})..."
  ros2 launch turtlebot3_bringup robot.launch.py &
  BRINGUP_PID=$!
  echo "turtlebot3_bringup started (pid=${BRINGUP_PID}). Waiting 5 seconds for hardware to come up..."
  sleep 5
fi

# ── 2) SLAM Toolbox + Nav2 ───────────────────────────────────────────────────

SLAM_LAUNCH="$(ros2 pkg prefix slam_toolbox)/share/slam_toolbox/launch/online_async_launch.py"
NAV2_LAUNCH="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch/navigation_launch.py"

echo "Starting SLAM Toolbox (use_sim_time=false)..."
ros2 launch "${SLAM_LAUNCH}" \
  use_sim_time:=false \
  slam_params_file:="${SLAM_PARAMS}" &
SLAM_PID=$!
echo "SLAM started (pid=${SLAM_PID}). Waiting 8 seconds for first map publication..."
sleep 8

echo "Starting Nav2 (use_sim_time=false)..."
ros2 launch "${NAV2_LAUNCH}" \
  use_sim_time:=false \
  params_file:="${NAVIGATION_PARAMS}" &
NAV_PID=$!
echo "Nav2 started (pid=${NAV_PID})."

# ── 3) Path tracker ──────────────────────────────────────────────────────────

ros2 run rviz_autonomous_exploration_benchmark frontier_path_tracker.py \
  --ros-args \
  --params-file "${TRACKER_PARAMS}" \
  -p use_sim_time:=false &
TRACKER_PID=$!
echo "frontier_path_tracker.py started (pid=${TRACKER_PID})."

# ── 4) Frontier exploration ───────────────────────────────────────────────────

EXPLORE_CONFIG="${PROJECT_ROOT}/config/frontier_exploration_ros2/config.yaml"
echo "Starting frontier_exploration_ros2 (params=${EXPLORE_CONFIG})..."
ros2 launch frontier_exploration_ros2 explore.launch.py \
  use_sim_time:=false \
  params_file:="${EXPLORE_CONFIG}" &
EXPLORE_PID=$!
echo "frontier_exploration_ros2 started (pid=${EXPLORE_PID})."

# ── Done ─────────────────────────────────────────────────────────────────────

echo ""
echo "Full stack is running. Press Ctrl+C to stop all."
wait
