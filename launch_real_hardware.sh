#!/usr/bin/env bash
# Launch the autonomous exploration stack on a real TurtleBot3 (ROS 2 Jazzy).
#
# Usage:
#   ./launch_real_hardware.sh [--local-bringup] [--model burger|waffle|waffle_pi]
#
# Flags:
#   --local-bringup   Also launch turtlebot3_bringup on this machine (useful when the
#                     workstation IS the robot, or when testing via USB-to-OpenCR).
#                     Omit this flag when the robot's Raspberry Pi is running its own
#                     bringup and you are only launching the navigation/exploration side.
#   --model <name>    TurtleBot3 model (burger | waffle | waffle_pi). Defaults to the
#                     TURTLEBOT3_MODEL env var, then falls back to burger.
#
# What this script does NOT do (run these yourself, on the robot or on another terminal):
#   - turtlebot3_bringup robot.launch.py  (unless --local-bringup is given)
#   - RViz (open it manually, or pass rviz:=true to navigation_with_slam.launch.py)
#   - The exploration node itself — start it separately, e.g.:
#       ros2 launch frontier_exploration_ros2 explore.launch.py \
#         use_sim_time:=false \
#         params_file:=config/frontier_exploration_ros2/config.yaml

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"

# ── Parse arguments ──────────────────────────────────────────────────────────

LOCAL_BRINGUP=false
TB3_MODEL="${TURTLEBOT3_MODEL:-}"

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
    *)
      echo "Unknown argument: $1" >&2
      echo "Usage: $0 [--local-bringup] [--model burger|waffle|waffle_pi]" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${TB3_MODEL}" ]]; then
  TB3_MODEL="burger"
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

check_pkg slam_toolbox
check_pkg nav2_bringup

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
  [[ -n "${BRINGUP_PID:-}" ]] && kill "${BRINGUP_PID}" 2>/dev/null || true
  [[ -n "${NAV_PID:-}"     ]] && kill "${NAV_PID}"     2>/dev/null || true
  [[ -n "${TRACKER_PID:-}" ]] && kill "${TRACKER_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cleanup_existing_nav2

# ── Config file paths ─────────────────────────────────────────────────────────

# Reuse the existing sim nav params — they are already tuned to TB3 velocity limits.
# If you have a robot-specific nav params file, point NAVIGATION_PARAMS at it instead.
NAVIGATION_PARAMS="${PROJECT_ROOT}/simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation/config/navigation.yaml"
SLAM_PARAMS="${PROJECT_ROOT}/simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation/config/slam_toolbox_mapping.yaml"

TRACKER_PARAMS="${PROJECT_ROOT}/install/rviz_autonomous_exploration_benchmark/share/rviz_autonomous_exploration_benchmark/config/frontier_path_tracker.yaml"
if [[ ! -f "${TRACKER_PARAMS}" ]]; then
  TRACKER_PARAMS="${PROJECT_ROOT}/rviz/src/frontier_path_tracker.yaml"
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
echo "SLAM started (pid=${SLAM_PID}). Waiting 3 seconds..."
sleep 3

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

# ── Done ─────────────────────────────────────────────────────────────────────

echo ""
echo "Stack is running. Start your exploration node separately, e.g.:"
echo "  ros2 launch frontier_exploration_ros2 explore.launch.py \\"
echo "    use_sim_time:=false \\"
echo "    params_file:=${PROJECT_ROOT}/config/frontier_exploration_ros2/config.yaml"
echo ""
echo "Press Ctrl+C to stop all."
wait
