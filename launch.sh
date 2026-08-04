#!/usr/bin/env bash
set -eo pipefail

# Resolve repository root from script location so the script works from any cwd.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"

# ROS setup scripts may read unset vars, so keep nounset disabled while sourcing.
source /opt/ros/jazzy/setup.bash
if [[ -f "${PROJECT_ROOT}/install/setup.bash" ]]; then
  source "${PROJECT_ROOT}/install/setup.bash"
fi
set -u

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [world_name]" >&2
  echo "  Env vars: ROBOT=<model>  NUM_ROBOTS=<n>" >&2
  exit 1
fi

# Default benchmark world. Users can override by passing a single world name argument.
WORLD="${1:-bookstore}"
NUM_ROBOTS="${NUM_ROBOTS:-1}"
ROBOT="${ROBOT:-mogi_bot}"

# Experiment parameters — forwarded by docker.sh from experiment.conf.
MAP_TRANSPORT="${MAP_TRANSPORT:-baseline}"
BANDWIDTH_KBPS="${BANDWIDTH_KBPS:-0}"
LOSS_PCT="${LOSS_PCT:-0.0}"
DELAY_MS="${DELAY_MS:-0}"
HAAR_LEVELS="${HAAR_LEVELS:-4}"
RANDOM_SEED="${RANDOM_SEED:--1}"
ROBOT_STARTUP_DELAY_S="${ROBOT_STARTUP_DELAY_S:-0.0}"

if ! [[ "${NUM_ROBOTS}" =~ ^[0-9]+$ ]] || (( NUM_ROBOTS < 1 )); then
  echo "NUM_ROBOTS must be a positive integer (got '${NUM_ROBOTS}')." >&2
  exit 1
fi

collect_available_worlds() {
  # A world is considered runnable when folder and world file name match:
  # simulation/worlds/<name>/<name>.world or <name>.sdf
  local -a worlds=()
  local name dir

  while IFS= read -r -d '' dir; do
    name="$(basename "${dir}")"
    if [[ -f "${dir}/${name}.world" || -f "${dir}/${name}.sdf" ]]; then
      worlds+=("${name}")
    fi
  done < <(find "${PROJECT_ROOT}/simulation/worlds" -mindepth 1 -maxdepth 1 -type d -print0 2>/dev/null || true)

  printf '%s\n' "${worlds[@]}" | sort -u
}

# Precompute world list once so validation and error messages stay consistent.
mapfile -t AVAILABLE_WORLDS < <(collect_available_worlds)

WORLD_VALID=false
for name in "${AVAILABLE_WORLDS[@]}"; do
  if [[ "${name}" == "${WORLD}" ]]; then
    WORLD_VALID=true
    break
  fi
done

if [[ "${WORLD_VALID}" == false ]]; then
  # Fail early with explicit choices to avoid launching into an unintended world.
  echo "Unknown world: ${WORLD}" >&2
  if [[ ${#AVAILABLE_WORLDS[@]} -gt 0 ]]; then
    echo "Available worlds: ${AVAILABLE_WORLDS[*]}" >&2
  fi
  exit 1
fi

# Baseline spawn tuned for the benchmark maps.
SPAWN_X="2.5"
SPAWN_Y="1.5"
SPAWN_YAW="-1.5707"

# Robot-specific spawn height above the floor.
case "${ROBOT}" in
  turtlebot3_waffle) SPAWN_Z="0.05" ;;
  mogi_bot)          SPAWN_Z="0.10" ;;
  *)                 SPAWN_Z="0.05" ;;
esac

# Map-specific spawn overrides for x/y/yaw; z is set per robot above.
if [[ "${WORLD}" == "bookstore" ]]; then
  SPAWN_X="1.23"
  SPAWN_Y="6.35"
  SPAWN_YAW="-3.10"
fi

if [[ "${WORLD}" == "corridor" ]]; then
  SPAWN_X="-8.11"
  SPAWN_Y="0.31"
  SPAWN_YAW="0.00"
fi

if [[ "${WORLD}" == "warehouse" ]]; then
  SPAWN_X="-12.97"
  SPAWN_Y="8.23"
  SPAWN_YAW="1.58"
fi

SPAWN_PRESET="${SPAWN_PRESET:-default}"
SPAWN_PRESETS_FILE="${PROJECT_ROOT}/spawn_presets.yaml"

# Load per-robot spawn positions from config file.
# Returns a JSON array like [{"x":1.0,"y":2.0,"yaw":0.0}, ...].
# Empty array ("[]") means "use the default grid/line offset logic".
SPAWN_POSITIONS_JSON="[]"
if [[ -f "${SPAWN_PRESETS_FILE}" ]]; then
  SPAWN_POSITIONS_JSON=$(WORLD="${WORLD}" SPAWN_PRESET="${SPAWN_PRESET}" \
    SPAWN_PRESETS_FILE="${SPAWN_PRESETS_FILE}" python3 - <<'PYEOF'
import yaml, json, os, sys
world  = os.environ["WORLD"]
preset = os.environ["SPAWN_PRESET"]
path   = os.environ["SPAWN_PRESETS_FILE"]
with open(path) as f:
    data = yaml.safe_load(f)
positions = data.get(world, {}).get(preset)
if positions is None:
    if preset != "default":
        print(f"[spawn_presets] WARNING: preset '{preset}' not found for world '{world}'."
              " Falling back to default spawn.", file=sys.stderr)
    positions = []
print(json.dumps(positions))
PYEOF
  )
fi

# For single robot: override base coords from the first preset position (if any).
# The "default" preset either has no entry or its entry matches the hardcoded values,
# so this is a no-op for normal runs.
if [[ "${NUM_ROBOTS}" == "1" ]] && [[ "${SPAWN_POSITIONS_JSON}" != "[]" ]]; then
  read -r SPAWN_X SPAWN_Y SPAWN_YAW < <(SPAWN_POSITIONS_JSON="${SPAWN_POSITIONS_JSON}" python3 - <<'PYEOF'
import json, os
p = json.loads(os.environ["SPAWN_POSITIONS_JSON"])
if p:
    print(p[0]["x"], p[0]["y"], p[0].get("yaw", 0))
PYEOF
  )
fi

# Verify critical overlay packages before launching anything.
if ! ros2 pkg prefix bme_ros2_navigation >/dev/null 2>&1; then
  echo "Workspace overlay is not available. Expected package 'bme_ros2_navigation' was not found." >&2
  echo "Rebuild the image/container so /opt/benchmark_ws/install/setup.bash contains the workspace packages." >&2
  exit 1
fi

if ! ros2 pkg prefix rviz_autonomous_exploration_benchmark >/dev/null 2>&1; then
  echo "Workspace overlay is not available. Expected package 'rviz_autonomous_exploration_benchmark' was not found." >&2
  echo "Rebuild the image/container so /opt/benchmark_ws/install/setup.bash contains the workspace packages." >&2
  exit 1
fi

URDF_PATH="$(ros2 pkg prefix bme_ros2_navigation)/share/bme_ros2_navigation/urdf/${ROBOT}.urdf"
if [[ ! -f "${URDF_PATH}" ]]; then
  echo "Unknown robot model: '${ROBOT}'. No URDF found at: ${URDF_PATH}" >&2
  exit 1
fi

# Nav2 can leave lifecycle-managed processes around between runs. We detect and
# terminate known nodes up front to avoid namespace conflicts and flaky bringup.
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
  # Fast pre-check avoids unnecessary pkill loops when nothing is running.
  local pattern
  local found=false

  for pattern in "${NAV2_PATTERNS[@]}"; do
    if pgrep -f "${pattern}" >/dev/null 2>&1; then
      found=true
      break
    fi
  done

  if [[ "${found}" == false ]]; then
    return
  fi

  # Two-phase shutdown: try TERM first, then escalate to KILL.
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
  # Prevent recursive trap calls while we are already in cleanup.
  trap - EXIT INT TERM
  # Best-effort shutdown of launched child processes.
  [[ -n "${SPAWN_PID:-}" ]] && kill "${SPAWN_PID}" 2>/dev/null || true
  [[ -n "${NAV_PID:-}" ]] && kill "${NAV_PID}" 2>/dev/null || true
  [[ -n "${TRACKER_PID:-}" ]] && kill "${TRACKER_PID}" 2>/dev/null || true
  [[ -n "${STACK_PID:-}" ]] && kill "${STACK_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Ensure a clean process baseline before starting the new run.
cleanup_existing_nav2

if (( NUM_ROBOTS > 1 )); then
  # LAUNCH_DEBUG=true prints a full traceback for launch-time exceptions
  # (e.g. "Caught exception in launch (see debug for traceback)") instead of
  # just the one-line summary -- set it when you need to find where a
  # launch-time error actually originates.
  LAUNCH_DEBUG_FLAG=()
  if [[ "${LAUNCH_DEBUG:-false}" == true ]]; then
    LAUNCH_DEBUG_FLAG=(--debug)
    [[ -d "${PROJECT_ROOT}/debug" ]] && export PYTHONPATH="${PROJECT_ROOT}/debug:${PYTHONPATH:-}"
  fi

  ros2 launch "${LAUNCH_DEBUG_FLAG[@]}" bme_ros2_navigation multi_robot_vxch_experiment.launch.py \
    world:="${WORLD}" \
    num_robots:="${NUM_ROBOTS}" \
    model:="${ROBOT}.urdf" \
    x:="${SPAWN_X}" \
    y:="${SPAWN_Y}" \
    z:="${SPAWN_Z}" \
    yaw:="${SPAWN_YAW}" \
    map_transport:="${MAP_TRANSPORT}" \
    bandwidth_kbps:="${BANDWIDTH_KBPS}" \
    loss_pct:="${LOSS_PCT}" \
    delay_ms:="${DELAY_MS}" \
    haar_levels:="${HAAR_LEVELS}" \
    rng_seed:="${RANDOM_SEED}" \
    robot_startup_delay_s:="${ROBOT_STARTUP_DELAY_S}" \
    spawn_positions_json:="${SPAWN_POSITIONS_JSON}" &
  STACK_PID=$!

  echo "multi_robot_vxch_experiment.launch.py started (pid=${STACK_PID}, world=${WORLD}, robot=${ROBOT}, num_robots=${NUM_ROBOTS}, map_transport=${MAP_TRANSPORT}, bandwidth_kbps=${BANDWIDTH_KBPS}, loss_pct=${LOSS_PCT}, delay_ms=${DELAY_MS}, rng_seed=${RANDOM_SEED})."
  echo "All processes are running. Press Ctrl+C to stop all."
  wait
  exit $?
fi

# 1) Start world + robot spawning.
ros2 launch bme_ros2_navigation spawn_robot.launch.py \
  world:="${WORLD}" \
  model:="${ROBOT}.urdf" \
  x:="${SPAWN_X}" \
  y:="${SPAWN_Y}" \
  z:="${SPAWN_Z}" \
  yaw:="${SPAWN_YAW}" &
SPAWN_PID=$!

# Give simulation/spawn a short head start before Nav2 + tracker.
echo "spawn_robot.launch.py started (pid=${SPAWN_PID}, world=${WORLD}, robot=${ROBOT}, x=${SPAWN_X}, y=${SPAWN_Y}, z=${SPAWN_Z}, yaw=${SPAWN_YAW}). Waiting 5 seconds..."
sleep 5

# 2) Start Nav2 + SLAM stack.
ros2 launch bme_ros2_navigation navigation_with_slam.launch.py &
NAV_PID=$!
echo "navigation_with_slam.launch.py started (pid=${NAV_PID})."

# Prefer installed tracker params; fall back to source file for pre-install/dev runs.
TRACKER_PARAMS_FILE="${PROJECT_ROOT}/install/rviz_autonomous_exploration_benchmark/share/rviz_autonomous_exploration_benchmark/config/frontier_path_tracker.yaml"
if [[ ! -f "${TRACKER_PARAMS_FILE}" ]]; then
  TRACKER_PARAMS_FILE="${PROJECT_ROOT}/rviz/src/frontier_path_tracker.yaml"
fi

# 3) Start path tracker for traveled-path telemetry and reset integration.
ros2 run rviz_autonomous_exploration_benchmark frontier_path_tracker.py \
  --ros-args \
  --params-file "${TRACKER_PARAMS_FILE}" \
  -p use_sim_time:=true &
TRACKER_PID=$!
echo "frontier_path_tracker.py started (pid=${TRACKER_PID})."

# Block while children run; Ctrl+C triggers trap-driven cleanup.
echo "All processes are running. Press Ctrl+C to stop all."
wait
