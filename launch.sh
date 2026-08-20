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
# pure_pursuit (default) mirrors navigation_hw.yaml's real-hardware controller;
# mppi is sim's original controller, kept for A/B comparison against it.
CONTROLLER_TYPE="${CONTROLLER_TYPE:-pure_pursuit}"
BANDWIDTH_KBPS="${BANDWIDTH_KBPS:-0}"
LOSS_PCT="${LOSS_PCT:-0.0}"
DELAY_MS="${DELAY_MS:-0}"
HAAR_LEVELS="${HAAR_LEVELS:-4}"
COMPRESSION="${COMPRESSION:-zstd}"
VARINT_ENCODING="${VARINT_ENCODING:-true}"
TILE_SIZE_M="${TILE_SIZE_M:-2.0}"
SCHEDULE_MODE="${SCHEDULE_MODE:-smart}"
RANDOM_SEED="${RANDOM_SEED:--1}"
ROBOT_STARTUP_DELAY_S="${ROBOT_STARTUP_DELAY_S:-0.0}"
RECORD_METRICS="${RECORD_METRICS:-false}"
# sim (default): ddil_proxy_node's in-process token-bucket/loss/delay simulation.
# tc: real netns + veth + `tc netem` per robot (see setup_ddil_netns.sh) -- the
# same tool used to throttle the wireless interface in the hardware test, instead
# of software-simulated impairment. Requires the container to have been started
# with CAP_NET_ADMIN (docker.sh adds this automatically when IMPAIRMENT_MODE=tc).
IMPAIRMENT_MODE="${IMPAIRMENT_MODE:-sim}"
# tc mode only: seconds to let ROS2/DDS discovery settle across the netns
# boundary (unshaped) before tc netem is actually applied. Discovery involves
# multiple SPDP announce/response round-trips; applying tight bandwidth/loss/
# delay limits before it completes can prevent it from ever completing.
IMPAIRMENT_DELAY_S="${IMPAIRMENT_DELAY_S:-10}"

if [[ "${IMPAIRMENT_MODE}" != "sim" && "${IMPAIRMENT_MODE}" != "tc" ]]; then
  echo "IMPAIRMENT_MODE must be 'sim' or 'tc' (got '${IMPAIRMENT_MODE}')." >&2
  exit 1
fi

if ! [[ "${NUM_ROBOTS}" =~ ^[0-9]+$ ]] || (( NUM_ROBOTS < 1 )); then
  echo "NUM_ROBOTS must be a positive integer (got '${NUM_ROBOTS}')." >&2
  exit 1
fi

if [[ "${IMPAIRMENT_MODE}" == "tc" && "${NUM_ROBOTS}" -eq 1 ]]; then
  echo "IMPAIRMENT_MODE=tc has no effect with NUM_ROBOTS=1 (no peer link to shape); falling back to sim." >&2
  IMPAIRMENT_MODE="sim"
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

DDIL_NETNS_SCRIPT="${PROJECT_ROOT}/simulation/Week-7-8-ROS2-Navigation/bme_ros2_navigation/scripts/setup_ddil_netns.sh"

cleanup() {
  # Prevent recursive trap calls while we are already in cleanup.
  trap - EXIT INT TERM
  # Best-effort shutdown of launched child processes.
  [[ -n "${SPAWN_PID:-}" ]] && kill "${SPAWN_PID}" 2>/dev/null || true
  [[ -n "${NAV_PID:-}" ]] && kill "${NAV_PID}" 2>/dev/null || true
  [[ -n "${TRACKER_PID:-}" ]] && kill "${TRACKER_PID}" 2>/dev/null || true
  [[ -n "${STACK_PID:-}" ]] && kill "${STACK_PID}" 2>/dev/null || true
  [[ -n "${BAG_PID:-}" ]] && kill "${BAG_PID}" 2>/dev/null || true
  [[ -n "${BAG_PID:-}" ]] && wait "${BAG_PID}" 2>/dev/null || true
  [[ -n "${IMPAIRMENT_DELAY_PID:-}" ]] && kill "${IMPAIRMENT_DELAY_PID}" 2>/dev/null || true
  if [[ "${IMPAIRMENT_MODE}" == "tc" && "${NUM_ROBOTS}" -gt 1 ]]; then
    "${DDIL_NETNS_SCRIPT}" down "${NUM_ROBOTS}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# Ensure a clean process baseline before starting the new run.
cleanup_existing_nav2

# ── Bag recording (RECORD_METRICS=true) ─────────────────────────────────────
# Bags this run's traffic to experiment_runs/, mirroring launch_real_hardware.sh's
# RECORD_METRICS behavior so sim runs are inspectable the same way hw runs are.
if [[ "${RECORD_METRICS}" == true ]]; then
  RUN_DIR="${PROJECT_ROOT}/experiment_runs/$(date +%Y%m%d_%H%M%S)_${MAP_TRANSPORT}_${WORLD}"
  mkdir -p "${RUN_DIR}"
  export ROS_LOG_DIR="${RUN_DIR}/ros_logs"
  echo "Recording metrics to ${RUN_DIR} (RECORD_METRICS=true)"

  BAG_TOPICS=()
  if (( NUM_ROBOTS > 1 )); then
    for ((i = 1; i <= NUM_ROBOTS; i++)); do
      name="robot${i}"
      BAG_TOPICS+=(
        "/${name}/explore/traversed_path"
        "/${name}/explore/frontiers"
        "/${name}/map"
        "/${name}/team_map_ddil"
        "/${name}/nav_map"
      )
      if [[ "${MAP_TRANSPORT}" == "vxch" ]]; then
        for ((band = 0; band <= HAAR_LEVELS; band++)); do
          BAG_TOPICS+=("/${name}/vxch/map/band_${band}")
        done
        BAG_TOPICS+=("/${name}/vxch/map/manifest")
        # Peer-to-peer topology: each robot has its own independent downlink
        # from every other robot (no centralized base station), so the DDIL'd
        # bands live under /{name}/incoming/{peer}/... per peer.
        for ((j = 1; j <= NUM_ROBOTS; j++)); do
          (( j == i )) && continue
          peer="robot${j}"
          for ((band = 0; band <= HAAR_LEVELS; band++)); do
            BAG_TOPICS+=("/${name}/incoming/${peer}/band_${band}")
          done
          BAG_TOPICS+=("/${name}/incoming/${peer}/manifest")
        done
      fi
      if [[ "${MAP_TRANSPORT}" == "zstd" ]]; then
        BAG_TOPICS+=("/${name}/zstd/map")
        for ((j = 1; j <= NUM_ROBOTS; j++)); do
          (( j == i )) && continue
          peer="robot${j}"
          BAG_TOPICS+=("/${name}/incoming/${peer}/zstd_map")
        done
      fi
    done
  else
    BAG_TOPICS+=(
      "/explore/traversed_path"
      "/explore/frontiers"
      "/map"
    )
  fi

  ros2 bag record -o "${RUN_DIR}/bag" "${BAG_TOPICS[@]}" \
    >"${RUN_DIR}/bag_record.log" 2>&1 &
  BAG_PID=$!
fi

if (( NUM_ROBOTS > 1 )); then
  if [[ "${IMPAIRMENT_MODE}" == "tc" ]]; then
    echo "Setting up per-robot netns links (unshaped)..."
    "${DDIL_NETNS_SCRIPT}" up "${NUM_ROBOTS}"
  fi

  ros2 launch bme_ros2_navigation multi_robot_vxch_experiment.launch.py \
    world:="${WORLD}" \
    num_robots:="${NUM_ROBOTS}" \
    model:="${ROBOT}.urdf" \
    x:="${SPAWN_X}" \
    y:="${SPAWN_Y}" \
    z:="${SPAWN_Z}" \
    yaw:="${SPAWN_YAW}" \
    map_transport:="${MAP_TRANSPORT}" \
    controller_type:="${CONTROLLER_TYPE}" \
    impairment_mode:="${IMPAIRMENT_MODE}" \
    bandwidth_kbps:="${BANDWIDTH_KBPS}" \
    loss_pct:="${LOSS_PCT}" \
    delay_ms:="${DELAY_MS}" \
    haar_levels:="${HAAR_LEVELS}" \
    compression:="${COMPRESSION}" \
    varint_encoding:="${VARINT_ENCODING}" \
    tile_size_m:="${TILE_SIZE_M}" \
    schedule_mode:="${SCHEDULE_MODE}" \
    rng_seed:="${RANDOM_SEED}" \
    robot_startup_delay_s:="${ROBOT_STARTUP_DELAY_S}" \
    spawn_positions_json:="${SPAWN_POSITIONS_JSON}" &
  STACK_PID=$!

  if [[ "${IMPAIRMENT_MODE}" == "tc" ]]; then
    (
      sleep "${IMPAIRMENT_DELAY_S}"
      echo "Applying tc netem shaping after ${IMPAIRMENT_DELAY_S}s delay (bandwidth_kbps=${BANDWIDTH_KBPS}, loss_pct=${LOSS_PCT}, delay_ms=${DELAY_MS})..."
      "${DDIL_NETNS_SCRIPT}" update "${NUM_ROBOTS}" "${BANDWIDTH_KBPS}" "${LOSS_PCT}" "${DELAY_MS}"
    ) &
    IMPAIRMENT_DELAY_PID=$!
  fi

  echo "multi_robot_vxch_experiment.launch.py started (pid=${STACK_PID}, world=${WORLD}, robot=${ROBOT}, num_robots=${NUM_ROBOTS}, map_transport=${MAP_TRANSPORT}, impairment_mode=${IMPAIRMENT_MODE}, bandwidth_kbps=${BANDWIDTH_KBPS}, loss_pct=${LOSS_PCT}, delay_ms=${DELAY_MS}, impairment_delay_s=${IMPAIRMENT_DELAY_S}, rng_seed=${RANDOM_SEED})."
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
