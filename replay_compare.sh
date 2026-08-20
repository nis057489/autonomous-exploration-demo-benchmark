#!/usr/bin/env bash
set -euo pipefail

# Replays the latest run of each selected condition side by side in separate
# rviz2 windows, each fed by a single `ros2 bag play` that plays all robots'
# bags for that condition together (bags are per-robot; topic names like
# /robot1/explore/traversed_path are namespaced so playing them concurrently
# in one process is safe).
#
# Each condition runs in its own Docker container on its own ROS_DOMAIN_ID so
# their (identically-named) topics don't collide with each other. Host
# experiment_runs/ is mounted read-only -- if a run is missing its bag
# metadata.yaml (recorder was killed before it could finalize), it's
# reindexed into a scratch copy inside the container, never touching the
# original recording.
#
# Usage: ./replay_compare.sh [rate]
#   rate   Playback speed multiplier (default 8).
#
# Each side's bag playback loops automatically (ros2 bag play --loop) so it
# restarts from the beginning once it reaches the end, until the rviz window
# is closed.
#
# By default compares baseline vs. vxch. Set REPLAY_CONDITIONS to a
# comma-separated subset of baseline,vxch,zstd (2 or 3 of them) to change
# which conditions are shown, e.g. REPLAY_CONDITIONS="baseline,zstd" or
# REPLAY_CONDITIONS="baseline,vxch,zstd" for all three side by side.
#
# By default the latest run of each condition is picked per robot. To compare
# specific (e.g. older) runs instead, set BASELINE_RUN_OVERRIDE,
# VXCH_RUN_OVERRIDE, and/or ZSTD_RUN_OVERRIDE to a comma-separated
# robot:run_dir_name list, e.g.:
#   BASELINE_RUN_OVERRIDE="robot1:20260710_011013_baseline_robot1,robot2:20260710_011500_baseline_robot2" \
#   ./replay_compare.sh
# Robots omitted from an override fall back to their latest run for that
# condition. This is how replay_gui.py drives specific-run comparisons.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
DOCKER_IMAGE="${BENCHMARK_DOCKER_IMAGE:-autonomous-exploration-benchmark:jazzy-harmonic}"
RUNS_DIR="${PROJECT_ROOT}/experiment_runs"
RVIZ_CONFIG="${PROJECT_ROOT}/real.rviz"

RATE="${1:-8}"

IFS=',' read -ra CONDITIONS <<< "${REPLAY_CONDITIONS:-baseline,vxch}"
for c in "${CONDITIONS[@]}"; do
  if [[ "${c}" != "baseline" && "${c}" != "vxch" && "${c}" != "zstd" ]]; then
    echo "REPLAY_CONDITIONS entries must be baseline, vxch, or zstd -- got '${c}'." >&2
    exit 1
  fi
done
if [[ ${#CONDITIONS[@]} -lt 2 ]]; then
  echo "REPLAY_CONDITIONS needs at least 2 conditions to compare." >&2
  exit 1
fi

declare -A RUN_OVERRIDES_baseline=()
declare -A RUN_OVERRIDES_vxch=()
declare -A RUN_OVERRIDES_zstd=()

parse_overrides() {
  local -n _out=$1
  local spec="$2"
  local pair robot run_dir
  [[ -z "${spec}" ]] && return 0
  IFS=',' read -ra pairs <<< "${spec}"
  for pair in "${pairs[@]}"; do
    robot="${pair%%:*}"
    run_dir="${pair#*:}"
    [[ -z "${robot}" || -z "${run_dir}" || "${robot}" == "${pair}" ]] && continue
    _out["${robot}"]="${run_dir}"
  done
}

parse_overrides RUN_OVERRIDES_baseline "${BASELINE_RUN_OVERRIDE:-}"
parse_overrides RUN_OVERRIDES_vxch "${VXCH_RUN_OVERRIDE:-}"
parse_overrides RUN_OVERRIDES_zstd "${ZSTD_RUN_OVERRIDE:-}"

if [[ ! -d "${RUNS_DIR}" ]]; then
  echo "No experiment_runs/ directory found at ${RUNS_DIR}." >&2
  exit 1
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "rviz config not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi

# ── Find the latest run per robot per condition ─────────────────────────
declare -a BAG_LIST_baseline=()
declare -a BAG_LIST_vxch=()
declare -a BAG_LIST_zstd=()

# Simulator runs (launch.sh) are written flat as
# experiment_runs/<timestamp>_<condition>_<world>/bag -- one dir per whole
# multi-robot run, not per robot. Real-hardware runs (launch_real_hardware.sh
# / recover_metrics.sh) nest one dir per robot:
# experiment_runs/<robot>/<timestamp>_<condition>_<robot>. run_bag_dir below
# resolves a robot:run_dir pair against whichever layout it's actually in,
# mirroring replay_gui.py's run_bag_dir().
RUN_DIR_RE='^[0-9]{8}_[0-9]{6}_(baseline|vxch|zstd)_([[:alnum:]_]+)$'

run_bag_dir() {
  local robot=$1 run_dir=$2
  if [[ -d "${RUNS_DIR}/${run_dir}" && "${run_dir}" =~ ${RUN_DIR_RE} ]]; then
    echo "${RUNS_DIR}/${run_dir}/bag"
  else
    echo "${RUNS_DIR}/${robot}/${run_dir}/bag"
  fi
}

condition_selected() {
  local want=$1 c
  for c in "${CONDITIONS[@]}"; do
    [[ "${c}" == "${want}" ]] && return 0
  done
  return 1
}

for robot_dir in "${RUNS_DIR}"/*/; do
  [[ -d "${robot_dir}" ]] || continue
  robot="$(basename "${robot_dir}")"

  # Flat simulator run dirs land here too (RUNS_DIR/*/ globs them the same
  # as robot dirs) -- skip them as "robots" since they're runs, not robots.
  if [[ "${robot}" =~ ${RUN_DIR_RE} ]]; then
    continue
  fi

  for condition in "${CONDITIONS[@]}"; do
    case "${condition}" in
      baseline) declare -n overrides_ref=RUN_OVERRIDES_baseline; declare -n list_ref=BAG_LIST_baseline ;;
      vxch)     declare -n overrides_ref=RUN_OVERRIDES_vxch;     declare -n list_ref=BAG_LIST_vxch ;;
      zstd)     declare -n overrides_ref=RUN_OVERRIDES_zstd;     declare -n list_ref=BAG_LIST_zstd ;;
    esac

    if [[ -n "${overrides_ref[${robot}]:-}" ]]; then
      latest="${overrides_ref[${robot}]}"
    else
      latest="$(find "${robot_dir}" -maxdepth 1 -type d -name "*_${condition}_${robot}" -printf '%f\n' 2>/dev/null | sort | tail -1)"
    fi

    if [[ -n "${latest}" && -d "$(run_bag_dir "${robot}" "${latest}")" ]]; then
      list_ref+=("${robot}:$(run_bag_dir "${robot}" "${latest}")")
      echo "${condition} / ${robot}: ${latest}"
    else
      echo "${condition} / ${robot}: no run found -- skipping" >&2
    fi
  done
done

# Simulator runs: pick the latest flat run per (condition, world) unless
# overridden -- names sort lexically the same as chronologically since
# they're timestamp-prefixed, so track the max name seen per world first
# rather than appending every match (multiple runs for the same world would
# otherwise all mount to the same /bags/<world> path in run_side below).
declare -A LATEST_NAME_baseline=()
declare -A LATEST_NAME_vxch=()
declare -A LATEST_NAME_zstd=()
for entry in "${RUNS_DIR}"/*/; do
  [[ -d "${entry}" ]] || continue
  name="$(basename "${entry}")"
  [[ "${name}" =~ ${RUN_DIR_RE} ]] || continue
  [[ -d "${entry}bag" ]] || continue
  condition="${BASH_REMATCH[1]}"
  world="${BASH_REMATCH[2]}"

  condition_selected "${condition}" || continue

  case "${condition}" in
    baseline) declare -n latest_ref=LATEST_NAME_baseline ;;
    vxch)     declare -n latest_ref=LATEST_NAME_vxch ;;
    zstd)     declare -n latest_ref=LATEST_NAME_zstd ;;
  esac
  if [[ -z "${latest_ref[${world}]:-}" || "${name}" > "${latest_ref[${world}]}" ]]; then
    latest_ref["${world}"]="${name}"
  fi
done

for condition in "${CONDITIONS[@]}"; do
  case "${condition}" in
    baseline) declare -n overrides_ref=RUN_OVERRIDES_baseline; declare -n latest_ref=LATEST_NAME_baseline; declare -n list_ref=BAG_LIST_baseline ;;
    vxch)     declare -n overrides_ref=RUN_OVERRIDES_vxch;     declare -n latest_ref=LATEST_NAME_vxch;     declare -n list_ref=BAG_LIST_vxch ;;
    zstd)     declare -n overrides_ref=RUN_OVERRIDES_zstd;     declare -n latest_ref=LATEST_NAME_zstd;     declare -n list_ref=BAG_LIST_zstd ;;
  esac
  for world in "${!latest_ref[@]}"; do
    name="${latest_ref[${world}]}"
    if [[ -n "${overrides_ref[${world}]:-}" ]]; then
      name="${overrides_ref[${world}]}"
    fi
    [[ -d "${RUNS_DIR}/${name}/bag" ]] || continue
    list_ref+=("${world}:${RUNS_DIR}/${name}/bag")
    echo "${condition} / ${world}: ${name}"
  done
done

for condition in "${CONDITIONS[@]}"; do
  case "${condition}" in
    baseline) count=${#BAG_LIST_baseline[@]} ;;
    vxch)     count=${#BAG_LIST_vxch[@]} ;;
    zstd)     count=${#BAG_LIST_zstd[@]} ;;
  esac
  if [[ "${count}" -eq 0 ]]; then
    echo "Need at least one ${condition} run to compare (REPLAY_CONDITIONS=${REPLAY_CONDITIONS:-baseline,vxch})." >&2
    exit 1
  fi
done

# ── X11 / GUI setup (mirrors docker.sh) ──────────────────────────────────
if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is not set -- rviz needs a working X11 session." >&2
  exit 1
fi
command -v xhost >/dev/null 2>&1 && xhost +si:localuser:root >/dev/null 2>&1 || true

GUI_ARGS=(-e DISPLAY -e QT_X11_NO_MITSHM=1 -e QT_QPA_PLATFORM=xcb)
[[ -d /tmp/.X11-unix ]] && GUI_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
[[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY}" ]] && GUI_ARGS+=(-e XAUTHORITY -v "${XAUTHORITY}:${XAUTHORITY}:ro")
[[ -e /dev/dri ]] && GUI_ARGS+=(--device /dev/dri)

# ── Window geometry: screen split into one column per condition ─────────
SCREEN_W=1920
SCREEN_H=1200
if command -v xdpyinfo >/dev/null 2>&1; then
  dims="$(xdpyinfo | awk '/dimensions:/{print $2}')"
  SCREEN_W="${dims%%x*}"
  SCREEN_H="${dims##*x}"
fi
NUM_COLS=${#CONDITIONS[@]}
COL_W=$(( SCREEN_W / NUM_COLS - 10 ))
declare -a GEOMS=()
for ((i = 0; i < NUM_COLS; i++)); do
  GEOMS+=("${COL_W}x$((SCREEN_H - 60))+$((i * SCREEN_W / NUM_COLS))+0")
done

# ── Build the in-container command for one condition ─────────────────────
# Each robot's bag is mounted read-only at /bags/<robot>. If metadata.yaml
# is missing, reindex a copy under /tmp first rather than touching the
# mount.
build_inner_cmd() {
  local -n _bags=$1
  local cmd='set -e; declare -a inputs=(); '
  for entry in "${_bags[@]}"; do
    local robot="${entry%%:*}"
    cmd+="
if [[ -f /bags/${robot}/metadata.yaml ]]; then
  inputs+=(-i /bags/${robot} mcap)
else
  echo 'reindexing ${robot} (no metadata.yaml in recorded bag)...'
  mkdir -p /tmp/reindexed/${robot}
  cp /bags/${robot}/*.mcap /tmp/reindexed/${robot}/
  ros2 bag reindex -s mcap /tmp/reindexed/${robot}
  inputs+=(-i /tmp/reindexed/${robot} mcap)
fi
"
  done
  echo "${cmd}"
}

run_side() {
  local condition=$1 domain_id=$2 geometry=$3
  local -n bags_ref=$4
  local -a mount_args=()
  local entry robot host_bag
  for entry in "${bags_ref[@]}"; do
    robot="${entry%%:*}"
    host_bag="${entry#*:}"
    mount_args+=(-v "${host_bag}:/bags/${robot}:ro")
  done

  local inner
  inner="$(build_inner_cmd bags_ref)"
  inner+="
echo 'playing back ${#bags_ref[@]} bag(s) for ${condition} at ${RATE}x...'
ros2 bag play \"\${inputs[@]}\" -r ${RATE} --loop &
PLAY_PID=\$!
rviz2 -d /rviz/real.rviz -t '${condition}' -qwindowgeometry '${geometry}' &
RVIZ_PID=\$!
wait \$RVIZ_PID
kill \$PLAY_PID 2>/dev/null || true
"

  docker run --rm -i --name "replay_${condition}_$$" \
    -e "ROS_DOMAIN_ID=${domain_id}" \
    "${GUI_ARGS[@]}" \
    "${mount_args[@]}" \
    -v "${RVIZ_CONFIG}:/rviz/real.rviz:ro" \
    "${DOCKER_IMAGE}" bash -lc "${inner}"
}

cleanup() {
  local c
  for c in "${CONDITIONS[@]}"; do
    docker stop -t 2 "replay_${c}_$$" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

echo
echo "Launching ${CONDITIONS[*]} (domains 61+) at ${RATE}x..."
declare -a PIDS=()
for ((i = 0; i < NUM_COLS; i++)); do
  condition="${CONDITIONS[$i]}"
  case "${condition}" in
    baseline) bags_var=BAG_LIST_baseline ;;
    vxch)     bags_var=BAG_LIST_vxch ;;
    zstd)     bags_var=BAG_LIST_zstd ;;
  esac
  run_side "${condition}" $((61 + i)) "${GEOMS[$i]}" "${bags_var}" &
  PIDS+=("$!")
done

wait "${PIDS[@]}"
