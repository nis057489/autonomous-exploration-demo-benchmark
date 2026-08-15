#!/usr/bin/env bash
set -euo pipefail

# Replays the latest baseline run and the latest vxch run side by side in two
# rviz2 windows, each fed by a single `ros2 bag play` that plays all robots'
# bags for that condition together (bags are per-robot; topic names like
# /robot1/explore/traversed_path are namespaced so playing them concurrently
# in one process is safe).
#
# The two conditions run in separate Docker containers on different
# ROS_DOMAIN_IDs so their (identically-named) topics don't collide with each
# other. Host experiment_runs/ is mounted read-only -- if a run is missing
# its bag metadata.yaml (recorder was killed before it could finalize), it's
# reindexed into a scratch copy inside the container, never touching the
# original recording.
#
# Usage: ./replay_compare.sh [rate]
#   rate   Playback speed multiplier (default 8).
#
# By default the latest baseline/vxch run is picked per robot. To compare
# specific (e.g. older) runs instead, set BASELINE_RUN_OVERRIDE and/or
# VXCH_RUN_OVERRIDE to a comma-separated robot:run_dir_name list, e.g.:
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

declare -A BASELINE_RUN_OVERRIDES=()
declare -A VXCH_RUN_OVERRIDES=()

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

parse_overrides BASELINE_RUN_OVERRIDES "${BASELINE_RUN_OVERRIDE:-}"
parse_overrides VXCH_RUN_OVERRIDES "${VXCH_RUN_OVERRIDE:-}"

if [[ ! -d "${RUNS_DIR}" ]]; then
  echo "No experiment_runs/ directory found at ${RUNS_DIR}." >&2
  exit 1
fi
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "rviz config not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi

# ── Find the latest run per robot per condition ─────────────────────────
declare -a BASELINE_BAGS=()
declare -a VXCH_BAGS=()

for robot_dir in "${RUNS_DIR}"/*/; do
  [[ -d "${robot_dir}" ]] || continue
  robot="$(basename "${robot_dir}")"

  if [[ -n "${BASELINE_RUN_OVERRIDES[${robot}]:-}" ]]; then
    latest_baseline="${BASELINE_RUN_OVERRIDES[${robot}]}"
  else
    latest_baseline="$(find "${robot_dir}" -maxdepth 1 -type d -name "*_baseline_${robot}" -printf '%f\n' 2>/dev/null | sort | tail -1)"
  fi
  if [[ -n "${VXCH_RUN_OVERRIDES[${robot}]:-}" ]]; then
    latest_vxch="${VXCH_RUN_OVERRIDES[${robot}]}"
  else
    latest_vxch="$(find "${robot_dir}" -maxdepth 1 -type d -name "*_vxch_${robot}" -printf '%f\n' 2>/dev/null | sort | tail -1)"
  fi

  if [[ -n "${latest_baseline}" && -d "${robot_dir}${latest_baseline}/bag" ]]; then
    BASELINE_BAGS+=("${robot}:${robot_dir}${latest_baseline}/bag")
    echo "baseline / ${robot}: ${latest_baseline}"
  else
    echo "baseline / ${robot}: no run found -- skipping" >&2
  fi

  if [[ -n "${latest_vxch}" && -d "${robot_dir}${latest_vxch}/bag" ]]; then
    VXCH_BAGS+=("${robot}:${robot_dir}${latest_vxch}/bag")
    echo "vxch     / ${robot}: ${latest_vxch}"
  else
    echo "vxch     / ${robot}: no run found -- skipping" >&2
  fi
done

if [[ ${#BASELINE_BAGS[@]} -eq 0 || ${#VXCH_BAGS[@]} -eq 0 ]]; then
  echo "Need at least one baseline run and one vxch run to compare." >&2
  exit 1
fi

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

# ── Window geometry: left half for baseline, right half for vxch ────────
SCREEN_W=1920
SCREEN_H=1200
if command -v xdpyinfo >/dev/null 2>&1; then
  dims="$(xdpyinfo | awk '/dimensions:/{print $2}')"
  SCREEN_W="${dims%%x*}"
  SCREEN_H="${dims##*x}"
fi
HALF_W=$(( SCREEN_W / 2 - 10 ))
GEOM_LEFT="${HALF_W}x$((SCREEN_H - 60))+0+0"
GEOM_RIGHT="${HALF_W}x$((SCREEN_H - 60))+$((SCREEN_W / 2))+0"

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
ros2 bag play \"\${inputs[@]}\" -r ${RATE} &
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
  docker stop -t 2 "replay_baseline_$$" "replay_vxch_$$" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

echo
echo "Launching baseline (domain 61, left half) and vxch (domain 62, right half) at ${RATE}x..."
run_side "baseline" 61 "${GEOM_LEFT}" BASELINE_BAGS &
BASELINE_PID=$!
run_side "vxch" 62 "${GEOM_RIGHT}" VXCH_BAGS &
VXCH_PID=$!

wait "${BASELINE_PID}" "${VXCH_PID}"