#!/usr/bin/env bash
# Runs each MAP_TRANSPORT method 5x, 1500s per run:
#   ROBOT=turtlebot3_waffle ./docker.sh warehouse
#
# experiment.conf's MAP_TRANSPORT=... line takes precedence over an
# exported MAP_TRANSPORT env var (docker.sh sources the conf file
# unconditionally after reading args -- verified empirically), so this
# script edits experiment.conf in place for each run instead of just
# exporting the var. The original file is restored on exit.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONF_FILE="${SCRIPT_DIR}/experiment.conf"

WORLD="bookstore"
ROBOT="turtlebot3_waffle"
METHODS=(baseline vxch)
RUNS_PER_METHOD=1
RUN_DURATION_S=200
KILL_AFTER_S=30
LOG_DIR="${SCRIPT_DIR}/sweep_logs/$(date +%Y%m%d_%H%M%S)"

mkdir -p "${LOG_DIR}"

CONF_BACKUP="$(mktemp)"
cp "${CONF_FILE}" "${CONF_BACKUP}"
restore_conf() {
  cp "${CONF_BACKUP}" "${CONF_FILE}"
  rm -f "${CONF_BACKUP}"
}
trap restore_conf EXIT INT TERM

set_map_transport() {
  local method="$1"
  sed -i -E "s/^MAP_TRANSPORT=[^ \t]*/MAP_TRANSPORT=${method}/" "${CONF_FILE}"
}

total_runs=$(( ${#METHODS[@]} * RUNS_PER_METHOD ))
run_num=0

for method in "${METHODS[@]}"; do
  set_map_transport "${method}"
  for i in $(seq 1 "${RUNS_PER_METHOD}"); do
    run_num=$((run_num + 1))
    log_file="${LOG_DIR}/${method}_run${i}.log"
    echo "[$(date +%T)] (${run_num}/${total_runs}) MAP_TRANSPORT=${method} run ${i}/${RUNS_PER_METHOD} -- ${RUN_DURATION_S}s, log: ${log_file}"

    ROBOT="${ROBOT}" timeout --signal=TERM --kill-after="${KILL_AFTER_S}" "${RUN_DURATION_S}" \
      "${SCRIPT_DIR}/docker.sh" "${WORLD}" 2>&1 | tee "${log_file}"
    status="${PIPESTATUS[0]}"

    if [[ "${status}" -eq 124 || "${status}" -eq 137 ]]; then
      echo "[$(date +%T)] run finished (stopped at ${RUN_DURATION_S}s timeout, as expected)."
    elif [[ "${status}" -ne 0 ]]; then
      echo "[$(date +%T)] WARNING: docker.sh exited with status ${status} (see ${log_file})." >&2
    else
      echo "[$(date +%T)] run finished (exited on its own before the timeout)."
    fi
  done
done

echo "Sweep complete. Logs in ${LOG_DIR}, metrics in experiment_runs/ (RECORD_METRICS must be true in experiment.conf)."
