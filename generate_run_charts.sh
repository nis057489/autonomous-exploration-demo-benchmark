#!/usr/bin/env bash
set -euo pipefail

# Generates comparison figures for the latest baseline vs. vxch run per robot
# under experiment_runs/ (distance, map coverage, DDIL throughput, CPU load).
# The actual work is in results/generate_run_charts.py -- this just runs it in
# the project's Docker image, since it needs rosbag2_py + matplotlib and
# neither is on the host.
#
# Usage: ./generate_run_charts.sh [output_dir]
#   output_dir defaults to results/latest_run_comparison/

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"
DOCKER_IMAGE="${BENCHMARK_DOCKER_IMAGE:-autonomous-exploration-benchmark:jazzy-harmonic}"
RUNS_DIR="${PROJECT_ROOT}/experiment_runs"
OUT_DIR="$(cd "$(dirname "${1:-${PROJECT_ROOT}/results/latest_run_comparison}")" && pwd)/$(basename "${1:-latest_run_comparison}")"

if [[ ! -d "${RUNS_DIR}" ]]; then
  echo "No experiment_runs/ directory found at ${RUNS_DIR}." >&2
  exit 1
fi

mkdir -p "${OUT_DIR}"

docker run --rm \
  -v "${RUNS_DIR}:/data/experiment_runs:ro" \
  -v "${OUT_DIR}:/data/out" \
  -v "${PROJECT_ROOT}/results/generate_run_charts.py:/data/generate_run_charts.py:ro" \
  "${DOCKER_IMAGE}" python3 /data/generate_run_charts.py \
    --runs-dir /data/experiment_runs \
    --out-dir /data/out

echo
echo "Charts written to ${OUT_DIR}"
