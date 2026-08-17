#!/bin/bash
set -euo pipefail

# copy the latest run for each robot to the desktop.

SRC_BASE="/var/home/nick/code/autonomous-exploration-demo-benchmark/experiment_runs"
DEST_BASE="/home/nick/Desktop/vxch experiment results/bag_files/experiment_runs"

for robot_dir in "$SRC_BASE"/robot*/; do
    robot_name="$(basename "$robot_dir")"
    latest_run="$(find "$robot_dir" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort | tail -n 1)"

    if [ -z "$latest_run" ]; then
        echo "No runs found for $robot_name, skipping"
        continue
    fi

    dest="$DEST_BASE/$robot_name"
    mkdir -p "$dest"
    echo "Copying $robot_name latest run: $latest_run"
    cp -r "$robot_dir$latest_run" "$dest/"
done
