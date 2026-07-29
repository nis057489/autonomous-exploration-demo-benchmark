#!/usr/bin/env python3
"""
Generates comparison figures for the latest baseline vs. vxch experiment_runs/
per robot: distance traveled, own/team map coverage, DDIL link throughput, and
CPU load. Needs rosbag2_py + matplotlib, which live in the project's Docker
image (not the host) -- run via ../generate_run_charts.sh, not directly.
"""
import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Path as PathMsg, OccupancyGrid

CONDITIONS = ["baseline", "vxch"]
# references/palette.md categorical slots 1 (blue) and 2 (orange) -- already
# validated as a pair, fixed order (baseline always blue, vxch always orange).
COLORS = {"baseline": "#2a78d6", "vxch": "#eb6834"}


def find_latest_runs(runs_dir: Path):
    """{robot: {condition: run_dir or None}}, latest by directory-name timestamp."""
    runs = {}
    for robot_dir in sorted(runs_dir.iterdir()):
        if not robot_dir.is_dir():
            continue
        robot = robot_dir.name
        runs[robot] = {}
        for cond in CONDITIONS:
            candidates = sorted(robot_dir.glob(f"*_{cond}_{robot}"))
            runs[robot][cond] = candidates[-1] if candidates else None
    return runs


def ensure_indexed(bag_dir: Path, scratch_dir: Path) -> Path:
    """Reindex into a scratch copy if metadata.yaml is missing (recorder was
    killed before it could finalize) -- never touches the original recording."""
    if (bag_dir / "metadata.yaml").exists():
        return bag_dir
    dest = scratch_dir / bag_dir.parent.name
    dest.mkdir(parents=True, exist_ok=True)
    for mcap in bag_dir.glob("*.mcap"):
        shutil.copy(mcap, dest / mcap.name)
    print(f"  reindexing {bag_dir} (no metadata.yaml)...")
    subprocess.run(["ros2", "bag", "reindex", "-s", "mcap", str(dest)], check=True,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return dest


def read_bag_metrics(bag_path: Path, robot: str) -> dict:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id='mcap'),
                ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))

    types_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    path_topic = f"/{robot}/explore/traversed_path"
    own_map_topic = f"/{robot}/map"
    team_map_topic = f"/{robot}/team_map_ddil"

    last_path, last_own_map, last_team_map = None, None, None
    t_min, t_max = None, None

    while reader.has_next():
        topic, data, t = reader.read_next()
        t_min = t if t_min is None else min(t_min, t)
        t_max = t if t_max is None else max(t_max, t)
        if topic == path_topic and types_by_topic.get(topic) == 'nav_msgs/msg/Path':
            last_path = deserialize_message(data, PathMsg)
        elif topic == own_map_topic and types_by_topic.get(topic) == 'nav_msgs/msg/OccupancyGrid':
            last_own_map = deserialize_message(data, OccupancyGrid)
        elif topic == team_map_topic and types_by_topic.get(topic) == 'nav_msgs/msg/OccupancyGrid':
            last_team_map = deserialize_message(data, OccupancyGrid)

    duration_s = (t_max - t_min) / 1e9 if (t_min is not None and t_max is not None) else 0.0

    distance_m = 0.0
    if last_path is not None:
        poses = last_path.poses
        for i in range(1, len(poses)):
            p0, p1 = poses[i - 1].pose.position, poses[i].pose.position
            distance_m += ((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2) ** 0.5

    def known_area_m2(grid):
        if grid is None:
            return None
        res = grid.info.resolution
        known = sum(1 for c in grid.data if c != -1)
        return known * res * res

    return {
        "duration_s": duration_s,
        "distance_m": distance_m,
        "own_map_m2": known_area_m2(last_own_map),
        "team_map_m2": known_area_m2(last_team_map),
    }


def parse_ddil_bandwidth(run_dir: Path) -> dict:
    total_kb, total_msgs = 0.0, 0
    for log in run_dir.glob("ros_logs/ddil_proxy_node_*.log"):
        text = log.read_text(errors="ignore")
        matches = re.findall(r"stats \| rcvd=\d+\s+sent=(\d+)\s+\((\d+) KB\)", text)
        if matches:
            sent, kb = matches[-1]
            total_msgs += int(sent)
            total_kb += float(kb)
    return {"bandwidth_kb": total_kb, "bandwidth_msgs": total_msgs}


def parse_cpu_load(run_dir: Path) -> dict:
    log = run_dir / "cpu_load.log"
    if not log.exists():
        return {"cpu_avg": None, "cpu_max": None}
    loads = []
    for line in log.read_text(errors="ignore").splitlines():
        parts = line.split()
        if len(parts) >= 2:
            try:
                loads.append(float(parts[1]))
            except ValueError:
                pass
    if not loads:
        return {"cpu_avg": None, "cpu_max": None}
    return {"cpu_avg": sum(loads) / len(loads), "cpu_max": max(loads)}


def per_minute(value, duration_s):
    if value is None or duration_s <= 0:
        return None
    return value / duration_s * 60.0


def grouped_bar_chart(robots, series, title, ylabel, fmt, out_path, footer):
    """series: {condition: [value_or_None, ...] aligned with robots}"""
    fig, ax = plt.subplots(figsize=(10, 6))
    x = range(len(robots))
    width = 0.32
    for i, cond in enumerate(CONDITIONS):
        offset = (i - 0.5) * width
        vals = [v if v is not None else 0.0 for v in series[cond]]
        bars = ax.bar([xi + offset for xi in x], vals, width=width,
                      color=COLORS[cond], label=cond, edgecolor='white', linewidth=1, zorder=3)
        for bar, v, raw in zip(bars, vals, series[cond]):
            if raw is None:
                continue
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(),
                     fmt.format(v), ha='center', va='bottom', fontsize=10, fontweight='bold', color='#333333')

    ax.set_title(title, fontsize=15, fontweight='bold', pad=14, color='#1a1a1a', loc='left')
    ax.set_ylabel(ylabel, fontsize=12, color='#444444')
    ax.set_xticks(list(x))
    ax.set_xticklabels(robots, fontsize=11)
    ax.grid(axis='y', linestyle='--', alpha=0.35, color='#cccccc', zorder=0)
    ax.grid(axis='x', visible=False)
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)
    for spine in ('left', 'bottom'):
        ax.spines[spine].set_color('#cccccc')
    ax.legend(frameon=False, fontsize=11)
    fig.text(0.01, 0.01, footer, fontsize=8, color='#999999', ha='left')
    plt.tight_layout(rect=(0, 0.03, 1, 1))
    plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs-dir", type=Path, default=Path("experiment_runs"))
    parser.add_argument("--out-dir", type=Path, default=Path("results/latest_run_comparison"))
    parser.add_argument("--scratch-dir", type=Path, default=Path("/tmp/generate_run_charts_scratch"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    args.scratch_dir.mkdir(parents=True, exist_ok=True)

    latest = find_latest_runs(args.runs_dir)
    robots = [r for r in latest if latest[r]["baseline"] and latest[r]["vxch"]]
    if not robots:
        print("No robot has both a baseline and a vxch run under "
              f"{args.runs_dir} -- nothing to compare.", file=sys.stderr)
        sys.exit(1)

    run_dirs = {}  # {robot: {condition: run_dir}}
    metrics = {}   # {robot: {condition: {...}}}
    for robot in robots:
        run_dirs[robot] = {}
        metrics[robot] = {}
        for cond in CONDITIONS:
            run_dir = latest[robot][cond]
            run_dirs[robot][cond] = run_dir
            print(f"{cond:8s} / {robot}: {run_dir.name}")
            bag_dir = ensure_indexed(run_dir / "bag", args.scratch_dir)
            m = read_bag_metrics(bag_dir, robot)
            m.update(parse_ddil_bandwidth(run_dir))
            m.update(parse_cpu_load(run_dir))
            metrics[robot][cond] = m

    footer_bits = [f"{robot}: {run_dirs[robot]['baseline'].name} vs {run_dirs[robot]['vxch'].name}"
                   for robot in robots]
    footer = "Source -- " + " | ".join(footer_bits)

    def series_for(key, rate=False):
        out = {}
        for cond in CONDITIONS:
            vals = []
            for robot in robots:
                m = metrics[robot][cond]
                v = m.get(key)
                if rate:
                    v = per_minute(v, m["duration_s"])
                vals.append(v)
            out[cond] = vals
        return out

    charts = [
        ("Chart_Distance.png", "Distance Traveled — latest baseline vs vxch", "m / min", "{:.1f}",
         series_for("distance_m", rate=True)),
        ("Chart_OwnMapCoverage.png", "Own SLAM Map Coverage — latest baseline vs vxch", "m² / min", "{:.1f}",
         series_for("own_map_m2", rate=True)),
        ("Chart_TeamMapCoverage.png", "Team-Fused Map Coverage — latest baseline vs vxch", "m² / min", "{:.1f}",
         series_for("team_map_m2", rate=True)),
        ("Chart_Bandwidth.png", "DDIL Link Throughput — latest baseline vs vxch", "KB / min delivered", "{:.0f}",
         series_for("bandwidth_kb", rate=True)),
        ("Chart_CPULoad.png", "CPU Load (1-min avg) — latest baseline vs vxch", "loadavg", "{:.1f}",
         series_for("cpu_avg", rate=False)),
    ]

    for filename, title, ylabel, fmt, series in charts:
        grouped_bar_chart(robots, series, title, ylabel, fmt, args.out_dir / filename, footer)
        print(f"wrote {args.out_dir / filename}")

    fig, axes = plt.subplots(2, 3, figsize=(20, 11))
    axes = axes.flatten()
    for ax, (filename, title, ylabel, fmt, series) in zip(axes, charts):
        x = range(len(robots))
        width = 0.32
        for i, cond in enumerate(CONDITIONS):
            offset = (i - 0.5) * width
            vals = [v if v is not None else 0.0 for v in series[cond]]
            ax.bar([xi + offset for xi in x], vals, width=width, color=COLORS[cond],
                   label=cond, edgecolor='white', linewidth=1, zorder=3)
        ax.set_title(title, fontsize=12, fontweight='bold', loc='left')
        ax.set_ylabel(ylabel, fontsize=10)
        ax.set_xticks(list(x))
        ax.set_xticklabels(robots, fontsize=9)
        ax.grid(axis='y', linestyle='--', alpha=0.35, color='#cccccc', zorder=0)
        for spine in ('top', 'right'):
            ax.spines[spine].set_visible(False)
        ax.legend(frameon=False, fontsize=9)
    axes[-1].axis('off')
    fig.text(0.01, 0.01, footer, fontsize=8, color='#999999', ha='left')
    plt.tight_layout(rect=(0, 0.02, 1, 1))
    plt.savefig(args.out_dir / "Chart_AllMetrics.png", dpi=150, bbox_inches='tight', facecolor='white')
    plt.close(fig)
    print(f"wrote {args.out_dir / 'Chart_AllMetrics.png'}")

    summary_path = args.out_dir / "summary.csv"
    with open(summary_path, "w") as f:
        f.write("robot,condition,run,duration_s,distance_m,own_map_m2,team_map_m2,bandwidth_kb,bandwidth_msgs,cpu_avg,cpu_max\n")
        for robot in robots:
            for cond in CONDITIONS:
                m = metrics[robot][cond]
                f.write(",".join(str(x) for x in [
                    robot, cond, run_dirs[robot][cond].name,
                    round(m["duration_s"], 1), round(m["distance_m"], 2),
                    m["own_map_m2"], m["team_map_m2"],
                    round(m["bandwidth_kb"], 1), m["bandwidth_msgs"],
                    m["cpu_avg"], m["cpu_max"],
                ]) + "\n")
    print(f"wrote {summary_path}")


if __name__ == "__main__":
    main()
