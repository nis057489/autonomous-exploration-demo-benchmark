#!/usr/bin/env python3
"""Produce a scientific comparison figure for a baseline run vs. a vxch run:
how much data was exchanged between robots to share the map, and how much of
the map got explored, for each condition.

Reads each robot's recorded rosbag2 (mcap) directly:
  - inter-robot map traffic: bytes on every /<robot>/incoming/<peer>/... topic
    (baseline: .../map; vxch: .../band_*+manifest) -- per RECORD_METRICS's
    doc in experiment.conf, this is the real traffic between robots, already
    past DDIL throttling, not the free-running node-log counters.
  - exploration coverage: /<robot>/nav_map (nav_msgs/OccupancyGrid), decoded
    per message into (seconds since bag start, known-cell area in m^2).

Must run where rosbag2_py and rclpy are importable, e.g. inside the
jazzy_env distrobox with a login shell so its ROS setup gets sourced:
  distrobox enter jazzy_env -- bash -lc "python3 generate_comparison_figure.py ..."

Usage:
  ./generate_comparison_figure.py \\
      --baseline robot1=<bag_dir> robot2=<bag_dir> \\
      --vxch     robot1=<bag_dir> robot2=<bag_dir> \\
      --out figures/compare.png
"""
import argparse
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from matplotlib.lines import Line2D

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid

COLOR_BASELINE = "#eb6834"
COLOR_VXCH = "#2a78d6"
TEXT_PRIMARY = "#1a1a1a"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#cccccc"
CONDITIONS = ("baseline", "vxch")
LINESTYLES = ("-", "--", ":", "-.")


def parse_robot_paths(pairs):
    out = {}
    for pair in pairs:
        robot, sep, path = pair.partition("=")
        if not sep or not robot or not path:
            raise ValueError(f"expected robot=bag_dir, got {pair!r}")
        out[robot] = Path(path)
    return out


def read_bag(robot, bag_dir):
    """Returns (incoming_bytes, coverage) where coverage is a list of
    (seconds_since_start, known_area_m2), or None if the bag can't be read."""
    reader = SequentialReader()
    try:
        reader.open(
            StorageOptions(uri=str(bag_dir), storage_id=""),
            ConverterOptions(input_serialization_format="", output_serialization_format=""),
        )
    except RuntimeError as e:
        print(f"warning: failed to open bag for {robot} ({bag_dir}): {e}", file=sys.stderr)
        print(f"  if metadata.yaml is missing, try: ros2 bag reindex -s mcap {bag_dir}", file=sys.stderr)
        return None

    incoming_re = re.compile(rf"^/{re.escape(robot)}/incoming/")
    nav_map_topic = f"/{robot}/nav_map"

    incoming_bytes = 0
    coverage = []
    start_ns = None

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if start_ns is None:
            start_ns = t_ns
        if incoming_re.match(topic):
            incoming_bytes += len(data)
        elif topic == nav_map_topic:
            msg = deserialize_message(data, OccupancyGrid)
            known = int(np.count_nonzero(np.asarray(msg.data) != -1))
            area_m2 = known * (msg.info.resolution ** 2)
            coverage.append(((t_ns - start_ns) / 1e9, area_m2))

    coverage.sort(key=lambda p: p[0])
    return incoming_bytes, coverage


def style_ax(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(GRID_COLOR)
    ax.spines["bottom"].set_color(GRID_COLOR)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=10)
    ax.grid(axis="y", color=GRID_COLOR, linewidth=1, alpha=0.6, zorder=0)
    ax.set_axisbelow(True)


def plot_bandwidth(ax, results):
    robots = sorted({r for cond in results.values() for r in cond})
    x = np.arange(len(CONDITIONS))
    bottoms = np.zeros(len(CONDITIONS))

    for robot in robots:
        heights_kb = np.array([results[c].get(robot, (0, []))[0] / 1024 for c in CONDITIONS])
        ax.bar(x, heights_kb, bottom=bottoms, width=0.45,
               color=[COLOR_BASELINE, COLOR_VXCH], edgecolor="white", linewidth=2, zorder=3)
        bottoms += heights_kb

    totals = bottoms
    for xi, total in zip(x, totals):
        ax.annotate(f"{total:,.0f} KB", xy=(xi, total), xytext=(0, 6),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=11, fontweight="bold", color=TEXT_PRIMARY)

    if totals[0] > 0 and totals[1] > 0 and totals[0] != totals[1]:
        bigger, smaller = max(totals), min(totals)
        winner = "VXCH" if totals[1] < totals[0] else "baseline"
        ax.text(0.5, 0.92, f"{bigger / smaller:.1f}× less data ({winner})",
                transform=ax.transAxes, ha="center", va="top",
                fontsize=10.5, color=TEXT_SECONDARY, style="italic")

    if totals[0] > 0 and totals[1] > 0 and max(totals) / min(totals) > 8:
        ax.set_yscale("log")
        ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda v, _: f"{v:,.0f}"))
    else:
        ax.set_ylim(0, max(totals) * 1.25 if max(totals) > 0 else 1)

    ax.set_xticks(x)
    ax.set_xticklabels(["Baseline", "VXCH"], fontsize=11, color=TEXT_PRIMARY, fontweight="bold")
    ax.set_ylabel("Inter-robot map traffic (KB)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_title("Map-sharing bandwidth", fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)


def plot_coverage(ax, results):
    colors = {"baseline": COLOR_BASELINE, "vxch": COLOR_VXCH}
    robots = sorted({r for cond in results.values() for r in cond})
    robot_style = {r: LINESTYLES[i % len(LINESTYLES)] for i, r in enumerate(robots)}

    for cond in CONDITIONS:
        for robot, (_, coverage) in sorted(results[cond].items()):
            if not coverage:
                continue
            ts = [p[0] for p in coverage]
            areas = [p[1] for p in coverage]
            ax.plot(ts, areas, color=colors[cond], linestyle=robot_style[robot],
                     linewidth=2, solid_capstyle="round", zorder=3)
            ax.annotate(robot, (ts[-1], areas[-1]), textcoords="offset points",
                        xytext=(6, 0), fontsize=8.5, color=TEXT_SECONDARY, va="center")

    ax.set_xlabel("Time since run start (s)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_ylabel("Explored area (m²)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_title("Exploration coverage", fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)

    handles = [Line2D([0], [0], color=colors[c], lw=2, label=c.capitalize())
               for c in CONDITIONS if results[c]]
    if handles:
        ax.legend(handles=handles, frameon=False, fontsize=10, loc="lower right")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--baseline", nargs="+", required=True, metavar="robot=bag_dir")
    parser.add_argument("--vxch", nargs="+", required=True, metavar="robot=bag_dir")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()

    robot_paths = {"baseline": parse_robot_paths(args.baseline), "vxch": parse_robot_paths(args.vxch)}

    results = {"baseline": {}, "vxch": {}}
    for condition, robots in robot_paths.items():
        for robot, bag_dir in sorted(robots.items()):
            r = read_bag(robot, bag_dir)
            if r is not None:
                results[condition][robot] = r

    for condition in CONDITIONS:
        if not results[condition]:
            print(f"error: no readable bags for condition {condition!r}", file=sys.stderr)
            sys.exit(1)

    fig, (ax_bytes, ax_coverage) = plt.subplots(1, 2, figsize=(13, 5.5))
    plot_bandwidth(ax_bytes, results)
    plot_coverage(ax_coverage, results)
    fig.suptitle("Map sharing: baseline vs. vxch", fontsize=15, fontweight="bold",
                 color=TEXT_PRIMARY, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=200, facecolor="white")

    baseline_total = sum(b for b, _ in results["baseline"].values())
    vxch_total = sum(b for b, _ in results["vxch"].values())
    print(f"baseline total: {baseline_total} bytes ({baseline_total / 1024:.1f} KB)")
    print(f"vxch total:     {vxch_total} bytes ({vxch_total / 1024:.1f} KB)")
    if baseline_total > 0 and vxch_total > 0:
        print(f"ratio: {max(baseline_total, vxch_total) / min(baseline_total, vxch_total):.2f}x")
    for condition in CONDITIONS:
        for robot, (_, coverage) in sorted(results[condition].items()):
            final = coverage[-1][1] if coverage else 0.0
            print(f"{condition} {robot}: final coverage {final:.1f} m^2")
    print(str(args.out.resolve()))


if __name__ == "__main__":
    main()
