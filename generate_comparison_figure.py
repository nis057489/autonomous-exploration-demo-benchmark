#!/usr/bin/env python3
"""Produce a scientific comparison figure across map-sharing conditions --
baseline, vxch (Wavestream), and/or zstd -- pass any 2 or all 3: how much
data was exchanged between robots to share the map, and how much of the map
got explored, for each condition given.

Reads each robot's recorded rosbag2 (mcap) directly:
  - received bandwidth: bytes on every /<robot>/incoming/<peer>/... topic
    (baseline: .../map; vxch: .../band_*+manifest; zstd: .../zstd_map) --
    what this robot actually received from its peers after DDIL throttling,
    per RECORD_METRICS's doc in experiment.conf, not the free-running
    node-log counters.
  - sent bandwidth: bytes this robot itself published for peers to pull --
    baseline: /<robot>/map (peers' ddil_proxy instances pull straight from
    it); vxch: /<robot>/vxch/map/band_*+manifest (the encoder's own output,
    pre-DDIL-relay-fanout); zstd: /<robot>/zstd/map (the compressor's own
    output, same pre-fanout point). Received and sent are read from disjoint
    topics, so a robot that explores little but has active peers can easily
    receive more than it sends -- that's not a contradiction, it's DDIL
    relaying its peers' progress to it regardless of its own.
  - communicated map coverage: /<robot>/nav_map (nav_msgs/OccupancyGrid),
    decoded per message into (seconds since bag start, known-cell area in
    m^2). nav_map is each robot's post-fusion team map -- it includes cells
    the robot only knows about because a peer relayed them, not just what
    the robot itself observed. This is NOT a measure of first-hand
    exploration; it's how much of the map each robot knows about, team-wide.
  - locally-observed coverage: /<robot>/map (nav_msgs/OccupancyGrid), same
    known-cell-area calculation, but this topic is slam_toolbox's raw local
    output -- it never receives peer-communicated cells (those only ever
    land in team_map_ddil, which per team_map_fusion.py excludes the
    robot's own map). So /map and team_map_ddil are disjoint by
    construction, and /map's known-area is a genuine, non-double-counted
    measure of what this robot itself has seen.

Must run where rosbag2_py and rclpy are importable, e.g. inside the
jazzy_env distrobox with a login shell so its ROS setup gets sourced:
  distrobox enter jazzy_env -- bash -lc "python3 generate_comparison_figure.py ..."

Usage:
  ./generate_comparison_figure.py \\
      --baseline robot1=<bag_dir> robot2=<bag_dir> \\
      --vxch     robot1=<bag_dir> robot2=<bag_dir> \\
      --zstd     robot1=<bag_dir> robot2=<bag_dir> \\
      --out figures/compare.png
  (any 2 of --baseline/--vxch/--zstd also works, e.g. just --baseline --zstd)
"""
import argparse
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid

TEXT_PRIMARY = "#1a1a1a"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#cccccc"
ALL_CONDITIONS = ("baseline", "vxch", "zstd")  # internal keys -- match run-dir/CLI naming, unrelated to display
DISPLAY_NAMES = {"baseline": "Baseline", "vxch": "Wavestream", "zstd": "Zstd"}
CONDITION_COLORS = {"baseline": "#eb6834", "vxch": "#2a78d6", "zstd": "#3fa15c"}
LINESTYLES = ("-", "--", ":", "-.")


def parse_robot_paths(pairs):
    out = {}
    for pair in pairs:
        robot, sep, path = pair.partition("=")
        if not sep or not robot or not path:
            raise ValueError(f"expected robot=bag_dir, got {pair!r}")
        out[robot] = Path(path)
    return out


def _reindexed_copy(bag_dir):
    """metadata.yaml is missing (recorder killed before finalizing, e.g. an
    interrupted run) -- copy the raw .mcap into a scratch dir and reindex
    there, mirroring replay_compare.sh's same fallback, so the original
    recording is never touched. Returns the scratch dir, or raises if there's
    nothing recoverable (no .mcap file, or reindexing itself fails)."""
    mcap_files = sorted(bag_dir.glob("*.mcap"))
    if not mcap_files:
        raise FileNotFoundError(f"no .mcap file found in {bag_dir}")
    scratch = Path(tempfile.mkdtemp(prefix="vxch_figure_reindex_"))
    for mcap_file in mcap_files:
        shutil.copy2(mcap_file, scratch)
    subprocess.run(
        ["ros2", "bag", "reindex", "-s", "mcap", str(scratch)],
        check=True, capture_output=True, text=True,
    )
    return scratch


def read_bag(robot, bag_dir, condition):
    """Returns (received_bytes, sent_bytes, coverage, local_coverage) where
    coverage and local_coverage are each a list of
    (seconds_since_start, known_area_m2) -- coverage from /<robot>/nav_map
    (post-fusion team map), local_coverage from /<robot>/map (this robot's
    own raw SLAM output, never touched by fusion) -- or None if the bag
    can't be read. received_bytes is traffic on /<robot>/incoming/<peer>/...
    (what peers sent this robot); sent_bytes is what this robot itself
    published for peers to pull -- /<robot>/map for baseline,
    /<robot>/vxch/map/band_*+manifest for vxch (see module docstring)."""
    bag_dir = Path(bag_dir)
    scratch_dir = None
    try:
        open_dir = bag_dir
        if not (bag_dir / "metadata.yaml").is_file():
            scratch_dir = _reindexed_copy(bag_dir)
            open_dir = scratch_dir

        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(open_dir), storage_id=""),
            ConverterOptions(input_serialization_format="", output_serialization_format=""),
        )
    except Exception as e:
        print(f"warning: failed to open bag for {robot} ({bag_dir}): {e}", file=sys.stderr)
        print(f"  if metadata.yaml is missing, try: ros2 bag reindex -s mcap {bag_dir}", file=sys.stderr)
        if scratch_dir is not None:
            shutil.rmtree(scratch_dir, ignore_errors=True)
        return None

    incoming_re = re.compile(rf"^/{re.escape(robot)}/incoming/")
    vxch_own_re = re.compile(rf"^/{re.escape(robot)}/vxch/map/")
    zstd_own_topic = f"/{robot}/zstd/map"
    nav_map_topic = f"/{robot}/nav_map"
    local_map_topic = f"/{robot}/map"

    received_bytes = 0
    sent_bytes = 0
    coverage = []
    local_coverage = []
    start_ns = None

    def known_area_m2(msg):
        known = int(np.count_nonzero(np.asarray(msg.data) != -1))
        return known * (msg.info.resolution ** 2)

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if start_ns is None:
            start_ns = t_ns
        if incoming_re.match(topic):
            received_bytes += len(data)
        elif topic == nav_map_topic:
            msg = deserialize_message(data, OccupancyGrid)
            coverage.append(((t_ns - start_ns) / 1e9, known_area_m2(msg)))
        elif topic == local_map_topic:
            msg = deserialize_message(data, OccupancyGrid)
            local_coverage.append(((t_ns - start_ns) / 1e9, known_area_m2(msg)))
            if condition == "baseline":
                sent_bytes += len(data)
        elif condition == "vxch" and vxch_own_re.match(topic):
            sent_bytes += len(data)
        elif condition == "zstd" and topic == zstd_own_topic:
            sent_bytes += len(data)

    del reader
    if scratch_dir is not None:
        shutil.rmtree(scratch_dir, ignore_errors=True)

    coverage.sort(key=lambda p: p[0])
    local_coverage.sort(key=lambda p: p[0])
    return received_bytes, sent_bytes, coverage, local_coverage


def style_ax(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(GRID_COLOR)
    ax.spines["bottom"].set_color(GRID_COLOR)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=10)
    ax.grid(axis="y", color=GRID_COLOR, linewidth=1, alpha=0.6, zorder=0)
    ax.set_axisbelow(True)


def plot_bandwidth(ax, results, conditions, byte_index, title, ylabel):
    """byte_index selects which per-robot byte count to plot out of the
    (received_bytes, sent_bytes, coverage, local_coverage) tuple in results
    -- 0 for received (what peers sent this robot), 1 for sent (what this
    robot itself published for peers to pull). conditions fixes the
    left-to-right bar order; results may hold any subset/superset of it."""
    robots = sorted({r for cond in results.values() for r in cond})
    x = np.arange(len(conditions))
    heights_by_robot = {
        robot: np.array([results[c].get(robot, (0, 0, [], []))[byte_index] / 1024 for c in conditions])
        for robot in robots
    }
    totals = np.sum(list(heights_by_robot.values()), axis=0) if robots else np.zeros(len(conditions))

    bottoms = np.zeros(len(conditions))
    for robot in robots:
        heights_kb = heights_by_robot[robot]
        ax.bar(x, heights_kb, bottom=bottoms, width=0.45,
               color=[CONDITION_COLORS[c] for c in conditions], edgecolor="white", linewidth=2,
               zorder=3)
        # Label each brick with the robot it belongs to, directly on the
        # segment -- skip slivers too thin for the label to fit legibly.
        for xi, height, base, total in zip(x, heights_kb, bottoms, totals):
            if total > 0 and height / total > 0.04:
                ax.annotate(robot, xy=(xi, base + height / 2), ha="center", va="center",
                            fontsize=8.5, fontweight="bold", color="white", zorder=4)
        bottoms += heights_kb

    for xi, total in zip(x, totals):
        ax.annotate(f"{total:,.0f} KB", xy=(xi, total), xytext=(0, 6),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=11, fontweight="bold", color=TEXT_PRIMARY)

    nonzero = [(c, t) for c, t in zip(conditions, totals) if t > 0]
    if len(nonzero) >= 2:
        (_, biggest_val) = max(nonzero, key=lambda ct: ct[1])
        (winner, smallest_val) = min(nonzero, key=lambda ct: ct[1])
        if biggest_val != smallest_val:
            ax.text(0.5, 0.99, f"{biggest_val / smallest_val:.1f}× less data ({DISPLAY_NAMES[winner]})",
                    transform=ax.transAxes, ha="center", va="top",
                    fontsize=10.5, color=TEXT_SECONDARY, style="italic")

    ax.set_ylim(0, max(totals) * 1.25 if max(totals) > 0 else 1)

    ax.set_xticks(x)
    ax.set_xticklabels([DISPLAY_NAMES[c] for c in conditions], fontsize=11, color=TEXT_PRIMARY, fontweight="bold")
    ax.set_ylabel(ylabel, fontsize=11, color=TEXT_SECONDARY)
    ax.set_title(title, fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)


def plot_coverage(ax, results, conditions, series_index, title, ylabel):
    """series_index selects which per-robot coverage series to plot out of
    the (received_bytes, sent_bytes, coverage, local_coverage) tuple in
    results -- 2 for communicated (nav_map), 3 for locally-observed (map)."""
    robots = sorted({r for cond in results.values() for r in cond})
    robot_style = {r: LINESTYLES[i % len(LINESTYLES)] for i, r in enumerate(robots)}

    for cond in conditions:
        for robot, entry in sorted(results[cond].items()):
            coverage = entry[series_index]
            if not coverage:
                continue
            ts = [p[0] for p in coverage]
            areas = [p[1] for p in coverage]
            ax.plot(ts, areas, color=CONDITION_COLORS[cond], linestyle=robot_style[robot],
                     linewidth=2, solid_capstyle="round", zorder=3)
            ax.annotate(robot, (ts[-1], areas[-1]), textcoords="offset points",
                        xytext=(6, 0), fontsize=8.5, color=TEXT_SECONDARY, va="center")

    ax.set_xlabel("Time since run start (s)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_ylabel(ylabel, fontsize=11, color=TEXT_SECONDARY)
    ax.set_title(title, fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)

    handles = [Line2D([0], [0], color=CONDITION_COLORS[c], lw=2, label=DISPLAY_NAMES[c])
               for c in conditions if results[c]]
    if handles:
        ax.legend(handles=handles, frameon=False, fontsize=10, loc="lower right")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--baseline", nargs="+", metavar="robot=bag_dir")
    parser.add_argument("--vxch", nargs="+", metavar="robot=bag_dir")
    parser.add_argument("--zstd", nargs="+", metavar="robot=bag_dir")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()

    robot_paths = {}
    for condition in ALL_CONDITIONS:
        pairs = getattr(args, condition)
        if pairs:
            robot_paths[condition] = parse_robot_paths(pairs)
    if len(robot_paths) < 2:
        print("error: need at least 2 of --baseline/--vxch/--zstd to compare", file=sys.stderr)
        sys.exit(1)
    conditions = tuple(c for c in ALL_CONDITIONS if c in robot_paths)

    results = {c: {} for c in conditions}
    for condition, robots in robot_paths.items():
        for robot, bag_dir in sorted(robots.items()):
            r = read_bag(robot, bag_dir, condition)
            if r is not None:
                results[condition][robot] = r

    for condition in conditions:
        if not results[condition]:
            print(f"error: no readable bags for condition {condition!r}", file=sys.stderr)
            sys.exit(1)

    fig, (ax_sent, ax_received, ax_coverage, ax_local) = plt.subplots(1, 4, figsize=(25, 5.5))
    plot_bandwidth(ax_sent, results, conditions, 1, "Map-sharing bandwidth (sent)", "Sent to peers (KB)")
    plot_bandwidth(ax_received, results, conditions, 0, "Map-sharing bandwidth (received)", "Received from peers (KB)")
    plot_coverage(ax_coverage, results, conditions, 2, "Communicated map coverage", "Known map area (m²)")
    plot_coverage(ax_local, results, conditions, 3, "Locally-observed coverage (self only)", "Self-observed area (m²)")
    fig.suptitle("Map sharing: " + " vs. ".join(DISPLAY_NAMES[c] for c in conditions), fontsize=15, fontweight="bold",
                 color=TEXT_PRIMARY, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=200, facecolor="white")

    totals_received = {c: sum(r for r, _, _, _ in results[c].values()) for c in conditions}
    totals_sent = {c: sum(s for _, s, _, _ in results[c].values()) for c in conditions}
    for c in conditions:
        print(f"{DISPLAY_NAMES[c]:>10} received: {totals_received[c]} bytes ({totals_received[c] / 1024:.1f} KB), "
              f"sent: {totals_sent[c]} bytes ({totals_sent[c] / 1024:.1f} KB)")
    nonzero_received = {c: v for c, v in totals_received.items() if v > 0}
    if len(nonzero_received) >= 2:
        print(f"received ratio (max/min): {max(nonzero_received.values()) / min(nonzero_received.values()):.2f}x")
    for condition in conditions:
        for robot, (_, _, coverage, local_coverage) in sorted(results[condition].items()):
            final = coverage[-1][1] if coverage else 0.0
            final_local = local_coverage[-1][1] if local_coverage else 0.0
            print(f"{DISPLAY_NAMES[condition]} {robot}: final known map area {final:.1f} m^2 (communicated, incl. peer-relayed cells), "
                  f"{final_local:.1f} m^2 self-observed")
    print(str(args.out.resolve()))


if __name__ == "__main__":
    main()
