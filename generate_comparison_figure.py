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
    interrupted run) -- symlink the raw .mcap into a scratch dir and reindex
    there, mirroring replay_compare.sh's same fallback, so the original
    recording is never touched. A symlink (not a copy) because these files
    run multi-GB and reindexing only ever reads them -- copying risked
    filling /tmp outright (seen as ENOSPC/disk-quota-exceeded on a real run).
    Returns the scratch dir, or raises if there's nothing recoverable (no
    .mcap file, or reindexing itself fails)."""
    mcap_files = sorted(bag_dir.glob("*.mcap"))
    if not mcap_files:
        raise FileNotFoundError(f"no .mcap file found in {bag_dir}")
    scratch = Path(tempfile.mkdtemp(prefix="vxch_figure_reindex_"))
    for mcap_file in mcap_files:
        (scratch / mcap_file.name).symlink_to(mcap_file.resolve())
    subprocess.run(
        ["ros2", "bag", "reindex", "-s", "mcap", str(scratch)],
        check=True, capture_output=True, text=True,
    )
    return scratch


def read_bag(robot, bag_dir, condition):
    """Returns (received_bytes, sent_bytes, coverage, local_coverage,
    local_cell_series, nav_cell_series) where coverage and local_coverage
    are each a list of (seconds_since_start, known_area_m2) -- coverage
    from /<robot>/nav_map (post-fusion team map), local_coverage from
    /<robot>/map (this robot's own raw SLAM output, never touched by
    fusion) -- or None if the bag can't be read. received_bytes is traffic
    on /<robot>/incoming/<peer>/... (what peers sent this robot);
    sent_bytes is what this robot itself published for peers to pull --
    /<robot>/map for baseline, /<robot>/vxch/map/band_*+manifest for vxch
    (see module docstring). local_cell_series is a list of
    (seconds_since_start, int64 numpy array of packed cell keys newly known
    as of that /<robot>/map message -- not a full snapshot each time, see
    the loop below), used to union robots' own observations over time
    without double-counting cells more than one robot saw.
    nav_cell_series is the same shape but diffed off /<robot>/nav_map, i.e.
    it includes peer-relayed cells -- unioning it across robots shows what
    the team collectively knows, which is where communication's effect is
    actually visible (unioning local_cell_series instead structurally can't
    show it, since that series never receives fused/peer-relayed cells)."""
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

    def known_cells(msg):
        """Grid-cell coordinates keyed in a global, resolution-sized lattice
        (not this message's local row/col indices) so that cells from
        different robots -- whose grids can have different origins/extents
        as slam_toolbox grows each robot's own map independently -- line up
        and can be unioned. Assumes all robots share the same map frame and
        resolution, which holds for this benchmark's shared SLAM config.
        Returns a sorted int64 numpy array, each element packing (gx, gy)
        into one 64-bit key (32 bits each) -- cheap to build and diff with
        numpy's setdiff1d/isin, vs. a Python set of (int, int) tuples which
        is much slower to hash and was the actual bottleneck here."""
        data = np.asarray(msg.data)
        known_idx = np.flatnonzero(data != -1)
        if known_idx.size == 0:
            return np.empty(0, dtype=np.int64)
        width = msg.info.width
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        xs = known_idx % width
        ys = known_idx // width
        gx = np.round((ox + (xs + 0.5) * res) / res).astype(np.int64)
        gy = np.round((oy + (ys + 0.5) * res) / res).astype(np.int64)
        keys = (gx << np.int64(32)) | (gy & np.int64(0xFFFFFFFF))
        keys.sort()
        return keys

    local_cell_series = []
    nav_cell_series = []
    seen_cells = np.empty(0, dtype=np.int64)
    seen_nav_cells = np.empty(0, dtype=np.int64)
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if start_ns is None:
            start_ns = t_ns
        if incoming_re.match(topic):
            received_bytes += len(data)
        elif topic == nav_map_topic:
            msg = deserialize_message(data, OccupancyGrid)
            t = (t_ns - start_ns) / 1e9
            coverage.append((t, known_area_m2(msg)))
            cells_now = known_cells(msg)
            new_cells = np.setdiff1d(cells_now, seen_nav_cells, assume_unique=True)
            if new_cells.size:
                nav_cell_series.append((t, new_cells))
            seen_nav_cells = cells_now
        elif topic == local_map_topic:
            msg = deserialize_message(data, OccupancyGrid)
            t = (t_ns - start_ns) / 1e9
            local_coverage.append((t, known_area_m2(msg)))
            # Store only the cells newly known since this robot's last /map
            # message, not a full snapshot every time -- occupancy grids are
            # cumulative (once known, cells stay known), so a full-set
            # snapshot per message would retain O(cells * messages) instead
            # of O(final cell count) and OOM on longer runs. Diffs union
            # together identically to full snapshots downstream since set
            # union already dedupes across messages/robots.
            cells_now = known_cells(msg)
            new_cells = np.setdiff1d(cells_now, seen_cells, assume_unique=True)
            if new_cells.size:
                local_cell_series.append((t, new_cells))
            seen_cells = cells_now
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
    local_cell_series.sort(key=lambda p: p[0])
    nav_cell_series.sort(key=lambda p: p[0])
    return received_bytes, sent_bytes, coverage, local_coverage, local_cell_series, nav_cell_series


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
        robot: np.array([results[c].get(robot, (0, 0, [], [], [], []))[byte_index] / 1024 for c in conditions])
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


def union_coverage_over_time(cell_series_by_robot):
    """cell_series_by_robot is an iterable of local_cell_series lists (each
    a (t, int64 array of packed cell keys) list, per robot, from read_bag).
    Merges every robot's messages into one timeline in time order and
    returns a list of (t, running_union_tile_count) as cells accumulate --
    a cell counts once the first time ANY robot sees it, so two robots
    re-observing the same tile doesn't inflate the curve. Uses a plain
    Python int set for the running total (each element already a single
    packed int64, not a tuple) since the merge is across robots'
    already-deduped-per-robot diffs, where numpy set ops would need
    re-sorting/re-uniquifying on every step."""
    events = [event for series in cell_series_by_robot for event in series]
    events.sort(key=lambda e: e[0])
    running = set()
    out = []
    for t, cells in events:
        if cells.size:
            running.update(cells.tolist())
        out.append((t, len(running)))
    return out


def plot_union_coverage(ax, results, conditions, cell_index, title):
    """Team-wide map coverage over time, counted as the running UNION of
    every robot's cells (see known_cells) as messages arrive -- a cell two
    robots both know about only counts once, unlike summing each robot's
    coverage area. cell_index picks which per-robot cell series to union:
    4 for local_cell_series (/<robot>/map, self-observed only -- "how much
    has the team physically laid eyes on"), 5 for nav_cell_series
    (/<robot>/nav_map, post-fusion -- "how much does the team collectively
    know", including peer-relayed cells. This is the one that actually
    moves when communication improves, since local_cell_series structurally
    excludes anything a robot only learned from a peer)."""
    for cond in conditions:
        merged = union_coverage_over_time(entry[cell_index] for entry in results[cond].values())
        if not merged:
            continue
        ts = [p[0] for p in merged]
        tiles = [p[1] for p in merged]
        ax.plot(ts, tiles, color=CONDITION_COLORS[cond], linewidth=2, solid_capstyle="round", zorder=3)
        ax.annotate(f"{tiles[-1]:,} tiles", (ts[-1], tiles[-1]), textcoords="offset points",
                    xytext=(6, 0), fontsize=8.5, fontweight="bold", color=CONDITION_COLORS[cond], va="center")

    ax.set_xlabel("Time since run start (s)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_ylabel("Known tiles (union across robots)", fontsize=11, color=TEXT_SECONDARY)
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

    fig, (ax_sent, ax_received, ax_coverage, ax_local, ax_union, ax_union_nav) = plt.subplots(1, 6, figsize=(37, 5.5))
    plot_bandwidth(ax_sent, results, conditions, 1, "Map-sharing bandwidth (sent)", "Sent to peers (KB)")
    plot_bandwidth(ax_received, results, conditions, 0, "Map-sharing bandwidth (received)", "Received from peers (KB)")
    plot_coverage(ax_coverage, results, conditions, 2, "Communicated map coverage", "Known map area (m²)")
    plot_coverage(ax_local, results, conditions, 3, "Locally-observed coverage (self only)", "Self-observed area (m²)")
    plot_union_coverage(ax_union, results, conditions, 4,
                         "Team physical coverage over time (union, self-observed only)")
    plot_union_coverage(ax_union_nav, results, conditions, 5,
                         "Team known coverage over time (union, incl. peer-relayed)")
    fig.suptitle("Map sharing: " + " vs. ".join(DISPLAY_NAMES[c] for c in conditions), fontsize=15, fontweight="bold",
                 color=TEXT_PRIMARY, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=200, facecolor="white")

    totals_received = {c: sum(r for r, _, _, _, _, _ in results[c].values()) for c in conditions}
    totals_sent = {c: sum(s for _, s, _, _, _, _ in results[c].values()) for c in conditions}
    for c in conditions:
        print(f"{DISPLAY_NAMES[c]:>10} received: {totals_received[c]} bytes ({totals_received[c] / 1024:.1f} KB), "
              f"sent: {totals_sent[c]} bytes ({totals_sent[c] / 1024:.1f} KB)")
    nonzero_received = {c: v for c, v in totals_received.items() if v > 0}
    if len(nonzero_received) >= 2:
        print(f"received ratio (max/min): {max(nonzero_received.values()) / min(nonzero_received.values()):.2f}x")
    for condition in conditions:
        for robot, (_, _, coverage, local_coverage, _, _) in sorted(results[condition].items()):
            final = coverage[-1][1] if coverage else 0.0
            final_local = local_coverage[-1][1] if local_coverage else 0.0
            print(f"{DISPLAY_NAMES[condition]} {robot}: final known map area {final:.1f} m^2 (communicated, incl. peer-relayed cells), "
                  f"{final_local:.1f} m^2 self-observed")
    for condition in conditions:
        merged_local = union_coverage_over_time(entry[4] for entry in results[condition].values())
        merged_nav = union_coverage_over_time(entry[5] for entry in results[condition].values())
        final_local_tiles = merged_local[-1][1] if merged_local else 0
        final_nav_tiles = merged_nav[-1][1] if merged_nav else 0
        print(f"{DISPLAY_NAMES[condition]:>10} team physical coverage (union, self-observed only): {final_local_tiles:,} tiles; "
              f"team known coverage (union, incl. peer-relayed): {final_nav_tiles:,} tiles")
    print(str(args.out.resolve()))


if __name__ == "__main__":
    main()
