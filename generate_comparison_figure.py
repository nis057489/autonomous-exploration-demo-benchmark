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

  Add --max-duration <seconds> to clip each bag to that much time since its
  start (e.g. to match a replay clipped with REPLAY_MAX_DURATION).

  Repeat --baseline/--vxch/--zstd to average a condition over several runs,
  each occurrence being one run's robot=bag_dir set:
    ./generate_comparison_figure.py \\
        --baseline robot1=<run1_bag_dir> robot2=<run1_bag_dir> \\
        --baseline robot1=<run2_bag_dir> robot2=<run2_bag_dir> \\
        --vxch     robot1=<run1_bag_dir> robot2=<run1_bag_dir> \\
        --vxch     robot1=<run2_bag_dir> robot2=<run2_bag_dir> \\
        --out figures/compare.png
  With more than one run for a condition, bar charts show mean +/- std
  error bars and time-series plots show a mean line with a +/- std band,
  computed across those runs.
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


def read_bag(robot, bag_dir, condition, max_duration=None):
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
        if max_duration is not None and (t_ns - start_ns) / 1e9 > max_duration:
            break
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


def resample_step(series, grid):
    """Sample a cumulative (t, value) series -- known-area or known-cell-
    count, both non-decreasing over a bag -- onto arbitrary grid times via
    step-hold (the value as of the last message at or before each grid
    time; 0 before the first message). Used to line up runs of slightly
    different length/sampling onto a shared x-axis so mean/std can be
    computed pointwise across them."""
    if not series:
        return np.zeros(len(grid))
    ts = np.array([p[0] for p in series])
    ys = np.array([p[1] for p in series], dtype=float)
    idx = np.searchsorted(ts, grid, side="right") - 1
    return np.where(idx >= 0, ys[np.clip(idx, 0, None)], 0.0)


def plot_bandwidth(ax, results, conditions, byte_index, title, ylabel):
    """byte_index selects which per-robot byte count to plot out of the
    (received_bytes, sent_bytes, coverage, local_coverage, ...) tuple --
    0 for received (what peers sent this robot), 1 for sent (what this
    robot itself published for peers to pull). conditions fixes the
    left-to-right bar order. results[c] is a list of per-run {robot: entry}
    dicts -- with more than one run, bars show the mean total across runs
    with a +/- std error bar; per-robot stack segments use each robot's mean
    across runs (a robot missing from a run counts as 0 bytes that run, so
    the segments still sum to the mean total)."""
    robots = sorted({r for run in results.values() for entry in run for r in entry})
    x = np.arange(len(conditions))

    def robot_bytes(run, robot):
        entry = run.get(robot)
        return entry[byte_index] / 1024 if entry is not None else 0.0

    heights_by_robot = {
        robot: np.array([
            np.mean([robot_bytes(run, robot) for run in results[c]]) if results[c] else 0.0
            for c in conditions
        ])
        for robot in robots
    }
    totals = np.sum(list(heights_by_robot.values()), axis=0) if robots else np.zeros(len(conditions))
    totals_std = np.array([
        np.std([sum(robot_bytes(run, r) for r in robots) for run in results[c]]) if len(results[c]) > 1 else 0.0
        for c in conditions
    ])

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

    ax.errorbar(x, totals, yerr=totals_std, fmt="none", ecolor=TEXT_PRIMARY, elinewidth=1.5, capsize=5, zorder=5)

    for xi, total, std, n in zip(x, totals, totals_std, (len(results[c]) for c in conditions)):
        label = f"{total:,.0f} KB" + (f" ± {std:,.0f}" if n > 1 else "")
        ax.annotate(label, xy=(xi, total + std), xytext=(0, 6),
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

    ax.set_ylim(0, max(totals + totals_std) * 1.25 if max(totals) > 0 else 1)

    ax.set_xticks(x)
    ax.set_xticklabels([DISPLAY_NAMES[c] for c in conditions], fontsize=11, color=TEXT_PRIMARY, fontweight="bold")
    ax.set_ylabel(ylabel, fontsize=11, color=TEXT_SECONDARY)
    ax.set_title(title, fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)


def plot_coverage(ax, results, conditions, series_index, title, ylabel):
    """series_index selects which per-robot coverage series to plot out of
    the (received_bytes, sent_bytes, coverage, local_coverage, ...) tuple --
    2 for communicated (nav_map), 3 for locally-observed (map). results[c]
    is a list of per-run {robot: entry} dicts; each robot's line is the
    mean of that robot's series across the runs it appears in, resampled
    onto a shared time grid (resample_step) -- with a +/- std shaded band
    when more than one run contributes."""
    robots = sorted({r for run in results.values() for entry in run for r in entry})
    robot_style = {r: LINESTYLES[i % len(LINESTYLES)] for i, r in enumerate(robots)}

    for cond in conditions:
        runs = results[cond]
        max_ts = [entry[series_index][-1][0] for run in runs for entry in run.values() if entry[series_index]]
        if not max_ts:
            continue
        grid = np.linspace(0, min(max_ts), 200)
        for robot in robots:
            series_per_run = [run[robot][series_index] for run in runs if robot in run and run[robot][series_index]]
            if not series_per_run:
                continue
            sampled = np.array([resample_step(s, grid) for s in series_per_run])
            mean = sampled.mean(axis=0)
            ax.plot(grid, mean, color=CONDITION_COLORS[cond], linestyle=robot_style[robot],
                     linewidth=2, solid_capstyle="round", zorder=3)
            if sampled.shape[0] > 1:
                std = sampled.std(axis=0)
                ax.fill_between(grid, mean - std, mean + std, color=CONDITION_COLORS[cond], alpha=0.15,
                                 linewidth=0, zorder=2)
            ax.annotate(robot, (grid[-1], mean[-1]), textcoords="offset points",
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


def cumulative_count_series(cell_series):
    """Turn one robot's local_cell_series ((t, new_cells) diffs, already
    deduped against that robot's own earlier observations) into a running
    (t, cumulative_count) series -- how many cells *this robot alone* has
    now seen, ignoring what any teammate has seen."""
    total = 0
    out = []
    for t, cells in cell_series:
        total += cells.size
        out.append((t, total))
    return out


def run_redundant_series(run, cell_index):
    """For one run, (t, redundant_cell_count) over time: the sum of every
    robot's own cumulative cell count minus the team union
    (union_coverage_over_time) at the same instant. Both quantities are
    non-decreasing and the union can never exceed the sum (a cell known
    team-wide is known by at least one robot), so this difference is
    itself non-decreasing -- it only grows when a robot observes a cell
    some teammate already claimed, i.e. genuinely wasted, redundant
    physical exploration of ground someone else already covered. cell_index
    should be 4 (local_cell_series, self-observed only) -- redundancy in
    the fused/nav_map series (5) would count peer-relayed knowledge as
    "coverage" too, which was never independently (re)observed and isn't
    wasted effort."""
    cell_series_by_robot = {r: entry[cell_index] for r, entry in run.items()}
    merged_union = union_coverage_over_time(cell_series_by_robot.values())
    if not merged_union:
        return []
    grid = np.array([p[0] for p in merged_union])
    union_vals = np.array([p[1] for p in merged_union], dtype=float)
    total = np.zeros(len(grid))
    for series in cell_series_by_robot.values():
        total += resample_step(cumulative_count_series(series), grid)
    redundant_vals = total - union_vals
    return list(zip(grid.tolist(), redundant_vals.tolist()))


def plot_redundant_coverage(ax, results, conditions, cell_index, title):
    """Redundant physical coverage over time: cells more than one robot
    independently drove to and observed with its own sensors, i.e. wasted
    territory-overlap effort (see run_redundant_series). results[c] is a
    list of per-run {robot: entry} dicts -- each run's redundant series is
    resampled (resample_step) onto a shared time grid and averaged, with a
    +/- std band across runs when more than one is given."""
    for cond in conditions:
        runs = results[cond]
        redundant_per_run = [run_redundant_series(run, cell_index) for run in runs]
        max_ts = [m[-1][0] for m in redundant_per_run if m]
        if not max_ts:
            continue
        grid = np.linspace(0, min(max_ts), 200)
        sampled = np.array([resample_step(m, grid) for m in redundant_per_run if m])
        mean = sampled.mean(axis=0)
        ax.plot(grid, mean, color=CONDITION_COLORS[cond], linewidth=2, solid_capstyle="round", zorder=3)
        label = f"{mean[-1]:,.0f} cells"
        if sampled.shape[0] > 1:
            std = sampled.std(axis=0)
            ax.fill_between(grid, np.maximum(mean - std, 0), mean + std, color=CONDITION_COLORS[cond],
                             alpha=0.15, linewidth=0, zorder=2)
            label += f" ± {std[-1]:,.0f}"
        ax.annotate(label, (grid[-1], mean[-1]), textcoords="offset points",
                    xytext=(6, 0), fontsize=8.5, fontweight="bold", color=CONDITION_COLORS[cond], va="center")

    ax.set_xlabel("Time since run start (s)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_ylabel("Redundant tiles (seen by >1 robot)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_title(title, fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)

    handles = [Line2D([0], [0], color=CONDITION_COLORS[c], lw=2, label=DISPLAY_NAMES[c])
               for c in conditions if results[c]]
    if handles:
        ax.legend(handles=handles, frameon=False, fontsize=10, loc="lower right")


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
    excludes anything a robot only learned from a peer). results[c] is a
    list of per-run {robot: entry} dicts -- the union is computed per run,
    then those per-run curves are resampled (resample_step) onto a shared
    time grid and averaged, with a +/- std band when more than one run
    contributes."""
    for cond in conditions:
        runs = results[cond]
        merged_per_run = [union_coverage_over_time(entry[cell_index] for entry in run.values()) for run in runs]
        max_ts = [m[-1][0] for m in merged_per_run if m]
        if not max_ts:
            continue
        grid = np.linspace(0, min(max_ts), 200)
        sampled = np.array([resample_step(m, grid) for m in merged_per_run if m])
        mean = sampled.mean(axis=0)
        ax.plot(grid, mean, color=CONDITION_COLORS[cond], linewidth=2, solid_capstyle="round", zorder=3)
        label = f"{mean[-1]:,.0f} tiles"
        if sampled.shape[0] > 1:
            std = sampled.std(axis=0)
            ax.fill_between(grid, mean - std, mean + std, color=CONDITION_COLORS[cond], alpha=0.15,
                             linewidth=0, zorder=2)
            label += f" ± {std[-1]:,.0f}"
        ax.annotate(label, (grid[-1], mean[-1]), textcoords="offset points",
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
    parser.add_argument("--baseline", nargs="+", metavar="robot=bag_dir", action="append",
                         help="One run's robot=bag_dir set. Repeat --baseline for additional runs to average.")
    parser.add_argument("--vxch", nargs="+", metavar="robot=bag_dir", action="append",
                         help="One run's robot=bag_dir set. Repeat --vxch for additional runs to average.")
    parser.add_argument("--zstd", nargs="+", metavar="robot=bag_dir", action="append",
                         help="One run's robot=bag_dir set. Repeat --zstd for additional runs to average.")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--max-duration", type=float, default=None,
                         help="Clip each bag to this many seconds of bag time since its start.")
    parser.add_argument("--separate-figures", action="store_true",
                         help="Save each panel as its own image file (named <out stem>_<panel>.<out suffix>) "
                              "instead of one combined multi-panel image.")
    args = parser.parse_args()

    # Each condition's runs (from repeated --<condition> occurrences), each
    # a {robot: bag_dir} dict for one run.
    robot_paths = {}
    for condition in ALL_CONDITIONS:
        runs = getattr(args, condition)
        if runs:
            robot_paths[condition] = [parse_robot_paths(pairs) for pairs in runs]
    if len(robot_paths) < 2:
        print("error: need at least 2 of --baseline/--vxch/--zstd to compare", file=sys.stderr)
        sys.exit(1)
    conditions = tuple(c for c in ALL_CONDITIONS if c in robot_paths)

    # results[c] is a list of per-run {robot: read_bag() result} dicts, one
    # per --<condition> occurrence -- more than one run per condition drives
    # the mean +/- std error bars/bands in the plot_* functions below.
    results = {c: [] for c in conditions}
    for condition, runs in robot_paths.items():
        for run_robots in runs:
            run_results = {}
            for robot, bag_dir in sorted(run_robots.items()):
                r = read_bag(robot, bag_dir, condition, max_duration=args.max_duration)
                if r is not None:
                    run_results[robot] = r
            if run_results:
                results[condition].append(run_results)

    for condition in conditions:
        if not results[condition]:
            print(f"error: no readable bags for condition {condition!r}", file=sys.stderr)
            sys.exit(1)

    # (panel name, plot function, args) -- shared between the combined
    # multi-panel layout and --separate-figures' one-file-per-panel layout.
    panels = [
        ("sent", plot_bandwidth, (results, conditions, 1, "Map-sharing bandwidth (sent)", "Sent to peers (KB)")),
        ("received", plot_bandwidth, (results, conditions, 0, "Map-sharing bandwidth (received)", "Received from peers (KB)")),
        ("coverage", plot_coverage, (results, conditions, 2, "Communicated map coverage", "Known map area (m²)")),
        ("local", plot_coverage, (results, conditions, 3, "Locally-observed coverage (self only)", "Self-observed area (m²)")),
        ("union", plot_union_coverage, (results, conditions, 4, "Team physical coverage over time (union, self-observed only)")),
        ("union_nav", plot_union_coverage, (results, conditions, 5, "Team known coverage over time (union, incl. peer-relayed)")),
        ("redundant", plot_redundant_coverage, (results, conditions, 4, "Redundant physical coverage (territory overlap)")),
    ]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    if args.separate_figures:
        for name, plot_fn, plot_args in panels:
            panel_fig, ax = plt.subplots(figsize=(7, 5.5))
            plot_fn(ax, *plot_args)
            panel_fig.tight_layout()
            panel_out = args.out.with_name(f"{args.out.stem}_{name}{args.out.suffix}")
            panel_fig.savefig(panel_out, dpi=200, facecolor="white")
            plt.close(panel_fig)
    else:
        fig, axes = plt.subplots(1, len(panels), figsize=(43, 5.5))
        for ax, (name, plot_fn, plot_args) in zip(axes, panels):
            plot_fn(ax, *plot_args)
        fig.suptitle("Map sharing: " + " vs. ".join(DISPLAY_NAMES[c] for c in conditions), fontsize=15,
                     fontweight="bold", color=TEXT_PRIMARY, x=0.02, ha="left")
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(args.out, dpi=200, facecolor="white")

    # Per-run totals (summed across robots), then mean/std across runs.
    run_received = {c: [sum(r for r, _, _, _, _, _ in run.values()) for run in results[c]] for c in conditions}
    run_sent = {c: [sum(s for _, s, _, _, _, _ in run.values()) for run in results[c]] for c in conditions}
    totals_received = {c: float(np.mean(run_received[c])) for c in conditions}
    totals_sent = {c: float(np.mean(run_sent[c])) for c in conditions}
    for c in conditions:
        n = len(results[c])
        recv_std = f" ± {np.std(run_received[c]):.0f}" if n > 1 else ""
        sent_std = f" ± {np.std(run_sent[c]):.0f}" if n > 1 else ""
        print(f"{DISPLAY_NAMES[c]:>10} received: {totals_received[c]:.0f}{recv_std} bytes "
              f"({totals_received[c] / 1024:.1f} KB), sent: {totals_sent[c]:.0f}{sent_std} bytes "
              f"({totals_sent[c] / 1024:.1f} KB) across {n} run(s)")
    nonzero_received = {c: v for c, v in totals_received.items() if v > 0}
    if len(nonzero_received) >= 2:
        print(f"received ratio (max/min): {max(nonzero_received.values()) / min(nonzero_received.values()):.2f}x")
    for condition in conditions:
        robots = sorted({r for run in results[condition] for r in run})
        for robot in robots:
            finals = [run[robot][2][-1][1] for run in results[condition] if robot in run and run[robot][2]]
            finals_local = [run[robot][3][-1][1] for run in results[condition] if robot in run and run[robot][3]]
            final = np.mean(finals) if finals else 0.0
            final_local = np.mean(finals_local) if finals_local else 0.0
            std_suffix = f" ± {np.std(finals):.1f}" if len(finals) > 1 else ""
            std_local_suffix = f" ± {np.std(finals_local):.1f}" if len(finals_local) > 1 else ""
            print(f"{DISPLAY_NAMES[condition]} {robot}: final known map area {final:.1f}{std_suffix} m^2 "
                  f"(communicated, incl. peer-relayed cells), {final_local:.1f}{std_local_suffix} m^2 self-observed")
    for condition in conditions:
        local_finals = [union_coverage_over_time(entry[4] for entry in run.values()) for run in results[condition]]
        nav_finals = [union_coverage_over_time(entry[5] for entry in run.values()) for run in results[condition]]
        local_tiles = [m[-1][1] for m in local_finals if m]
        nav_tiles = [m[-1][1] for m in nav_finals if m]
        final_local_tiles = np.mean(local_tiles) if local_tiles else 0
        final_nav_tiles = np.mean(nav_tiles) if nav_tiles else 0
        local_suffix = f" ± {np.std(local_tiles):.0f}" if len(local_tiles) > 1 else ""
        nav_suffix = f" ± {np.std(nav_tiles):.0f}" if len(nav_tiles) > 1 else ""
        print(f"{DISPLAY_NAMES[condition]:>10} team physical coverage (union, self-observed only): "
              f"{final_local_tiles:,.0f}{local_suffix} tiles; "
              f"team known coverage (union, incl. peer-relayed): {final_nav_tiles:,.0f}{nav_suffix} tiles")
    for condition in conditions:
        local_finals = [union_coverage_over_time(entry[4] for entry in run.values()) for run in results[condition]]
        local_tiles = [m[-1][1] for m in local_finals if m]
        final_local_tiles = np.mean(local_tiles) if local_tiles else 0
        redundant_finals = [run_redundant_series(run, 4) for run in results[condition]]
        redundant_tiles = [m[-1][1] for m in redundant_finals if m]
        final_redundant = np.mean(redundant_tiles) if redundant_tiles else 0
        redundant_suffix = f" ± {np.std(redundant_tiles):.0f}" if len(redundant_tiles) > 1 else ""
        pct = f" ({100 * final_redundant / final_local_tiles:.1f}% of physical coverage)" if final_local_tiles else ""
        print(f"{DISPLAY_NAMES[condition]:>10} redundant physical coverage (territory overlap): "
              f"{final_redundant:,.0f}{redundant_suffix} tiles{pct}")
    print(str(args.out.resolve()))


if __name__ == "__main__":
    main()
