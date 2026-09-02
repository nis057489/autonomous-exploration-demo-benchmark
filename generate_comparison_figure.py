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
import csv
import io
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
    local_cell_series, nav_cell_series, resolution) where coverage and
    local_coverage are each a list of (seconds_since_start, known_area_m2) -- coverage
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
    show it, since that series never receives fused/peer-relayed cells).
    resolution is the grid resolution (m/cell) behind local_cell_series' and
    nav_cell_series' packed keys, for converting union "tile" counts back to
    m^2 downstream -- 0.0 if this robot never published a /map or /nav_map
    message with any known cells."""
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
    # First grid resolution actually seen on either /map or /nav_map for
    # this robot -- lets callers convert union "tile" counts (built from
    # known_cells' packed keys, one per cell at THIS resolution) into m^2,
    # so union/redundant coverage can be compared on the same units as
    # known_area_m2's m^2 figures instead of raw, resolution-dependent
    # tile counts. A single scalar, not per-message, because a mismatch
    # between /map's and /nav_map's resolution would silently break
    # known_cells' cross-message key comparability too -- if that ever
    # happens the tile counts themselves are already wrong, not just their
    # unit label.
    resolution_seen = []

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
        if not resolution_seen:
            resolution_seen.append(res)
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        xs = known_idx % width
        ys = known_idx // width
        gx = np.round((ox + (xs + 0.5) * res) / res).astype(np.int64)
        gy = np.round((oy + (ys + 0.5) * res) / res).astype(np.int64)
        keys = (gx << np.int64(32)) | (gy & np.int64(0xFFFFFFFF))
        # np.unique, not just sort: two cells of ONE message can land on the
        # same key when the grid's real row/column spacing disagrees with the
        # resolution it declares (seen on /nav_map -- consecutive rows
        # collapsing onto one world cell, ~70% of a message's cells duplicated).
        # setdiff1d(assume_unique=True) downstream has undefined behaviour on
        # a non-unique input, so leaving duplicates in silently corrupts every
        # diff built from this.
        return np.unique(keys)

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
    resolution = resolution_seen[0] if resolution_seen else 0.0
    return received_bytes, sent_bytes, coverage, local_coverage, local_cell_series, nav_cell_series, resolution


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


def run_resolution(run):
    """Grid resolution (m/cell) for one run's tile counts -- the first
    nonzero resolution reported by any robot's read_bag entry (index 6; see
    read_bag's docstring). All robots share one SLAM config so these should
    agree; 0.0 (tile counts left unconverted to area) if no robot in the
    run reported one."""
    return next((entry[6] for entry in run.values() if entry[6]), 0.0)


def to_area_m2(series, resolution):
    """Rescale a (t, tile_count) series (union_coverage_over_time or
    run_redundant_series output) into (t, area_m2), so it's on the same
    units as known_area_m2's coverage/local_coverage figures instead of a
    raw, resolution-dependent tile count."""
    return [(t, v * resolution ** 2) for t, v in series]


def local_physical_union_series(run):
    """Team physical coverage for one run, in m^2: a true keyed union of
    every robot's self-observed cells, so ground two robots both drove over
    counts once. Trustworthy because /map's cells key cleanly -- summing
    local_cell_series' per-message diffs lands within ~2% of the final
    /map snapshot's own known_area_m2, i.e. the keys really are 1:1 with
    distinct grid cells (unlike /nav_map's, see team_known_coverage_series)."""
    return to_area_m2(union_coverage_over_time(entry[4] for entry in run.values()), run_resolution(run))


def team_known_coverage_series(run):
    """Team-wide known coverage for one run, in m^2, as the MAX over robots
    of each robot's own /nav_map known_area_m2 -- deliberately NOT a keyed
    union across robots like local_physical_union_series.

    /nav_map's cells cannot be keyed to world positions reliably: within a
    single message, consecutive rows collapse onto the same computed world
    cell (~70% of a message's cells are duplicates of another cell in that
    same message), because the grid's real row spacing disagrees with the
    resolution it advertises. That aliasing is lossy and irreversible here
    -- two genuinely different patches of ground arrive sharing one key, so
    unioning by key undercounts by ~4x (67.6 m^2 built from the cell series
    vs. 263.8 m^2 in that same robot's own final /nav_map snapshot). The
    real fix belongs upstream in per_robot_map_compositor.py, which builds
    /nav_map; this is the best the plotting side can do until then.

    known_area_m2 just counts non-unknown cells in one message and never
    derives world positions, so it is immune to that aliasing. Taking the
    max across robots (rather than a sum) keeps the number honest: fusion
    is supposed to converge every robot's nav_map onto the same team-wide
    picture, so the best-informed robot's own map is a sound stand-in for
    what the team collectively knows, and no cell is ever counted twice."""
    series_by_robot = [entry[2] for entry in run.values() if entry[2]]
    if not series_by_robot:
        return []
    max_t = max(s[-1][0] for s in series_by_robot)
    grid = np.linspace(0, max_t, 200)
    sampled = np.array([resample_step(s, grid) for s in series_by_robot])
    return list(zip(grid.tolist(), sampled.max(axis=0).tolist()))


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
        redundant_per_run = [to_area_m2(run_redundant_series(run, cell_index), run_resolution(run)) for run in runs]
        max_ts = [m[-1][0] for m in redundant_per_run if m]
        if not max_ts:
            continue
        grid = np.linspace(0, min(max_ts), 200)
        sampled = np.array([resample_step(m, grid) for m in redundant_per_run if m])
        mean = sampled.mean(axis=0)
        ax.plot(grid, mean, color=CONDITION_COLORS[cond], linewidth=2, solid_capstyle="round", zorder=3)
        label = f"{mean[-1]:,.1f} m²"
        if sampled.shape[0] > 1:
            std = sampled.std(axis=0)
            ax.fill_between(grid, np.maximum(mean - std, 0), mean + std, color=CONDITION_COLORS[cond],
                             alpha=0.15, linewidth=0, zorder=2)
            label += f" ± {std[-1]:,.1f}"
        ax.annotate(label, (grid[-1], mean[-1]), textcoords="offset points",
                    xytext=(6, 0), fontsize=8.5, fontweight="bold", color=CONDITION_COLORS[cond], va="center")

    ax.set_xlabel("Time since run start (s)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_ylabel("Redundant coverage (m², seen by >1 robot)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_title(title, fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)

    handles = [Line2D([0], [0], color=CONDITION_COLORS[c], lw=2, label=DISPLAY_NAMES[c])
               for c in conditions if results[c]]
    if handles:
        ax.legend(handles=handles, frameon=False, fontsize=10, loc="lower right")


def summarize(results, conditions):
    """Per-condition end-of-run numbers, computed once and shared by every
    output format (the per-condition lines below and --table's renderers,
    which used to recompute the same unions separately). Each entry is
    {metric: (mean, std)} across that condition's runs, std 0.0 for a single
    run, plus "runs". Bytes are raw bytes, areas m^2, percentages relative
    to that condition's own physical coverage."""
    def mean_std(values):
        if not values:
            return 0.0, 0.0
        return float(np.mean(values)), float(np.std(values))

    summary = {}
    for cond in conditions:
        runs = results[cond]
        physical = [m[-1][1] for m in (local_physical_union_series(run) for run in runs) if m]
        known = [m[-1][1] for m in (team_known_coverage_series(run) for run in runs) if m]
        redundant = [m[-1][1] for m in
                     (to_area_m2(run_redundant_series(run, 4), run_resolution(run)) for run in runs) if m]
        physical_mean, physical_std = mean_std(physical)
        known_mean, known_std = mean_std(known)
        redundant_mean, redundant_std = mean_std(redundant)
        summary[cond] = {
            "runs": len(runs),
            "received": mean_std([sum(entry[0] for entry in run.values()) for run in runs]),
            "sent": mean_std([sum(entry[1] for entry in run.values()) for run in runs]),
            "physical": (physical_mean, physical_std),
            "known": (known_mean, known_std),
            "redundant": (redundant_mean, redundant_std),
            # Share of what the team physically covered that reached the
            # rest of the team, and share that was re-covered ground -- both
            # meaningless without a denominator, hence 0.0 when there is none.
            "propagated_pct": (100 * known_mean / physical_mean if physical_mean else 0.0, 0.0),
            "redundant_pct": (100 * redundant_mean / physical_mean if physical_mean else 0.0, 0.0),
        }
    return summary


# (metric key, display header, csv column name, scale applied to the stored
# value, decimal places). One spec drives all three --table styles so they
# can't drift apart. The csv name carries the unit, since a spreadsheet
# column has no header row to explain itself the way the display table does.
SUMMARY_COLUMNS = (
    ("runs", "Runs", "runs", 1.0, 0),
    ("sent", "Sent (KB)", "sent_kb", 1 / 1024, 1),
    ("received", "Received (KB)", "received_kb", 1 / 1024, 1),
    ("physical", "Physical (m²)", "physical_m2", 1.0, 1),
    ("known", "Known (m²)", "known_m2", 1.0, 1),
    ("propagated_pct", "Propagated (%)", "propagated_pct", 1.0, 1),
    ("redundant", "Redundant (m²)", "redundant_m2", 1.0, 1),
    ("redundant_pct", "Redundant (%)", "redundant_pct", 1.0, 1),
)


def format_summary_table(summary, conditions, style):
    """Render summarize()'s numbers as a table. style is "text" (aligned
    columns for a terminal), "markdown" (a pipe table to paste into notes or
    a PR), or "csv" (a spreadsheet-ready flat form: every mean gets its own
    column plus a <name>_std sibling, so nothing has to parse a ± back out
    of a cell). text and markdown fold std into the cell as "mean ± std",
    and omit it where a condition has only one run or the metric is a ratio
    of means with no spread of its own."""
    def cell(cond, key, scale, places):
        if key == "runs":
            return f"{summary[cond]['runs']}"
        mean, std = summary[cond][key]
        text = f"{mean * scale:,.{places}f}"
        if std and summary[cond]["runs"] > 1:
            text += f" ± {std * scale:,.{places}f}"
        return text

    if style == "csv":
        buf = io.StringIO()
        writer = csv.writer(buf)
        header = ["condition"]
        for _, _, csv_name, _, _ in SUMMARY_COLUMNS:
            header.append(csv_name)
            if csv_name != "runs":
                header.append(f"{csv_name}_std")
        writer.writerow(header)
        for cond in conditions:
            row = [DISPLAY_NAMES[cond]]
            for key, _, _, scale, places in SUMMARY_COLUMNS:
                if key == "runs":
                    row.append(summary[cond]["runs"])
                    continue
                mean, std = summary[cond][key]
                row.append(f"{mean * scale:.{places}f}")
                row.append(f"{std * scale:.{places}f}")
            writer.writerow(row)
        return buf.getvalue().rstrip("\n")

    headers = ["Condition"] + [h for _, h, _, _, _ in SUMMARY_COLUMNS]
    rows = [[DISPLAY_NAMES[c]] + [cell(c, k, s, p) for k, _, _, s, p in SUMMARY_COLUMNS] for c in conditions]
    widths = [max(len(str(r[i])) for r in [headers] + rows) for i in range(len(headers))]

    def line(values, pad=" "):
        # First column left-aligned (labels), numbers right-aligned.
        cells = [str(v).ljust(widths[0], pad) if i == 0 else str(v).rjust(widths[i], pad)
                 for i, v in enumerate(values)]
        return ("| " + " | ".join(cells) + " |") if style == "markdown" else "  ".join(cells).rstrip()

    if style == "markdown":
        rule = "| " + " | ".join(
            (":" + "-" * (w - 1)) if i == 0 else ("-" * (w - 1) + ":") for i, w in enumerate(widths)) + " |"
        return "\n".join([line(headers), rule] + [line(r) for r in rows])
    return "\n".join([line(headers), "  ".join("-" * w for w in widths)] + [line(r) for r in rows])


def plot_union_coverage(ax, results, conditions, series_fn, title, ylabel):
    """Team-wide map coverage over time, in m^2. series_fn turns one run's
    {robot: entry} dict into that run's (t, area_m2) curve -- pass
    local_physical_union_series for "how much ground has the team
    physically laid eyes on" (a real keyed union across robots, so a patch
    two robots both drove over counts once), or team_known_coverage_series
    for "how much does the team collectively know", including peer-relayed
    cells -- the one that moves when communication improves, since
    self-observed coverage structurally can't show it. results[c] is a list
    of per-run {robot: entry} dicts -- each run's curve is resampled
    (resample_step) onto a shared time grid and averaged, with a +/- std
    band when more than one run contributes."""
    for cond in conditions:
        runs = results[cond]
        merged_per_run = [series_fn(run) for run in runs]
        max_ts = [m[-1][0] for m in merged_per_run if m]
        if not max_ts:
            continue
        grid = np.linspace(0, min(max_ts), 200)
        sampled = np.array([resample_step(m, grid) for m in merged_per_run if m])
        mean = sampled.mean(axis=0)
        ax.plot(grid, mean, color=CONDITION_COLORS[cond], linewidth=2, solid_capstyle="round", zorder=3)
        label = f"{mean[-1]:,.1f} m²"
        if sampled.shape[0] > 1:
            std = sampled.std(axis=0)
            ax.fill_between(grid, mean - std, mean + std, color=CONDITION_COLORS[cond], alpha=0.15,
                             linewidth=0, zorder=2)
            label += f" ± {std[-1]:,.1f}"
        ax.annotate(label, (grid[-1], mean[-1]), textcoords="offset points",
                    xytext=(6, 0), fontsize=8.5, fontweight="bold", color=CONDITION_COLORS[cond], va="center")

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
    parser.add_argument("--table", nargs="?", const="text", choices=("text", "markdown", "csv"),
                         help="Also print a one-row-per-condition summary table (sent/received bandwidth, "
                              "team physical and known coverage, redundant overlap). Default style 'text'; "
                              "'markdown' to paste into notes, 'csv' for a spreadsheet.")
    parser.add_argument("--table-file", type=Path,
                         help="Also write the summary table to this path, in the style its extension implies "
                              "(.csv, .md, else text) regardless of --table. Use it to keep stdout pipe-clean.")
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
                # A bag can open fine and still hold no map messages at all
                # (an aborted recording -- a few tens of KB, no /map, no
                # /nav_map). Its coverage series drop out of the averages on
                # their own, but its zero bytes do NOT: they average in and
                # silently halve the condition's reported bandwidth. Say so
                # rather than let a dead run quietly move every number.
                if not any(entry[2] or entry[3] for entry in run_results.values()):
                    paths = ", ".join(str(p) for p in sorted(run_robots.values()))
                    print(f"warning: {condition} run has no /map or /nav_map messages, so it contributes "
                          f"0 bytes to that condition's mean -- drop it or check the recording: {paths}",
                          file=sys.stderr)
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
        ("union", plot_union_coverage, (results, conditions, local_physical_union_series,
                                        "Team physical coverage over time (union, self-observed only)",
                                        "Known area (m², union across robots)")),
        ("union_nav", plot_union_coverage, (results, conditions, team_known_coverage_series,
                                            "Team known coverage over time (incl. peer-relayed)",
                                            "Known area (m², best-informed robot)")),
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

    # Per-condition totals, computed once for both the lines below and --table.
    summary = summarize(results, conditions)
    for c in conditions:
        n = summary[c]["runs"]
        recv_mean, recv_std_val = summary[c]["received"]
        sent_mean, sent_std_val = summary[c]["sent"]
        recv_std = f" ± {recv_std_val:.0f}" if n > 1 else ""
        sent_std = f" ± {sent_std_val:.0f}" if n > 1 else ""
        print(f"{DISPLAY_NAMES[c]:>10} received: {recv_mean:.0f}{recv_std} bytes "
              f"({recv_mean / 1024:.1f} KB), sent: {sent_mean:.0f}{sent_std} bytes "
              f"({sent_mean / 1024:.1f} KB) across {n} run(s)")
    nonzero_received = {c: summary[c]["received"][0] for c in conditions if summary[c]["received"][0] > 0}
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
        n = summary[condition]["runs"]
        physical_mean, physical_std = summary[condition]["physical"]
        known_mean, known_std = summary[condition]["known"]
        local_suffix = f" ± {physical_std:.1f}" if n > 1 else ""
        nav_suffix = f" ± {known_std:.1f}" if n > 1 else ""
        print(f"{DISPLAY_NAMES[condition]:>10} team physical coverage (union, self-observed only): "
              f"{physical_mean:,.1f}{local_suffix} m²; "
              f"team known coverage (best-informed robot, incl. peer-relayed): "
              f"{known_mean:,.1f}{nav_suffix} m²")
    for condition in conditions:
        n = summary[condition]["runs"]
        redundant_mean, redundant_std = summary[condition]["redundant"]
        redundant_suffix = f" ± {redundant_std:.1f}" if n > 1 else ""
        pct_val = summary[condition]["redundant_pct"][0]
        pct = f" ({pct_val:.1f}% of physical coverage)" if summary[condition]["physical"][0] else ""
        print(f"{DISPLAY_NAMES[condition]:>10} redundant physical coverage (territory overlap): "
              f"{redundant_mean:,.1f}{redundant_suffix} m²{pct}")

    if args.table or args.table_file:
        if args.table_file:
            # The file's own extension picks its style, not --table (which
            # only governs stdout) -- writing a markdown pipe table into a
            # .csv because of an unrelated stdout flag is never what's meant.
            style = {".csv": "csv", ".md": "markdown", ".markdown": "markdown"}.get(
                args.table_file.suffix.lower(), "text")
            args.table_file.parent.mkdir(parents=True, exist_ok=True)
            args.table_file.write_text(format_summary_table(summary, conditions, style) + "\n")
        if args.table:
            print()
            print(format_summary_table(summary, conditions, args.table))
        if args.table_file:
            print(str(args.table_file.resolve()))

    print(str(args.out.resolve()))


if __name__ == "__main__":
    main()
