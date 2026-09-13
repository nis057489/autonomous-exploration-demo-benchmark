#!/usr/bin/env python3
"""Produce a scientific comparison figure across map-sharing conditions --
baseline, vxch (Wavestream), and/or zstd -- pass any 2 or all 3: how much
data was exchanged between robots to share the map, and how much of the map
got explored, for each condition given.

Reads each robot's recorded rosbag2 (mcap) directly:
  - Coordination traffic: published /<robot>/explore/reservation bytes are
    included in sent totals for sharing arms; received reservation bytes are
    included through the /incoming/ topics. These use the same DDIL budget.
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
  - peer-relayed coverage: derived, not a topic -- the part of each robot's
    /nav_map that a peer observed and the robot's own /map never did (see
    peer_derived_series). This is the only coverage figure communication is
    responsible for: the team-wide union of /nav_map is by construction the
    same set of cells as the union of every /map, in every condition, since
    a robot's own observations always reach its own nav_map.
  - locally-observed coverage: /<robot>/map (nav_msgs/OccupancyGrid), same
    known-cell-area calculation, but this topic is slam_toolbox's raw local
    output -- it never receives peer-communicated cells (those only ever
    land in team_map_ddil, which per team_map_fusion.py excludes the
    robot's own map). These sources may overlap spatially; /map measures
    this robot's own sensor observations, including occupied cells.

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

Reading a bag is the entire cost here (minutes per multi-GB run); the plots
themselves are pure functions of what comes out of it. So each bag is decoded
once into figure_cache/ (--cache-dir, --no-cache) and re-plotting the same runs
never opens it again, and every number a figure draws is written beside the
image as <out stem>.json (--data-out, --no-data-out) so the figure's data can
be read or re-plotted without the bags at all.
"""
import argparse
import csv
import hashlib
import io
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import yaml

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

TEXT_PRIMARY = "#1a1a1a"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#cccccc"
ALL_CONDITIONS = ("none", "baseline", "vxch", "zstd", "oracle")  # internal keys -- match run-dir/CLI naming, unrelated to display
DISPLAY_NAMES = {"baseline": "Baseline", "vxch": "Wavestream", "zstd": "Zstd",
                 "none": "No sharing", "oracle": "Unimpaired sharing"}
# Control arms change communication, not the exploration policy.
CONDITION_COLORS = {"baseline": "#eb6834", "vxch": "#2a78d6", "zstd": "#3fa15c",
                    "none": "#8a8a8a", "oracle": "#d4a017"}
LINESTYLES = ("-", "--", ":", "-.")

# Publication type scale: 1.5x the sizes these panels used when they were
# screen-sized, so a panel still reads at journal column width after the whole
# figure is scaled down. vxch_visual_test/compression_size_sweep.py carries the
# same scale (same numbers, one panel) so the two figures typeset alike.
FS_AXIS = 16.5      # axis labels, categorical tick labels
FS_TICK = 15        # numeric tick labels
FS_LEGEND = 15      # legend entries
FS_VALUE = 15       # end-of-series / on-bar value labels
FS_INLINE = 13      # labels drawn inside a bar segment
# Each bar prints its value just past its end; without headroom those labels
# run off the axes. The bandwidth axis is logarithmic and its span depends on
# the run, so the headroom is a fraction of the span in decades rather than a
# fixed multiplier -- this much of the axis width is left blank for labels.
BAR_LABEL_DECADES = 0.25
# One panel's figure size, used both for --separate-figures and for each cell
# of the combined grid, so a panel's type is the same physical size either way.
PANEL_SIZE = (7.6, 6.0)


def parse_robot_paths(pairs):
    out = {}
    for pair in pairs:
        robot, sep, path = pair.partition("=")
        if not sep or not robot or not path:
            raise ValueError(f"expected robot=bag_dir, got {pair!r}")
        out[robot] = Path(path)
    return out


def apply_pose(px, py, pyaw, x, y):
    """Compose a 2D pose (px, py, pyaw) with a point (x, y) expressed in that
    pose's frame. Same math as team_map_fusion.py's _apply_pose -- kept
    identical on purpose, since the whole point is to key cells into the very
    frame fusion builds /nav_map in."""
    cos_yaw = math.cos(pyaw)
    sin_yaw = math.sin(pyaw)
    return px + cos_yaw * x - sin_yaw * y, py + sin_yaw * x + cos_yaw * y


def yaw_from_quaternion(q):
    """Yaw only -- these are 2D grid origins, so roll/pitch are always 0."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def parse_robot_offsets(pairs):
    """--robot-offset robot1=X,Y,YAW pairs into {robot: (x, y, yaw)}."""
    out = {}
    for pair in pairs or ():
        robot, sep, values = pair.partition("=")
        if not sep or not robot:
            raise ValueError(f"expected robot=X,Y,YAW, got {pair!r}")
        parts = [p for p in values.split(",") if p != ""]
        if len(parts) != 3:
            raise ValueError(f"expected robot=X,Y,YAW (3 numbers), got {pair!r}")
        out[robot] = tuple(float(p) for p in parts)
    return out


def default_spawn_preset():
    """Which spawn_presets.yaml preset the runs were launched with. Read from
    the environment, else experiment.conf (the file docker.sh sources, so it's
    the same value the launch actually used), else 'default'. Hardcoding
    'distributed' here would silently produce wrong offsets -- and therefore a
    wrong coverage union -- for any run launched with a different preset."""
    env = os.environ.get("SPAWN_PRESET")
    if env:
        return env
    conf = Path(__file__).parent / "experiment.conf"
    if conf.is_file():
        for line in conf.read_text(encoding="utf-8").splitlines():
            m = re.match(r"\s*SPAWN_PRESET\s*=\s*([^\s#]+)", line)
            if m:
                return m.group(1)
    return "default"


# Built from ALL_CONDITIONS rather than a hand-written alternation: a
# condition added there but forgotten here silently fails to match its run
# dirs, so world_from_bag_dir returns None, spawn_offsets_for returns {}, and
# every robot's /map stays in its own SLAM frame -- which inflates team
# physical coverage to roughly the SUM of the robots' maps and collapses
# redundant coverage to ~0 for that condition alone. That is exactly what
# happened to `none`/`oracle` when they were added.
_RUN_DIR_RE = re.compile(
    r"^\d{8}_\d{6}_(?:" + "|".join(re.escape(c) for c in ALL_CONDITIONS) + r")_(.+)$")


def world_from_bag_dir(bag_dir):
    """Run dirs are experiment_runs/<timestamp>_<condition>_<world>/bag, so the
    world name (which selects a spawn_presets.yaml block) is recoverable from
    the path. Returns None if the path doesn't follow that shape."""
    for part in (Path(bag_dir).resolve()).parts[::-1]:
        m = re.match(_RUN_DIR_RE, part)
        if m:
            return m.group(1)
    return None


def spawn_offsets_for(world, preset, robots, presets_path=None):
    """Per-robot spawn poses (x, y, yaw) from spawn_presets.yaml -- the same
    source multi_robot_vxch_experiment.launch.py feeds to team_map_fusion as
    offsets_x/offsets_y/offsets_yaw. Robot N takes positions[N-1], cycling if
    there are more robots than listed positions (matching that file's own
    documented rule). Returns {} if the world/preset isn't found."""
    path = Path(presets_path or (Path(__file__).parent / "spawn_presets.yaml"))
    if not path.is_file() or not world:
        return {}
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}
    positions = (data.get(world) or {}).get(preset)
    if not positions:
        return {}
    out = {}
    for robot in robots:
        m = re.search(r"(\d+)$", robot)
        if not m:
            continue
        pos = positions[(int(m.group(1)) - 1) % len(positions)]
        out[robot] = (float(pos["x"]), float(pos["y"]), float(pos.get("yaw", 0.0)))
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


def known_cells(msg, offset=None):
    """Grid-cell coordinates keyed in a shared, resolution-sized world
    lattice (not this message's local row/col indices) so that cells from
    different robots -- whose grids have different origins/extents as
    slam_toolbox grows each robot's own map independently -- line up and
    can be unioned.

    `offset` is this robot's spawn pose (x, y, yaw), i.e. the pose of its
    private SLAM `map` frame within the shared team frame, and MUST be
    supplied for any per-robot topic (/map). Every robot runs its own
    async_slam_toolbox_node anchored at its own start pose, so /map
    coordinates are in that robot's private frame -- in long_t/distributed
    the two robots spawn 16 m apart, so without this the same physical
    ground keys 16 m apart, two robots' cells never dedupe, and the
    "union" inflates to roughly the sum (it read 505 m^2 for a world whose
    entire navigable floor is ~260 m^2). This is the exact transform chain
    team_map_fusion.py applies when it builds /nav_map: cell centre ->
    through the grid's own origin pose -> through the robot's spawn
    offset -> shared team frame.

    Pass offset=None only for a topic already published in the shared
    frame (/nav_map, which fusion emits with an identical origin for every
    robot); offsetting those again would move correct data off-frame.

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

    # Cell centres in the grid's own pixel space...
    local_x = (xs + 0.5) * res
    local_y = (ys + 0.5) * res
    # ...through the grid's origin pose (yaw included: slam_toolbox's
    # origin is normally axis-aligned, but nothing guarantees it, and
    # fusion honours it, so honour it here too rather than assuming)...
    origin_yaw = yaw_from_quaternion(msg.info.origin.orientation)
    map_x, map_y = apply_pose(ox, oy, origin_yaw, local_x, local_y)
    # ...and then through this robot's spawn offset into the team frame.
    if offset is not None:
        map_x, map_y = apply_pose(offset[0], offset[1], offset[2], map_x, map_y)

    gx = np.floor(map_x / res).astype(np.int64)
    gy = np.floor(map_y / res).astype(np.int64)
    keys = (gx << np.int64(32)) | (gy & np.int64(0xFFFFFFFF))
    # Containing-cell indices use floor. Rounding centres uses ties-to-even:
    # 1.5 and 2.5 both become 2, collapsing adjacent rows/columns on a
    # perfectly valid aligned grid. Rotation may still merge centre samples.
    return np.unique(keys)


def _read_bag_uncached(robot, bag_dir, condition, max_duration=None, map_offset=None):
    """Returns (received_bytes, sent_bytes, coverage, local_coverage,
    local_cell_series, nav_cell_series, resolution, link_stats) where coverage and
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
    it includes peer-relayed cells. Communication also changes exploration
    decisions, so local coverage may differ between independent runs.
    resolution is the grid resolution (m/cell) behind local_cell_series' and
    nav_cell_series' packed keys, for converting union "tile" counts back to
    m^2 downstream -- 0.0 if this robot never published a /map or /nav_map
    message with any known cells.

    link_stats is {peer: {"bandwidth"|"send_rate"|"queued"|"shed": [(t, value)]}}
    read off /ddil_proxy_<robot>_from_<peer>/ddil_stats -- the capacity actually
    applied to each of this robot's downlinks over time (which a LINK_PROFILE
    schedule varies during the run) together with the transport's response to
    it. Empty for bags recorded before that topic was bagged, or when
    voxelcodec_msgs is not importable.

    map_offset is this robot's spawn pose (x, y, yaw) -- the pose of its
    private SLAM `map` frame within the shared team frame -- and is required
    for local_cell_series' keys to be comparable across robots at all; see
    known_cells. None keeps every robot in its own frame, which is only
    correct for a single-robot run."""
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import OccupancyGrid

    try:
        from voxelcodec_msgs.msg import DdilStats
    except ImportError:  # workspace not sourced, or a bag predating ddil_stats recording
        DdilStats = None

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
    # /ddil_proxy_{robot}_from_{peer}/ddil_stats -- this robot's own downlink
    # from each peer, i.e. the links whose capacity a LINK_PROFILE schedule
    # drives and whose backlog/shedding is the response to it.
    ddil_stats_re = re.compile(
        rf"^/ddil_proxy_{re.escape(robot)}_from_(?P<peer>[^/]+)/ddil_stats$")
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


    local_cell_series = []
    nav_cell_series = []
    link_stats = {}
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
        elif topic == f"/{robot}/explore/reservation" and condition != "none":
            sent_bytes += len(data)
        elif topic == nav_map_topic:
            msg = deserialize_message(data, OccupancyGrid)
            t = (t_ns - start_ns) / 1e9
            if not resolution_seen:
                resolution_seen.append(msg.info.resolution)
            elif not math.isclose(resolution_seen[0], msg.info.resolution):
                raise ValueError("Mixed map resolutions cannot share coverage cell keys")
            coverage.append((t, known_area_m2(msg)))
            # No offset: /nav_map is fusion's output, already in the shared
            # team frame (every robot publishes it with the same origin).
            cells_now = known_cells(msg)
            new_cells = np.setdiff1d(cells_now, seen_nav_cells, assume_unique=True)
            if new_cells.size:
                nav_cell_series.append((t, new_cells))
            seen_nav_cells = np.union1d(seen_nav_cells, cells_now)
        elif topic == local_map_topic:
            msg = deserialize_message(data, OccupancyGrid)
            t = (t_ns - start_ns) / 1e9
            if not resolution_seen:
                resolution_seen.append(msg.info.resolution)
            elif not math.isclose(resolution_seen[0], msg.info.resolution):
                raise ValueError("Mixed map resolutions cannot share coverage cell keys")
            local_coverage.append((t, known_area_m2(msg)))
            # Store only the cells newly known since this robot's last /map
            # message, not a full snapshot every time: a full-set snapshot per
            # message would retain O(cells * messages) instead of O(final cell
            # count) and OOM on longer runs.
            #
            # seen_cells must be a running UNION of everything this robot has
            # ever reported, not just the previous message's cells. An
            # occupancy grid is cumulative in its OWN frame, but known_cells
            # keys by world position, and slam_toolbox is a pose-graph SLAM:
            # every loop closure re-optimises the graph and shifts where the
            # map believes the ground is, so the same physical cell re-keys
            # and setdiff1d re-emits the entire map as "new". Diffing against
            # only the previous message therefore leaks a full map's worth of
            # cells per re-anchoring -- measured at 17x on one robot of
            # 20260910_040102_baseline_office (3.42M cells emitted, 200k
            # distinct), while a robot that happened not to close a loop came
            # out at 1.1x. union_coverage_over_time survives that because a
            # set dedupes the re-emissions, but cumulative_count_series sums
            # diff sizes and cannot, which inflated run_redundant_series past
            # its own (n_robots - 1) * union ceiling.
            # Offset applied: /map is this robot's private SLAM frame.
            cells_now = known_cells(msg, offset=map_offset)
            new_cells = np.setdiff1d(cells_now, seen_cells, assume_unique=True)
            if new_cells.size:
                local_cell_series.append((t, new_cells))
            # union1d returns sorted-unique, which is what the
            # assume_unique=True setdiff1d above needs next iteration.
            seen_cells = np.union1d(seen_cells, cells_now)
            # oracle relays the same raw OccupancyGrid as baseline, just
            # unimpaired, so its own-published bytes are counted identically.
            # `none` matches no branch here and correctly reports 0 sent.
            if condition in ("baseline", "oracle"):
                sent_bytes += len(data)
        elif condition == "vxch" and vxch_own_re.match(topic):
            sent_bytes += len(data)
        elif condition == "zstd" and topic == zstd_own_topic:
            sent_bytes += len(data)
        elif DdilStats is not None and ddil_stats_re.match(topic):
            peer = ddil_stats_re.match(topic).group("peer")
            msg = deserialize_message(data, DdilStats)
            t = (t_ns - start_ns) / 1e9
            link = link_stats.setdefault(
                peer, {"bandwidth": [], "send_rate": [], "queued": [], "shed": []})
            # bandwidth_kbps is the capacity ACTUALLY in force at this instant,
            # read back from the proxy rather than from the schedule file --
            # so a link the scheduler failed to reach shows up here as a flat
            # line instead of silently being assumed to have followed along.
            link["bandwidth"].append((t, float(msg.bandwidth_kbps)))
            link["send_rate"].append((t, float(msg.send_rate_bps)))
            link["queued"].append((t, float(msg.queued_bytes)))
            link["shed"].append((t, float(msg.shed_bytes)))

    del reader
    if scratch_dir is not None:
        shutil.rmtree(scratch_dir, ignore_errors=True)

    coverage.sort(key=lambda p: p[0])
    local_coverage.sort(key=lambda p: p[0])
    local_cell_series.sort(key=lambda p: p[0])
    nav_cell_series.sort(key=lambda p: p[0])
    for link in link_stats.values():
        for series in link.values():
            series.sort(key=lambda p: p[0])
    resolution = resolution_seen[0] if resolution_seen else 0.0
    # link_stats appended LAST so every existing positional index into this
    # tuple (see plot_bandwidth's byte_index / plot_coverage's series_index)
    # keeps meaning what it did.
    return (received_bytes, sent_bytes, coverage, local_coverage,
            local_cell_series, nav_cell_series, resolution, link_stats)


# Bumped whenever the shape or meaning of what _read_bag_uncached returns
# changes -- a stale entry written by an older version would otherwise be
# unpacked into the new code's positions and silently plot wrong numbers.
CACHE_VERSION = 1
DEFAULT_CACHE_DIR = Path(__file__).resolve().parent / "figure_cache"


def _bag_fingerprint(bag_dir):
    """Identity of a bag's contents: every file's name, size and mtime.
    Bags are written once and never appended to, so this is really just a
    cheap "is this the same recording" check -- but including size/mtime
    means a re-recorded or repaired bag at the same path invalidates its
    cache entry instead of serving numbers from the old recording."""
    parts = []
    for f in sorted(bag_dir.iterdir()) if bag_dir.is_dir() else []:
        try:
            st = f.stat()
        except OSError:
            continue
        # A list, not a tuple: this goes into the cache entry's own JSON and is
        # compared against what comes back out of it, where a tuple would have
        # become a list and never matched again.
        parts.append([f.name, st.st_size, st.st_mtime_ns])
    return parts


def _cache_key(robot, bag_dir, condition, max_duration, map_offset):
    payload = {
        "version": CACHE_VERSION,
        "robot": robot,
        "bag": str(Path(bag_dir).resolve()),
        "condition": condition,
        "max_duration": max_duration,
        # Rounded: offsets come from spawn_presets.yaml as short decimals, and
        # float noise in the last bits must not spawn a second cache entry for
        # what is the same spawn pose.
        "offset": None if map_offset is None else [round(float(v), 9) for v in map_offset],
        "files": _bag_fingerprint(Path(bag_dir)),
    }
    digest = hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()[:16]
    return f"{robot}_{condition}_{digest}", payload


def _pack_cell_series(series):
    """Ragged [(t, int64 array)] -> three flat arrays npz can hold: the
    times, each entry's length, and every cell key concatenated. Storing one
    npz member per entry instead would mean thousands of members per bag."""
    times = np.array([t for t, _ in series], dtype=np.float64)
    lengths = np.array([cells.size for _, cells in series], dtype=np.int64)
    if series:
        cells = np.concatenate([c for _, c in series]).astype(np.int64)
    else:
        cells = np.empty(0, dtype=np.int64)
    return times, lengths, cells


def _unpack_cell_series(times, lengths, cells):
    chunks = np.split(cells, np.cumsum(lengths)[:-1]) if lengths.size else []
    return [(float(t), chunk) for t, chunk in zip(times, chunks)]


def _cache_paths(cache_dir, name):
    """Two files per entry, on purpose. The .json holds everything a human
    (or a plotting script with no numpy) would want to read back -- byte
    totals, the coverage time series, the link stats -- so the numbers behind
    a figure stay inspectable without the bag. The .npz holds only the packed
    cell-key arrays, which are hundreds of thousands of int64 per robot and
    would be both enormous and slow as JSON text. A hit needs both."""
    return cache_dir / f"{name}.json", cache_dir / f"{name}.npz"


def _cache_load(cache_dir, name, key_payload):
    json_path, npz_path = _cache_paths(cache_dir, name)
    if not (json_path.is_file() and npz_path.is_file()):
        return None
    try:
        meta = json.loads(json_path.read_text())
        # The digest already covers this, but a hash collision or a
        # hand-edited file would otherwise be unpackable into the wrong run's
        # numbers -- compare the key itself, not just its name.
        if meta.get("key") != key_payload:
            return None
        with np.load(npz_path) as z:
            local_cell_series = _unpack_cell_series(
                z["local_times"], z["local_lengths"], z["local_cells"])
            nav_cell_series = _unpack_cell_series(
                z["nav_times"], z["nav_lengths"], z["nav_cells"])
    except (OSError, ValueError, KeyError, json.JSONDecodeError):
        return None
    data = meta["data"]
    link_stats = {
        peer: {k: [(float(t), float(v)) for t, v in series] for k, series in per_peer.items()}
        for peer, per_peer in data["link_stats"].items()
    }
    return (
        int(data["received_bytes"]),
        int(data["sent_bytes"]),
        [(float(t), float(v)) for t, v in data["coverage"]],
        [(float(t), float(v)) for t, v in data["local_coverage"]],
        local_cell_series,
        nav_cell_series,
        float(data["resolution"]),
        link_stats,
    )


def _cache_store(cache_dir, name, key_payload, result):
    (received_bytes, sent_bytes, coverage, local_coverage,
     local_cell_series, nav_cell_series, resolution, link_stats) = result
    json_path, npz_path = _cache_paths(cache_dir, name)
    cache_dir.mkdir(parents=True, exist_ok=True)
    local = _pack_cell_series(local_cell_series)
    nav = _pack_cell_series(nav_cell_series)
    # Write both files to temporaries and rename: an interrupted run must not
    # leave a half-written entry that a later run reads back as a hit.
    # The temporary still has to end in .npz: np.savez_compressed appends
    # that suffix itself when the name lacks it, and would then write to a
    # path neither the rename nor the cleanup below knows about.
    tmp_npz = npz_path.with_name(npz_path.name + ".tmp.npz")
    tmp_json = json_path.with_name(json_path.name + ".tmp")
    try:
        np.savez_compressed(
            tmp_npz,
            local_times=local[0], local_lengths=local[1], local_cells=local[2],
            nav_times=nav[0], nav_lengths=nav[1], nav_cells=nav[2],
        )
        tmp_json.write_text(json.dumps({
            "key": key_payload,
            "data": {
                "received_bytes": int(received_bytes),
                "sent_bytes": int(sent_bytes),
                "coverage": [[t, v] for t, v in coverage],
                "local_coverage": [[t, v] for t, v in local_coverage],
                "resolution": float(resolution),
                "link_stats": {
                    peer: {k: [[t, v] for t, v in series] for k, series in per_peer.items()}
                    for peer, per_peer in link_stats.items()
                },
            },
        }))
        # npz first, json last: the json is what _cache_load checks the key
        # against, so an entry only becomes visible once its arrays are there.
        tmp_npz.replace(npz_path)
        tmp_json.replace(json_path)
    except OSError as e:
        print(f"warning: could not write bag cache for {name}: {e}", file=sys.stderr)
        for tmp in (tmp_npz, tmp_json):
            tmp.unlink(missing_ok=True)


def read_bag(robot, bag_dir, condition, max_duration=None, map_offset=None,
             cache_dir=DEFAULT_CACHE_DIR):
    """_read_bag_uncached, but reading a bag only the first time it's asked
    for. Decoding a multi-GB bag's every OccupancyGrid into cell keys is the
    whole cost of making a figure (minutes per run); every panel here is a
    pure function of that decoded result, so re-plotting the same runs -- a
    tweaked style, a different condition combination, one more run added to
    an average -- should never touch the bag again.

    cache_dir=None disables the cache entirely (--no-cache). Failures to read
    a bag are never cached: a missing/corrupt bag returns None every time, so
    the error surfaces on each run rather than being frozen in."""
    if cache_dir is None:
        return _read_bag_uncached(robot, bag_dir, condition,
                                  max_duration=max_duration, map_offset=map_offset)

    cache_dir = Path(cache_dir)
    name, key_payload = _cache_key(robot, bag_dir, condition, max_duration, map_offset)
    cached = _cache_load(cache_dir, name, key_payload)
    if cached is not None:
        print(f"cache hit: {robot} ({Path(bag_dir).parent.name}) -- not re-reading bag")
        return cached

    result = _read_bag_uncached(robot, bag_dir, condition,
                                max_duration=max_duration, map_offset=map_offset)
    if result is not None:
        _cache_store(cache_dir, name, key_payload, result)
    return result


def style_ax(ax, grid_axis="y"):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(GRID_COLOR)
    ax.spines["bottom"].set_color(GRID_COLOR)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=FS_TICK)
    ax.grid(axis=grid_axis, color=GRID_COLOR, linewidth=1, alpha=0.6, zorder=0)
    ax.set_axisbelow(True)


def panel_legend(ax, handles, loc="best"):
    """Every panel's legend, placed with loc="best" and given a soft white
    backing. "best" rather than a fixed corner because these legends now carry
    the per-series end-of-run numbers that used to be annotated at the line
    ends -- moving them into the legend is what keeps two curves that finish
    close together from printing their labels on top of each other, but it
    also makes the legend big enough that a hardcoded corner would sooner or
    later sit on the data. The backing keeps it legible wherever it lands."""
    if not handles:
        return
    ax.legend(handles=handles, fontsize=FS_LEGEND, loc=loc,
              frameon=True, facecolor="white", edgecolor="none", framealpha=0.85)


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


# (byte_index into read_bag's tuple, legend label, bar hatch). Sent first so
# the solid bar of each pair is the one the transport itself emitted, with the
# hatched "after DDIL throttling" bar beside it.
DIRECTIONS = ((1, "Published before relay", ""), (0, "Received from peers", "///"))


def format_kb(mean_kb, std_kb=None):
    """A KB total as a short label in whichever of KB/MB/GB keeps it readable.

    These totals span five or more orders of magnitude across conditions, so a
    single unit forces either "0 KB" or "2,582,266 KB" -- the latter being long
    enough that, with a +/- std beside it, no amount of axis headroom fits it.
    Mean and std always share one unit, picked from the mean."""
    for unit, scale in (("GB", 1024 * 1024), ("MB", 1024), ("KB", 1)):
        if mean_kb >= scale or unit == "KB":
            break
    places = 0 if unit == "KB" else 2
    text = f"{mean_kb / scale:,.{places}f}"
    if std_kb:
        text += f" ± {std_kb / scale:,.{places}f}"
    return f"{text} {unit}"


def plot_bandwidth(ax, results, conditions, xlabel):
    """Both directions of map-sharing traffic in a single panel: per condition,
    a pair of bars -- what the robots published for peers to pull (solid)
    beside what they actually received after DDIL throttling (hatched). These
    were two separate panels; they share a unit and a scale, so pairing them
    lets the sent/received gap for one transport be read directly instead of
    across two figures with independently-scaled axes.

    Horizontal bars on a log axis, and team totals rather than the per-robot
    stack these bars used to carry. Both follow from the spread in the data: a
    transport that sends three orders of magnitude less than the baseline
    leaves every efficient condition a flat invisible line on a linear axis,
    and segments stacked on a log axis would misstate each robot's share, since
    a segment's drawn length there is not proportional to its value. Horizontal
    because five condition names set at publication size will not fit side by
    side as x tick labels without rotating or truncating them.

    results[c] is a list of per-run {robot: entry} dicts -- with more than one
    run, bars show the mean total across runs with a +/- std error bar."""
    robots = sorted({r for run in results.values() for entry in run for r in entry})
    y = np.arange(len(conditions))
    bar_height = 0.34
    ratio_notes = []
    drawn = []  # (y, total, std) for every bar, to size the axis afterwards

    for (byte_index, direction_label, hatch), offset in zip(
            DIRECTIONS, (bar_height / 2 + 0.02, -bar_height / 2 - 0.02)):

        def run_total(run, byte_index=byte_index):
            return sum(run[r][byte_index] / 1024 for r in robots if r in run)

        totals = np.array([
            np.mean([run_total(run) for run in results[c]]) if results[c] else 0.0
            for c in conditions
        ])
        totals_std = np.array([
            np.std([run_total(run) for run in results[c]]) if len(results[c]) > 1 else 0.0
            for c in conditions
        ])

        ax.barh(y + offset, totals, height=bar_height,
                color=[CONDITION_COLORS[c] for c in conditions], edgecolor="white",
                linewidth=1.5, hatch=hatch, zorder=3)
        ax.errorbar(totals, y + offset, xerr=totals_std, fmt="none", ecolor=TEXT_PRIMARY,
                    elinewidth=1.5, capsize=5, zorder=5)
        drawn.extend(zip(y + offset, totals, totals_std))

        nonzero = [(c, t) for c, t in zip(conditions, totals) if t > 0]
        if len(nonzero) >= 2:
            (_, biggest_val) = max(nonzero, key=lambda ct: ct[1])
            (winner, smallest_val) = min(nonzero, key=lambda ct: ct[1])
            if biggest_val != smallest_val:
                ratio_notes.append(
                    f"{direction_label.split()[0]}: {biggest_val / smallest_val:,.0f}× "
                    f"less data ({DISPLAY_NAMES[winner]})")

    # A log axis cannot place 0, and a condition that shares nothing at all
    # (the no-sharing control) is legitimately 0 -- so the axis starts below the
    # smallest real value and those bars are drawn as nothing, labelled in place.
    positive = [t for _, t, _ in drawn if t > 0]
    left = min(positive) / 3 if positive else 0.1
    right = max(t + sd for _, t, sd in drawn) if drawn else 1.0
    right *= (right / left) ** BAR_LABEL_DECADES
    ax.set_xscale("log")
    ax.set_xlim(left, right)

    for yi, total, std in drawn:
        ax.annotate(format_kb(total, std), xy=(max(total + std, left), yi),
                    xytext=(8, 0), textcoords="offset points", ha="left", va="center",
                    fontsize=FS_VALUE, fontweight="bold", color=TEXT_PRIMARY)

    ax.set_yticks(y)
    ax.set_yticklabels([DISPLAY_NAMES[c] for c in conditions], fontsize=FS_AXIS,
                       color=TEXT_PRIMARY, fontweight="bold")
    # Descending (first condition on top, matching every other panel's legend
    # order), with a blank band above the first bar for the legend to sit in.
    # loc="best" is not enough here: it avoids bars but not the value labels
    # beside them, and it parked the legend on top of one. Reserving the band
    # keeps both. Its size is in category slots, worked back from the ~1.4in
    # the legend needs out of a panel's data area, so it holds whether two
    # conditions are plotted or five.
    ax.set_ylim(len(conditions) - 0.5, -0.5 - (0.32 * len(conditions) + 0.2))
    ax.set_xlabel(xlabel, fontsize=FS_AXIS, color=TEXT_SECONDARY)
    style_ax(ax, grid_axis="x")

    # The direction key and the headline ratios go in one legend rather than as
    # free-floating text, which at this type size lands on a bar or a label.
    handles = [Patch(facecolor="#b9b9b9", edgecolor="white", linewidth=1.5, hatch=hatch,
                     label=label) for _, label, hatch in DIRECTIONS]
    handles += [Line2D([], [], linestyle="none", label=note) for note in ratio_notes]
    panel_legend(ax, handles, loc="upper right")


def _step_value_at(series, t):
    """Value of a piecewise-constant (t, value) series at time t (0 before it starts)."""
    lo, hi = 0, len(series)
    while lo < hi:
        mid = (lo + hi) // 2
        if series[mid][0] <= t:
            lo = mid + 1
        else:
            hi = mid
    return series[lo - 1][1] if lo else 0.0


def capacity_timeline(results, conditions):
    """The run's shared link-capacity (t, kbps) step series, or None.

    Built from the bagged ddil_stats rather than from link_schedule.json, so it
    reports the capacity the proxies ACTUALLY had, not the capacity they were
    asked to have.

    Returns None if the links disagree with each other. That is not a fallback
    to be papered over: LINK_PROFILE drives every link with one shared schedule,
    so disagreement means some link was never reached and the run has no single
    capacity timeline. Shading one anyway would assert something false about the
    conditions the other links were under.
    """
    all_series = [link["bandwidth"]
                  for cond in conditions
                  for run in results.get(cond, [])
                  for entry in run.values()
                  if len(entry) > 7
                  for link in entry[7].values()
                  if link.get("bandwidth")]
    if not all_series:
        return None

    ref = all_series[0]
    # Probe on the reference's own transition times, plus a second after each,
    # which is where any disagreement shows up.
    probes = sorted({t for t, _ in ref} | {t + 1.0 for t, _ in ref})
    for other in all_series[1:]:
        for t in probes:
            a, b = _step_value_at(ref, t), _step_value_at(other, t)
            # 1 kbps tolerance: these are floats round-tripped through a message.
            if abs(a - b) > 1.0:
                print(f"warning: DDIL links disagree on capacity at t={t:.0f}s "
                      f"({a:.0f} vs {b:.0f} kbps) -- some link was not driven by "
                      "the schedule, so this run has no single capacity timeline "
                      "and is not comparable to one that does. Not shading.",
                      file=sys.stderr)
                return None

    # Collapse the 5 Hz samples down to the transitions themselves.
    collapsed = []
    for t, kbps in ref:
        if not collapsed or abs(collapsed[-1][1] - kbps) > 1.0:
            collapsed.append((t, kbps))
    return collapsed


def capacity_spans(timeline, t_end):
    """[(t0, t1, kbps)] from a capacity timeline."""
    if not timeline:
        return []
    spans = []
    for i, (t, kbps) in enumerate(timeline):
        end = timeline[i + 1][0] if i + 1 < len(timeline) else t_end
        if end > t:
            spans.append((t, end, kbps))
    return spans


def shade_capacity(ax, spans):
    """Grey out every stretch where the link was below its best capacity.

    This is what makes a varying-capacity run readable at a glance: whether a
    transport's coverage curve keeps climbing through the shaded stretches, or
    flattens and then dumps a stale burst on the far side, IS the
    graceful-degradation claim.
    """
    if not spans:
        return
    best = max(kbps for _, _, kbps in spans)
    constrained = [sp for sp in spans if sp[2] < best - 1.0]
    if not constrained:
        return
    worst = min(kbps for _, _, kbps in constrained)
    for t0, t1, kbps in constrained:
        # Deeper cut = darker band, so a staircase profile reads as a gradient
        # instead of one undifferentiated block.
        depth = (best - kbps) / (best - worst) if best > worst else 1.0
        ax.axvspan(t0, t1, color="#7a7a7a", alpha=0.06 + 0.10 * depth,
                   linewidth=0, zorder=0)


def plot_coverage(ax, results, conditions, series_index, ylabel):
    """series_index selects which per-robot coverage series to plot out of
    the (received_bytes, sent_bytes, coverage, local_coverage, ...) tuple --
    2 for communicated (nav_map), 3 for locally-observed (map). results[c]
    is a list of per-run {robot: entry} dicts; each robot's line is the
    mean of that robot's series across the runs it appears in, resampled
    onto a shared time grid (resample_step) -- with a +/- std shaded band
    when more than one run contributes."""
    robots = sorted({r for run in results.values() for entry in run for r in entry})
    robot_style = {r: LINESTYLES[i % len(LINESTYLES)] for i, r in enumerate(robots)}

    timeline = capacity_timeline(results, conditions)
    all_max_ts = [entry[series_index][-1][0]
                  for runs in results.values() for run in runs
                  for entry in run.values() if entry[series_index]]
    if timeline and all_max_ts:
        shade_capacity(ax, capacity_spans(timeline, max(all_max_ts)))

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
                     linewidth=2.5, solid_capstyle="round", zorder=3)
            if sampled.shape[0] > 1:
                std = sampled.std(axis=0)
                ax.fill_between(grid, mean - std, mean + std, color=CONDITION_COLORS[cond], alpha=0.15,
                                 linewidth=0, zorder=2)

    ax.set_xlabel("Time since run start (s)", fontsize=FS_AXIS, color=TEXT_SECONDARY)
    ax.set_ylabel(ylabel, fontsize=FS_AXIS, color=TEXT_SECONDARY)
    style_ax(ax)

    # Robots are identified by linestyle in the legend rather than by a name
    # annotated at each line's right end: one label per robot PER condition
    # meant several of them stacked up in the same corner of the axes, and at
    # publication type size they overlapped each other and ran past the axes.
    handles = [Line2D([0], [0], color=CONDITION_COLORS[c], lw=2.5, label=DISPLAY_NAMES[c])
               for c in conditions if results[c]]
    if len(robots) > 1:
        handles += [Line2D([0], [0], color=TEXT_SECONDARY, lw=2, linestyle=robot_style[r], label=r)
                    for r in robots]
    if timeline and len({kbps for _, kbps in timeline}) > 1:
        handles.append(Patch(facecolor="#7a7a7a", alpha=0.16, linewidth=0,
                             label="Reduced link capacity"))
    panel_legend(ax, handles)


def report_degradation(results, conditions):
    """Print how much of each transport's healthy coverage rate survives an outage.

    This is the transient a constant-bandwidth run cannot show, stated as a
    number. For each condition: mean rate of communicated-coverage growth
    (m^2/s, off /nav_map) while the link was at its best capacity, versus while
    it was constrained. The ratio between the two IS the graceful-degradation
    claim -- a transport that keeps delivering coarse coverage of the whole map
    through an outage retains a large fraction of its healthy rate; one that
    stalls and then dumps a stale burst on the far side does not.

    Deliberately NOT a slope-matched "time to recover": the coverage curves are
    step functions sampled at SLAM's own irregular cadence, so fitting a slope
    to them and asking when a later slope matches it is dominated by the choice
    of smoothing window rather than by the transport. A ratio of areas over
    spans whose boundaries the schedule defines exactly has no such free
    parameter.
    """
    timeline = capacity_timeline(results, conditions)
    if not timeline or len({kbps for _, kbps in timeline}) < 2:
        return  # constant-capacity run: nothing to say

    max_ts = [entry[2][-1][0] for cond in conditions
              for run in results.get(cond, []) for entry in run.values() if entry[2]]
    if not max_ts:
        return
    spans = capacity_spans(timeline, max(max_ts))
    best = max(kbps for _, _, kbps in spans)
    healthy = [(a, b) for a, b, k in spans if k >= best - 1.0]
    starved = [(a, b) for a, b, k in spans if k < best - 1.0]
    if not healthy or not starved:
        return

    def rate_over(series, windows):
        """m^2 gained per second across the given windows of a cumulative series."""
        gained = sum(max(_step_value_at(series, b) - _step_value_at(series, a), 0.0)
                     for a, b in windows)
        secs = sum(b - a for a, b in windows)
        return gained / secs if secs > 0 else 0.0

    print()
    print("Coverage growth vs link capacity "
          f"(healthy = {best:.0f} kbps, {sum(b - a for a, b in healthy):.0f}s; "
          f"constrained, {sum(b - a for a, b in starved):.0f}s):")
    for condition in conditions:
        runs = results.get(condition, [])
        pairs = [(rate_over(e[2], healthy), rate_over(e[2], starved))
                 for run in runs for e in run.values() if e[2]]
        if not pairs:
            continue
        h = float(np.mean([x for x, _ in pairs]))
        c = float(np.mean([y for _, y in pairs]))
        retained = f"{100.0 * c / h:.0f}% retained" if h > 0 else "n/a"
        print(f"{DISPLAY_NAMES[condition]:>10}: {h:7.2f} m²/s healthy, "
              f"{c:7.2f} m²/s constrained -- {retained}")


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
    """Union of ever-observed SLAM cells in a shared frame, in m².

    This is a map-based estimate, not ground-truth travelled floor area.
    Includes occupied cells and can grow when SLAM moves map boundaries.
    Spawn offsets are required for cross-robot spatial comparisons.
    """
    return to_area_m2(union_coverage_over_time(entry[4] for entry in run.values()), run_resolution(run))


def team_known_coverage_series(run):
    """Largest individual post-fusion map area, not collective team knowledge.

    This includes the robot's own observations, so it is not an amount of
    communicated information or a delivery fraction. Independent runs explore
    different ground; an unimpaired run is not a numerical upper bound.
    """
    series_by_robot = [entry[2] for entry in run.values() if entry[2]]
    if not series_by_robot:
        return []
    max_t = max(s[-1][0] for s in series_by_robot)
    grid = np.linspace(0, max_t, 200)
    sampled = np.array([resample_step(s, grid) for s in series_by_robot])
    return list(zip(grid.tolist(), sampled.max(axis=0).tolist()))


def cumulative_count_series(cell_series):
    """Turn one robot's local_cell_series ((t, new_cells) diffs) into a
    running (t, cumulative_count) series -- how many cells *this robot alone*
    has now seen, ignoring what any teammate has seen.

    Summing diff sizes is only valid because read_bag builds those diffs
    against a running union of everything the robot has reported, so no cell
    is ever emitted twice. If that ever regresses to diffing against just the
    previous message, pose-graph re-anchoring re-emits whole maps and this
    count inflates without bound -- see the comment in read_bag, and the
    overlap accounting in run_redundant_series."""
    total = 0
    out = []
    for t, cells in cell_series:
        total += cells.size
        out.append((t, total))
    return out


def run_redundant_series(run, cell_index):
    """Area tiles observed by at least two distinct robots, counted once.

    Three robots observing one tile still contribute one overlap tile.
    Merge simultaneous updates before reporting to avoid order-dependent values.
    """
    events = sorted(((t, robot, cells) for robot, entry in run.items()
                     for t, cells in entry[cell_index]), key=lambda e: e[0])
    seen = {robot: set() for robot in run}
    owners = {}
    overlap = 0
    out = []
    for t, robot, cells in events:
        for cell in cells.tolist():
            if cell in seen[robot]:
                continue
            seen[robot].add(cell)
            owners[cell] = owners.get(cell, 0) + 1
            if owners[cell] == 2:
                overlap += 1
        if out and out[-1][0] == t:
            out[-1] = (t, overlap)
        else:
            out.append((t, overlap))
    return out


def peer_derived_series(run):
    """Per-robot peer-relayed coverage over time -- the only coverage number
    communication is actually responsible for.

    The team-wide union of /nav_map is NOT that number. Every cell sits in the
    nav_map of at least the robot that observed it, whose "delivery" to itself
    cannot fail, so unioning nav_map across robots reproduces
    local_physical_union_series exactly -- in every condition, `none`
    included. Sharing adds no ground to the team union; it moves ground
    between individual robots' maps. What a transport changes is how much of a
    PEER's ground each robot ends up holding, which is what this measures.

    For robot i at time t, in tiles:
      delivered_i = |(nav_i \\ own_i) & (U_{j!=i} own_j)|
      peers_i     = |(U_{j!=i} own_j) \\ own_i|
    The denominator is the peer-observed ground robot i needs communication
    to learn. Exclude its own observations from BOTH sets: otherwise even
    complete sharing scores below 100% and exploration overlap confounds the
    comparison. Intersecting with peers' observations rejects fusion-only
    cells. Later self-observation removes a tile from both sets.

    This is map completeness, not packet delivery: it uses cumulative map
    cells, and still depends on map alignment and fusion. An unimpaired run
    is an empirical comparator, not a mathematical upper bound across runs.
    Ideal instantaneous sharing of all eligible cells scores 100%.

    own_i comes from local_cell_series (offset into the shared team frame),
    nav_i from nav_cell_series (already in it). own_i retains every key a
    re-anchored map ever reported, which makes it a generous set and therefore
    delivered_i a conservative estimate -- it under-counts rather than
    inflates.

    Returns (robots, events): `robots` sorted, and `events` a time-ordered
    list of (t, delivered_per_robot, peers_per_robot), each a tuple in
    `robots` order. Tile counts, not m^2 -- to_area_m2 with run_resolution
    converts, as for every other cell-set series here."""
    # Older GUI selections included relay-node namespaces with no map streams.
    robots = sorted(r for r, entry in run.items() if entry[4] or entry[5])
    bit = {r: 1 << i for i, r in enumerate(robots)}
    # Both streams merged into one timeline, so every cell's own/nav state is
    # evaluated against everything the team knew at that instant. entry[4] is
    # local_cell_series, entry[5] nav_cell_series (see read_bag).
    events = sorted(
        [(t, r, cells, True) for r in robots for t, cells in run[r][4]]
        + [(t, r, cells, False) for r in robots for t, cells in run[r][5]],
        key=lambda e: e[0])

    # cell -> bitmask of robots that have it, kept as ints rather than a set
    # per cell: a run holds a few hundred thousand distinct cells, and the
    # counts below are maintained incrementally off the mask transition, so
    # no step ever rescans a robot's whole map.
    own_mask = {}
    nav_mask = {}
    delivered = {r: 0 for r in robots}
    peers = {r: 0 for r in robots}
    out = []

    def counted(own_m, nav_m, r):
        """Does a cell in state (own_m, nav_m) count toward robot r's
        delivered total -- in its nav_map, not its own map, and observed by
        some peer?"""
        b = bit[r]
        return bool(nav_m & b) and not own_m & b and bool(own_m & ~b)

    for t, robot, cells, is_own in events:
        for cell in cells.tolist():
            om = own_mask.get(cell, 0)
            nm = nav_mask.get(cell, 0)
            new_om = om | bit[robot] if is_own else om
            new_nm = nm if is_own else nm | bit[robot]
            if new_om == om and new_nm == nm:
                continue
            for r in robots:
                delivered[r] += counted(new_om, new_nm, r) - counted(om, nm, r)
                if is_own:
                    b = bit[r]
                    peers[r] += (bool(new_om & ~b) and not bool(new_om & b)) - (
                        bool(om & ~b) and not bool(om & b))
            if is_own:
                own_mask[cell] = new_om
            else:
                nav_mask[cell] = new_nm
        point = (t, tuple(delivered[r] for r in robots), tuple(peers[r] for r in robots))
        # Simultaneous updates collapse to one point, so the series doesn't
        # depend on the order sorted() happened to give same-timestamp events.
        if out and out[-1][0] == t:
            out[-1] = point
        else:
            out.append(point)
    return robots, out


def delivery_fractions(delivered, peers):
    """Peer-map completeness as percentages, excluding self-observed ground.

    An empty eligible set scores 100%: no peer-only knowledge is missing.
    This includes startup and single-robot runs, not evidence of traffic.
    """
    return [100.0 * d / p if p else 100.0 for d, p in zip(delivered, peers)]


def peer_derived_area_series(run):
    """Mean per-robot peer-relayed coverage, in m^2 (see peer_derived_series)."""
    robots, events = peer_derived_series(run)
    if not events:
        return []
    return to_area_m2([(t, float(np.mean(delivered))) for t, delivered, _ in events],
                      run_resolution(run))


def delivery_fraction_series(run):
    """Mean per-robot completeness of eligible peer-observed ground, in
    percent (see peer_derived_series). Averaging over robots hides an
    asymmetric link -- one fully-fed robot and one blind one reads the same as
    two half-fed ones -- so summarize() also keeps the per-robot spread, which
    main() prints beside this."""
    _, events = peer_derived_series(run)
    if not events:
        return []
    return [(t, float(np.mean(delivery_fractions(delivered, peers))))
            for t, delivered, peers in events]


def plot_redundant_coverage(ax, results, conditions, cell_index):
    """Redundant physical coverage over time: cells more than one robot
    independently drove to and observed with its own sensors, i.e. sensor-map overlap (see run_redundant_series). results[c] is a
    list of per-run {robot: entry} dicts -- each run's redundant series is
    resampled (resample_step) onto a shared time grid and averaged, with a
    +/- std band across runs when more than one is given."""
    # Each curve's end-of-run value goes into its legend entry instead of being
    # annotated past the right end of the line: curves that finish close
    # together printed their labels on top of one another, and the annotation
    # itself sat outside the axes.
    handles = []
    for cond in conditions:
        runs = results[cond]
        redundant_per_run = [to_area_m2(run_redundant_series(run, cell_index), run_resolution(run)) for run in runs]
        max_ts = [m[-1][0] for m in redundant_per_run if m]
        if not max_ts:
            continue
        grid = np.linspace(0, min(max_ts), 200)
        sampled = np.array([resample_step(m, grid) for m in redundant_per_run if m])
        mean = sampled.mean(axis=0)
        ax.plot(grid, mean, color=CONDITION_COLORS[cond], linewidth=2.5, solid_capstyle="round", zorder=3)
        label = f"{DISPLAY_NAMES[cond]} — {mean[-1]:,.1f} m²"
        if sampled.shape[0] > 1:
            std = sampled.std(axis=0)
            ax.fill_between(grid, np.maximum(mean - std, 0), mean + std, color=CONDITION_COLORS[cond],
                             alpha=0.15, linewidth=0, zorder=2)
            label += f" ± {std[-1]:,.1f}"
        handles.append(Line2D([0], [0], color=CONDITION_COLORS[cond], lw=2.5, label=label))

    ax.set_xlabel("Time since run start (s)", fontsize=FS_AXIS, color=TEXT_SECONDARY)
    ax.set_ylabel("Redundant coverage (m², seen by >1 robot)", fontsize=FS_AXIS, color=TEXT_SECONDARY)
    style_ax(ax)
    panel_legend(ax, handles)


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
        # One peer_derived_series pass per run, shared by the area mean, the
        # delivered fraction and the per-robot spread: it walks every cell
        # event in the run, so it's the one metric here worth not recomputing
        # once per consumer.
        peer_area, peer_pct = [], []
        pct_by_robot = {}
        for run in runs:
            robots, events = peer_derived_series(run)
            if not events:
                continue
            _, delivered, peers = events[-1]
            peer_area.append(float(np.mean(delivered)) * run_resolution(run) ** 2)
            fractions = delivery_fractions(delivered, peers)
            peer_pct.append(float(np.mean(fractions)))
            for robot, fraction in zip(robots, fractions):
                pct_by_robot.setdefault(robot, []).append(fraction)
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
            # Peer-only knowledge and completeness of eligible peer ground.
            # These still depend on exploration, alignment, and map fusion.
            "peer_derived": mean_std(peer_area),
            "delivered_pct": mean_std(peer_pct),
            # Per-robot, because the mean above cannot show an asymmetric link.
            "delivered_pct_by_robot": {r: mean_std(v) for r, v in sorted(pct_by_robot.items())},
            # Descriptive map-area ratio, not a communication delivery fraction.
            # Overlap is unique area seen by two or more robots.
            "known_to_physical_pct": (100 * known_mean / physical_mean if physical_mean else 0.0, 0.0),
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
    ("known", "Largest map (m²)", "known_m2", 1.0, 1),
    ("known_to_physical_pct", "Largest map / union (%)", "known_to_physical_pct", 1.0, 1),
    ("peer_derived", "Peer-relayed (m²)", "peer_derived_m2", 1.0, 1),
    ("delivered_pct", "Peer-map completeness (%)", "delivered_pct", 1.0, 1),
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


def plot_union_coverage(ax, results, conditions, series_fn, ylabel, unit="m²"):
    """Plot a per-run map statistic with mean and standard deviation.

    local_physical_union_series estimates observed map union;
    team_known_coverage_series measures the largest individual known map.
    delivery_fraction_series measures completeness of peer-only map knowledge,
    including transport and fusion effects.
    `unit` only labels the legend value; the series carries its own units.
    """
    # End-of-run value carried in the legend entry, not annotated past the
    # right end of the line -- see plot_redundant_coverage.
    handles = []
    for cond in conditions:
        runs = results[cond]
        merged_per_run = [series_fn(run) for run in runs]
        max_ts = [m[-1][0] for m in merged_per_run if m]
        if not max_ts:
            continue
        grid = np.linspace(0, min(max_ts), 200)
        sampled = np.array([resample_step(m, grid) for m in merged_per_run if m])
        mean = sampled.mean(axis=0)
        ax.plot(grid, mean, color=CONDITION_COLORS[cond], linewidth=2.5, solid_capstyle="round", zorder=3)
        # Unit on the mean only, the spread bare -- same shape as
        # plot_redundant_coverage's labels ("10.0 m² ± 0.4").
        value = f"{mean[-1]:,.1f}%" if unit == "%" else f"{mean[-1]:,.1f} {unit}"
        label = f"{DISPLAY_NAMES[cond]} — {value}"
        if sampled.shape[0] > 1:
            std = sampled.std(axis=0)
            ax.fill_between(grid, mean - std, mean + std, color=CONDITION_COLORS[cond], alpha=0.15,
                             linewidth=0, zorder=2)
            label += f" ± {std[-1]:,.1f}"
        handles.append(Line2D([0], [0], color=CONDITION_COLORS[cond], lw=2.5, label=label))

    ax.set_xlabel("Time since run start (s)", fontsize=FS_AXIS, color=TEXT_SECONDARY)
    ax.set_ylabel(ylabel, fontsize=FS_AXIS, color=TEXT_SECONDARY)
    style_ax(ax)
    if series_fn is delivery_fraction_series:
        handles.append(ax.axhline(100, color=TEXT_SECONDARY, linestyle="--",
                                  linewidth=1, label="Ideal complete sharing (100%)"))
        ax.set_ylim(0, 105)
    panel_legend(ax, handles)


def figure_data(results, conditions, summary, robot_paths, max_duration):
    """Every number the figure draws, as plain JSON-able lists.

    Each panel's series is taken from the same function that panel plots, so
    the two cannot drift; what is NOT reproduced here is the averaging the
    plot functions do at draw time (resample onto a shared grid, mean and std
    across runs) -- this keeps one entry per run, which is strictly more
    information and lets a reader re-average however they like.

    The bandwidth panel has no entry under "series": it is a bar chart, and
    its bars are the sent/received means in "summary" like every other bar.

    Between this and the per-bag cache, a figure's numbers survive without the
    3 GB of bags behind them: this file for reading and re-plotting elsewhere,
    the cache for regenerating the figure itself."""
    def series(pairs):
        return [[float(t), float(v)] for t, v in pairs]

    per_run_panels = {
        "union": local_physical_union_series,
        "union_nav": team_known_coverage_series,
        "delivered": delivery_fraction_series,
        "redundant": lambda run: to_area_m2(run_redundant_series(run, 4), run_resolution(run)),
    }

    data = {
        "schema": 1,
        "max_duration": max_duration,
        "conditions": list(conditions),
        "display_names": {c: DISPLAY_NAMES[c] for c in conditions},
        # Which bags produced this -- so a figure's numbers can be traced back
        # to the runs, and the same figure re-made from them later.
        "runs": {c: [{r: str(b) for r, b in sorted(run.items())} for run in robot_paths[c]]
                 for c in conditions},
        # The bar panels: one (mean, std) pair per metric per condition.
        "summary": {c: {k: (list(v) if isinstance(v, tuple)
                            else {r: list(pair) for r, pair in v.items()} if isinstance(v, dict)
                            else v)
                        for k, v in summary[c].items()}
                    for c in conditions},
        "series": {},
    }

    # Per-robot time series (the "coverage" and "local" panels), indices 2/3
    # of a read_bag entry.
    for name, index in (("coverage", 2), ("local", 3)):
        data["series"][name] = {
            c: [{robot: series(entry[index]) for robot, entry in sorted(run.items()) if entry[index]}
                for run in results[c]]
            for c in conditions
        }
    # Whole-run time series (the union/delivered/redundant panels).
    for name, fn in per_run_panels.items():
        data["series"][name] = {c: [series(fn(run)) for run in results[c]] for c in conditions}
    # The link capacity actually in force, which the coverage panels shade.
    # One shared timeline across all conditions, exactly as the panels build
    # it -- per-condition calls would re-emit its disagreement warnings and
    # could report a timeline the figure itself declined to shade. None when
    # the links disagreed (or no ddil_stats were bagged).
    timeline = capacity_timeline(results, conditions)
    data["capacity_timeline"] = (
        None if timeline is None else [[float(t), float(kbps)] for t, kbps in timeline])
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--baseline", nargs="+", metavar="robot=bag_dir", action="append",
                         help="One run's robot=bag_dir set. Repeat --baseline for additional runs to average.")
    parser.add_argument("--vxch", nargs="+", metavar="robot=bag_dir", action="append",
                         help="One run's robot=bag_dir set. Repeat --vxch for additional runs to average.")
    parser.add_argument("--zstd", nargs="+", metavar="robot=bag_dir", action="append",
                         help="One run's robot=bag_dir set. Repeat --zstd for additional runs to average.")
    parser.add_argument("--none", nargs="+", metavar="robot=bag_dir", action="append",
                         help="Control arm: no map sharing at all. Repeat for additional runs to average.")
    parser.add_argument("--oracle", nargs="+", metavar="robot=bag_dir", action="append",
                         help="Control arm: unimpaired map sharing. Repeat for additional runs to average.")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--max-duration", type=float, default=None,
                         help="Clip each bag to this many seconds of bag time since its start.")
    parser.add_argument("--separate-figures", action="store_true",
                         help="Save each panel as its own image file (named <out stem>_<panel>.<out suffix>) "
                              "instead of one combined multi-panel image.")
    parser.add_argument("--table", nargs="?", const="text", choices=("text", "markdown", "csv"),
                         help="Also print a one-row-per-condition summary table (sent/received bandwidth, "
                              "team physical and known coverage, peer-relayed coverage and the share of "
                              "peer-observed ground it represents, redundant overlap). Default style 'text'; "
                              "'markdown' to paste into notes, 'csv' for a spreadsheet.")
    parser.add_argument("--table-file", type=Path,
                         help="Also write the summary table to this path, in the style its extension implies "
                              "(.csv, .md, else text) regardless of --table. Use it to keep stdout pipe-clean.")
    parser.add_argument("--robot-offset", metavar="robot=X,Y,YAW", action="append",
                         help="This robot's spawn pose, i.e. the pose of its private SLAM 'map' frame in the "
                              "shared team frame. Required to union different robots' self-observed coverage, "
                              "since each robot's /map is in its own frame. Normally inferred from "
                              "spawn_presets.yaml via the run directory's world name; use this to override "
                              "or to supply offsets for bags stored outside experiment_runs/.")
    parser.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE_DIR,
                         help="Where to keep decoded per-bag data so re-plotting the same runs does not "
                              "re-read their bags (default: figure_cache/ beside this script). Entries are "
                              "keyed by bag contents, so a re-recorded bag invalidates its own entry.")
    parser.add_argument("--no-cache", action="store_true",
                         help="Read every bag from scratch and write no cache entries.")
    parser.add_argument("--data-out", type=Path, default=None,
                         help="Write the figure's plotted numbers (every panel's series plus the summary "
                              "bar values) to this JSON path. Defaults to <out stem>.json; pass an explicit "
                              "path to put it elsewhere.")
    parser.add_argument("--no-data-out", action="store_true",
                         help="Skip writing that JSON sidecar next to the figure.")
    parser.add_argument("--spawn-preset", default=None,
                         help="spawn_presets.yaml preset the runs were launched with, used to infer "
                              "--robot-offset values. Defaults to $SPAWN_PRESET, else experiment.conf's "
                              "SPAWN_PRESET, else 'default'.")
    args = parser.parse_args()

    # Each condition's runs (from repeated --<condition> occurrences), each
    # a {robot: bag_dir} dict for one run.
    robot_paths = {}
    for condition in ALL_CONDITIONS:
        runs = getattr(args, condition)
        if runs:
            robot_paths[condition] = [parse_robot_paths(pairs) for pairs in runs]
    if len(robot_paths) < 2:
        print("error: need at least 2 of "
              "--none/--baseline/--vxch/--zstd/--oracle to compare", file=sys.stderr)
        sys.exit(1)
    conditions = tuple(c for c in ALL_CONDITIONS if c in robot_paths)

    # results[c] is a list of per-run {robot: read_bag() result} dicts, one
    # per --<condition> occurrence -- more than one run per condition drives
    # the mean +/- std error bars/bands in the plot_* functions below.
    cli_offsets = parse_robot_offsets(args.robot_offset)
    spawn_preset = args.spawn_preset or default_spawn_preset()

    results = {c: [] for c in conditions}
    for condition, runs in robot_paths.items():
        for run_robots in runs:
            # Each robot's /map is in its own SLAM frame, anchored at that
            # robot's spawn pose, so unioning them requires putting them in
            # one frame first. Prefer explicit --robot-offset, else infer from
            # spawn_presets.yaml using the world named by the run directory.
            robots = sorted(run_robots)
            offsets = dict(spawn_offsets_for(
                world_from_bag_dir(next(iter(run_robots.values()))),
                spawn_preset, robots))
            offsets.update(cli_offsets)
            if len(robots) > 1 and not all(r in offsets for r in robots):
                print(
                    f"warning: no spawn offset for {[r for r in robots if r not in offsets]} "
                    f"({condition}) -- each robot's /map stays in its own SLAM frame, so team "
                    f"physical coverage will double-count ground more than one robot covered. "
                    f"Pass --robot-offset robot=X,Y,YAW (or --spawn-preset).",
                    file=sys.stderr)

            run_results = {}
            for robot, bag_dir in sorted(run_robots.items()):
                r = read_bag(robot, bag_dir, condition, max_duration=args.max_duration,
                             map_offset=offsets.get(robot),
                             cache_dir=None if args.no_cache else args.cache_dir)
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
    # No panel titles anywhere: each y-axis label now states what the panel
    # measures, so the figure carries no text a journal caption would repeat.
    # "sent" and "received" are one panel -- plot_bandwidth draws both.
    panels = [
        ("bandwidth", plot_bandwidth, (results, conditions, "Map and coordination data volume (KB, log scale)")),
        ("coverage", plot_coverage, (results, conditions, 2, "Known map area (m², incl. peer-relayed)")),
        ("local", plot_coverage, (results, conditions, 3, "Self-observed map area (m²)")),
        ("union", plot_union_coverage, (results, conditions, local_physical_union_series,
                                        "Observed map union (m², SLAM estimate)")),
        ("union_nav", plot_union_coverage, (results, conditions, team_known_coverage_series,
                                            "Largest individual known map (m²)")),
        ("delivered", plot_union_coverage, (results, conditions, delivery_fraction_series,
                                            "Peer-map completeness (%, excluding own observations)", "%")),
        ("redundant", plot_redundant_coverage, (results, conditions, 4)),
    ]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    if args.separate_figures:
        for name, plot_fn, plot_args in panels:
            panel_fig, ax = plt.subplots(figsize=PANEL_SIZE)
            plot_fn(ax, *plot_args)
            panel_fig.tight_layout(pad=1.2)
            panel_out = args.out.with_name(f"{args.out.stem}_{name}{args.out.suffix}")
            panel_fig.savefig(panel_out, dpi=200, facecolor="white")
            plt.close(panel_fig)
    else:
        # Grid rather than one long row: at publication type size a 1xN strip
        # has to be scaled down so far to fit a page that the larger text buys
        # nothing back. Each cell keeps PANEL_SIZE, the same geometry
        # --separate-figures gives a panel, so type reads identically either way.
        ncols = min(3, len(panels))
        nrows = math.ceil(len(panels) / ncols)
        fig, axes = plt.subplots(nrows, ncols, squeeze=False,
                                 figsize=(PANEL_SIZE[0] * ncols, PANEL_SIZE[1] * nrows))
        flat = axes.ravel()
        for ax, (name, plot_fn, plot_args) in zip(flat, panels):
            plot_fn(ax, *plot_args)
        for ax in flat[len(panels):]:
            ax.set_visible(False)
        # No suptitle -- see panels above.
        fig.tight_layout(pad=2.0)
        fig.savefig(args.out, dpi=200, facecolor="white")

    # Per-condition totals, computed once for both the lines below and --table.
    summary = summarize(results, conditions)

    if not args.no_data_out:
        data_out = args.data_out or args.out.with_suffix(".json")
        data_out.parent.mkdir(parents=True, exist_ok=True)
        data_out.write_text(json.dumps(
            figure_data(results, conditions, summary, robot_paths, args.max_duration),
            indent=1) + "\n")
        print(f"figure data written to {data_out.resolve()}")
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
    report_degradation(results, conditions)
    for condition in conditions:
        n = summary[condition]["runs"]
        physical_mean, physical_std = summary[condition]["physical"]
        known_mean, known_std = summary[condition]["known"]
        local_suffix = f" ± {physical_std:.1f}" if n > 1 else ""
        nav_suffix = f" ± {known_std:.1f}" if n > 1 else ""
        print(f"{DISPLAY_NAMES[condition]:>10} team physical coverage (union, self-observed only): "
              f"{physical_mean:,.1f}{local_suffix} m²; "
              f"largest individual known map (incl. peer-relayed): "
              f"{known_mean:,.1f}{nav_suffix} m²")
    for condition in conditions:
        n = summary[condition]["runs"]
        peer_mean, peer_std = summary[condition]["peer_derived"]
        pct_mean, pct_std = summary[condition]["delivered_pct"]
        peer_suffix = f" ± {peer_std:.1f}" if n > 1 else ""
        pct_suffix = f" ± {pct_std:.1f}" if n > 1 else ""
        print(f"{DISPLAY_NAMES[condition]:>10} peer-relayed coverage (mean per robot, own observations "
              f"excluded): {peer_mean:,.1f}{peer_suffix} m² -- "
              f"{pct_mean:.1f}{pct_suffix}% of peer-observed ground not observed itself")
        by_robot = summary[condition]["delivered_pct_by_robot"]
        # The mean above reads the same for one fully-fed robot beside a blind
        # one as for two half-fed ones, which is exactly what an asymmetric
        # link budget produces -- so print the robots separately too.
        if len(by_robot) > 1:
            spread = ", ".join(f"{robot} {mean:.1f}%" for robot, (mean, _) in by_robot.items())
            print(f"{'':>10}   peer-map completeness per robot: {spread}")
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
