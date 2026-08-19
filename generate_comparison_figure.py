#!/usr/bin/env python3
"""Produce a scientific comparison figure for a baseline run vs. a vxch run:
how much data was exchanged between robots to share the map, and how much of
the map got explored, for each condition.

Reads each robot's recorded rosbag2 (mcap) directly:
  - received bandwidth: bytes on every /<robot>/incoming/<peer>/... topic
    (baseline: .../map; vxch: .../band_*+manifest) -- what this robot
    actually received from its peers after DDIL throttling, per
    RECORD_METRICS's doc in experiment.conf, not the free-running node-log
    counters.
  - sent bandwidth: bytes this robot itself published for peers to pull --
    baseline: /<robot>/map (peers' ddil_proxy instances pull straight from
    it); vxch: /<robot>/vxch/map/band_*+manifest (the encoder's own output,
    pre-DDIL-relay-fanout). Received and sent are read from disjoint topics,
    so a robot that explores little but has active peers can easily receive
    more than it sends -- that's not a contradiction, it's DDIL relaying its
    peers' progress to it regardless of its own.
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
  - hypothetical "Baseline (zstd)" bandwidth: replays every message that
    counted toward baseline's real sent/received bytes and re-measures it
    as len(zstd.compress(msg, --zstd-level)) instead of len(msg) -- i.e.
    what baseline's bandwidth would have been if every map message had
    been zstd-compressed before going out, message-by-message (no shared
    dictionary/stream state across messages). This is a derived what-if,
    not a separately recorded run -- it reuses baseline's real coverage
    curves unchanged, since compression doesn't change what gets shared,
    only how many bytes it costs.

Must run where rosbag2_py, rclpy, and (for --zstd-level) the python3-zstd
apt package are importable, e.g. inside the jazzy_env distrobox with a
login shell so its ROS setup gets sourced:
  distrobox enter jazzy_env -- bash -lc "python3 generate_comparison_figure.py ..."

Usage:
  ./generate_comparison_figure.py \\
      --baseline robot1=<bag_dir> robot2=<bag_dir> \\
      --vxch     robot1=<bag_dir> robot2=<bag_dir> \\
      --out figures/compare.png
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
import matplotlib.ticker as ticker
import numpy as np
from matplotlib.lines import Line2D

from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
import zstd

COLOR_BASELINE = "#eb6834"
COLOR_BASELINE_ZSTD = "#c9a227"
COLOR_WAVESTREAM = "#2a78d6"
COLOR_WAVESTREAM_ZSTD = "#5aa89a"
COLOR_MANIFEST = "#d1495b"
TEXT_PRIMARY = "#1a1a1a"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#cccccc"
CONDITIONS = ("baseline", "vxch")  # internal keys -- match run-dir/CLI naming, unrelated to display
DISPLAY_NAMES = {"baseline": "Baseline", "vxch": "Wavestream"}
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


def read_bag(robot, bag_dir, condition, zstd_level):
    """Returns (received_bytes, sent_bytes, coverage, local_coverage,
    received_bytes_zstd, sent_bytes_zstd, received_bands, received_manifest,
    sent_bands, sent_manifest) where coverage and local_coverage are each a
    list of (seconds_since_start, known_area_m2) -- coverage from
    /<robot>/nav_map (post-fusion team map), local_coverage from
    /<robot>/map (this robot's own raw SLAM output, never touched by
    fusion) -- or None if the bag can't be read. received_bytes is traffic
    on /<robot>/incoming/<peer>/... (what peers sent this robot);
    sent_bytes is what this robot itself published for peers to pull --
    /<robot>/map for baseline, /<robot>/vxch/map/band_*+manifest for vxch
    (see module docstring).

    received_bytes_zstd/sent_bytes_zstd apply independent per-message
    zstd.compress(..., zstd_level) wherever that message isn't already
    compressed, and leave it alone where it already is:
      - baseline: every message re-measured (baseline never compresses
        anything).
      - vxch: only messages ending in "/manifest" are re-measured -- band_*
        payloads already go through voxelcodec_ros's own zstd compression
        (see occupancy_grid_vxch_node's "compression" param, default
        "zstd"), so re-compressing them again would just show
        double-compression noise, not a real opportunity. The manifest
        itself is currently sent uncompressed on every tick a band goes
        out (see occupancy_grid_vxch_node.cpp's send_pending_bands), so
        this measures what a compressed manifest would actually save.

    received_bands/received_manifest/sent_bands/sent_manifest split real
    (not zstd-hypothetical) vxch bytes into band_* payload vs manifest, so
    the two can be plotted separately -- always 0 for condition ==
    "baseline" (it has no band/manifest distinction, just one map
    message)."""
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
    nav_map_topic = f"/{robot}/nav_map"
    local_map_topic = f"/{robot}/map"

    received_bytes = 0
    sent_bytes = 0
    received_bytes_zstd = 0
    sent_bytes_zstd = 0
    received_bands = 0
    received_manifest = 0
    sent_bands = 0
    sent_manifest = 0
    coverage = []
    local_coverage = []
    start_ns = None

    def known_area_m2(msg):
        known = int(np.count_nonzero(np.asarray(msg.data) != -1))
        return known * (msg.info.resolution ** 2)

    def zstd_len(data):
        return len(zstd.compress(bytes(data), zstd_level))

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if start_ns is None:
            start_ns = t_ns
        if incoming_re.match(topic):
            received_bytes += len(data)
            if condition == "baseline" or topic.endswith("/manifest"):
                received_bytes_zstd += zstd_len(data)
            else:
                received_bytes_zstd += len(data)
            if condition == "vxch":
                if topic.endswith("/manifest"):
                    received_manifest += len(data)
                else:
                    received_bands += len(data)
        elif topic == nav_map_topic:
            msg = deserialize_message(data, OccupancyGrid)
            coverage.append(((t_ns - start_ns) / 1e9, known_area_m2(msg)))
        elif topic == local_map_topic:
            msg = deserialize_message(data, OccupancyGrid)
            local_coverage.append(((t_ns - start_ns) / 1e9, known_area_m2(msg)))
            if condition == "baseline":
                sent_bytes += len(data)
                sent_bytes_zstd += zstd_len(data)
        elif condition == "vxch" and vxch_own_re.match(topic):
            sent_bytes += len(data)
            if topic.endswith("/manifest"):
                sent_bytes_zstd += zstd_len(data)
                sent_manifest += len(data)
            else:
                sent_bytes_zstd += len(data)
                sent_bands += len(data)

    del reader
    if scratch_dir is not None:
        shutil.rmtree(scratch_dir, ignore_errors=True)

    coverage.sort(key=lambda p: p[0])
    local_coverage.sort(key=lambda p: p[0])
    return (received_bytes, sent_bytes, coverage, local_coverage, received_bytes_zstd, sent_bytes_zstd,
            received_bands, received_manifest, sent_bands, sent_manifest)


def style_ax(ax):
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(GRID_COLOR)
    ax.spines["bottom"].set_color(GRID_COLOR)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=10)
    ax.grid(axis="y", color=GRID_COLOR, linewidth=1, alpha=0.6, zorder=0)
    ax.set_axisbelow(True)


def plot_bandwidth(ax, results, series, title, ylabel, ratio_pair=(0, -1)):
    """series is an ordered list of (result_key, tuple_index, display_name,
    color), one per x-axis category. result_key indexes into `results`
    (e.g. "baseline", "vxch"); tuple_index selects which field of that
    condition's per-robot tuple to plot -- 0/1 for real received/sent
    bytes, 4/5 for the zstd-recompressed received/sent bytes read_bag()
    computes per its docstring. This lets a hypothetical variant (e.g.
    "Baseline (zstd)") sit as its own bar next to a real, recorded
    condition without being mistaken for one. ratio_pair is the (index,
    index) into `series` of the two categories the "N× less data" callout
    should compare -- defaults to first vs. last, but pass explicit
    indices once hypothetical bars are sandwiched between the real ones."""
    robots = sorted({r for key, _, _, _ in series for r in results.get(key, {})})
    x = np.arange(len(series))
    colors = [color for _, _, _, color in series]
    heights_by_robot = {
        robot: np.array([
            results.get(key, {}).get(robot, (0, 0, [], [], 0, 0, 0, 0, 0, 0))[idx] / 1024
            for key, idx, _, _ in series
        ])
        for robot in robots
    }
    totals = np.sum(list(heights_by_robot.values()), axis=0) if robots else np.zeros(len(series))

    bottoms = np.zeros(len(series))
    for robot in robots:
        heights_kb = heights_by_robot[robot]
        ax.bar(x, heights_kb, bottom=bottoms, width=0.45,
               color=colors, edgecolor="white", linewidth=2,
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

    ia, ib = ratio_pair
    if totals[ia] > 0 and totals[ib] > 0 and totals[ia] != totals[ib]:
        bigger, smaller = max(totals[ia], totals[ib]), min(totals[ia], totals[ib])
        winner = series[ib][2] if totals[ib] < totals[ia] else series[ia][2]
        ax.text(0.5, 0.99, f"{bigger / smaller:.1f}× less data ({winner})",
                transform=ax.transAxes, ha="center", va="top",
                fontsize=10.5, color=TEXT_SECONDARY, style="italic")

    positive = totals[totals > 0]
    if positive.size >= 2 and positive.max() / positive.min() > 8:
        ax.set_yscale("log")
        ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda v, _: f"{v:,.0f}"))
        # A segment stacked at bottom=0 (the alphabetically-first robot in
        # each bar) has no valid log position for its bottom edge -- if the
        # autoscaled lower ylim lands above that segment's top, the whole
        # segment (bar AND label) silently renders off-screen. Floor the
        # axis below the smallest real segment anywhere in the chart so
        # that can't happen, instead of trusting autoscale.
        segment_heights = [h for heights in heights_by_robot.values() for h in heights if h > 0]
        if segment_heights:
            ax.set_ylim(bottom=min(segment_heights) / 4, top=totals.max() * 1.6)
    else:
        ax.set_ylim(0, totals.max() * 1.25 if totals.max() > 0 else 1)

    ax.set_xticks(x)
    ax.set_xticklabels([name for _, _, name, _ in series], fontsize=10.5, color=TEXT_PRIMARY, fontweight="bold")
    ax.set_ylabel(ylabel, fontsize=11, color=TEXT_SECONDARY)
    ax.set_title(title, fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)


def plot_vxch_breakdown(ax, results):
    """Real (not zstd-hypothetical) vxch bytes, aggregated across robots,
    split into band_* payload vs manifest -- one stacked bar each for sent
    and received. Complements the "manifest zstd" hypothetical bars in the
    main bandwidth panels by showing the actual, uncompressed proportion
    manifest accounts for today."""
    categories = ["Sent", "Received"]
    x = np.arange(len(categories))
    entries = list(results.get("vxch", {}).values())
    # tuple layout: (received, sent, coverage, local_coverage,
    # received_zstd, sent_zstd, received_bands, received_manifest,
    # sent_bands, sent_manifest) -- see read_bag().
    bands = np.array([
        sum(e[8] for e in entries),
        sum(e[6] for e in entries),
    ]) / 1024
    manifest = np.array([
        sum(e[9] for e in entries),
        sum(e[7] for e in entries),
    ]) / 1024
    totals = bands + manifest

    ax.bar(x, bands, width=0.45, color=COLOR_WAVESTREAM, edgecolor="white", linewidth=2,
           label="Bands (already zstd'd)", zorder=3)
    ax.bar(x, manifest, bottom=bands, width=0.45, color=COLOR_MANIFEST, edgecolor="white", linewidth=2,
           label="Manifest (uncompressed)", zorder=3)

    for xi, total, m in zip(x, totals, manifest):
        ax.annotate(f"{total:,.1f} KB", xy=(xi, total), xytext=(0, 6),
                    textcoords="offset points", ha="center", va="bottom",
                    fontsize=11, fontweight="bold", color=TEXT_PRIMARY)
        if total > 0:
            ax.annotate(f"{100 * m / total:.0f}% manifest", xy=(xi, total), xytext=(0, 22),
                        textcoords="offset points", ha="center", va="bottom",
                        fontsize=9, color=TEXT_SECONDARY, style="italic")

    ax.set_ylim(0, totals.max() * 1.35 if totals.max() > 0 else 1)
    ax.set_xticks(x)
    ax.set_xticklabels(categories, fontsize=11, color=TEXT_PRIMARY, fontweight="bold")
    ax.set_ylabel("Wavestream bytes (KB)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_title("Wavestream: bands vs. manifest", fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)
    ax.legend(frameon=False, fontsize=9.5, loc="upper right")


def plot_coverage(ax, results, series_index, title, ylabel):
    """series_index selects which per-robot coverage series to plot out of
    the (received_bytes, sent_bytes, coverage, local_coverage) tuple in
    results -- 2 for communicated (nav_map), 3 for locally-observed (map)."""
    colors = {"baseline": COLOR_BASELINE, "vxch": COLOR_WAVESTREAM}
    robots = sorted({r for cond in results.values() for r in cond})
    robot_style = {r: LINESTYLES[i % len(LINESTYLES)] for i, r in enumerate(robots)}

    for cond in CONDITIONS:
        for robot, entry in sorted(results[cond].items()):
            coverage = entry[series_index]
            if not coverage:
                continue
            ts = [p[0] for p in coverage]
            areas = [p[1] for p in coverage]
            ax.plot(ts, areas, color=colors[cond], linestyle=robot_style[robot],
                     linewidth=2, solid_capstyle="round", zorder=3)
            ax.annotate(robot, (ts[-1], areas[-1]), textcoords="offset points",
                        xytext=(6, 0), fontsize=8.5, color=TEXT_SECONDARY, va="center")

    ax.set_xlabel("Time since run start (s)", fontsize=11, color=TEXT_SECONDARY)
    ax.set_ylabel(ylabel, fontsize=11, color=TEXT_SECONDARY)
    ax.set_title(title, fontsize=13, fontweight="bold", color=TEXT_PRIMARY, loc="left")
    style_ax(ax)

    handles = [Line2D([0], [0], color=colors[c], lw=2, label=DISPLAY_NAMES[c])
               for c in CONDITIONS if results[c]]
    if handles:
        ax.legend(handles=handles, frameon=False, fontsize=10, loc="lower right")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--baseline", nargs="+", required=True, metavar="robot=bag_dir")
    parser.add_argument("--vxch", nargs="+", required=True, metavar="robot=bag_dir")
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--zstd-level", type=int, default=3, metavar="N",
                         help="zstd.compress level (1-22, default 3 == the library's own out-of-the-box "
                              "default) used for the hypothetical 'Baseline (zstd)' bandwidth bar.")
    args = parser.parse_args()

    robot_paths = {"baseline": parse_robot_paths(args.baseline), "vxch": parse_robot_paths(args.vxch)}

    results = {"baseline": {}, "vxch": {}}
    for condition, robots in robot_paths.items():
        for robot, bag_dir in sorted(robots.items()):
            r = read_bag(robot, bag_dir, condition, args.zstd_level)
            if r is not None:
                results[condition][robot] = r

    for condition in CONDITIONS:
        if not results[condition]:
            print(f"error: no readable bags for condition {condition!r}", file=sys.stderr)
            sys.exit(1)

    baseline_zstd_label = f"Baseline\n(zstd, L{args.zstd_level})"
    vxch_zstd_label = f"Wavestream\n(manifest zstd, L{args.zstd_level})"
    received_series = [
        ("baseline", 0, DISPLAY_NAMES["baseline"], COLOR_BASELINE),
        ("baseline", 4, baseline_zstd_label, COLOR_BASELINE_ZSTD),
        ("vxch", 0, DISPLAY_NAMES["vxch"], COLOR_WAVESTREAM),
        ("vxch", 4, vxch_zstd_label, COLOR_WAVESTREAM_ZSTD),
    ]
    sent_series = [
        ("baseline", 1, DISPLAY_NAMES["baseline"], COLOR_BASELINE),
        ("baseline", 5, baseline_zstd_label, COLOR_BASELINE_ZSTD),
        ("vxch", 1, DISPLAY_NAMES["vxch"], COLOR_WAVESTREAM),
        ("vxch", 5, vxch_zstd_label, COLOR_WAVESTREAM_ZSTD),
    ]

    fig, (ax_sent, ax_received, ax_breakdown, ax_coverage, ax_local) = plt.subplots(1, 5, figsize=(34, 5.5))
    plot_bandwidth(ax_sent, results, sent_series, "Map-sharing bandwidth (sent)", "Sent to peers (KB)", ratio_pair=(0, 2))
    plot_bandwidth(ax_received, results, received_series, "Map-sharing bandwidth (received)", "Received from peers (KB)", ratio_pair=(0, 2))
    plot_vxch_breakdown(ax_breakdown, results)
    plot_coverage(ax_coverage, results, 2, "Communicated map coverage", "Known map area (m²)")
    plot_coverage(ax_local, results, 3, "Locally-observed coverage (self only)", "Self-observed area (m²)")
    fig.suptitle(f"Map sharing: {DISPLAY_NAMES['baseline']} vs. {DISPLAY_NAMES['vxch']}", fontsize=15, fontweight="bold",
                 color=TEXT_PRIMARY, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=200, facecolor="white")

    baseline_received = sum(e[0] for e in results["baseline"].values())
    vxch_received = sum(e[0] for e in results["vxch"].values())
    baseline_sent = sum(e[1] for e in results["baseline"].values())
    vxch_sent = sum(e[1] for e in results["vxch"].values())
    baseline_received_zstd = sum(e[4] for e in results["baseline"].values())
    baseline_sent_zstd = sum(e[5] for e in results["baseline"].values())
    vxch_received_zstd = sum(e[4] for e in results["vxch"].values())
    vxch_sent_zstd = sum(e[5] for e in results["vxch"].values())
    vxch_sent_bands = sum(e[8] for e in results["vxch"].values())
    vxch_sent_manifest = sum(e[9] for e in results["vxch"].values())
    vxch_received_bands = sum(e[6] for e in results["vxch"].values())
    vxch_received_manifest = sum(e[7] for e in results["vxch"].values())
    print(f"{DISPLAY_NAMES['baseline']} received: {baseline_received} bytes ({baseline_received / 1024:.1f} KB), "
          f"sent: {baseline_sent} bytes ({baseline_sent / 1024:.1f} KB)")
    print(f"{DISPLAY_NAMES['baseline']} zstd (level {args.zstd_level}) received: {baseline_received_zstd} bytes "
          f"({baseline_received_zstd / 1024:.1f} KB), sent: {baseline_sent_zstd} bytes ({baseline_sent_zstd / 1024:.1f} KB)")
    print(f"{DISPLAY_NAMES['vxch']} received:     {vxch_received} bytes ({vxch_received / 1024:.1f} KB), "
          f"sent: {vxch_sent} bytes ({vxch_sent / 1024:.1f} KB)")
    print(f"{DISPLAY_NAMES['vxch']} sent split: bands {vxch_sent_bands} bytes ({vxch_sent_bands / 1024:.1f} KB), "
          f"manifest {vxch_sent_manifest} bytes ({vxch_sent_manifest / 1024:.1f} KB)"
          + (f" [{100 * vxch_sent_manifest / vxch_sent:.0f}% manifest]" if vxch_sent > 0 else ""))
    print(f"{DISPLAY_NAMES['vxch']} received split: bands {vxch_received_bands} bytes ({vxch_received_bands / 1024:.1f} KB), "
          f"manifest {vxch_received_manifest} bytes ({vxch_received_manifest / 1024:.1f} KB)"
          + (f" [{100 * vxch_received_manifest / vxch_received:.0f}% manifest]" if vxch_received > 0 else ""))
    print(f"{DISPLAY_NAMES['vxch']} manifest-zstd (level {args.zstd_level}) received: {vxch_received_zstd} bytes "
          f"({vxch_received_zstd / 1024:.1f} KB), sent: {vxch_sent_zstd} bytes ({vxch_sent_zstd / 1024:.1f} KB)")
    if baseline_received > 0 and vxch_received > 0:
        print(f"received ratio: {max(baseline_received, vxch_received) / min(baseline_received, vxch_received):.2f}x")
    for condition in CONDITIONS:
        for robot, entry in sorted(results[condition].items()):
            coverage, local_coverage = entry[2], entry[3]
            final = coverage[-1][1] if coverage else 0.0
            final_local = local_coverage[-1][1] if local_coverage else 0.0
            print(f"{DISPLAY_NAMES[condition]} {robot}: final known map area {final:.1f} m^2 (communicated, incl. peer-relayed cells), "
                  f"{final_local:.1f} m^2 self-observed")
    print(str(args.out.resolve()))


if __name__ == "__main__":
    main()
