#!/usr/bin/env python3
"""Cache one run's bag-derived series so figures can be rebuilt without the bags.

A run's bag is large and awkward to keep around; everything the comparison
figure actually draws is a much smaller set of series. This extracts exactly
that set -- by calling generate_comparison_figure.py's own read_bag(), not a
reimplementation, so the cache cannot drift from what the figure would have
computed -- and writes it to a compressed .npz plus a JSON sidecar.

What is stored, per robot, is precisely read_bag()'s return value:
    received_bytes, sent_bytes          scalars (bandwidth bars)
    coverage, local_coverage            (t, known_area_m2) series
    local_cell_series, nav_cell_series  (t, newly-known packed cell keys)
    resolution                          metres per cell
    link_stats                          per-peer (t, value) series for the
                                        capacity actually applied to each
                                        downlink plus the response to it
                                        (send rate, backlog, bytes shed)
The cell series are diffs (new cells per message), not snapshots, so they stay
small -- they are what the union/redundant-coverage plots need in order to
combine robots without double-counting.

Must run where rosbag2_py and rclpy are importable, i.e. inside jazzy_env:

    distrobox enter jazzy_env -- bash -lc \\
        "python3 tools/export_run_summary.py --condition vxch \\
             --bag robot1=experiment_runs/<run>/bag robot2=... \\
             --out summaries/<run>.npz"

Reading one back (anywhere numpy is available -- no ROS needed):

    from tools.export_run_summary import load_summary
    meta, results = load_summary("summaries/<run>.npz")
    # results is {robot: read_bag()-shaped tuple}, ready to hand to the
    # plotting functions in generate_comparison_figure.py.
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
# 2: added per-link ddil_stats series (link_stats), so a varying-capacity run
#    can be read against the capacity that produced it. Version 1 caches still
#    load -- link_stats comes back empty for them.
# 3: read_bag now diffs each /map message against a running union rather than
#    against only the previous message. Versions 1-2 cached the pre-fix cell
#    series, in which pose-graph re-anchoring re-emitted whole maps (up to 17x
#    the distinct cell count on one robot), so any redundancy figure built
#    from them is inflated. Those caches are REJECTED rather than loaded --
#    unlike the version-2 bump, the payload itself is wrong, not just absent,
#    and silently loading it would keep reporting the old numbers.
# 4: floor-based cell keys fix ties-to-even aliasing; nav diffs are cumulative.
# Older cell arrays cannot be repaired without reading the original maps.
SUMMARY_VERSION = 4
MIN_LOADABLE_VERSION = 4


def _import_figure_module():
    """generate_comparison_figure.py lives at the repo root, not on sys.path."""
    sys.path.insert(0, str(REPO))
    try:
        import generate_comparison_figure as gcf
    except ImportError as exc:  # rosbag2_py/rclpy missing -> wrong environment
        raise SystemExit(
            f"cannot import generate_comparison_figure ({exc}).\n"
            "This must run where rosbag2_py and rclpy are importable, e.g.\n"
            '  distrobox enter jazzy_env -- bash -lc "python3 '
            'tools/export_run_summary.py ..."') from exc
    return gcf


def _pack_cell_series(series):
    """[(t, int64 array)] -> (times, offsets, concatenated keys).

    Stored flat because .npz holds arrays, not ragged lists; offsets are the
    split points so the original per-message arrays come back exactly.
    """
    times = np.asarray([t for t, _ in series], dtype=np.float64)
    arrays = [np.asarray(keys, dtype=np.int64).ravel() for _, keys in series]
    lengths = np.asarray([a.size for a in arrays], dtype=np.int64)
    offsets = np.concatenate(([0], np.cumsum(lengths))).astype(np.int64)
    keys = np.concatenate(arrays) if arrays else np.empty(0, dtype=np.int64)
    return times, offsets, keys


def _unpack_cell_series(times, offsets, keys):
    return [(float(times[i]), keys[offsets[i]:offsets[i + 1]])
            for i in range(len(times))]


def _pack_xy(series):
    """[(t, value)] -> 2 float arrays."""
    if not series:
        return np.empty(0), np.empty(0)
    arr = np.asarray(series, dtype=np.float64)
    return arr[:, 0], arr[:, 1]


def export_run(bags, condition, out_path, max_duration=None, spawn_preset=None,
               cli_offsets=None):
    """Read each robot's bag and write the cache. Returns the metadata dict."""
    gcf = _import_figure_module()

    robots = sorted(bags)
    # Mirror generate_comparison_figure.main() exactly: each robot's /map sits
    # in its own SLAM frame anchored at its spawn pose, so the union plots need
    # the same offsets the figure would have used. Getting these wrong would
    # silently change physical-coverage numbers.
    preset = spawn_preset or gcf.default_spawn_preset()
    world = gcf.world_from_bag_dir(next(iter(bags.values())))
    offsets = dict(gcf.spawn_offsets_for(world, preset, robots))
    if cli_offsets:
        offsets.update(cli_offsets)
    missing = [r for r in robots if r not in offsets]
    if len(robots) > 1 and missing:
        print(f"warning: no spawn offset for {missing} -- team physical coverage "
              "will double-count overlapping ground (same caveat as the figure).",
              file=sys.stderr)

    arrays = {}
    per_robot_meta = {}
    for robot in robots:
        bag_dir = bags[robot]
        result = gcf.read_bag(robot, bag_dir, condition,
                              max_duration=max_duration,
                              map_offset=offsets.get(robot))
        if result is None:
            print(f"warning: {robot}: unreadable bag at {bag_dir} -- skipped",
                  file=sys.stderr)
            continue
        (received, sent, coverage, local_coverage,
         local_cells, nav_cells, resolution, link_stats) = result

        cov_t, cov_a = _pack_xy(coverage)
        loc_t, loc_a = _pack_xy(local_coverage)
        lc_t, lc_o, lc_k = _pack_cell_series(local_cells)
        nc_t, nc_o, nc_k = _pack_cell_series(nav_cells)
        arrays.update({
            f"{robot}/coverage_t": cov_t, f"{robot}/coverage_a": cov_a,
            f"{robot}/local_coverage_t": loc_t, f"{robot}/local_coverage_a": loc_a,
            f"{robot}/local_cells_t": lc_t, f"{robot}/local_cells_o": lc_o,
            f"{robot}/local_cells_k": lc_k,
            f"{robot}/nav_cells_t": nc_t, f"{robot}/nav_cells_o": nc_o,
            f"{robot}/nav_cells_k": nc_k,
        })
        # One key group per (robot, peer) downlink. Kept per-link rather than
        # averaged: a link the scheduler never reached shows as a flat
        # bandwidth line here, and collapsing the links together would hide
        # exactly that failure.
        for peer, series in sorted(link_stats.items()):
            for field, points in sorted(series.items()):
                t_arr, v_arr = _pack_xy(points)
                arrays[f"{robot}/link/{peer}/{field}_t"] = t_arr
                arrays[f"{robot}/link/{peer}/{field}_v"] = v_arr
        per_robot_meta[robot] = {
            "received_bytes": int(received),
            "sent_bytes": int(sent),
            "resolution": None if resolution is None else float(resolution),
            "bag_dir": str(bag_dir),
            "offset": list(offsets[robot]) if robot in offsets else None,
            "coverage_points": int(cov_t.size),
            "local_cell_messages": int(lc_t.size),
            "nav_cell_messages": int(nc_t.size),
            "links": sorted(link_stats),
            "link_fields": sorted(next(iter(link_stats.values()))) if link_stats else [],
        }

    if not per_robot_meta:
        raise SystemExit("no readable bags -- nothing exported")

    meta = {
        "version": SUMMARY_VERSION,
        "condition": condition,
        "world": world,
        "spawn_preset": preset,
        "max_duration": max_duration,
        "robots": sorted(per_robot_meta),
        "per_robot": per_robot_meta,
    }

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out_path, meta=json.dumps(meta), **arrays)
    out_path.with_suffix(".json").write_text(json.dumps(meta, indent=2))
    return meta


class StaleSummary(Exception):
    """Cache predates a fix that changed what the cached arrays mean."""


def load_summary(path):
    """Return (meta, {robot: read_bag()-shaped tuple}) from a cache file.

    Raises StaleSummary for caches older than MIN_LOADABLE_VERSION; callers
    should fall back to re-reading the bag.
    """
    with np.load(path, allow_pickle=False) as data:
        meta = json.loads(str(data["meta"]))
        if meta.get("version", 1) < MIN_LOADABLE_VERSION:
            raise StaleSummary(
                f"{path}: summary version {meta.get('version', 1)} predates the "
                f"read_bag cell-coordinate fix (need >= {MIN_LOADABLE_VERSION}); "
                f"re-export it from the bag."
            )
        results = {}
        for robot in meta["robots"]:
            rmeta = meta["per_robot"][robot]
            coverage = list(zip(data[f"{robot}/coverage_t"].tolist(),
                                data[f"{robot}/coverage_a"].tolist()))
            local_coverage = list(zip(data[f"{robot}/local_coverage_t"].tolist(),
                                      data[f"{robot}/local_coverage_a"].tolist()))
            local_cells = _unpack_cell_series(data[f"{robot}/local_cells_t"],
                                              data[f"{robot}/local_cells_o"],
                                              data[f"{robot}/local_cells_k"])
            nav_cells = _unpack_cell_series(data[f"{robot}/nav_cells_t"],
                                            data[f"{robot}/nav_cells_o"],
                                            data[f"{robot}/nav_cells_k"])
            # .get() so a version-1 cache (no link keys, no "links" entry)
            # still loads, just with nothing to say about capacity over time.
            link_stats = {}
            for peer in rmeta.get("links", []):
                link_stats[peer] = {
                    field: list(zip(data[f"{robot}/link/{peer}/{field}_t"].tolist(),
                                    data[f"{robot}/link/{peer}/{field}_v"].tolist()))
                    for field in rmeta.get("link_fields", [])
                }
            results[robot] = (
                rmeta["received_bytes"], rmeta["sent_bytes"],
                coverage, local_coverage, local_cells, nav_cells,
                rmeta["resolution"], link_stats,
            )
    return meta, results


def parse_pairs(pairs):
    out = {}
    for pair in pairs:
        if "=" not in pair:
            raise SystemExit(f"expected robot=bag_dir, got '{pair}'")
        robot, _, path = pair.partition("=")
        out[robot.strip()] = path.strip()
    return out


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", nargs="+", metavar="robot=bag_dir", required=True)
    p.add_argument("--condition", required=True,
                   choices=("baseline", "vxch", "zstd", "none", "oracle"))
    p.add_argument("--out", required=True, type=Path)
    p.add_argument("--max-duration", type=float, default=None)
    p.add_argument("--spawn-preset", default=None)
    p.add_argument("--robot-offset", action="append", metavar="robot=X,Y,YAW",
                   help="override a robot's spawn offset (same meaning as the "
                        "figure script's flag of the same name)")
    args = p.parse_args()

    cli_offsets = {}
    for item in args.robot_offset or []:
        robot, _, triple = item.partition("=")
        cli_offsets[robot.strip()] = tuple(float(v) for v in triple.split(","))

    meta = export_run(parse_pairs(args.bag), args.condition, args.out,
                      max_duration=args.max_duration,
                      spawn_preset=args.spawn_preset,
                      cli_offsets=cli_offsets)
    size = os.path.getsize(args.out)
    print(f"wrote {args.out} ({size / 1e6:.2f} MB) "
          f"for {len(meta['robots'])} robot(s), condition={meta['condition']}")


if __name__ == "__main__":
    main()
