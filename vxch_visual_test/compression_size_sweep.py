#!/usr/bin/env python3
"""Sweep occupancy grid size x vxch's compression/varint ablation matrix (see
experiment.conf's COMPRESSION / VARINT_ENCODING) through the real codec via
./build/vxch_cli, and plot how encoded band-stream size scales with map size
under each configuration.

The grids are cut from a REAL map -- by default the office world the benchmark
runs actually explore (simulation/worlds/office/office_part1/map) -- as nested
crops centred on the building, one per sweep size. That is the point of this
figure: the numbers are measured bytes out of the codec on real floorplan
geometry, so the size scaling reflects the structure the codec exploits
(long uniform runs broken by walls) rather than a synthetic generator's. Pass
--map-yaml to point at any other ROS map_server yaml -- e.g. a slam_toolbox
export, which additionally carries unknown cells that a ground-truth world map
does not -- or --synthetic to fall back to vxch_cli's generated grids.

There is deliberately no "raw" reference series. The obvious one, 1 byte per
cell, is an assumption about an encoding nothing here actually performs, and
plotting it alongside measured bytes invites reading a modelled number as a
measured one. What each configuration costs on a real map, absolutely, is the
result; a ratio against a hypothetical baseline is not.

Needs matplotlib + numpy + Pillow, which live in the jazzy_env distrobox (see
gui/vxch_gui.py's docstring for the same constraint), so run this via:
    distrobox enter jazzy_env -- python3 compression_size_sweep.py

Requires vxch_cli to already be built:
    distrobox enter jazzy_env -- bash -lc "cmake -S . -B build && cmake --build build -j"
"""
import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import numpy as np
import yaml

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(PROJECT_ROOT)
CLI_BIN = os.path.join(PROJECT_ROOT, "build", "vxch_cli")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "figures")
DEFAULT_MAP_YAML = os.path.join(
    REPO_ROOT, "simulation", "worlds", "office", "office_part1", "map", "map.yaml")

# Roughly-doubling map sizes (cells), matching how an exploration map grows
# from a small room-scale SLAM map up to a large multi-room one.
DEFAULT_SIZES = [
    (40, 30), (60, 45), (80, 60), (120, 90),
    (160, 120), (240, 180), (320, 240), (480, 360), (640, 480),
]

# Ablation matrix: (label, compression, varint). Order fixed so color
# assignment below is stable regardless of which configs happen to run.
CONFIGS = [
    ("zstd + varint (default)", "zstd", True),
    ("zstd + fixed-width", "zstd", False),
    ("none + varint", "none", True),
    ("none + fixed-width", "none", False),
]

# First four slots of the dataviz skill's validated categorical palette
# (references/palette.md), in fixed order -- passes the adjacent-pair CVD/
# contrast gates for a line chart.
SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]

# Publication type scale and panel geometry, matching
# generate_comparison_figure.py so the two figures typeset alike.
FS_AXIS = 16.5
FS_TICK = 15
FS_LEGEND = 15
PANEL_SIZE = (7.6, 6.0)
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#cccccc"

# grid_io.hpp's format: magic | width u32 | height u32 | resolution f32 |
# width*height int8 cells (-1 unknown, 0 free, 100 occupied).
GRID_MAGIC = b"VXG1"


def run_cli(*args):
    proc = subprocess.run(
        [CLI_BIN, *[str(a) for a in args]],
        cwd=PROJECT_ROOT, capture_output=True, text=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"vxch_cli {' '.join(map(str, args))} failed:\n{proc.stderr.strip()}")
    return json.loads(proc.stdout)


def write_grid(path, cells, resolution):
    """Write an int8 occupancy grid in grid_io.hpp's format, so vxch_cli's
    `encode` reads a real map through exactly the path it reads a generated
    one -- no separate ingest to diverge."""
    height, width = cells.shape
    with open(path, "wb") as f:
        f.write(GRID_MAGIC)
        f.write(struct.pack("<IIf", width, height, resolution))
        f.write(cells.astype(np.int8).tobytes())


def load_ros_map(map_yaml):
    """A ROS map_server yaml+image pair as (int8 occupancy array, resolution).

    Applies the yaml's own negate/occupied_thresh/free_thresh exactly as
    map_server does, so the cells handed to the codec are the cells a robot
    would have had, not a re-thresholding of our own invention."""
    from PIL import Image  # Pillow lives in jazzy_env; imported late so --help works without it

    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    image_path = meta["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), image_path)

    pixels = np.asarray(Image.open(image_path).convert("L"), dtype=np.float64)
    occupancy = pixels / 255.0 if meta.get("negate", 0) else (255.0 - pixels) / 255.0

    cells = np.full(occupancy.shape, -1, dtype=np.int8)  # between the thresholds = unknown
    cells[occupancy >= meta.get("occupied_thresh", 0.65)] = 100
    cells[occupancy <= meta.get("free_thresh", 0.25)] = 0
    return cells, float(meta["resolution"])


def crop_windows(cells, sizes):
    """Nested (width, height) crops of a real map, centred on its occupied
    structure rather than on the image.

    A map image is mostly padding -- the office map's walls occupy rows
    148-601 of 1200 -- so an image-centred crop at the small sizes would be
    uniform free space and would tell us how well the codec encodes nothing.
    Centring on the occupied centroid keeps real floorplan structure in every
    window, which is what the sweep is measuring. Windows are clamped to the
    image, and a size larger than the map is skipped with a warning rather
    than silently padded."""
    occupied_rows, occupied_cols = np.nonzero(cells == 100)
    if len(occupied_rows):
        center_row, center_col = int(occupied_rows.mean()), int(occupied_cols.mean())
    else:
        center_row, center_col = cells.shape[0] // 2, cells.shape[1] // 2

    for width, height in sizes:
        if height > cells.shape[0] or width > cells.shape[1]:
            print(f"warning: skipping {width}x{height} -- larger than the "
                  f"{cells.shape[1]}x{cells.shape[0]} map", file=sys.stderr)
            continue
        row0 = min(max(center_row - height // 2, 0), cells.shape[0] - height)
        col0 = min(max(center_col - width // 2, 0), cells.shape[1] - width)
        yield width, height, cells[row0:row0 + height, col0:col0 + width]


def sweep(sizes, levels, tile_size_cells, seed, workdir, map_yaml=None):
    """Returns {config_label: {"cells": [...], "compressed_bytes": [...]}}.

    map_yaml=None uses vxch_cli's synthetic generator instead of a real map."""
    results = {label: {"cells": [], "compressed_bytes": []} for label, _, _ in CONFIGS}

    if map_yaml:
        grid, resolution = load_ros_map(map_yaml)
        print(f"map: {map_yaml} ({grid.shape[1]}x{grid.shape[0]} cells @ {resolution} m/cell)")
        windows = list(crop_windows(grid, sizes))
    else:
        windows = [(w, h, None) for w, h in sizes]

    for width, height, window in windows:
        map_path = os.path.join(workdir, f"map_{width}x{height}.grid")
        if window is None:
            run_cli("gen-map", "--out", map_path, "--width", width, "--height", height, "--seed", seed)
            mix = ""
        else:
            write_grid(map_path, window, resolution)
            total = window.size
            mix = (f"  [{100.0 * (window == 0).sum() / total:.0f}% free, "
                   f"{100.0 * (window == 100).sum() / total:.0f}% occupied, "
                   f"{100.0 * (window == -1).sum() / total:.0f}% unknown]")
        cells = width * height

        for label, compression, use_varint in CONFIGS:
            session_path = os.path.join(workdir, f"session_{width}x{height}_{compression}_{use_varint}.vxch")
            result = run_cli(
                "encode", "--map", map_path, "--out", session_path,
                "--levels", levels, "--tile-size-cells", tile_size_cells,
                "--compression", compression, "--varint", "true" if use_varint else "false",
            )
            encoded = result["total_compressed_bytes"]
            results[label]["cells"].append(cells)
            results[label]["compressed_bytes"].append(encoded)
            # Bytes per cell is measured output over input extent -- unlike the
            # "% of raw" this used to print, it assumes no encoding we did not run.
            print(f"{width:>4}x{height:<4} {label:<26} {encoded:>8} B "
                  f"({encoded / cells:.3f} B/cell){mix if label == CONFIGS[0][0] else ''}")

    return results


def plot(results, out_path):
    fig, ax = plt.subplots(figsize=PANEL_SIZE)

    for (label, _, _), color in zip(CONFIGS, SERIES_COLORS):
        data = results[label]
        ax.plot(data["cells"], data["compressed_bytes"],
                marker="o", markersize=7, linewidth=2.5, color=color, label=label, zorder=2)

    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Map size (cells)", fontsize=FS_AXIS, color=TEXT_SECONDARY)
    ax.set_ylabel("Encoded band stream (bytes)", fontsize=FS_AXIS, color=TEXT_SECONDARY)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=FS_TICK)
    ax.grid(True, which="both", color=GRID_COLOR, linewidth=0.8, alpha=0.5)
    ax.set_axisbelow(True)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["left"].set_color(GRID_COLOR)
    ax.spines["bottom"].set_color(GRID_COLOR)
    ax.xaxis.set_major_formatter(mticker.ScalarFormatter())
    # Upper left: the series all rise left-to-right, so that corner is the only
    # one a legend at this type size can occupy without covering a line.
    ax.legend(fontsize=FS_LEGEND, loc="upper left", frameon=True,
              facecolor="white", edgecolor="none", framealpha=0.85)

    fig.tight_layout(pad=1.2)
    fig.savefig(out_path, dpi=200)
    print(f"\nSaved figure to {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--levels", type=int, default=4, help="Haar wavelet levels")
    parser.add_argument("--tile-size-cells", type=int, default=32)
    parser.add_argument("--seed", type=int, default=42, help="only used with --synthetic")
    parser.add_argument("--map-yaml", default=DEFAULT_MAP_YAML,
                        help="ROS map_server yaml to cut the swept grids from")
    parser.add_argument("--synthetic", action="store_true",
                        help="use vxch_cli's generated grids instead of a real map")
    parser.add_argument(
        "--out", default=os.path.join(FIGURES_DIR, "compression_size_sweep.png"),
    )
    args = parser.parse_args()

    if not os.path.isfile(CLI_BIN):
        print(
            f"{CLI_BIN} not found -- build it first:\n"
            f"  cd {PROJECT_ROOT} && distrobox enter jazzy_env -- "
            f"bash -lc 'cmake -S . -B build && cmake --build build -j'",
            file=sys.stderr,
        )
        sys.exit(1)

    map_yaml = None if args.synthetic else args.map_yaml
    if map_yaml and not os.path.isfile(map_yaml):
        print(f"map yaml not found: {map_yaml} (pass --map-yaml, or --synthetic)", file=sys.stderr)
        sys.exit(1)

    os.makedirs(FIGURES_DIR, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="vxch_sweep_") as workdir:
        results = sweep(DEFAULT_SIZES, args.levels, args.tile_size_cells, args.seed,
                        workdir, map_yaml=map_yaml)
    plot(results, args.out)


if __name__ == "__main__":
    main()
