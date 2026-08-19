#!/usr/bin/env python3
"""Sweep synthetic occupancy grid size x vxch's compression/coefficient-encoding
ablation matrix (see experiment.conf's COMPRESSION / COEFF_ENCODING) through the
real codec via ./build/vxch_cli, and plot how encoded band-stream size scales with
map size under each configuration.

Needs matplotlib + numpy, which live in the jazzy_env distrobox (see
gui/vxch_gui.py's docstring for the same constraint), so run this via:
    distrobox enter jazzy_env -- python3 compression_size_sweep.py

Requires vxch_cli to already be built:
    distrobox enter jazzy_env -- bash -lc "cmake -S . -B build && cmake --build build -j"
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
CLI_BIN = os.path.join(PROJECT_ROOT, "build", "vxch_cli")
FIGURES_DIR = os.path.join(PROJECT_ROOT, "figures")

# Roughly-doubling map sizes (cells), matching how an exploration map grows
# from a small room-scale SLAM map up to a large multi-room one.
DEFAULT_SIZES = [
    (40, 30), (60, 45), (80, 60), (120, 90),
    (160, 120), (240, 180), (320, 240), (480, 360), (640, 480),
]

# Ablation matrix: (label, compression, encoding). Order fixed so color
# assignment below is stable regardless of which configs happen to run.
# encoding is vxch_cli's --encoding value -- see kHaarEncoding* in
# voxelcodec_ros/types.hpp: "varint" (default), "fixed_width", "sparse_rle"
# (separate gap-length/magnitude streams -- wins on sparse/mostly-flat
# bands, can lose on dense ones), "auto" (try all 3 per band, keep
# whichever compresses smallest -- never worse than the best single mode).
CONFIGS = [
    ("zstd + varint (default)", "zstd", "varint"),
    ("zstd + fixed-width", "zstd", "fixed_width"),
    ("zstd + sparse-RLE", "zstd", "sparse_rle"),
    ("zstd + auto", "zstd", "auto"),
    ("none + varint", "none", "varint"),
    ("none + fixed-width", "none", "fixed_width"),
    ("none + sparse-RLE", "none", "sparse_rle"),
    ("none + auto", "none", "auto"),
]

# All 8 slots of the dataviz skill's validated categorical palette
# (references/palette.md), in fixed order -- the full 8 passes every
# adjacent-pair CVD/contrast gate in both modes for a line chart (this
# script's chart form); only all-pairs contexts (scatter, small multiples)
# need the 3-slot cap. The raw-bytes reference line uses the palette's
# muted "text-secondary" ink instead of a categorical hue, since it's a
# baseline for scale, not a series identity.
SERIES_COLORS = [
    "#2a78d6", "#eb6834", "#1baf7a", "#eda100",
    "#e87ba4", "#008300", "#4a3aa7", "#e34948",
]
RAW_REFERENCE_COLOR = "#767568"


def run_cli(*args):
    proc = subprocess.run(
        [CLI_BIN, *[str(a) for a in args]],
        cwd=PROJECT_ROOT, capture_output=True, text=True,
    )
    if proc.returncode != 0:
        raise RuntimeError(f"vxch_cli {' '.join(map(str, args))} failed:\n{proc.stderr.strip()}")
    return json.loads(proc.stdout)


def sweep(sizes, levels, tile_size_cells, seed, workdir):
    """Returns {config_label: {"cells": [...], "raw_bytes": [...], "compressed_bytes": [...]}}."""
    results = {label: {"cells": [], "raw_bytes": [], "compressed_bytes": []}
               for label, _, _ in CONFIGS}

    for width, height in sizes:
        map_path = os.path.join(workdir, f"map_{width}x{height}.grid")
        run_cli("gen-map", "--out", map_path, "--width", width, "--height", height, "--seed", seed)
        cells = width * height

        for label, compression, encoding in CONFIGS:
            session_path = os.path.join(workdir, f"session_{width}x{height}_{compression}_{encoding}.vxch")
            result = run_cli(
                "encode", "--map", map_path, "--out", session_path,
                "--levels", levels, "--tile-size-cells", tile_size_cells,
                "--compression", compression, "--encoding", encoding,
            )
            results[label]["cells"].append(cells)
            results[label]["raw_bytes"].append(result["raw_bytes"])
            results[label]["compressed_bytes"].append(result["total_compressed_bytes"])
            print(
                f"{width:>4}x{height:<4} {label:<26} "
                f"{result['total_compressed_bytes']:>8} B "
                f"({100.0 * result['total_compressed_bytes'] / result['raw_bytes']:.1f}% of raw)"
            )

    return results


def plot(results, out_path):
    fig, (ax_bytes, ax_ratio) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle("vxch encoded size vs. map size, by compression configuration")

    raw_cells = results[CONFIGS[0][0]]["cells"]
    raw_bytes = results[CONFIGS[0][0]]["raw_bytes"]
    # "none + varint" tracks close to 100% of raw for most sizes, so this dashed
    # reference needs to sit ABOVE the series lines (not below) or it disappears
    # under whichever series happens to overlap it.
    ax_bytes.plot(
        raw_cells, raw_bytes, linestyle="--", linewidth=2, color=RAW_REFERENCE_COLOR,
        label="raw OccupancyGrid (1 B/cell)", zorder=3,
    )

    for (label, _, _), color in zip(CONFIGS, SERIES_COLORS):
        data = results[label]
        ax_bytes.plot(
            data["cells"], data["compressed_bytes"],
            marker="o", markersize=5, linewidth=2, color=color, label=label, zorder=2,
        )
        ratio = [100.0 * c / r for c, r in zip(data["compressed_bytes"], data["raw_bytes"])]
        ax_ratio.plot(
            data["cells"], ratio,
            marker="o", markersize=5, linewidth=2, color=color, label=label, zorder=2,
        )

    ax_bytes.set_xscale("log")
    ax_bytes.set_yscale("log")
    ax_bytes.set_xlabel("map size (cells)")
    ax_bytes.set_ylabel("encoded band stream (bytes)")
    ax_bytes.set_title("Absolute size")
    ax_bytes.grid(True, which="both", linewidth=0.5, alpha=0.3)
    ax_bytes.xaxis.set_major_formatter(mticker.ScalarFormatter())
    ax_bytes.legend(fontsize=8, loc="upper left")

    ax_ratio.set_xscale("log")
    ax_ratio.set_xlabel("map size (cells)")
    ax_ratio.set_ylabel("% of raw OccupancyGrid size")
    ax_ratio.set_title("Relative to raw (lower is better)")
    ax_ratio.set_ylim(bottom=0)
    ax_ratio.grid(True, which="both", linewidth=0.5, alpha=0.3)
    ax_ratio.xaxis.set_major_formatter(mticker.ScalarFormatter())
    ax_ratio.legend(fontsize=8, loc="upper right")

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_path, dpi=150)
    print(f"\nSaved figure to {out_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--levels", type=int, default=4, help="Haar wavelet levels")
    parser.add_argument("--tile-size-cells", type=int, default=32)
    parser.add_argument("--seed", type=int, default=42)
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

    os.makedirs(FIGURES_DIR, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="vxch_sweep_") as workdir:
        results = sweep(DEFAULT_SIZES, args.levels, args.tile_size_cells, args.seed, workdir)
    plot(results, args.out)


if __name__ == "__main__":
    main()
