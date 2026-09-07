#!/usr/bin/env python3
"""3D (sparse voxel) mode viewer for the vxch visual test.

Companion to vxch_gui.py, which covers the 2D dense-grid mode. The two modes
are different representations, and watching them side by side is the point:

  2D  a dense occupancy grid, tiled, each tile carrying a 2D Haar pyramid.
      Adding a band sharpens a blurry image.
  3D  a sparse voxel list split into per-axis coordinate channels (x, y, z),
      each Haar-transformed along its own sequence. The first k bands are a
      lossless representation of the coordinate sequence downsampled by
      2^(levels-k), so adding a band roughly DOUBLES the voxel count. Coarse
      levels are not blurry -- they are fewer voxels spanning the same volume.

Like vxch_gui.py, this shells out to vxch_cli for anything touching the codec
and never reimplements Haar/decode math itself. It only reads back the plain
"x y z" voxel dumps the CLI writes.

Usage:
    python3 gui/vxch_3d_gui.py [--archive datasets/new_college_0p2.vxch]

Generate an archive from a .graph dataset with the Go reference encoder:
    graph2vxch -resolution 0.2 -o datasets/new_college_0p2.vxch \\
        datasets/new_college.graph
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np
from PIL import Image, ImageTk

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLI_BIN = os.path.join(PROJECT_ROOT, "build", "vxch_cli")
DEFAULT_ARCHIVE = os.path.join(PROJECT_ROOT, "datasets", "new_college_0p2.vxch")
VIEW_PX = 340


def fmt_bytes(n):
    """Human-readable size. Bits are shown too at small sizes, where the whole
    point is that a usable coarse map costs a startlingly small number of them."""
    if n < 1024:
        return f"{n} B ({n * 8} bits)"
    if n < 1024 * 1024:
        return f"{n / 1024:.1f} KB"
    return f"{n / (1024 * 1024):.2f} MB"


def run_cli(*args):
    if not os.path.isfile(CLI_BIN):
        raise RuntimeError(
            f"{CLI_BIN} not found -- build it first:\n"
            f"  cmake -S {PROJECT_ROOT} -B {PROJECT_ROOT}/build && "
            f"cmake --build {PROJECT_ROOT}/build")
    proc = subprocess.run(
        [CLI_BIN, *[str(a) for a in args]], capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(proc.stderr.strip() or "vxch_cli failed")
    return json.loads(proc.stdout)


def read_voxels(path):
    """Reads the CLI's "voxels N" header plus one "x y z" line per voxel."""
    with open(path) as handle:
        header = handle.readline().split()
        count = int(header[1]) if len(header) > 1 else 0
        if count == 0:
            return np.zeros((0, 3), dtype=np.int64)
        data = np.loadtxt(handle, dtype=np.int64)
    return data.reshape(-1, 3)


def project_image(voxels, axis_a, axis_b, extent):
    """Orthographic density projection onto two axes, as a PIL image.

    Kept separate from the Tk wrapping below so the projection can be exercised
    (and eyeballed as a PNG) without a display or a root window.

    Density rather than a binary hit: a single pixel can stand for many voxels
    along the collapsed axis, and showing that count is what makes a wall read
    as a wall instead of a smear.
    """
    size_a, size_b = extent[axis_a], extent[axis_b]
    grid = np.zeros((size_b + 1, size_a + 1), dtype=np.int32)
    if len(voxels):
        np.add.at(grid, (voxels[:, axis_b], voxels[:, axis_a]), 1)

    img = np.zeros(grid.shape + (3,), dtype=np.uint8)
    img[..., :] = 24                                     # empty
    hit = grid > 0
    if hit.any():
        # Log scale: raw counts are dominated by a few very deep columns.
        depth = np.log1p(grid[hit].astype(float))
        depth = depth / depth.max() if depth.max() > 0 else depth
        img[hit, 0] = (40 + 215 * depth).astype(np.uint8)
        img[hit, 1] = (90 + 140 * depth).astype(np.uint8)
        img[hit, 2] = (200 - 120 * depth).astype(np.uint8)

    image = Image.fromarray(np.flipud(img), mode="RGB")
    scale = VIEW_PX / max(image.width, image.height)
    return image.resize(
        (max(1, round(image.width * scale)), max(1, round(image.height * scale))),
        Image.NEAREST if scale >= 1 else Image.BOX)


def project(voxels, axis_a, axis_b, extent):
    """project_image, wrapped for Tk. Requires a live root window."""
    return ImageTk.PhotoImage(project_image(voxels, axis_a, axis_b, extent))


class ViewPanel(ttk.Frame):
    def __init__(self, parent, title):
        super().__init__(parent)
        ttk.Label(self, text=title, font=("TkDefaultFont", 10, "bold")).pack()
        self.canvas = tk.Label(self, background="#181818")
        self.canvas.pack()
        self._photo = None

    def show(self, photo):
        self._photo = photo                    # keep a reference or Tk drops it
        self.canvas.configure(image=photo)


class Vxch3DGui(tk.Tk):
    def __init__(self, archive):
        super().__init__()
        self.title("vxch 3D mode -- progressive voxel level of detail")
        self.archive = archive
        self.tmpdir = tempfile.mkdtemp(prefix="vxch3d_")
        self.extent = None
        self.full_voxels = None

        try:
            self.info = run_cli("voxels-info", "--archive", archive)
        except Exception as exc:                          # noqa: BLE001
            messagebox.showerror("vxch", str(exc))
            raise SystemExit(1)

        self.max_bands = max(1, int(self.info["max_bands"]))

        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text=os.path.basename(archive),
                  font=("TkDefaultFont", 11, "bold")).pack(side="left")
        ttk.Label(top, text=f"  {self.info['archive_bytes']/1024:.0f} KB archive"
                            f"  |  {self.max_bands} bands").pack(side="left")

        # Per-channel cost AT THE CURRENT LEVEL, not the stored total -- which
        # axis is expensive is the central question in 3D mode, and it depends
        # on the sort order the encoder used.
        chan = ttk.Frame(self, padding=(8, 2))
        chan.pack(fill="x")
        self.chan_labels = {}
        for c in self.info["channels"]:
            lbl = ttk.Label(chan, text=f"{c['name']}: -")
            lbl.pack(side="left", padx=(0, 16))
            self.chan_labels[c["name"]] = lbl

        views = ttk.Frame(self, padding=8)
        views.pack()
        self.view_xy = ViewPanel(views, "top (X-Y)")
        self.view_xz = ViewPanel(views, "front (X-Z)")
        self.view_yz = ViewPanel(views, "side (Y-Z)")
        for i, v in enumerate((self.view_xy, self.view_xz, self.view_yz)):
            v.grid(row=0, column=i, padx=6)

        controls = ttk.Frame(self, padding=8)
        controls.pack(fill="x")
        ttk.Label(controls, text="bands:").pack(side="left")
        self.band_var = tk.IntVar(value=min(12, self.max_bands))
        self.slider = ttk.Scale(
            controls, from_=1, to=self.max_bands, orient="horizontal",
            command=self._on_slide, length=380)
        self.slider.set(self.band_var.get())
        self.slider.pack(side="left", padx=8)
        ttk.Button(controls, text="Full detail",
                   command=lambda: self._set_bands(self.max_bands)).pack(side="left")

        # Channel-separation ablation: the same coordinates regrouped three
        # ways, so the only variable is byte layout. Splits the usual
        # "separation helps" claim into the part that is really about keeping an
        # axis's values contiguous (grouping) and the part that is about giving
        # each axis its own compressor (separation).
        abl = ttk.LabelFrame(
            self, text="ablation: does splitting x/y/z do anything?", padding=8)
        abl.pack(fill="x", padx=8, pady=(4, 0))
        self.abl_rows = {}
        for i, (key, label) in enumerate((
            ("interleaved", "interleaved   x0,y0,z0, x1,y1,z1, ...  (1 stream)"),
            ("concatenated", "concatenated  x0..xn, y0..yn, z0..zn  (1 stream)"),
            ("split", "split         x | y | z               (3 streams)"),
        )):
            ttk.Label(abl, text=label, font=("TkFixedFont", 9)).grid(
                row=i, column=0, sticky="w")
            value = ttk.Label(abl, text="-", font=("TkFixedFont", 9, "bold"))
            value.grid(row=i, column=1, sticky="e", padx=(18, 0))
            self.abl_rows[key] = value
        self.abl_verdict = ttk.Label(abl, text="", wraplength=1000)
        self.abl_verdict.grid(row=3, column=0, columnspan=2, sticky="w", pady=(6, 0))

        self.size_label = ttk.Label(
            self, padding=(8, 4), text="", font=("TkDefaultFont", 13, "bold"))
        self.size_label.pack(fill="x")

        self.status = ttk.Label(self, padding=(8, 0, 8, 8), text="")
        self.status.pack(fill="x")

        self._render(self.band_var.get())

    def _on_slide(self, _value):
        bands = int(round(float(self.slider.get())))
        if bands != self.band_var.get():
            self._set_bands(bands)

    def _set_bands(self, bands):
        self.band_var.set(bands)
        self.slider.set(bands)
        self._render(bands)

    def _render(self, bands):
        out = os.path.join(self.tmpdir, f"voxels_{bands}.txt")
        try:
            result = run_cli("voxels-decode", "--archive", self.archive,
                             "--bands", bands, "--out", out)
            voxels = read_voxels(out)
        except Exception as exc:                          # noqa: BLE001
            self.status.configure(text=f"error: {exc}")
            return

        # Fix the projection extent from the full-detail decode once, so coarse
        # levels render at the same scale instead of each rescaling to its own
        # bounding box (which would hide that they cover the same volume).
        if self.extent is None:
            full_out = os.path.join(self.tmpdir, "voxels_full.txt")
            full = run_cli("voxels-decode", "--archive", self.archive,
                           "--bands", 0, "--out", full_out)
            self.full_voxels = full["voxels"]
            full_data = read_voxels(full_out)
            self.extent = full_data.max(axis=0) if len(full_data) else np.array([1, 1, 1])

        extent = self.extent
        self.view_xy.show(project(voxels, 0, 1, extent))
        self.view_xz.show(project(voxels, 0, 2, extent))
        self.view_yz.show(project(voxels, 1, 2, extent))

        share = 100.0 * result["voxels"] / max(1, self.full_voxels)
        level_bytes = result["level_bytes"]
        full_bytes = result["full_channel_bytes"]
        pct = 100.0 * level_bytes / max(1, full_bytes)

        self.size_label.configure(
            text=f"level cost: {fmt_bytes(level_bytes)}"
                 f"   ({pct:.1f}% of the full {fmt_bytes(full_bytes)})"
                 f"   —  {result['voxels']:,} voxels")

        for c in result["channels"]:
            note = c["encoding"] if c["progressive"] else f"{c['encoding']}, no ladder"
            self.chan_labels[c["name"]].configure(
                text=f"{c['name']}: {fmt_bytes(c['bytes'])}  ({note})")

        self._render_ablation(bands)

        self.status.configure(
            text=f"bands 1..{bands} of {self.max_bands}   "
                 f"{share:.1f}% of full {self.full_voxels:,} voxels   "
                 f"— a band prefix gives fewer, coarser voxels spanning the "
                 f"same volume, not a blurrier grid. Sizes are the coefficient "
                 f"prefix re-compressed, so full detail lands within a few "
                 f"percent of the stored archive rather than exactly on it.")


    def _render_ablation(self, bands):
        try:
            a = run_cli("voxels-ablate", "--archive", self.archive, "--bands", bands)
        except Exception as exc:                          # noqa: BLE001
            self.abl_verdict.configure(text=f"ablation failed: {exc}")
            return

        for key in ("interleaved", "concatenated", "split"):
            self.abl_rows[key].configure(text=fmt_bytes(a[key]))
        self.abl_rows["split"].configure(
            text=f"{fmt_bytes(a['split'])}   "
                 f"(x {fmt_bytes(a['split_x'])}, y {fmt_bytes(a['split_y'])}, "
                 f"z {fmt_bytes(a['split_z'])})")

        grouping, separation = a["grouping_gain"], a["separation_gain"]
        # Three regimes, and the small one is genuinely uninformative rather
        # than a null result: with only a few hundred voxels the payloads are
        # smaller than zstd's own framing, so splitting into three streams pays
        # three frame headers and can come out BEHIND (0.88x at 24 voxels).
        if a["voxels"] < 1000:
            verdict = (f"too few voxels ({a['voxels']:,}) to conclude anything -- at this "
                       f"size zstd's per-stream framing dominates, and splitting into three "
                       f"streams can even lose. Slide right to compare on real volumes.")
        elif separation < 1.02:
            verdict = (f"grouping is worth {grouping:.2f}x; separate compressors add "
                       f"essentially nothing ({separation:.2f}x). The win is keeping each "
                       f"axis's values contiguous, not giving each its own stream -- "
                       f"per-channel splitting earns its place through progressive "
                       f"per-axis LOD and per-axis encoding choices, not compression.")
        else:
            verdict = (f"grouping {grouping:.2f}x, separate compressors a further "
                       f"{separation:.2f}x, {a['total_gain']:.2f}x overall. Grouping's share "
                       f"grows with volume; separation's shrinks toward 1.00x at full detail.")
        self.abl_verdict.configure(text=verdict)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", default=DEFAULT_ARCHIVE)
    args = parser.parse_args()
    if not os.path.isfile(args.archive):
        print(f"archive not found: {args.archive}\n"
              f"generate one with:\n"
              f"  graph2vxch -resolution 0.2 -o {args.archive} "
              f"datasets/new_college.graph", file=sys.stderr)
        return 1
    Vxch3DGui(args.archive).mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
