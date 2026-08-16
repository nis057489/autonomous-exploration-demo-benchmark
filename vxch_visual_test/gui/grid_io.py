"""Reader for the .grid format written by ../src/grid_io.hpp.

Layout: magic "VXG1" (4 bytes) | width u32 LE | height u32 LE |
resolution f32 LE | width*height bytes of int8 row-major occupancy data
(-1 = unknown, 0 = free, 100 = occupied, same semantics as
nav_msgs/OccupancyGrid.data). Kept dependency-free (just struct + numpy) so
the GUI never needs to link against the C++ codec -- it only ever reads
grids the vxch_cli binary already produced.
"""
import struct

import numpy as np

MAGIC = b"VXG1"
HEADER = struct.Struct("<4sIIf")


def read_grid(path):
    with open(path, "rb") as f:
        raw = f.read()
    magic, width, height, resolution = HEADER.unpack_from(raw, 0)
    if magic != MAGIC:
        raise ValueError(f"{path}: bad magic {magic!r}, expected {MAGIC!r}")
    data = np.frombuffer(raw, dtype=np.int8, count=width * height, offset=HEADER.size)
    return width, height, resolution, data.reshape((height, width))


def grid_to_rgb(cells):
    """Map occupancy values to grayscale (unknown=mid gray, free=white,
    occupied=black, linear in between), as an (H, W, 3) uint8 array."""
    cells = cells.astype(np.int32)
    gray = np.where(
        cells < 0,
        190,
        np.clip(255 - (cells * 255 // 100), 0, 255),
    ).astype(np.uint8)
    return np.stack([gray, gray, gray], axis=-1)
