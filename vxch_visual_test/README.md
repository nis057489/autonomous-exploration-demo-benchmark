# vxch visual test

Exercise the real vxch Haar-band codec (`exploration_packages/vxch/voxelcodec_ros`)
against a synthetic occupancy grid, and watch it progressively stream over a
simulated bandwidth-limited link -- no ROS, no robots, no DDS discovery.

It's split in two pieces:

- `src/` -- a small standalone C++ CLI (`vxch_cli`) that compiles
  `voxelcodec_ros/src/codec.cpp` directly from `exploration_packages/vxch`
  (by path, not copied, so it can never drift from what actually ships) and
  wraps it with `gen-map` / `encode` / `step` subcommands. This is the only
  place that touches codec logic.
- `gui/` -- a Tkinter GUI that shells out to `vxch_cli` for every action and
  otherwise only reads the plain grid files it writes. It never reimplements
  any codec/Haar-wavelet math itself.

## Build

Same toolchain as the rest of the repo -- run it inside the `jazzy_env`
distrobox (needed for tkinter/numpy/Pillow, and for a consistent
zstd/nlohmann_json setup):

```
cd vxch_visual_test
distrobox enter jazzy_env -- bash -lc "cmake -S . -B build && cmake --build build -j"
```

## Run

```
distrobox enter jazzy_env -- python3 gui/vxch_gui.py
```

1. **Generate Map** -- procedurally synthesizes an occupancy grid (outer
   walls, a couple of rooms with doorways, scattered obstacles, and an
   irregular unexplored fringe) and writes it to `maps/synthetic_map.grid`.
2. **Encode** -- runs `make_haar_bands` per tile (same tiling
   `occupancy_grid_vxch_node` does) and builds the ordered band send queue
   in `maps/session.vxch`. Compression (zstd/none) and varint packing
   (on/off) are both exposed here for ablation -- e.g. compression=none +
   varint off isolates the wavelet transform's own bandwidth win from
   everything layered on top of it. Mirrors the real node's `compression`
   and `varint_encoding` parameters (see `experiment.conf`).
3. **Send Next Band** / **Send All** -- pops the next queued band, decodes
   it into the receiver's per-tile state via the same
   `reconstruct_haar_from_bands` the real `vxch_occupancy_grid_node` uses,
   and re-renders the receiver's current reconstruction next to the
   sender's ground truth. Each send is paced by `payload_bytes * 8 /
   bandwidth_kbps` -- an honest (not fudged/animated) simulated wire delay --
   at whatever bandwidth is set, with presets lifted from
   `../wifi_profiles.json` (good/degraded/denied).

State persists across restarts in `maps/` (`.grid`/`.vxch`/`state.json`,
all gitignored) -- reopening the GUI resumes wherever the last run left off.

## 3D mode (sparse voxels)

Everything above is the **2D mode**: a dense occupancy grid, tiled, each tile
carrying its own 2D Haar pyramid. The **3D mode** is a genuinely different
representation, not the same algorithm with an extra axis:

| | 2D mode | 3D mode |
|---|---|---|
| input | dense occupancy grid | sparse voxel list |
| transform | per-tile 2D Haar (row + column pass) | per-axis Haar over the x/y/z coordinate *sequences* |
| adding a band | sharpens a blurry image | roughly **doubles the voxel count** |

The first *k* bands of a coordinate channel are a lossless representation of
that sequence downsampled by `2^(levels-k)`, so a coarse level is fewer voxels
spanning the same volume -- not a blurrier field. The run structure a 3D map
compresses on lives in the coordinate sequences (column sorting makes a
floor-to-ceiling obstacle emit `z, z+1, z+2, ...` within one column), which is
why reasoning about this mode as if it were a dense volume gives the wrong
answer about which axis compresses well.

### Getting a dataset

`datasets/new_college.graph` (Oxford New College, ~393 MB, gitignored) is the
source map. Convert it to a `.vxch` archive with the Go reference encoder from
`voxel_channels_2` -- C++ can currently *read* these archives but not write
them:

```sh
go build -o /tmp/graph2vxch ./cmd/graph2vxch      # in voxel_channels_2
/tmp/graph2vxch -resolution 0.2 \
    -o datasets/new_college_0p2.vxch datasets/new_college.graph
```

That reduces 14.5 M raw points to 775,677 voxels in a 611 KB archive with a
19-band ladder.

### Run

```sh
./build/vxch_cli voxels-info --archive datasets/new_college_0p2.vxch
./build/vxch_cli voxels-decode --archive datasets/new_college_0p2.vxch \
    --bands 12 --out /tmp/voxels.txt
python3 gui/vxch_3d_gui.py --archive datasets/new_college_0p2.vxch
```

The GUI shows three orthographic density projections (top X-Y, front X-Z, side
Y-Z) with a band slider. Panels keep a fixed extent taken from the full-detail
decode, so coarse levels visibly cover the same volume rather than rescaling to
their own bounding box. It also prints the per-channel byte split, because
*which axis is expensive* is the central question in 3D mode and depends on the
sort order the encoder used -- on the New College archive `x` costs 7 KB while
`z` costs 382 KB, since that encoder sorts by `x` and leaves `z` unsorted.

The band slider reports what reaching that level of detail actually **costs on
the wire**, per channel and in total. That number is measured, not estimated: a
Haar channel stores its coefficients coarsest-first in one varint stream, so
"the first k bands" is literally a prefix of that stream, and the CLI walks the
varints to find where the prefix ends and re-compresses exactly that.

Measured LOD ladder on New College at 0.2 m:

| bands | voxels | level cost | share of full | x | y | z |
|---|---|---|---|---|---|---|
| 1..1 | 3 | 42 B | 0.0% | 15 B | 15 B | 12 B |
| 1..4 | 24 | 120 B | 0.0% | 41 B | 46 B | 33 B |
| 1..8 | 379 | 974 B | 0.2% | 228 B | 486 B | 260 B |
| 1..12 | 6,060 | 13.0 KB | 2.2% | 978 B | 7.8 KB | 4.2 KB |
| 1..16 | 96,960 | 121.9 KB | 20.5% | 3.3 KB | 60.2 KB | 58.5 KB |
| 1..19 | 775,677 | 575.9 KB | 96.7% | 6.3 KB | 201.6 KB | 368.1 KB |

That is the streaming claim in one table: a recognisable map of the whole site
costs **13 KB, about 2% of the archive**, and a peer can stop there if that is
enough to plan against.

Full detail reports 96.7% rather than 100% because the level cost re-compresses
the coefficient prefix rather than reusing the encoder's original zstd
invocation, so it lands a few percent off the stored size. Treat the level
costs as "what this prefix costs to send", not as a byte-exact slice of the
file on disk.

### Ablation: does splitting x/y/z actually do anything?

The GUI's ablation panel (and `vxch_cli voxels-ablate --archive A --bands N`)
takes the **same coordinate values** at the current level of detail and
regroups them three ways, so the only variable is byte layout:

| arrangement | layout | streams |
|---|---|---|
| interleaved | `x0,y0,z0, x1,y1,z1, ...` | 1 |
| concatenated | `x0..xn, y0..yn, z0..zn` | 1 |
| split | `x` \| `y` \| `z` | 3 |

That separates two claims usually bundled together as "channel separation":
interleaved→concatenated is **grouping** (keeping an axis's values contiguous),
concatenated→split is **separation** proper (a compression context per axis).

Measured on New College at 0.2 m:

| bands | voxels | interleaved | concatenated | split | grouping | separation |
|---|---|---|---|---|---|---|
| 1..4 | 24 | 128 B | 128 B | 146 B | 1.00x | 0.88x |
| 1..8 | 379 | 1,630 B | 1,632 B | 1,562 B | 1.00x | 1.04x |
| 1..12 | 6,060 | 24.3 KB | 16.6 KB | 15.0 KB | 1.46x | 1.11x |
| 1..16 | 96,960 | 318.1 KB | 139.9 KB | 136.1 KB | 2.27x | 1.03x |
| 1..19 | 775,677 | 1,718 KB | 492.5 KB | 492.8 KB | **3.49x** | **1.00x** |

**Grouping is the entire effect, and it scales with volume (1.00x → 3.49x).
Separate compressors add nothing at full size** -- at 775k voxels split is a
hair *larger* than concatenated, being three zstd frames instead of one.

This reproduces the recorded `fr_079` benchmark in `voxel_channels_2`
independently: `interleaved_zstd / split_colsorted_zstd` = 3.68x there against
3.49x here, and its `split_colsorted` vs `perchannel_colsorted` differ by
1.001x -- the same null result for separation proper.

So per-channel splitting should not be justified on compression grounds. It
earns its place functionally: progressive per-axis LOD, and per-axis encoding
choices (`column-run` on a column-sorted `z`, delta on `x`/`y`) that a single
merged stream could not express.

Below ~1,000 voxels the ablation is uninformative rather than null: the payloads
are smaller than zstd's own framing, so three streams pay three frame headers
and split comes out *behind* (0.88x at 24 voxels). The GUI says so instead of
reporting a ratio.

The table above was produced against zstd 1.5.7. The **grouping** magnitude is
sensitive to the zstd version -- the same input gives 3.49x under 1.5.7 and
3.65x under 1.5.5 (the version inside `jazzy_env`), because the binary links
whichever libzstd its runtime provides. The **separation** null is not
sensitive: it stays within 0.1% of 1.00x in both. Quote the null freely; quote
the grouping magnitude with its environment. See
`docs/claim_grouping_not_splitting.md`.

## Compression/varint sweep

`compression_size_sweep.py` sweeps map size against the full compression x
varint ablation matrix (the same four points as the GUI's Compression/Varint
controls) via `vxch_cli gen-map`/`encode` and plots encoded band-stream size,
absolute and as a percentage of the raw `OccupancyGrid` size:

```
distrobox enter jazzy_env -- python3 compression_size_sweep.py
```

Writes `figures/compression_size_sweep.png` (gitignored) and prints the
underlying numbers to stdout. `none + fixed-width` is a useful sanity check
in its own right -- it should sit at a flat 400% of raw at every size (int32
per coefficient vs. int8 per raw cell), independent of map content.

## What's real vs. simplified

The encode/decode math (Haar forward/inverse, zigzag-varint, zstd
compression, per-tile reconstruction, nearest-neighbour upsampling of a
partially-received tile) is the exact code the real nodes run.

The one thing this tool does *not* reuse is `occupancy_grid_vxch_node`'s
send-scheduling state machine (per-tile fingerprinting, recency-based
round robin) -- that machinery exists to handle a map that keeps changing
while it's being sent. A synthetic snapshot never changes, so the send
order here is the simplified static-map case of the same round robin:
every tile's band 0 (coarsest), then every tile's band 1, and so on --
which is exactly what the real scheduler converges to once nothing is
re-changing. See `build_send_order` in `src/main.cpp` for the full
reasoning.

## Files

```
CMakeLists.txt        builds vxch_cli against ../exploration_packages/vxch
src/grid_io.hpp        .grid file format (read/write)
src/map_gen.hpp        synthetic occupancy grid generator
src/main.cpp           gen-map / encode / step subcommands
gui/grid_io.py          Python .grid reader (numpy) + grayscale colorizing
gui/vxch_gui.py         Tkinter GUI
compression_size_sweep.py  map-size x compression/varint ablation sweep + plot
maps/                   generated grids/sessions/state (gitignored)
figures/                generated plots (gitignored)
```
