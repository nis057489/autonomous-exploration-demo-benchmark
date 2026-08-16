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
maps/                   generated grids/sessions/state (gitignored)
```
