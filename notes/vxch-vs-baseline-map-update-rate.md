# VXCH vs baseline map update triggers — history and final decision

Date: 2026-08-06 (updated same day after design discussion)

## Where this landed

VXCH and baseline now fire on the **same trigger** (every incoming `/map` message from
slam_toolbox) with **no independent scheduling on either side**. This was a deliberate
simplification for a paper that's about the VXCH codec specifically (delta +
multiresolution Haar encoding), not about link scheduling — see "Why the scheduling was
removed" below for the reasoning.

### Current behavior

File: `exploration_packages/vxch/voxelcodec_ros/src/occupancy_grid_vxch_node.cpp`

- `on_map()` runs synchronously on every `/map` callback: encodes the whole grid as one
  Haar pyramid (`make_haar_bands()`), fingerprints each band, and publishes the manifest
  + only the bands that changed since the last transmission. No timer, no tiling, no
  per-tick caps, no debounce.
- Baseline (`ddil_proxy_node` relaying raw `nav_msgs/OccupancyGrid`) also fires on every
  incoming `/map` message, with no diffing — it relays the full grid every time.
- Both paths pass through the same token-bucket bandwidth/loss/delay simulator in
  `ddil_proxy_node.cpp`.

So the only remaining variable between the two arms is the codec itself: VXCH sends
only-changed, multiresolution-encoded bands; baseline sends the full raw grid,
unconditionally, every publish. That is the intended apples-to-apples comparison.

## Why the scheduling was removed

Earlier iterations added several traffic-shaping mechanisms on top of the codec:
per-tile Haar pyramids (`tile_size_m`), round-robin fairness across tiles, per-tick
band/tile send caps (`max_bands_per_update`, `max_tiles_per_update`), a per-tile resend
debounce (`min_resend_interval_s`), and a fixed 1 Hz send timer decoupled from `/map`
(`send_rate_hz`). These were introduced over several commits (`c5d899f`, `2a7edd1`,
`028f951`, `df71938`/`dd51d9a`) to solve real link-contention problems for live
multi-robot demos — see history below.

None of that is part of the paper's claim. Keeping it in would have invited an
ablation request ("how much of your bandwidth win is the codec vs. the scheduling?")
that's hard to answer cleanly in a paper scoped to the codec. Decision: strip it all
back to the pre-tiling, pre-scheduling form (matching commit `5e1af5c`, "2D haar" —
the commit right before `c5d899f` started adding scheduling), keeping only the
band-level diff/fingerprint skip, which *is* the codec.

A pull-based "request higher detail for specific tiles" mechanism was also considered
(broadcast coarse bands to everyone, only send fine detail to a peer that's actually
planning to go there) but was scoped out for the same reason — it would be a second,
genuinely novel systems contribution bolted onto a codec paper, requiring its own
ablation. No such mechanism exists in the codebase today; `ddil_proxy_node` is strictly
one-way (source → peer), and the existing `SetBand`/`SetLevel` services
(`voxelcodec_msgs/srv/`) are only wired up locally in the RViz `progressive_panel` for
manual debugging, not cross-robot. Noted here as a candidate follow-on, out of scope
for this paper.

## Commit history (for context)

- `5e1af5c` "2D haar" — original single-pyramid, event-driven, diff-only encoder. This
  is what the node was reverted back to.
- `c5d899f` "only send one band at a time", `2a7edd1` "tiling based map transmission
  with weighted age-out", `028f951` "round-robin fairness for band transmission in
  active tiles" — added tiling + per-tick scheduling to reduce link contention across
  robots for live demos.
- `df71938` "smarter vxch update schedule" (Aug 4) — added `min_resend_interval_s` with
  an 8.0s default baked into the node.
- `ad22c52` "Revert..." — reverted it; the 8s floor made the DDIL map look stalled/slow
  to converge on real hardware.
- `dd51d9a` "smarter vxch update schedule" (Aug 5) — reintroduced the mechanism with a
  2.0s opt-in default instead of 8.0s.
- This change (same day) — removed all of the above scheduling/tiling machinery
  entirely for the paper-scoped comparison, restoring the `5e1af5c` architecture with a
  `tile_size_cells` manifest field kept for decoder backward-compatibility (any nonzero
  value works; the decoder's untagged/untiled fallback already treats the whole grid as
  one tile).
