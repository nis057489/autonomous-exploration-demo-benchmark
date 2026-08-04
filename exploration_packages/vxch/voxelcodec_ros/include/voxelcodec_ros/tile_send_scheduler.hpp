#pragma once

#include <optional>

namespace voxelcodec_ros
{

// True iff a tile is eligible to be serviced (have pending bands actually
// published) again right now, given when it was last actually sent and a
// minimum resend interval.
//
// A tile that has never been sent (nullopt) is always eligible -- debouncing
// only throttles *repeat* sends of a tile already delivered at least once.
// Without this, a tile sitting near an actively-exploring robot re-dirties on
// nearly every SLAM map update (ordinary occupancy-probability noise, not
// just genuine new information) and gets resent every send tick, defeating
// the coalescing that pending-band storage already does for content that
// hasn't been sent yet (a redirty before the tile's turn just overwrites its
// still-queued payload) -- coalescing only has a chance to fire if the tile
// is actually made to wait.
inline bool tile_resend_eligible(
  const std::optional<double> & last_sent_seconds, double now_seconds,
  double min_resend_interval_s)
{
  if (!last_sent_seconds.has_value()) {
    return true;
  }
  return (now_seconds - *last_sent_seconds) >= min_resend_interval_s;
}

}  // namespace voxelcodec_ros
