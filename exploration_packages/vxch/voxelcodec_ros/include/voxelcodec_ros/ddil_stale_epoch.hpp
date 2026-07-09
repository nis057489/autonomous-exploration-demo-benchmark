#pragma once

#include <cstdint>
#include <optional>

namespace voxelcodec_ros
{

// Plain, ROS-free timestamp -- mirrors builtin_interfaces/msg/Time's fields
// (sec is a signed int32, valid over its full range; nanosec is unsigned,
// valid in [0, 1e9)) without pulling in any ROS message headers.
struct Stamp
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0};
};

// Chronological "a happened before b". Never compare nanosec alone -- e.g.
// {100, 999999999} is older than {101, 0} despite the larger nanosec.
inline bool stamp_less(const Stamp & a, const Stamp & b)
{
  if (a.sec != b.sec) {
    return a.sec < b.sec;
  }
  return a.nanosec < b.nanosec;
}

// True iff a VXCH band carrying band_stamp is stale relative to the newest
// manifest already sent (latest_manifest_stamp) and can be safely dropped
// without spending bandwidth on data the receiver would discard anyway.
//
// Strict less-than, not equality: DDS gives no cross-topic delivery-order
// guarantee, and manifest (RELIABLE) vs bands (BEST_EFFORT) take different
// delivery paths, so a brand-new epoch's band can legitimately reach the
// front of the relay queue before that same epoch's manifest has been sent
// yet. An equality check would incorrectly drop that good, new band. Only a
// band strictly OLDER than the newest manifest already sent is truly stale.
//
// No manifest sent yet (nullopt) means nothing can be judged stale -- note
// this is NOT the same as a manifest with stamp {0,0}, which is a real,
// reachable timestamp under use_sim_time before /clock has advanced.
inline bool should_drop_as_stale(
  const Stamp & band_stamp, const std::optional<Stamp> & latest_manifest_stamp)
{
  if (!latest_manifest_stamp.has_value()) {
    return false;
  }
  return stamp_less(band_stamp, *latest_manifest_stamp);
}

}  // namespace voxelcodec_ros
