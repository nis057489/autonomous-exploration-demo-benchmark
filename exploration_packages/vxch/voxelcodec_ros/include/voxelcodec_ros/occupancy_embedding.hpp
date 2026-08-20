#pragma once

#include <algorithm>
#include <cstdint>

// Encoder/decoder-shared mapping between OccupancyGrid's int8 cell values
// (-1 = unknown, 0..100 = occupancy percent) and the uint32 domain the Haar
// pyramid is built over. This is the one occupancy-specific piece of an
// otherwise domain-agnostic tiling/transform/scheduling pipeline (see
// TileScheduler / TileReconstructor / make_haar_bands) -- kept in its own
// header, rather than inlined into those generic classes, so a future
// non-occupancy voxel domain (TSDF, semantic labels, ...) has an obvious,
// separate seam to plug its own embedding into instead of occupancy's
// assumptions leaking into code meant to work for any channel.
//
// unknown sits at the numeric MIDPOINT of the embedded range, not adjacent
// to free. Under the previous mapping (v+1: unknown=0, free=1, occupied=101),
// a cell resolving unknown->free was a magnitude-1 step while unknown->
// occupied was magnitude-101 and free->occupied was magnitude-100 -- so any
// signal built on coefficient/value magnitude (e.g. a rate-distortion send
// priority derived from Haar coefficient energy) would treat newly-
// discovered free space, exactly what frontier exploration produces most
// of, as ~100x less informative than an occupied-state flip, for no reason
// tied to actual uncertainty reduction. Placing unknown equidistant from
// both known extremes fixes that: unknown->free and unknown->occupied now
// produce comparably large steps (101 and 99).
//
// Known values are doubled (0,2,...,200) to open the one odd slot (101)
// unknown needs, so the mapping stays an exact bijection over all 102 legal
// OccupancyGrid values -- lossless, same guarantee the mapping it replaces
// made.
namespace voxelcodec_ros
{

inline std::uint32_t occupancy_to_embedded(std::int8_t v)
{
  if (v == -1) {
    return 101U;
  }
  return static_cast<std::uint32_t>(v) * 2U;
}

// Inverse of occupancy_to_embedded. Values outside the legitimate
// {0,2,...,200} ∪ {101} range (e.g. from a corrupted/foreign payload) clamp
// into OccupancyGrid's valid [-1,100] range rather than wrapping or
// producing garbage -- same defensive behavior the mapping it replaces had.
inline std::int8_t embedded_to_occupancy(std::uint32_t e)
{
  if (e == 101U) {
    return -1;
  }
  const int known = static_cast<int>(e / 2U);
  return static_cast<std::int8_t>(std::max(0, std::min(100, known)));
}

}  // namespace voxelcodec_ros
