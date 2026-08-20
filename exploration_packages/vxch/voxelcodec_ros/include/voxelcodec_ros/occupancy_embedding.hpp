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
// unknown sits adjacent to free (0 -> 1), at the LOW end of the range, not
// at the midpoint. An earlier version of this mapping placed unknown at the
// numeric midpoint between free and occupied specifically to fix a
// different problem -- the wavelet DETAIL bands treating a newly-resolved
// unknown->free cell as far less informative (magnitude-1 step) than an
// unknown->occupied or free->occupied one (magnitude ~100-101) -- but that
// same embedded value also feeds band_0, the coarsest band, which is a
// literal per-tile AVERAGE. Once unknown sat at the midpoint, any tile with
// substantial unexplored territory averaged toward "medium," which decodes
// back to a medium occupancy percentage: a tile that was 100% unexplored
// could decode, at coarse/partial-band fidelity, to as much as 87% "occupied"
// -- phantom obstacles across ground nobody has ever seen. Confirmed against
// a real generated map (whole-map coarse average moved from ~6.6% to ~21%
// occupied purely from this remapping, with individual unexplored cells
// reading as high as 87%).
//
// Any nonzero gap between unknown and free necessarily leaks into that
// average whenever a tile mixes unknown and free cells (the common case,
// especially near a frontier) -- there's no shared linear embedding that's
// simultaneously "unknown reads as equidistant from both known states" for
// the detail bands and "unknown doesn't bias the coarse average" for band_0.
// Since band_0 is what a teammate actually sees first and the coarse
// reconstruction has to be trustworthy on its own, correctness of that
// average wins: unknown stays adjacent to free, same relative placement as
// the original v+1 mapping this header replaced. The detail-band
// under-weighting of newly-discovered free space this reintroduces is a
// real but much lower-severity issue (it affects send *priority*, not what
// the map claims is there) -- fixing it, if ever needed, requires a rate-
// distortion signal computed separately from the transmitted/reconstructed
// value, not another remapping of this shared embedding.
namespace voxelcodec_ros
{

inline std::uint32_t occupancy_to_embedded(std::int8_t v)
{
  return static_cast<std::uint32_t>(static_cast<int>(v) + 1);
}

// Inverse of occupancy_to_embedded. Values outside the legitimate [0,101]
// range (e.g. from a corrupted/foreign payload) clamp into OccupancyGrid's
// valid [-1,100] range rather than wrapping or producing garbage.
inline std::int8_t embedded_to_occupancy(std::uint32_t e)
{
  const int shifted = static_cast<int>(e) - 1;
  return static_cast<std::int8_t>(std::max(-1, std::min(100, shifted)));
}

}  // namespace voxelcodec_ros
