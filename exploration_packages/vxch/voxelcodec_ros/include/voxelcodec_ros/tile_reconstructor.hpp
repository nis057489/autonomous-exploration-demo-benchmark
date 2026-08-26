#pragma once

// Pure tile-decode/reconstruction logic behind vxch_occupancy_grid_node.cpp's
// Node, pulled into a header so it's testable without an rclcpp::Node fixture
// -- same reasoning as ddil_stale_epoch.hpp. Node-specific glue (parameter
// declaration, subscriptions, the wall timer, RCLCPP logging, and the final
// nav_msgs::msg::OccupancyGrid message construction) stays in the .cpp;
// TileReconstructor returns a plain ReconstructedGrid for the caller to wrap.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/ddil_stale_epoch.hpp"
#include "voxelcodec_ros/occupancy_embedding.hpp"
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{

// Zigzag-varint decode (mirrors zigzag_varint_encode in haar_forward.hpp).
inline std::vector<std::int64_t> zigzag_varint_decode(
  const std::vector<std::uint8_t> & raw, std::size_t count)
{
  std::vector<std::int64_t> out;
  out.reserve(count);
  std::size_t offset = 0;
  while (out.size() < count) {
    std::uint64_t value = 0;
    int shift = 0;
    while (offset < raw.size()) {
      const std::uint8_t byte = raw[offset++];
      value |= static_cast<std::uint64_t>(byte & 0x7FU) << shift;
      if ((byte & 0x80U) == 0) {break;}
      shift += 7;
      if (shift >= 64) {throw std::runtime_error("varint overflow");}
    }
    const std::int64_t decoded = (value & 1U)
      ? -static_cast<std::int64_t>((value >> 1U) + 1U)
      : static_cast<std::int64_t>(value >> 1U);
    out.push_back(decoded);
  }
  return out;
}

// Mirrors fixed_width_encode in haar_forward.hpp -- the ablation-mode counterpart to
// zigzag_varint_decode above, selected per band via the descriptor's kHaarVarintKey.
inline std::vector<std::int64_t> fixed_width_decode(
  const std::vector<std::uint8_t> & raw, std::size_t count)
{
  std::vector<std::int64_t> out(count);
  for (std::size_t i = 0; i < count; ++i) {
    std::int32_t v;
    std::memcpy(&v, &raw[i * 4], 4);
    out[i] = v;
  }
  return out;
}

using TileKey = std::pair<int, int>;

// Per-tile band state: stores decoded int64 coefficients per received band,
// scoped to that tile's own (possibly smaller-than-nominal, at grid edges)
// cell extent.
struct TileBandState
{
  int width{0};
  int height{0};
  int levels{0};
  int total_bands{0};
  std::vector<std::vector<std::int64_t>> band_coeffs;  // [band_index] → coefficients
  std::vector<bool> received;
};

struct GridGeometry
{
  bool have_manifest{false};
  std::uint32_t grid_width{0};
  std::uint32_t grid_height{0};
  int tile_size_cells{0};
  float grid_resolution{0.05f};
  double origin_x{0.0};
  double origin_y{0.0};
  std::string frame_id{"map"};
  Stamp manifest_stamp{};
};

struct ReconstructedGrid
{
  std::vector<std::int8_t> data;  // row-major, size width*height
  std::uint32_t width{0};
  std::uint32_t height{0};
  float resolution{0.05f};
  double origin_x{0.0};
  double origin_y{0.0};
  std::string frame_id{"map"};
  Stamp stamp{};
};

// Accumulates manifest + per-tile band state from a (possibly tiled) vxch
// occupancy stream and reconstructs the best-available full-resolution grid
// at any point -- tiles with only coarse bands received so far are
// nearest-neighbour upsampled into their place in the output.
class TileReconstructor
{
public:
  explicit TileReconstructor(int haar_levels)
  : haar_levels_(haar_levels)
  {}

  // Parses grid geometry out of manifest metadata. Returns false (metadata
  // incomplete -- no grid_width/grid_height) without changing any state.
  // Bands are sent progressively, so a fresher manifest stamp alone does NOT
  // mean every tile/band needs to arrive again -- only clears accumulated
  // tile state when the grid geometry or tile partition actually changes.
  bool ingest_manifest(const Metadata & metadata, const Stamp & stamp)
  {
    const auto get_meta = [&](const std::string & key) -> std::string {
        auto it = metadata.find(key);
        return (it != metadata.end()) ? it->second : "";
      };

    const std::string w_str = get_meta("grid_width");
    const std::string h_str = get_meta("grid_height");
    if (w_str.empty() || h_str.empty()) {
      return false;
    }

    const auto new_width = static_cast<std::uint32_t>(std::stoul(w_str));
    const auto new_height = static_cast<std::uint32_t>(std::stoul(h_str));
    const std::string tile_size_str = get_meta("tile_size_cells");
    const int new_tile_size_cells = tile_size_str.empty() ? 0 : std::stoi(tile_size_str);
    // Parsed before the clear-decision below so geometry_.origin_x/y still hold the
    // previous manifest's values for comparison.
    const std::string ox = get_meta("origin_x");
    const double new_origin_x = std::stod(ox.empty() ? "0" : ox);
    const std::string oy = get_meta("origin_y");
    const double new_origin_y = std::stod(oy.empty() ? "0" : oy);

    // tile_row/tile_col are now world-anchored (see TileScheduler::
    // compute_axis_tile_spans): a tile's key is which cell of a fixed
    // world-cell lattice it falls in, not an array-local offset. That makes a
    // tile's identity, and reconstruct()'s placement of it, independent of
    // this array's own origin/width/height -- so NEITHER grid_width/height
    // growing NOR an origin shift invalidates any already-decoded tile
    // anymore; reconstruct() re-derives each tile's current placement (which
    // can differ call to call as origin moves) from geometry_.origin_x/y and
    // the tile's world-anchored key every time it runs.
    //
    // Before world-anchoring, ANY origin change unconditionally cleared the
    // entire cache -- and slam_toolbox recomputes/republishes origin
    // routinely (not just on rare boundary extensions, often by less than one
    // cell, pure float noise relative to the actual array), so that cleared
    // far more often than any genuine reindexing ever occurred. Combined with
    // "smart" mode never resending a tile whose content hasn't changed, a
    // peer's whole accumulated map could and did go missing over origin
    // nudges unrelated to any specific tile's content. See
    // project_ddil_bandqueue_starvation_fix / the tile-reconstructor-reset-fix
    // memory for the investigation trail.
    //
    // Only a tile-partition-shape change (tile_size_cells itself changing --
    // a real resolution rescale) genuinely invalidates every existing key's
    // meaning; that still clears.
    if (new_tile_size_cells != 0 && new_tile_size_cells != geometry_.tile_size_cells) {
      tiles_.clear();
    }
    geometry_.grid_width = new_width;
    geometry_.grid_height = new_height;
    if (new_tile_size_cells != 0) {
      geometry_.tile_size_cells = new_tile_size_cells;
    }

    const std::string res_str = get_meta("resolution");
    geometry_.grid_resolution = res_str.empty() ? 0.05F : std::stof(res_str);
    geometry_.origin_x = new_origin_x;
    geometry_.origin_y = new_origin_y;
    geometry_.frame_id = get_meta("frame_id");
    if (geometry_.frame_id.empty()) {geometry_.frame_id = "map";}
    geometry_.manifest_stamp = stamp;
    geometry_.have_manifest = true;
    return true;
  }

  // Decodes one band message for one tile. compressed_payload is decompressed
  // internally per descriptor.compression. Returns an error description
  // instead of throwing (mirrors production's catch-and-RCLCPP_WARN) -- a
  // decode failure for one band must never take down the whole node.
  // No-ops (returns nullopt, nothing recorded) if no manifest has arrived yet.
  std::optional<std::string> ingest_band(
    int band_index, const ChannelDescriptor & descriptor,
    const std::vector<std::uint8_t> & compressed_payload)
  {
    if (!geometry_.have_manifest) {
      return std::nullopt;
    }

    try {
      const auto get_desc_meta = [&](const std::string & key, int fallback) -> int {
          auto it = descriptor.metadata.find(key);
          return (it != descriptor.metadata.end()) ? std::stoi(it->second) : fallback;
        };
      // Untagged (non-tiled) publishers are treated as a single tile (0,0)
      // covering the whole grid, for backward compatibility with an untiled encoder.
      const TileKey key{get_desc_meta("tile_row", 0), get_desc_meta("tile_col", 0)};
      const int tile_w = get_desc_meta("tile_width", static_cast<int>(geometry_.grid_width));
      const int tile_h = get_desc_meta("tile_height", static_cast<int>(geometry_.grid_height));

      auto & tile = tiles_[key];
      if (tile.width != tile_w || tile.height != tile_h || tile.levels != haar_levels_) {
        // A tile straddling the array's growing edge legitimately changes size
        // often (see TileScheduler::ingest_grid's covered_w/covered_h) -- the
        // Haar coefficients for one size aren't a valid prefix/subset of
        // another size's, so mixing old- and new-size bands would decode to
        // garbage, and wiping band_coeffs/received here is the only correct
        // response. But reconstruct() below has bands_received==0 for a
        // freshly-wiped tile until every earlier band re-arrives at the new
        // size -- with nothing else, that's a visible blank/flicker at this
        // tile's place on every single resize, i.e. exactly the "stripe" a
        // moving frontier produces. reconstruct() caches the last full
        // reconstruction it computed per tile and falls back to it whenever
        // the live one has nothing yet, so this wipe only affects internal
        // decode state, never what's actually shown.
        tile = TileBandState{};
        tile.width = tile_w;
        tile.height = tile_h;
        tile.levels = haar_levels_;
        tile.total_bands = haar_levels_ + 1;
        tile.band_coeffs.assign(static_cast<std::size_t>(tile.total_bands), {});
        tile.received.assign(static_cast<std::size_t>(tile.total_bands), false);
      }

      const std::size_t idx = static_cast<std::size_t>(band_index);
      if (idx >= tile.band_coeffs.size()) {
        return std::nullopt;
      }

      const auto raw = decompress_payload(descriptor, compressed_payload);
      const bool use_varint = descriptor.metadata.at(kHaarVarintKey) == "1";
      tile.band_coeffs[idx] = use_varint ?
        zigzag_varint_decode(raw, descriptor.element_count) :
        fixed_width_decode(raw, descriptor.element_count);
      tile.received[idx] = true;
      return std::nullopt;
    } catch (const std::exception & e) {
      return std::string("band decode failed: ") + e.what();
    }
  }

  // Reconstructs the best-available full grid from whatever bands have been
  // received so far. Returns nullopt under the same conditions publish_map()
  // used to skip publishing (no manifest, incomplete geometry, or nothing
  // decodable yet).
  std::optional<ReconstructedGrid> reconstruct() const
  {
    if (!geometry_.have_manifest || geometry_.grid_width == 0 || geometry_.grid_height == 0 ||
      geometry_.tile_size_cells <= 0)
    {
      return std::nullopt;
    }

    std::vector<std::int8_t> grid_data(
      static_cast<std::size_t>(geometry_.grid_width) *
      static_cast<std::size_t>(geometry_.grid_height), -1);
    bool any_tile_rendered = false;

    // World-cell coordinate that THIS array's own local index 0 currently sits
    // at, along each axis -- see TileScheduler::compute_axis_tile_spans for the
    // matching encoder-side computation. Every tile's world-anchored key gets
    // converted back to a local offset in this specific reconstruction via this
    // same reference, so a tile encoded under one origin still lands in the
    // right place when reconstructed under a later (or earlier-known) one.
    const double resolution_d = static_cast<double>(geometry_.grid_resolution);
    const long long origin_cell_x =
      resolution_d > 0.0 ? std::llround(geometry_.origin_x / resolution_d) : 0;
    const long long origin_cell_y =
      resolution_d > 0.0 ? std::llround(geometry_.origin_y / resolution_d) : 0;

    for (const auto & [key, tile] : tiles_) {
      if (tile.width <= 0 || tile.height <= 0) {continue;}

      int bands_received = 0;
      for (int k = 0; k < tile.total_bands; ++k) {
        if (tile.received[static_cast<std::size_t>(k)]) {
          bands_received = k + 1;
        } else {
          break;
        }
      }

      // tw_use/th_use describe whichever reconstruction (live or cached)
      // actually ends up used below -- NOT necessarily tile.width/height,
      // since a cache hit is, by construction, from before the tile's most
      // recent resize-triggered wipe (see ingest_band) and so belongs to a
      // different size than the live tile currently declares.
      const HaarReconstruction * recon_ptr = nullptr;
      int tw_use = tile.width;
      int th_use = tile.height;

      const auto cache_it = tile_cache_.find(key);
      const int cached_bands =
        cache_it != tile_cache_.end() ? cache_it->second.bands_received : 0;

      // Only replace what's cached once the live reconstruction is at least
      // as detailed -- otherwise a resize's fresh coarsest-band-only content
      // would visibly *downgrade* an already-fuller cached reconstruction for
      // however many ticks it takes the rest of that tile's bands to
      // re-arrive (max_bands_per_update sends one band per tile per tick),
      // which is exactly the repeating blurry-fringe-at-the-frontier "stripe"
      // this cache exists to prevent -- a one-off blank flicker is only the
      // most extreme case of the same underlying problem.
      if (bands_received > 0 && bands_received >= cached_bands) {
        HaarReconstruction fresh;
        try {
          fresh = reconstruct_haar_from_bands(
            tile.band_coeffs, static_cast<std::size_t>(tile.width),
            static_cast<std::size_t>(tile.height), tile.levels, bands_received);
        } catch (const std::exception &) {
          continue;
        }
        auto & cached = tile_cache_[key];
        cached.recon = std::move(fresh);
        cached.tile_w = tile.width;
        cached.tile_h = tile.height;
        cached.bands_received = bands_received;
        recon_ptr = &cached.recon;
      } else if (cache_it != tile_cache_.end()) {
        recon_ptr = &cache_it->second.recon;
        tw_use = cache_it->second.tile_w;
        th_use = cache_it->second.tile_h;
      } else {
        continue;  // never decoded anything for this tile yet
      }

      const auto & recon = *recon_ptr;
      const std::size_t tw = static_cast<std::size_t>(tw_use);
      const std::size_t th = static_cast<std::size_t>(th_use);
      const std::size_t w_prime = recon.width;
      const std::size_t h_prime = recon.height;

      // This tile's world-cell coordinate (key * lattice quantum) minus this
      // array's own origin (in cells) gives its LOCAL offset in the grid
      // being built right now -- can be negative (tile sits before this
      // array's own left/top edge) or run past grid_width/height (sits past
      // its right/bottom edge). Both are normal: it just means none (or only
      // part) of this tile is within what this array currently covers, not
      // an error -- e.g. a peer's array hasn't grown far enough yet to
      // include a tile this receiver already decoded from an earlier,
      // larger-extent manifest.
      const long long row0 = static_cast<long long>(key.first) *
        static_cast<long long>(geometry_.tile_size_cells) - origin_cell_y;
      const long long col0 = static_cast<long long>(key.second) *
        static_cast<long long>(geometry_.tile_size_cells) - origin_cell_x;

      // 2D nearest-neighbour upsample if we only have a coarse reconstruction
      // of this tile -- a real (blurry but spatially faithful) downsampled
      // patch, pasted at this tile's place in the full-resolution output grid.
      for (std::size_t r = 0; r < th; ++r) {
        const std::size_t r_src = (w_prime == tw && h_prime == th) ? r : (r * h_prime / th);
        const long long dst_row = row0 + static_cast<long long>(r);
        if (dst_row < 0) {continue;}
        if (dst_row >= static_cast<long long>(geometry_.grid_height)) {break;}
        for (std::size_t c = 0; c < tw; ++c) {
          const std::size_t c_src = (w_prime == tw && h_prime == th) ? c : (c * w_prime / tw);
          const long long dst_col = col0 + static_cast<long long>(c);
          if (dst_col < 0 || dst_col >= static_cast<long long>(geometry_.grid_width)) {continue;}
          grid_data[static_cast<std::size_t>(dst_row) * geometry_.grid_width +
            static_cast<std::size_t>(dst_col)] =
            embedded_to_occupancy(recon.values[r_src * w_prime + c_src]);
        }
      }
      any_tile_rendered = true;
    }

    if (!any_tile_rendered) {return std::nullopt;}

    ReconstructedGrid out;
    out.data = std::move(grid_data);
    out.width = geometry_.grid_width;
    out.height = geometry_.grid_height;
    out.resolution = geometry_.grid_resolution;
    out.origin_x = geometry_.origin_x;
    out.origin_y = geometry_.origin_y;
    out.frame_id = geometry_.frame_id;
    out.stamp = geometry_.manifest_stamp;
    return out;
  }

  const GridGeometry & geometry() const {return geometry_;}

private:
  // Last-known-good reconstruction for one tile, kept across a resize-
  // triggered wipe of that same key's live TileBandState (see ingest_band)
  // purely so reconstruct() has something other than blank/unknown to show
  // while the new size's bands re-accumulate. tile_w/tile_h are the size
  // this recon was computed at -- generally NOT equal to the live tile's
  // current (post-wipe) width/height.
  struct CachedTile
  {
    HaarReconstruction recon;
    int tile_w{0};
    int tile_h{0};
    // How many leading bands recon was built from -- lets reconstruct()
    // refuse to replace a fuller cached reconstruction with a thinner live
    // one just because the tile resized (see reconstruct()'s comment).
    int bands_received{0};
  };

  int haar_levels_;
  GridGeometry geometry_;
  // Not actively pruned if the grid ever shrinks below a tile's offset --
  // reconstruct()'s bounds checks already skip placing such a tile, so a stale entry
  // here is a correctness no-op, only a minor unbounded-growth risk. Not observed in
  // practice (grids only grow as SLAM explores); revisit if that changes.
  std::map<TileKey, TileBandState> tiles_;
  // Updated only from reconstruct(), which is otherwise a pure read of
  // tiles_/geometry_ -- mutable so reconstruct() can stay const (it's called
  // from a timer callback purely to read out the current best-available
  // grid, not to mutate decode state).
  mutable std::map<TileKey, CachedTile> tile_cache_;
};

}  // namespace voxelcodec_ros
