#pragma once

// Pure tile-decode/reconstruction logic behind vxch_occupancy_grid_node.cpp's
// Node, pulled into a header so it's testable without an rclcpp::Node fixture
// -- same reasoning as ddil_stale_epoch.hpp. Node-specific glue (parameter
// declaration, subscriptions, the wall timer, RCLCPP logging, and the final
// nav_msgs::msg::OccupancyGrid message construction) stays in the .cpp;
// TileReconstructor returns a plain ReconstructedGrid for the caller to wrap.

#include <algorithm>
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
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{

// Reverse the uint32 shift applied by the encoder: 0→-1, 1→0, 101→100, etc.
inline std::int8_t unshift_from_uint32(std::uint32_t v)
{
  const int shifted = static_cast<int>(v) - 1;
  return static_cast<std::int8_t>(std::max(-1, std::min(100, shifted)));
}

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

    if (new_width != geometry_.grid_width || new_height != geometry_.grid_height ||
      (new_tile_size_cells != 0 && new_tile_size_cells != geometry_.tile_size_cells))
    {
      tiles_.clear();
      geometry_.grid_width = new_width;
      geometry_.grid_height = new_height;
    }
    if (new_tile_size_cells != 0) {
      geometry_.tile_size_cells = new_tile_size_cells;
    }

    const std::string res_str = get_meta("resolution");
    geometry_.grid_resolution = res_str.empty() ? 0.05F : std::stof(res_str);
    const std::string ox = get_meta("origin_x");
    geometry_.origin_x = std::stod(ox.empty() ? "0" : ox);
    const std::string oy = get_meta("origin_y");
    geometry_.origin_y = std::stod(oy.empty() ? "0" : oy);
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
      if (bands_received == 0) {continue;}

      HaarReconstruction recon;
      try {
        recon = reconstruct_haar_from_bands(
          tile.band_coeffs, static_cast<std::size_t>(tile.width),
          static_cast<std::size_t>(tile.height), tile.levels, bands_received);
      } catch (const std::exception &) {
        continue;
      }

      const std::size_t tw = static_cast<std::size_t>(tile.width);
      const std::size_t th = static_cast<std::size_t>(tile.height);
      const std::size_t w_prime = recon.width;
      const std::size_t h_prime = recon.height;

      const std::size_t row0 = static_cast<std::size_t>(key.first) *
        static_cast<std::size_t>(geometry_.tile_size_cells);
      const std::size_t col0 = static_cast<std::size_t>(key.second) *
        static_cast<std::size_t>(geometry_.tile_size_cells);

      // 2D nearest-neighbour upsample if we only have a coarse reconstruction
      // of this tile -- a real (blurry but spatially faithful) downsampled
      // patch, pasted at this tile's place in the full-resolution output grid.
      for (std::size_t r = 0; r < th; ++r) {
        const std::size_t r_src = (w_prime == tw && h_prime == th) ? r : (r * h_prime / th);
        const std::size_t dst_row = row0 + r;
        if (dst_row >= geometry_.grid_height) {break;}
        for (std::size_t c = 0; c < tw; ++c) {
          const std::size_t c_src = (w_prime == tw && h_prime == th) ? c : (c * w_prime / tw);
          const std::size_t dst_col = col0 + c;
          if (dst_col >= geometry_.grid_width) {continue;}
          grid_data[dst_row * geometry_.grid_width + dst_col] =
            unshift_from_uint32(recon.values[r_src * w_prime + c_src]);
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
  int haar_levels_;
  GridGeometry geometry_;
  std::map<TileKey, TileBandState> tiles_;
};

}  // namespace voxelcodec_ros
