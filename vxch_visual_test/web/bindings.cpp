// Emscripten/Embind wrapper exposing the real vxch codec (codec/ -- vendored
// from voxelcodec_ros, see codec/VENDORED.md) to JS, so the browser demo
// calls the actual C++ encode/decode path directly instead of a
// reimplementation. This mirrors src/main.cpp's gen-map/encode/step
// subcommands, restructured as in-memory object state instead of a
// stateless CLI round-tripping through files (there's no subprocess
// boundary to work around in a single-threaded page, so VxchDemo just
// keeps its session/receiver state as member variables between calls).
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <emscripten/bind.h>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/haar_forward.hpp"
#include "voxelcodec_ros/types.hpp"

#include "../src/grid_io.hpp"
#include "../src/map_gen.hpp"

using namespace voxelcodec_ros;
using emscripten::val;
using vxch_test::SyntheticGrid;

namespace
{

// Mirrors voxelcodec_ros/occupancy_embedding.hpp -- not vendored alongside
// codec.hpp/haar_forward.hpp/types.hpp (see codec/VENDORED.md) because this
// file already hand-duplicates the int8<->uint32 mapping rather than
// including tile_scheduler.hpp/tile_reconstructor.hpp, so it's kept in sync
// by hand like the rest of this vendored copy. Unknown stays adjacent to
// free (not at the embedded range's midpoint) -- band_0 is a per-tile
// average, and placing unknown further away lets an unexplored tile's
// coarse reconstruction drift toward "medium occupied" -- see the real
// header for the full rationale.
std::uint32_t shift_to_uint32(std::int8_t v)
{
  return static_cast<std::uint32_t>(static_cast<int>(v) + 1);
}

std::int8_t unshift_from_uint32(std::uint32_t v)
{
  const int shifted = static_cast<int>(v) - 1;
  return static_cast<std::int8_t>(std::max(-1, std::min(100, shifted)));
}

// Mirrors zigzag_varint_encode in voxelcodec_ros/haar_forward.hpp -- same
// duplication convention as the native decoder (vxch_occupancy_grid_node.cpp)
// and the CLI (src/main.cpp): the decode side isn't shared library code.
std::vector<std::int64_t> zigzag_varint_decode(
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

// Mirrors fixed_width_encode in voxelcodec_ros/haar_forward.hpp.
std::vector<std::int64_t> fixed_width_decode(
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

struct QueueEntry
{
  int tile_row;
  int tile_col;
  int tile_width;
  int tile_height;
  int band_index;
  ChannelDescriptor descriptor;
  std::vector<std::uint8_t> payload;
};

struct TileState
{
  int width{0};
  int height{0};
  int levels{0};
  int total_bands{0};
  std::vector<std::vector<std::int64_t>> band_coeffs;
  std::vector<bool> received;
};

// Same static-map round-robin simplification as src/main.cpp's
// build_send_order: wave 0 = every tile's band 0 (coarsest), wave 1 = every
// tile's band 1, and so on -- what occupancy_grid_vxch_node's real
// fingerprint+recency scheduler converges to once nothing is re-changing.
std::vector<QueueEntry> build_send_order(
  const std::vector<std::vector<EncodedChannel>> & tile_bands,
  const std::vector<std::pair<int, int>> & tile_coords,
  const std::vector<std::pair<int, int>> & tile_dims,
  int total_bands)
{
  std::vector<QueueEntry> order;
  for (int k = 0; k < total_bands; ++k) {
    for (std::size_t t = 0; t < tile_bands.size(); ++t) {
      if (static_cast<std::size_t>(k) >= tile_bands[t].size()) {continue;}
      const auto & band = tile_bands[t][static_cast<std::size_t>(k)];
      QueueEntry e;
      e.tile_row = tile_coords[t].first;
      e.tile_col = tile_coords[t].second;
      e.tile_width = tile_dims[t].first;
      e.tile_height = tile_dims[t].second;
      e.band_index = k;
      e.descriptor = band.descriptor;
      e.payload = band.payload;
      order.push_back(std::move(e));
    }
  }
  return order;
}

class VxchDemo
{
public:
  val genMap(int width, int height, unsigned seed)
  {
    grid_ = vxch_test::generate_synthetic_grid(
      static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height), 0.05f, seed);
    send_order_.clear();
    tiles_.clear();
    sent_index_ = 0;
    cumulative_bytes_ = total_compressed_bytes_ = 0;
    raw_bytes_ = static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height);
    receiver_grid_.assign(grid_.data.size(), -1);

    std::size_t free_c = 0, occ_c = 0, unk_c = 0;
    for (const auto v : grid_.data) {
      if (v < 0) {++unk_c;} else if (v == 0) {++free_c;} else {++occ_c;}
    }
    val result = val::object();
    result.set("width", width);
    result.set("height", height);
    result.set("freeCells", static_cast<double>(free_c));
    result.set("occupiedCells", static_cast<double>(occ_c));
    result.set("unknownCells", static_cast<double>(unk_c));
    return result;
  }

  int width() const {return static_cast<int>(grid_.width);}
  int height() const {return static_cast<int>(grid_.height);}

  val getSenderGrid() const
  {
    return val::array(grid_.data.begin(), grid_.data.end());
  }

  val encode(int levels, int tile_size_cells, const std::string & compression, bool use_varint)
  {
    tile_size_cells_ = tile_size_cells;
    const int grid_w = static_cast<int>(grid_.width);
    const int grid_h = static_cast<int>(grid_.height);
    const int tiles_x = (grid_w + tile_size_cells - 1) / tile_size_cells;
    const int tiles_y = (grid_h + tile_size_cells - 1) / tile_size_cells;

    std::vector<std::vector<EncodedChannel>> tile_bands;
    std::vector<std::pair<int, int>> tile_coords;
    std::vector<std::pair<int, int>> tile_dims;

    for (int trow = 0; trow < tiles_y; ++trow) {
      for (int tcol = 0; tcol < tiles_x; ++tcol) {
        const int row0 = trow * tile_size_cells;
        const int col0 = tcol * tile_size_cells;
        const int tw = std::min(tile_size_cells, grid_w - col0);
        const int th = std::min(tile_size_cells, grid_h - row0);
        if (tw <= 0 || th <= 0) {continue;}

        std::vector<std::uint32_t> values(static_cast<std::size_t>(tw) * th);
        for (int r = 0; r < th; ++r) {
          for (int c = 0; c < tw; ++c) {
            const std::size_t src = static_cast<std::size_t>(row0 + r) * grid_w + (col0 + c);
            values[static_cast<std::size_t>(r) * tw + c] = shift_to_uint32(grid_.data[src]);
          }
        }

        auto bands = make_haar_bands(
          values, static_cast<std::size_t>(tw), static_cast<std::size_t>(th), levels,
          compression, use_varint);
        tile_bands.push_back(std::move(bands));
        tile_coords.emplace_back(trow, tcol);
        tile_dims.emplace_back(tw, th);
      }
    }

    const int total_bands = levels + 1;
    send_order_ = build_send_order(tile_bands, tile_coords, tile_dims, total_bands);
    tiles_.clear();
    sent_index_ = 0;
    cumulative_bytes_ = 0;
    total_compressed_bytes_ = 0;
    receiver_grid_.assign(grid_.data.size(), -1);
    for (const auto & e : send_order_) {total_compressed_bytes_ += e.descriptor.compressed_size;}

    val result = val::object();
    result.set("tiles", static_cast<double>(tile_bands.size()));
    result.set("bandsPerTile", total_bands);
    result.set("totalEntries", static_cast<double>(send_order_.size()));
    result.set("rawBytes", static_cast<double>(raw_bytes_));
    result.set("totalCompressedBytes", static_cast<double>(total_compressed_bytes_));
    result.set("varint", use_varint);
    return result;
  }

  val sendNext()
  {
    val result = val::object();
    if (sent_index_ >= send_order_.size()) {
      result.set("sent", false);
      result.set("done", true);
      return result;
    }

    const auto & entry = send_order_[sent_index_];
    ++sent_index_;

    const auto raw = decompress_payload(entry.descriptor, entry.payload);
    const bool use_varint = entry.descriptor.metadata.at(kHaarVarintKey) == "1";
    const auto coeffs = use_varint ?
      zigzag_varint_decode(raw, entry.descriptor.element_count) :
      fixed_width_decode(raw, entry.descriptor.element_count);

    const TileKey key{entry.tile_row, entry.tile_col};
    auto & tile = tiles_[key];
    if (tile.width != entry.tile_width || tile.height != entry.tile_height) {
      tile = TileState{};
      tile.width = entry.tile_width;
      tile.height = entry.tile_height;
      tile.levels = static_cast<int>(entry.descriptor.metadata.count(kHaarLevelsKey) ?
        std::stoi(entry.descriptor.metadata.at(kHaarLevelsKey)) : 0);
      tile.total_bands = tile.levels + 1;
      tile.band_coeffs.assign(static_cast<std::size_t>(tile.total_bands), {});
      tile.received.assign(static_cast<std::size_t>(tile.total_bands), false);
    }
    tile.band_coeffs[static_cast<std::size_t>(entry.band_index)] = coeffs;
    tile.received[static_cast<std::size_t>(entry.band_index)] = true;

    cumulative_bytes_ += entry.descriptor.compressed_size;
    updateReceiverTile(key, tile);

    result.set("sent", true);
    result.set("tileRow", entry.tile_row);
    result.set("tileCol", entry.tile_col);
    result.set("bandIndex", entry.band_index);
    result.set("compressedSize", static_cast<double>(entry.descriptor.compressed_size));
    result.set("uncompressedSize", static_cast<double>(entry.descriptor.uncompressed_size));
    result.set("sentIndex", static_cast<double>(sent_index_));
    result.set("totalEntries", static_cast<double>(send_order_.size()));
    result.set("cumulativeBytes", static_cast<double>(cumulative_bytes_));
    result.set("done", sent_index_ >= send_order_.size());
    return result;
  }

  void resetReceiver()
  {
    tiles_.clear();
    sent_index_ = 0;
    cumulative_bytes_ = 0;
    receiver_grid_.assign(grid_.data.size(), -1);
  }

  val getReceiverGrid() const
  {
    return val::array(receiver_grid_.begin(), receiver_grid_.end());
  }

  bool isDone() const {return !send_order_.empty() && sent_index_ >= send_order_.size();}
  double sentIndex() const {return static_cast<double>(sent_index_);}
  double totalEntries() const {return static_cast<double>(send_order_.size());}
  double cumulativeBytes() const {return static_cast<double>(cumulative_bytes_);}
  double totalCompressedBytes() const {return static_cast<double>(total_compressed_bytes_);}
  double rawBytes() const {return static_cast<double>(raw_bytes_);}

private:
  // Reconstructs and pastes only the ONE tile that just received a band, into
  // the persistent receiver_grid_ buffer -- O(this tile's size), not O(every
  // tracked tile), unlike re-deriving the whole grid from scratch on every
  // sendNext() call. That distinction matters once a map has enough tiles
  // that "Send All" means thousands of calls (a 640x480 map at the default
  // 32-cell tiles is 300 tiles x 5 bands = 1500 sendNext() calls) --
  // rebuilding every other tile's already-unchanged reconstruction 1500
  // times over was the dominant cost, not the codec math itself.
  void updateReceiverTile(const TileKey & key, const TileState & tile)
  {
    if (tile.width <= 0 || tile.height <= 0) {return;}

    int bands_received = 0;
    for (int k = 0; k < tile.total_bands; ++k) {
      if (tile.received[static_cast<std::size_t>(k)]) {
        bands_received = k + 1;
      } else {
        break;
      }
    }
    if (bands_received == 0) {return;}

    const auto recon = reconstruct_haar_from_bands(
      tile.band_coeffs, static_cast<std::size_t>(tile.width),
      static_cast<std::size_t>(tile.height), tile.levels, bands_received);

    const std::uint32_t grid_width = grid_.width;
    const std::uint32_t grid_height = grid_.height;
    const std::size_t row0 = static_cast<std::size_t>(key.first) *
      static_cast<std::size_t>(tile_size_cells_);
    const std::size_t col0 = static_cast<std::size_t>(key.second) *
      static_cast<std::size_t>(tile_size_cells_);
    const std::size_t w_prime = recon.width;
    const std::size_t h_prime = recon.height;
    const std::size_t tw = static_cast<std::size_t>(tile.width);
    const std::size_t th = static_cast<std::size_t>(tile.height);

    for (std::size_t r = 0; r < th; ++r) {
      const std::size_t r_src = (w_prime == tw && h_prime == th) ? r : (r * h_prime / th);
      const std::size_t dst_row = row0 + r;
      if (dst_row >= grid_height) {break;}
      for (std::size_t c = 0; c < tw; ++c) {
        const std::size_t c_src = (w_prime == tw && h_prime == th) ? c : (c * w_prime / tw);
        const std::size_t dst_col = col0 + c;
        if (dst_col >= grid_width) {continue;}
        receiver_grid_[dst_row * grid_width + dst_col] =
          unshift_from_uint32(recon.values[r_src * w_prime + c_src]);
      }
    }
  }

  SyntheticGrid grid_;
  int tile_size_cells_{0};
  std::vector<QueueEntry> send_order_;
  std::size_t sent_index_{0};
  std::uint64_t cumulative_bytes_{0};
  std::uint64_t total_compressed_bytes_{0};
  std::uint64_t raw_bytes_{0};
  std::map<TileKey, TileState> tiles_;
  std::vector<std::int8_t> receiver_grid_;
};

}  // namespace

EMSCRIPTEN_BINDINGS(vxch_demo)
{
  emscripten::class_<VxchDemo>("VxchDemo")
    .constructor<>()
    .function("genMap", &VxchDemo::genMap)
    .function("width", &VxchDemo::width)
    .function("height", &VxchDemo::height)
    .function("getSenderGrid", &VxchDemo::getSenderGrid)
    .function("encode", &VxchDemo::encode)
    .function("sendNext", &VxchDemo::sendNext)
    .function("resetReceiver", &VxchDemo::resetReceiver)
    .function("getReceiverGrid", &VxchDemo::getReceiverGrid)
    .function("isDone", &VxchDemo::isDone)
    .function("sentIndex", &VxchDemo::sentIndex)
    .function("totalEntries", &VxchDemo::totalEntries)
    .function("cumulativeBytes", &VxchDemo::cumulativeBytes)
    .function("totalCompressedBytes", &VxchDemo::totalCompressedBytes)
    .function("rawBytes", &VxchDemo::rawBytes);
}
