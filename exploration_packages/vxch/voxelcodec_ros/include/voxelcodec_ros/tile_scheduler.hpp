#pragma once

// Pure tiling/fingerprint/scheduling logic behind occupancy_grid_vxch_node.cpp's
// Node, pulled into a header so it's testable without an rclcpp::Node fixture --
// same reasoning as ddil_stale_epoch.hpp. Node-specific glue (parameter
// declaration, publishers, the wall timer, RCLCPP logging) stays in the .cpp;
// TileScheduler surfaces enough (IngestResult::tile_errors, the scheduled
// bands themselves) for the caller to log whatever it wants around this.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "voxelcodec_ros/haar_forward.hpp"
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{

// Shift int8 occupancy value to non-negative uint32 for Haar encoding.
// -1 (unknown) → 0, 0 (free) → 1, 100 (occupied) → 101, etc.
inline std::uint32_t shift_to_uint32(std::int8_t v)
{
  return static_cast<std::uint32_t>(static_cast<int>(v) + 1);
}

// (tile_row, tile_col) in the tile grid, row = along height, col = along width.
using TileKey = std::pair<int, int>;

// Tiles per axis and the cell extent of tile (row, col), clamped to the grid
// edge (the last row/col of tiles is <= tile_size_cells when W or H isn't an
// exact multiple of it).
struct TileGeom
{
  int row0, col0;      // top-left cell offset in the full grid
  int width, height;   // this tile's actual cell extent (<= tile_size_cells)
};

inline TileGeom compute_tile_geom(
  int tile_row, int tile_col, int tile_size_cells, int grid_w, int grid_h)
{
  TileGeom g;
  g.row0 = tile_row * tile_size_cells;
  g.col0 = tile_col * tile_size_cells;
  g.width = std::min(tile_size_cells, grid_w - g.col0);
  g.height = std::min(tile_size_cells, grid_h - g.row0);
  return g;
}

// Splits an occupancy grid into tile_size_m x tile_size_m tiles, each with
// its own independent Haar pyramid, fingerprints each tile's bands to detect
// real content changes, and schedules changed bands for sending with
// round-robin fairness across tiles (and, in "smart" mode, within a tile's
// own bands too). See occupancy_grid_vxch_node.cpp's on_map()/
// send_pending_bands() comments for the full rationale -- this class is an
// extraction of that logic, unchanged, so the ROS Node wrapper can declare
// parameters/publishers/logging around it instead of interleaving them.
class TileScheduler
{
public:
  TileScheduler(
    double tile_size_m, int haar_levels, std::string compression, std::string coeff_encoding,
    std::string schedule_mode)
  : tile_size_m_(tile_size_m),
    haar_levels_(haar_levels),
    compression_(std::move(compression)),
    coeff_encoding_(std::move(coeff_encoding)),
    schedule_mode_(std::move(schedule_mode))
  {}

  struct IngestResult
  {
    std::size_t total_changed{0};
    std::size_t queued_tiles{0};
    // One entry per tile whose make_haar_bands() call threw -- that tile is
    // skipped for this ingest, same as production's catch-and-continue.
    std::vector<std::string> tile_errors;
  };

  // Ingests one occupancy grid update: tiles it, Haar-encodes each tile,
  // fingerprints each band to skip re-queuing unchanged content (unless
  // schedule_mode is "simple", which always re-queues everything), and
  // enqueues changed bands for take_pending_bands(). grid_values.size() must
  // equal grid_w*grid_h.
  IngestResult ingest_grid(
    const std::vector<std::int8_t> & grid_values, int grid_w, int grid_h, double resolution)
  {
    IngestResult result;
    if (grid_w <= 0 || grid_h <= 0) {
      return result;
    }
    const std::size_t N = static_cast<std::size_t>(grid_w) * static_cast<std::size_t>(grid_h);
    if (grid_values.size() != N) {
      throw std::runtime_error("TileScheduler::ingest_grid: grid_values size mismatch");
    }

    // tile_size_cells_ is derived from resolution once we see it; recomputed every
    // call (cheap) so a resolution change (unusual, but SLAM maps can rescale) is
    // picked up rather than silently kept stale.
    const int new_tile_size_cells = std::max(
      1, static_cast<int>(std::llround(tile_size_m_ / resolution)));
    if (new_tile_size_cells != tile_size_cells_) {
      // Tile partition changed shape -- old per-tile fingerprints/backlog no longer
      // correspond to the same physical tiles, so start clean rather than mixing
      // old-partition and new-partition state.
      tile_size_cells_ = new_tile_size_cells;
      last_band_fingerprint_.clear();
      pending_by_tile_.clear();
      tile_queue_.clear();
      tiles_in_queue_.clear();
      last_sent_seq_.clear();
    }

    std::vector<std::uint32_t> values(N);
    for (std::size_t i = 0; i < N; ++i) {
      values[i] = shift_to_uint32(grid_values[i]);
    }

    const int tiles_x = (grid_w + tile_size_cells_ - 1) / tile_size_cells_;
    const int tiles_y = (grid_h + tile_size_cells_ - 1) / tile_size_cells_;

    for (int trow = 0; trow < tiles_y; ++trow) {
      for (int tcol = 0; tcol < tiles_x; ++tcol) {
        const TileGeom geom = compute_tile_geom(trow, tcol, tile_size_cells_, grid_w, grid_h);
        if (geom.width <= 0 || geom.height <= 0) {continue;}

        std::vector<std::uint32_t> tile_values(
          static_cast<std::size_t>(geom.width) * static_cast<std::size_t>(geom.height));
        for (int r = 0; r < geom.height; ++r) {
          const std::size_t src_off =
            static_cast<std::size_t>(geom.row0 + r) * static_cast<std::size_t>(grid_w) +
            static_cast<std::size_t>(geom.col0);
          const std::size_t dst_off = static_cast<std::size_t>(r) * static_cast<std::size_t>(geom.width);
          std::copy_n(values.begin() + static_cast<std::ptrdiff_t>(src_off), geom.width,
            tile_values.begin() + static_cast<std::ptrdiff_t>(dst_off));
        }

        std::vector<EncodedChannel> bands;
        try {
          bands = make_haar_bands(
            tile_values, static_cast<std::size_t>(geom.width),
            static_cast<std::size_t>(geom.height), haar_levels_, compression_,
            coeff_encoding_);
        } catch (const std::exception & e) {
          result.tile_errors.push_back(
            "tile (" + std::to_string(trow) + "," + std::to_string(tcol) + "): " + e.what());
          continue;
        }

        const TileKey key{trow, tcol};
        auto & fp_for_tile = last_band_fingerprint_[key];
        if (fp_for_tile.empty()) {
          fp_for_tile.assign(bands.size(), 0);
        }

        bool tile_changed = false;
        for (std::size_t k = 0; k < bands.size(); ++k) {
          auto & band = bands[k];
          band.descriptor.metadata["tile_row"] = std::to_string(trow);
          band.descriptor.metadata["tile_col"] = std::to_string(tcol);
          band.descriptor.metadata["tile_width"] = std::to_string(geom.width);
          band.descriptor.metadata["tile_height"] = std::to_string(geom.height);
          band.descriptor.metadata["tile_size_cells"] = std::to_string(tile_size_cells_);

          const auto & payload = band.payload;
          const std::size_t fp =
            payload.size() ^
            std::hash<std::string>{}(std::string(payload.begin(), payload.end()));
          if (schedule_mode_ == "simple" || fp != fp_for_tile[k]) {
            fp_for_tile[k] = fp;
            pending_by_tile_[key][static_cast<int>(k)] = std::move(band);
            tile_changed = true;
            ++result.total_changed;
          }
        }

        if (tile_changed && tiles_in_queue_.insert(key).second) {
          tile_queue_.push_back(key);
        }
      }
    }

    result.queued_tiles = tile_queue_.size();
    return result;
  }

  struct ScheduledBand
  {
    TileKey tile;
    int band_index;
    EncodedChannel channel;
  };

  // Round-robin dequeue across tiles with pending bands: each tile gets up to
  // max_bands_per_update of its own highest-priority pending bands this tick,
  // then -- if it still has more pending -- goes to the back of the queue.
  // max_tiles_per_update < 0 means no cap (every tile currently queued gets a turn).
  std::vector<ScheduledBand> take_pending_bands(int max_bands_per_update, int max_tiles_per_update)
  {
    std::vector<ScheduledBand> out;
    if (tile_queue_.empty()) {
      return out;
    }

    const int tiles_this_tick = max_tiles_per_update < 0 ?
      static_cast<int>(tile_queue_.size()) :
      std::min(max_tiles_per_update, static_cast<int>(tile_queue_.size()));

    for (int t = 0; t < tiles_this_tick; ++t) {
      const TileKey key = tile_queue_.front();
      tile_queue_.pop_front();
      tiles_in_queue_.erase(key);

      auto tile_it = pending_by_tile_.find(key);
      if (tile_it == pending_by_tile_.end() || tile_it->second.empty()) {
        continue;
      }
      auto & bands_for_tile = tile_it->second;
      auto & last_sent = last_sent_seq_[key];

      for (int i = 0; i < max_bands_per_update && !bands_for_tile.empty(); ++i) {
        // "smart": prefer the least-recently-sent pending band over strict
        // coarsest-first, so a band that was just delivered doesn't cut back
        // in line ahead of one that's been waiting longer.
        // "simple": always take the lowest pending band index (bands_for_tile
        // is keyed by band index, so begin() is already coarsest-first).
        auto it = bands_for_tile.begin();
        if (schedule_mode_ == "smart") {
          std::uint64_t best_seq = std::numeric_limits<std::uint64_t>::max();
          for (auto candidate = bands_for_tile.begin(); candidate != bands_for_tile.end();
            ++candidate)
          {
            auto sent_it = last_sent.find(candidate->first);
            const std::uint64_t seq = (sent_it == last_sent.end()) ? 0 : sent_it->second;
            if (seq < best_seq) {
              best_seq = seq;
              it = candidate;
              if (seq == 0) {break;}  // never sent -- can't do better than this
            }
          }
        }
        const int band_idx = it->first;
        EncodedChannel channel = std::move(it->second);
        bands_for_tile.erase(it);
        if (schedule_mode_ == "smart") {
          last_sent[band_idx] = ++send_seq_counter_;
        }
        out.push_back(ScheduledBand{key, band_idx, std::move(channel)});
      }

      if (bands_for_tile.empty()) {
        pending_by_tile_.erase(tile_it);
      } else if (tiles_in_queue_.insert(key).second) {
        tile_queue_.push_back(key);
      }
    }

    return out;
  }

  bool has_pending() const {return !tile_queue_.empty();}
  std::size_t queued_tile_count() const {return tile_queue_.size();}
  int tile_size_cells() const {return tile_size_cells_;}

private:
  double tile_size_m_;
  int haar_levels_;
  std::string compression_;
  std::string coeff_encoding_;
  std::string schedule_mode_;

  int tile_size_cells_{0};

  // Per-tile, per-band fingerprint of the last payload actually queued for that
  // (tile, band) -- lets ingest_grid() skip re-queuing a band whose content hasn't
  // changed since last time.
  std::map<TileKey, std::vector<std::size_t>> last_band_fingerprint_;
  // Bands queued but not yet sent, per tile; within a tile, keyed by band index
  // so begin() is always that tile's lowest (coarsest/highest-priority) pending entry.
  std::map<TileKey, std::map<int, EncodedChannel>> pending_by_tile_;
  // FIFO of tiles with at least one pending band, for round-robin fairness across tiles.
  std::deque<TileKey> tile_queue_;
  std::set<TileKey> tiles_in_queue_;
  // Per tile, per band index, the send_seq_counter_ value at which it was last
  // sent (absent = never sent). Lets take_pending_bands() prefer the
  // least-recently-sent pending band over strict coarsest-first.
  std::map<TileKey, std::map<int, std::uint64_t>> last_sent_seq_;
  std::uint64_t send_seq_counter_{0};
};

}  // namespace voxelcodec_ros
