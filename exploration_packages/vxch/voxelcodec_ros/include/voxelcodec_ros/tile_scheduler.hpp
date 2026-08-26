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
#include "voxelcodec_ros/occupancy_embedding.hpp"
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{

// (tile_row, tile_col) in the tile grid, row = along height, col = along width.
// World-anchored (see compute_axis_tile_spans below), so these are stable
// positions in a fixed world-cell lattice, NOT array-local indices -- they can
// legitimately be negative, and the same key always refers to the same patch
// of the world regardless of how any one robot's array origin happens to sit.
using TileKey = std::pair<int, int>;

// Floor division (rounds toward negative infinity), unlike C++'s truncating
// `/` -- needed because world-cell coordinates and tile indices can be
// negative once tiling is anchored to a fixed world lattice instead of each
// array's own local index 0. b must be > 0.
inline long long floor_div(long long a, long long b)
{
  const long long q = a / b;
  const long long r = a % b;
  return (r != 0 && ((r < 0) != (b < 0))) ? q - 1 : q;
}

// One axis' worth of (world-anchored tile index, local start, length) spans
// covering local indices [0, extent) of an array whose local index 0 sits at
// world-cell coordinate origin_cell (both in units of the grid's resolution).
// Tile boundaries fall wherever the world-tile lattice -- multiples of
// tile_size_cells, counted from world cell 0, independent of any one array's
// own origin -- crosses this axis, so the first and/or last span can be
// shorter than tile_size_cells when origin_cell isn't itself tile-aligned
// (the common case: array origins move continuously as SLAM refines them,
// they don't snap to tile boundaries). This is what makes a given tile's
// identity (its TileKey) survive an origin shift: the same world region
// always floor-divides to the same tile_index, regardless of where local
// index 0 currently sits.
struct AxisTileSpan
{
  int tile_index;
  int local_start;
  int length;
};

inline std::vector<AxisTileSpan> compute_axis_tile_spans(
  long long origin_cell, int extent, int tile_size_cells)
{
  std::vector<AxisTileSpan> spans;
  if (extent <= 0 || tile_size_cells <= 0) {
    return spans;
  }
  int local = 0;
  while (local < extent) {
    const long long world_cell = origin_cell + local;
    const long long tile_index = floor_div(world_cell, tile_size_cells);
    const long long tile_world_end = (tile_index + 1) * static_cast<long long>(tile_size_cells);
    const long long local_end = std::min<long long>(extent, tile_world_end - origin_cell);
    spans.push_back({static_cast<int>(tile_index), local, static_cast<int>(local_end - local)});
    local = static_cast<int>(local_end);
  }
  return spans;
}

// Splits an occupancy grid into tile_size_m x tile_size_m tiles, each with
// its own independent Haar pyramid, fingerprints each tile's bands to detect
// real content changes, and schedules changed bands for sending. "smart" and
// "simple" use round-robin fairness across tiles (and, in "smart" mode,
// least-recently-sent-first within a tile); "rd" instead scores every
// pending band by estimated distortion-reduction-per-byte and drains highest
// score first across ALL queued tiles at once -- see take_pending_bands_rd
// for the rationale. See occupancy_grid_vxch_node.cpp's on_map()/
// send_pending_bands() comments for the full rationale -- this class is an
// extraction of that logic, unchanged, so the ROS Node wrapper can declare
// parameters/publishers/logging around it instead of interleaving them.
class TileScheduler
{
public:
  TileScheduler(
    double tile_size_m, int haar_levels, std::string compression, bool varint_encoding,
    std::string schedule_mode)
  : tile_size_m_(tile_size_m),
    haar_levels_(haar_levels),
    compression_(std::move(compression)),
    varint_encoding_(varint_encoding),
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
  // equal grid_w*grid_h. origin_x/origin_y (default 0,0 -- exactly reproduces
  // pre-world-anchoring behavior for callers that don't have/need a real
  // origin, e.g. existing tests) anchor the tiling to a fixed world-cell
  // lattice instead of this array's own local index 0 -- see
  // compute_axis_tile_spans for why that's what makes a tile's identity
  // survive the array's origin moving out from under it between calls.
  IngestResult ingest_grid(
    const std::vector<std::int8_t> & grid_values, int grid_w, int grid_h, double resolution,
    double origin_x = 0.0, double origin_y = 0.0)
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
      // old-partition and new-partition state. Origin changes need no such reset:
      // tile identity is world-anchored (TileKey comment above), so the same real
      // patch of the world keeps the same key and comparable fingerprint no matter
      // how the array's own origin moves between calls.
      tile_size_cells_ = new_tile_size_cells;
      last_band_fingerprint_.clear();
      pending_by_tile_.clear();
      tile_queue_.clear();
      tiles_in_queue_.clear();
      last_sent_seq_.clear();
    }

    std::vector<std::uint32_t> values(N);
    for (std::size_t i = 0; i < N; ++i) {
      values[i] = occupancy_to_embedded(grid_values[i]);
    }

    // Round each axis' origin/resolution once (not per cell) to the world-cell
    // coordinate that this array's local index 0 currently sits at.
    const long long origin_cell_x = std::llround(origin_x / resolution);
    const long long origin_cell_y = std::llround(origin_y / resolution);
    const auto col_spans = compute_axis_tile_spans(origin_cell_x, grid_w, tile_size_cells_);
    const auto row_spans = compute_axis_tile_spans(origin_cell_y, grid_h, tile_size_cells_);

    for (const auto & row_span : row_spans) {
      for (const auto & col_span : col_spans) {
        const int trow = row_span.tile_index;
        const int tcol = col_span.tile_index;
        const int width = col_span.length;
        const int height = row_span.length;
        if (width <= 0 || height <= 0) {continue;}

        std::vector<std::uint32_t> tile_values(
          static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
        for (int r = 0; r < height; ++r) {
          const std::size_t src_off =
            static_cast<std::size_t>(row_span.local_start + r) * static_cast<std::size_t>(grid_w) +
            static_cast<std::size_t>(col_span.local_start);
          const std::size_t dst_off = static_cast<std::size_t>(r) * static_cast<std::size_t>(width);
          std::copy_n(values.begin() + static_cast<std::ptrdiff_t>(src_off), width,
            tile_values.begin() + static_cast<std::ptrdiff_t>(dst_off));
        }

        std::vector<EncodedChannel> bands;
        try {
          bands = make_haar_bands(
            tile_values, static_cast<std::size_t>(width),
            static_cast<std::size_t>(height), haar_levels_, compression_,
            varint_encoding_);
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
          band.descriptor.metadata["tile_width"] = std::to_string(width);
          band.descriptor.metadata["tile_height"] = std::to_string(height);
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

    if (schedule_mode_ == "rd") {
      return take_pending_bands_rd(max_bands_per_update, max_tiles_per_update);
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
  // "rd" (rate-distortion) selection: unlike the round-robin path above,
  // this scores every currently-pending (tile, band) at once and drains
  // highest score first, so a hot tile's high-value band can outrank a
  // quiet tile's low-value one instead of waiting a fixed turn behind it.
  //
  // score = l2_energy / bytes-on-the-wire. Haar is an orthonormal
  // transform, so by Parseval's theorem a band's coefficient energy
  // (Σcoefficient², computed in make_haar_bands while it still has the raw
  // coefficients -- see EncodedChannel::l2_energy) is exactly that band's
  // contribution to squared reconstruction error if it's withheld. Dividing
  // by payload bytes turns that into "distortion reduction per byte sent,"
  // the same bit-allocation principle JPEG2000/EZW use to decide which
  // wavelet coefficients earn their place in a byte budget first.
  //
  // Pure greedy-by-score would starve a tile whose bands are always
  // low-energy relative to whatever else is currently pending -- the same
  // failure mode BandQueue had in ddil_proxy_logic.hpp before it grew
  // wait-time aging (see that file's history). This reuses the same idea:
  // a band's effective score grows with how long it's gone since its last
  // send (via the existing last_sent_seq_/send_seq_counter_ bookkeeping
  // "smart" mode already relies on), so an ever-losing candidate eventually
  // outscores anything, guaranteeing it can't wait forever.
  std::vector<ScheduledBand> take_pending_bands_rd(int max_bands_per_update, int max_tiles_per_update)
  {
    struct Candidate
    {
      TileKey tile;
      int band_index;
      double effective_score;
    };

    std::vector<Candidate> candidates;
    for (const auto & key : tile_queue_) {
      auto tile_it = pending_by_tile_.find(key);
      if (tile_it == pending_by_tile_.end()) {
        continue;
      }
      const auto & last_sent = last_sent_seq_[key];
      for (const auto & [band_idx, channel] : tile_it->second) {
        const double bytes = static_cast<double>(std::max<std::size_t>(1, channel.payload.size()));
        const double raw_score = channel.l2_energy / bytes;

        auto sent_it = last_sent.find(band_idx);
        const std::uint64_t last_seq = (sent_it == last_sent.end()) ? 0 : sent_it->second;
        const double wait = static_cast<double>(send_seq_counter_ - last_seq);
        candidates.push_back({key, band_idx, raw_score * (1.0 + wait)});
      }
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & a, const Candidate & b) {return a.effective_score > b.effective_score;});

    std::map<TileKey, int> taken_per_tile;
    std::set<TileKey> tiles_touched;
    std::vector<ScheduledBand> out;
    for (const auto & c : candidates) {
      const bool tile_already_touched = tiles_touched.count(c.tile) > 0;
      if (!tile_already_touched && max_tiles_per_update >= 0 &&
        static_cast<int>(tiles_touched.size()) >= max_tiles_per_update)
      {
        continue;  // would exceed this tick's distinct-tile cap
      }
      int & taken = taken_per_tile[c.tile];
      if (taken >= max_bands_per_update) {
        continue;  // this tile already got its max_bands_per_update turn
      }

      auto tile_it = pending_by_tile_.find(c.tile);
      auto band_it = tile_it->second.find(c.band_index);
      EncodedChannel channel = std::move(band_it->second);
      tile_it->second.erase(band_it);

      last_sent_seq_[c.tile][c.band_index] = ++send_seq_counter_;
      ++taken;
      tiles_touched.insert(c.tile);
      out.push_back(ScheduledBand{c.tile, c.band_index, std::move(channel)});
    }

    // Drop tiles that were fully drained; anything left (skipped by a cap
    // above, or never a candidate) stays queued in its prior relative order.
    std::deque<TileKey> remaining;
    for (const auto & key : tile_queue_) {
      auto tile_it = pending_by_tile_.find(key);
      if (tile_it == pending_by_tile_.end() || tile_it->second.empty()) {
        pending_by_tile_.erase(key);
        tiles_in_queue_.erase(key);
      } else {
        remaining.push_back(key);
      }
    }
    tile_queue_ = std::move(remaining);

    return out;
  }

  double tile_size_m_;
  int haar_levels_;
  std::string compression_;
  bool varint_encoding_;
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
