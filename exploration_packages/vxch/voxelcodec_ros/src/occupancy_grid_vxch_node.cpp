#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/haar_forward.hpp"
#include "voxelcodec_ros/ros_messages.hpp"
#include "voxelcodec_ros/types.hpp"
#include "voxelcodec_msgs/msg/voxel_channel.hpp"
#include "voxelcodec_msgs/msg/voxel_manifest.hpp"

namespace voxelcodec_ros
{
namespace
{

// Shift int8 occupancy value to non-negative uint32 for Haar encoding.
// -1 (unknown) → 0, 0 (free) → 1, 100 (occupied) → 101, etc.
inline std::uint32_t shift_to_uint32(std::int8_t v)
{
  return static_cast<std::uint32_t>(static_cast<int>(v) + 1);
}

// (tile_row, tile_col) in the tile grid, row = along height, col = along width.
using TileKey = std::pair<int, int>;

// See band_qos comment below for why this needs to be well above 1 once tiling
// is in play. 64 comfortably covers tile_size_m_ down to well under 1m over a
// map several tens of meters across without needing to be a tunable parameter.
constexpr std::size_t kBandQueueDepth = 64;

class OccupancyGridVxchNode : public rclcpp::Node
{
public:
  OccupancyGridVxchNode()
  : Node("occupancy_grid_vxch_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/map");
    output_base_topic_ = declare_parameter<std::string>("output_base_topic", "/vxch/map");
    haar_levels_ = declare_parameter<int>("haar_levels", 4);
    compression_ = declare_parameter<std::string>("compression", "zstd");
    stream_id_ = declare_parameter<std::string>("stream_id", "map_stream");
    // Each Haar pyramid (band_0..band_L) is built over the whole grid it's given.
    // Encoding the full map as ONE pyramid means a single changed cell anywhere
    // invalidates every band's payload (the fingerprint hashes the whole band),
    // so an actively-exploring robot never lets its own detail bands (which is
    // exactly the info a teammate needs to know an area is already covered) win
    // the coarsest-first priority race against its own constantly-refreshing
    // low bands. Splitting the grid into tile_size_m x tile_size_m tiles, each
    // with its own independent 6-band pyramid, means only the tile(s) actually
    // near the robot right now stay dirty -- a tile the robot already left
    // quiets down and finishes draining its own detail within a few ticks,
    // independent of what's still changing elsewhere in the map.
    tile_size_m_ = declare_parameter<double>("tile_size_m", 2.0);
    if (tile_size_m_ <= 0.0) {
      throw std::runtime_error("tile_size_m must be > 0");
    }
    // Publishing every changed band in one burst monopolizes the shared DDIL link for the
    // whole burst's duration -- each ddil_proxy_node instance (one per peer link, possibly on
    // different robots) throttles independently with no cross-robot coordination, so nothing
    // stops multiple robots' bursts from overlapping and contending for the same real airtime.
    // Sending only this many of the highest-priority (coarsest) pending bands per send tick,
    // per tile, and carrying the rest over spaces this robot's own traffic out, leaving gaps
    // for peers to get a word in. 1 = strictly one band per tile per tick.
    max_bands_per_update_ = declare_parameter<int>("max_bands_per_update", 1);
    if (max_bands_per_update_ < 1) {
      throw std::runtime_error("max_bands_per_update must be >= 1");
    }
    // Safety valve on top of the per-tile cap above: how many DISTINCT tiles get
    // serviced per send tick. -1 = no cap (every tile currently holding pending
    // bands gets its max_bands_per_update_ turn every tick -- this is what the
    // tiling split above is actually for: a robot's own footprint touches at
    // most a handful of tiles at once at the tile_size_m_ default, so an
    // unbounded per-tick tile count stays cheap in practice; this parameter
    // exists purely as a guard against a pathological tile count, e.g. a much
    // smaller tile_size_m_ than the default without re-checking the tradeoff).
    max_tiles_per_update_ = declare_parameter<int>("max_tiles_per_update", -1);

    // Sending used to happen directly inside on_map(), so a robot's send *opportunities* were
    // tied to its own /map republish rate -- which is itself tied to how much SLAM CPU it has.
    // Live measurement: one robot's on_map fired 46 times against peers' 7-8 times in the same
    // window (same CPU contention this whole session has been chasing), so it got ~6x as many
    // chances to transmit regardless of any per-cycle band cap. A fixed-rate timer decoupled
    // from on_map gives every robot the same number of send opportunities per second no matter
    // how fast or slow its own SLAM is running.
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 1.0);
    if (send_rate_hz_ <= 0.0) {
      throw std::runtime_error("send_rate_hz must be > 0");
    }

    if (haar_levels_ < 1 || haar_levels_ > 12) {
      throw std::runtime_error("haar_levels must be between 1 and 12");
    }

    const auto map_qos = rclcpp::QoS(1)
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);
    // Tiling means one send tick can publish several *different* tiles' messages
    // back-to-back on the SAME fixed band_k topic (tile identity travels in the
    // payload, not the topic). A depth-1 queue only ever holds the single latest
    // sample published on a topic; if N tiles publish to band_k before the
    // subscriber's executor gets a chance to drain each one, only the last of
    // those N survives at the DDS layer -- silent data loss, not a dedup/logic
    // bug. Sizing the queue to comfortably exceed the largest expected number of
    // tiles serviced in one tick avoids that regardless of scheduling jitter.
    const auto band_qos = rclcpp::QoS(kBandQueueDepth).best_effort();

    manifest_pub_ = create_publisher<voxelcodec_msgs::msg::VoxelManifest>(
      output_base_topic_ + "/manifest", map_qos);

    const int total_bands = haar_levels_ + 1;
    band_pubs_.resize(static_cast<std::size_t>(total_bands));
    for (int k = 0; k < total_bands; ++k) {
      band_pubs_[static_cast<std::size_t>(k)] =
        create_publisher<voxelcodec_msgs::msg::VoxelChannel>(
          output_base_topic_ + "/band_" + std::to_string(k), band_qos);
    }

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_, map_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        on_map(msg);
      });

    send_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / send_rate_hz_),
      [this]() {send_pending_bands();});

    RCLCPP_INFO(
      get_logger(),
      "Encoding %s → %s/manifest + band_0..band_%d (haar_levels=%d, compression=%s, "
      "tile_size_m=%.2f, send_rate_hz=%.2f, max_bands_per_update=%d)",
      input_topic_.c_str(), output_base_topic_.c_str(), haar_levels_,
      haar_levels_, compression_.c_str(), tile_size_m_, send_rate_hz_, max_bands_per_update_);
  }

private:
  // Tiles per axis and the cell extent of tile (row, col), clamped to the grid
  // edge (the last row/col of tiles is <= tile_size_cells_ when W or H isn't
  // an exact multiple of it).
  struct TileGeom
  {
    int row0, col0;   // top-left cell offset in the full grid
    int width, height;  // this tile's actual cell extent (<= tile_size_cells_)
  };

  TileGeom tile_geom(int tile_row, int tile_col, int grid_w, int grid_h) const
  {
    TileGeom g;
    g.row0 = tile_row * tile_size_cells_;
    g.col0 = tile_col * tile_size_cells_;
    g.width = std::min(tile_size_cells_, grid_w - g.col0);
    g.height = std::min(tile_size_cells_, grid_h - g.row0);
    return g;
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0) {
      return;
    }

    const int grid_w = static_cast<int>(msg->info.width);
    const int grid_h = static_cast<int>(msg->info.height);
    const std::size_t N = static_cast<std::size_t>(grid_w) * static_cast<std::size_t>(grid_h);

    if (msg->data.size() != N) {
      RCLCPP_WARN(get_logger(), "OccupancyGrid data size mismatch: expected %zu got %zu",
        N, msg->data.size());
      return;
    }

    // tile_size_cells_ is derived from resolution once we see it; recomputed every
    // call (cheap) so a resolution change (unusual, but SLAM maps can rescale) is
    // picked up rather than silently kept stale.
    const int new_tile_size_cells = std::max(
      1, static_cast<int>(std::llround(tile_size_m_ / static_cast<double>(msg->info.resolution))));
    if (new_tile_size_cells != tile_size_cells_) {
      // Tile partition changed shape -- old per-tile fingerprints/backlog no longer
      // correspond to the same physical tiles, so start clean rather than mixing
      // old-partition and new-partition state.
      tile_size_cells_ = new_tile_size_cells;
      last_band_fingerprint_.clear();
      pending_by_tile_.clear();
      tile_queue_.clear();
      tiles_in_queue_.clear();
    }

    // Convert int8 → uint32 once for the whole grid; tiles below index into this.
    std::vector<std::uint32_t> values(N);
    for (std::size_t i = 0; i < N; ++i) {
      values[i] = shift_to_uint32(msg->data[i]);
    }

    const int tiles_x = (grid_w + tile_size_cells_ - 1) / tile_size_cells_;
    const int tiles_y = (grid_h + tile_size_cells_ - 1) / tile_size_cells_;

    std::size_t total_changed = 0;
    for (int trow = 0; trow < tiles_y; ++trow) {
      for (int tcol = 0; tcol < tiles_x; ++tcol) {
        const TileGeom geom = tile_geom(trow, tcol, grid_w, grid_h);
        if (geom.width <= 0 || geom.height <= 0) {continue;}

        // Gather this tile's sub-block, row-major, out of the full grid.
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
            static_cast<std::size_t>(geom.height), haar_levels_, compression_);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(
            get_logger(), "make_haar_bands failed for tile (%d,%d): %s", trow, tcol, e.what());
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
          // Tag the descriptor so the decoder can place this tile's reconstruction
          // back into the right spot in the full grid, and so ddil_proxy can key
          // dedup/priority per (tile, band) instead of colliding all tiles that
          // happen to share the same band_k topic.
          band.descriptor.metadata["tile_row"] = std::to_string(trow);
          band.descriptor.metadata["tile_col"] = std::to_string(tcol);
          band.descriptor.metadata["tile_width"] = std::to_string(geom.width);
          band.descriptor.metadata["tile_height"] = std::to_string(geom.height);
          band.descriptor.metadata["tile_size_cells"] = std::to_string(tile_size_cells_);

          const auto & payload = band.payload;
          const std::size_t fp =
            payload.size() ^
            std::hash<std::string>{}(std::string(payload.begin(), payload.end()));
          if (fp != fp_for_tile[k]) {
            fp_for_tile[k] = fp;
            pending_by_tile_[key][static_cast<int>(k)] = std::move(band);
            tile_changed = true;
            ++total_changed;
          }
        }

        if (tile_changed && tiles_in_queue_.insert(key).second) {
          tile_queue_.push_back(key);
        }
      }
    }

    manifest_.voxel_count = static_cast<std::uint32_t>(N);
    manifest_.metadata["grid_width"] = std::to_string(grid_w);
    manifest_.metadata["grid_height"] = std::to_string(grid_h);
    manifest_.metadata["resolution"] = std::to_string(msg->info.resolution);
    manifest_.metadata["origin_x"] = std::to_string(msg->info.origin.position.x);
    manifest_.metadata["origin_y"] = std::to_string(msg->info.origin.position.y);
    manifest_.metadata["frame_id"] = msg->header.frame_id;
    manifest_.metadata["stamp_sec"] = std::to_string(msg->header.stamp.sec);
    manifest_.metadata["stamp_nanosec"] = std::to_string(msg->header.stamp.nanosec);
    manifest_.metadata["tile_size_cells"] = std::to_string(tile_size_cells_);
    manifest_.channels.clear();

    latest_header_ = msg->header;
    has_map_ = true;

    RCLCPP_DEBUG(
      get_logger(), "%zu band(s) changed across %zu queued tile(s)",
      total_changed, tile_queue_.size());
  }

  void send_pending_bands()
  {
    if (!has_map_ || tile_queue_.empty()) {
      return;
    }

    // Publish manifest before bands (TRANSIENT_LOCAL — reaches late-joining decoders). Sent
    // every tick a band goes out, using the latest known geometry; the bands sent below are
    // stamped with this same header, so they can never be judged stale against it downstream
    // (see ddil_stale_epoch.hpp) even if their payload was computed several on_map calls ago.
    manifest_pub_->publish(manifest_to_msg(latest_header_, stream_id_, manifest_));

    // Round-robin across tiles that currently have pending bands: each tile gets
    // up to max_bands_per_update_ of its own lowest (coarsest-first) pending
    // bands this tick, then -- if it still has more pending -- goes to the back
    // of the queue. A tile that's quieted down (nothing new queued for it since
    // its last turn) drains fully within a few ticks and drops out, regardless
    // of how much churn is still happening in other tiles.
    const int tiles_this_tick = max_tiles_per_update_ < 0 ?
      static_cast<int>(tile_queue_.size()) :
      std::min(max_tiles_per_update_, static_cast<int>(tile_queue_.size()));

    std::size_t sent_count = 0;
    std::size_t sent_bytes = 0;
    std::ostringstream ss;
    ss << "[";

    for (int t = 0; t < tiles_this_tick; ++t) {
      const TileKey key = tile_queue_.front();
      tile_queue_.pop_front();
      tiles_in_queue_.erase(key);

      auto tile_it = pending_by_tile_.find(key);
      if (tile_it == pending_by_tile_.end() || tile_it->second.empty()) {
        continue;
      }
      auto & bands_for_tile = tile_it->second;

      for (int i = 0; i < max_bands_per_update_ && !bands_for_tile.empty(); ++i) {
        auto it = bands_for_tile.begin();
        const int band_idx = it->first;
        const EncodedChannel channel = std::move(it->second);
        bands_for_tile.erase(it);

        if (sent_count > 0) {ss << " ";}
        const double kb = static_cast<double>(channel.payload.size()) / 1024.0;
        ss << "t(" << key.first << "," << key.second << "):" << band_idx << ":"
           << std::fixed << std::setprecision(1) << kb << "KB";
        sent_bytes += channel.payload.size();
        ++sent_count;

        band_pubs_[static_cast<std::size_t>(band_idx)]->publish(
          channel_to_msg(latest_header_, stream_id_, channel.descriptor, channel.payload));
      }

      if (bands_for_tile.empty()) {
        pending_by_tile_.erase(tile_it);
      } else if (tiles_in_queue_.insert(key).second) {
        tile_queue_.push_back(key);
      }
    }
    ss << "]";

    RCLCPP_INFO(
      get_logger(), "send tick %s  total=%.1f KB  (%zu sent, %zu tile(s) still queued)",
      ss.str().c_str(),
      static_cast<double>(sent_bytes) / 1024.0,
      sent_count, tile_queue_.size());
  }

  std::string input_topic_;
  std::string output_base_topic_;
  int haar_levels_;
  std::string compression_;
  std::string stream_id_;
  double tile_size_m_;
  int tile_size_cells_{0};
  int max_bands_per_update_;
  int max_tiles_per_update_;
  double send_rate_hz_;

  rclcpp::Publisher<voxelcodec_msgs::msg::VoxelManifest>::SharedPtr manifest_pub_;
  std::vector<rclcpp::Publisher<voxelcodec_msgs::msg::VoxelChannel>::SharedPtr> band_pubs_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;

  // Per-tile, per-band fingerprint of the last payload actually queued for that
  // (tile, band) -- lets on_map() skip re-queuing a band whose content hasn't
  // changed since last time, same as the pre-tiling single-pyramid version did.
  std::map<TileKey, std::vector<std::size_t>> last_band_fingerprint_;
  // Bands queued but not yet sent, per tile; within a tile, keyed by band index
  // so begin() is always that tile's lowest (coarsest/highest-priority) pending
  // entry -- same invariant the pre-tiling version relied on, just scoped per tile.
  std::map<TileKey, std::map<int, EncodedChannel>> pending_by_tile_;
  // FIFO of tiles with at least one pending band, for round-robin fairness
  // across tiles -- this is what stops one still-churning tile from starving
  // every other (possibly already-settled) tile's turn to send.
  std::deque<TileKey> tile_queue_;
  std::set<TileKey> tiles_in_queue_;

  // Latest known map geometry/timestamp, refreshed on every on_map() call, used by
  // send_pending_bands() regardless of which on_map() call queued the band it's sending.
  std_msgs::msg::Header latest_header_;
  Manifest manifest_;
  bool has_map_{false};
};

}  // namespace
}  // namespace voxelcodec_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<voxelcodec_ros::OccupancyGridVxchNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("occupancy_grid_vxch_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
