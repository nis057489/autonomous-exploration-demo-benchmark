#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/ros_messages.hpp"
#include "voxelcodec_ros/tile_scheduler.hpp"
#include "voxelcodec_ros/types.hpp"
#include "voxelcodec_msgs/msg/voxel_channel.hpp"
#include "voxelcodec_msgs/msg/voxel_manifest.hpp"

namespace voxelcodec_ros
{
namespace
{

// See tile_scheduler.hpp's kBandQueueDepth-equivalent comment: tiling means one
// send tick can publish several *different* tiles' messages back-to-back on the
// SAME fixed band_k topic (tile identity travels in the payload, not the
// topic). A depth-1 queue only ever holds the single latest sample published
// on a topic; if N tiles publish to band_k before the subscriber's executor
// gets a chance to drain each one, only the last of those N survives at the
// DDS layer -- silent data loss, not a dedup/logic bug. Sizing the queue to
// comfortably exceed the largest expected number of tiles serviced in one
// tick avoids that regardless of scheduling jitter.
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
    compression_ = declare_parameter<std::string>("compression", kCompressionZstd);
    if (compression_ != kCompressionNone && compression_ != kCompressionZstd) {
      throw std::runtime_error("compression must be 'zstd' or 'none'");
    }
    // Ablation knob, independent of compression above: true (default) zigzag-varint
    // packs each band's Haar coefficients before compression; false packs them as
    // fixed-width int32 LE instead (see fixed_width_encode in haar_forward.hpp). With
    // compression=none this isolates varint packing's own contribution to bandwidth --
    // compression=none alone still leaves varint packing in place, so it only shows
    // "wavelet + varint, no zstd" vs. "wavelet + varint + zstd," not "wavelet vs. nothing."
    varint_encoding_ = declare_parameter<bool>("varint_encoding", true);
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

    // "smart" = only queue a band when its fingerprint actually changed, and within a tile
    // prefer whichever pending band has gone longest without a turn (see TileScheduler).
    // "simple" = every tile's every band is queued fresh on every on_map() call (no change
    // detection) and always sent strict coarsest-first (no recency reordering) -- this is the
    // same "just iterate over the tiles" scheme the baseline OccupancyGrid relay effectively
    // gets for free: ddil_proxy_node has no dedup/priority logic for a plain map topic (only
    // for band_N/manifest topics), so baseline already resends the whole map unconditionally
    // on every SLAM map_update_interval tick. "simple" exists so a vxch-vs-baseline comparison
    // can isolate what the wavelet/tiling encoding itself buys, independent of whether vxch's
    // scheduling is also doing extra work baseline never does.
    schedule_mode_ = declare_parameter<std::string>("schedule_mode", "smart");
    if (schedule_mode_ != "smart" && schedule_mode_ != "simple") {
      throw std::runtime_error("schedule_mode must be 'smart' or 'simple'");
    }

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

    scheduler_ = std::make_unique<TileScheduler>(
      tile_size_m_, haar_levels_, compression_, varint_encoding_, schedule_mode_);

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
      "varint_encoding=%s, tile_size_m=%.2f, send_rate_hz=%.2f, max_bands_per_update=%d, "
      "schedule_mode=%s)",
      input_topic_.c_str(), output_base_topic_.c_str(), haar_levels_,
      haar_levels_, compression_.c_str(), varint_encoding_ ? "true" : "false", tile_size_m_,
      send_rate_hz_, max_bands_per_update_, schedule_mode_.c_str());
  }

private:
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

    const auto result = scheduler_->ingest_grid(
      msg->data, grid_w, grid_h, static_cast<double>(msg->info.resolution));
    for (const auto & error : result.tile_errors) {
      RCLCPP_ERROR(get_logger(), "make_haar_bands failed for %s", error.c_str());
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
    manifest_.metadata["tile_size_cells"] = std::to_string(scheduler_->tile_size_cells());
    manifest_.channels.clear();

    latest_header_ = msg->header;
    has_map_ = true;

    RCLCPP_DEBUG(
      get_logger(), "%zu band(s) changed across %zu queued tile(s)",
      result.total_changed, result.queued_tiles);
  }

  void send_pending_bands()
  {
    if (!has_map_ || !scheduler_->has_pending()) {
      return;
    }

    // Publish manifest before bands (TRANSIENT_LOCAL — reaches late-joining decoders). Sent
    // every tick a band goes out, using the latest known geometry; the bands sent below are
    // stamped with this same header, so they can never be judged stale against it downstream
    // (see ddil_stale_epoch.hpp) even if their payload was computed several on_map calls ago.
    manifest_pub_->publish(manifest_to_msg(latest_header_, stream_id_, manifest_));

    const auto scheduled = scheduler_->take_pending_bands(
      max_bands_per_update_, max_tiles_per_update_);

    std::size_t sent_bytes = 0;
    std::ostringstream ss;
    ss << "[";
    for (std::size_t i = 0; i < scheduled.size(); ++i) {
      const auto & item = scheduled[i];
      if (i > 0) {ss << " ";}
      const double kb = static_cast<double>(item.channel.payload.size()) / 1024.0;
      ss << "t(" << item.tile.first << "," << item.tile.second << "):" << item.band_index << ":"
         << std::fixed << std::setprecision(1) << kb << "KB";
      sent_bytes += item.channel.payload.size();

      band_pubs_[static_cast<std::size_t>(item.band_index)]->publish(
        channel_to_msg(latest_header_, stream_id_, item.channel.descriptor, item.channel.payload));
    }
    ss << "]";

    RCLCPP_INFO(
      get_logger(), "send tick %s  total=%.1f KB  (%zu sent, %zu tile(s) still queued)",
      ss.str().c_str(),
      static_cast<double>(sent_bytes) / 1024.0,
      scheduled.size(), scheduler_->queued_tile_count());
  }

  std::string input_topic_;
  std::string output_base_topic_;
  int haar_levels_;
  std::string compression_;
  bool varint_encoding_;
  std::string stream_id_;
  double tile_size_m_;
  int max_bands_per_update_;
  int max_tiles_per_update_;
  double send_rate_hz_;
  std::string schedule_mode_;

  std::unique_ptr<TileScheduler> scheduler_;

  rclcpp::Publisher<voxelcodec_msgs::msg::VoxelManifest>::SharedPtr manifest_pub_;
  std::vector<rclcpp::Publisher<voxelcodec_msgs::msg::VoxelChannel>::SharedPtr> band_pubs_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;

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
