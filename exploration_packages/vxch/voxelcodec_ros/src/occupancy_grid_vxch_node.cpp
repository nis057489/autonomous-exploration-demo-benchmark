#include <algorithm>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
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
    // Publishing every changed band in one burst monopolizes the shared DDIL link for the
    // whole burst's duration -- each ddil_proxy_node instance (one per peer link, possibly on
    // different robots) throttles independently with no cross-robot coordination, so nothing
    // stops multiple robots' bursts from overlapping and contending for the same real airtime.
    // Sending only this many of the highest-priority (coarsest) pending bands per send tick
    // and carrying the rest over spaces this robot's own traffic out, leaving gaps for peers
    // to get a word in. 1 = strictly one band per tick.
    max_bands_per_update_ = declare_parameter<int>("max_bands_per_update", 1);
    if (max_bands_per_update_ < 1) {
      throw std::runtime_error("max_bands_per_update must be >= 1");
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

    const auto map_qos = rclcpp::QoS(1)
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);
    const auto band_qos = rclcpp::QoS(1).best_effort();

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

    last_band_fingerprint_.assign(static_cast<std::size_t>(total_bands), 0);

    send_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / send_rate_hz_),
      [this]() {send_pending_bands();});

    RCLCPP_INFO(
      get_logger(),
      "Encoding %s → %s/manifest + band_0..band_%d (haar_levels=%d, compression=%s, "
      "send_rate_hz=%.2f, max_bands_per_update=%d)",
      input_topic_.c_str(), output_base_topic_.c_str(), haar_levels_,
      haar_levels_, compression_.c_str(), send_rate_hz_, max_bands_per_update_);
  }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
  {
    if (msg->info.width == 0 || msg->info.height == 0) {
      return;
    }

    const std::size_t N = static_cast<std::size_t>(msg->info.width) *
      static_cast<std::size_t>(msg->info.height);

    if (msg->data.size() != N) {
      RCLCPP_WARN(get_logger(), "OccupancyGrid data size mismatch: expected %zu got %zu",
        N, msg->data.size());
      return;
    }

    // Convert int8 → uint32
    std::vector<std::uint32_t> values(N);
    for (std::size_t i = 0; i < N; ++i) {
      values[i] = shift_to_uint32(msg->data[i]);
    }

    // Forward Haar + split into per-band EncodedChannels
    std::vector<EncodedChannel> bands;
    try {
      bands = make_haar_bands(values, msg->info.width, msg->info.height, haar_levels_, compression_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "make_haar_bands failed: %s", e.what());
      return;
    }

    // Build manifest
    Manifest manifest;
    manifest.voxel_count = static_cast<std::uint32_t>(N);
    manifest.metadata["grid_width"] = std::to_string(msg->info.width);
    manifest.metadata["grid_height"] = std::to_string(msg->info.height);
    manifest.metadata["resolution"] = std::to_string(msg->info.resolution);
    manifest.metadata["origin_x"] = std::to_string(msg->info.origin.position.x);
    manifest.metadata["origin_y"] = std::to_string(msg->info.origin.position.y);
    manifest.metadata["frame_id"] = msg->header.frame_id;
    manifest.metadata["stamp_sec"] = std::to_string(msg->header.stamp.sec);
    manifest.metadata["stamp_nanosec"] = std::to_string(msg->header.stamp.nanosec);
    for (const auto & band : bands) {
      manifest.channels.push_back(band.descriptor);
    }

    // Determine which bands changed by fingerprinting compressed payloads.
    // Bands whose payload is identical to the last transmission are skipped —
    // the decoder already has them and doesn't need to re-download.
    std::vector<std::size_t> changed_indices;
    for (std::size_t k = 0; k < bands.size(); ++k) {
      const auto & payload = bands[k].payload;
      const std::size_t fp =
        payload.size() ^
        std::hash<std::string>{}(std::string(payload.begin(), payload.end()));
      if (fp != last_band_fingerprint_[k]) {
        changed_indices.push_back(k);
        last_band_fingerprint_[k] = fp;
      }
    }

    // Merge newly-changed bands into the pending backlog (latest payload wins per index --
    // same dedup semantics as ddil_proxy's queue downstream). Actual sending happens on
    // send_timer_'s own fixed schedule, not here -- see send_rate_hz_ above for why.
    for (std::size_t k : changed_indices) {
      pending_bands_[k] = bands[k];
    }

    // Manifest/header used by send_pending_bands() are always the latest known geometry,
    // regardless of how much (if any) of the backlog the timer has drained so far.
    latest_header_ = msg->header;
    latest_manifest_ = std::move(manifest);
    has_map_ = true;

    if (changed_indices.empty()) {
      RCLCPP_DEBUG(get_logger(), "all %zu bands unchanged — nothing queued", bands.size());
    } else {
      RCLCPP_DEBUG(
        get_logger(), "%zu band(s) changed, %zu now pending", changed_indices.size(),
        pending_bands_.size());
    }
  }

  void send_pending_bands()
  {
    if (!has_map_ || pending_bands_.empty()) {
      return;
    }

    // Publish manifest before bands (TRANSIENT_LOCAL — reaches late-joining decoders). Sent
    // every tick a band goes out, using the latest known geometry; the bands sent below are
    // stamped with this same header, so they can never be judged stale against it downstream
    // (see ddil_stale_epoch.hpp) even if their payload was computed several on_map calls ago.
    manifest_pub_->publish(manifest_to_msg(latest_header_, stream_id_, latest_manifest_));

    // Drain up to max_bands_per_update_ pending bands, coarsest (lowest index) first --
    // std::map keeps keys sorted, so begin() is always the next-highest-priority entry.
    std::size_t sent_count = 0;
    std::size_t sent_bytes = 0;
    std::ostringstream ss;
    ss << "[";
    for (int i = 0; i < max_bands_per_update_ && !pending_bands_.empty(); ++i) {
      auto it = pending_bands_.begin();
      const std::size_t k = it->first;
      const EncodedChannel channel = std::move(it->second);
      pending_bands_.erase(it);

      if (sent_count > 0) {ss << " ";}
      const double kb = static_cast<double>(channel.payload.size()) / 1024.0;
      ss << k << ":" << std::fixed << std::setprecision(1) << kb << "KB";
      sent_bytes += channel.payload.size();
      ++sent_count;

      band_pubs_[k]->publish(
        channel_to_msg(latest_header_, stream_id_, channel.descriptor, channel.payload));
    }
    ss << "]";

    RCLCPP_INFO(
      get_logger(), "send tick %s  total=%.1f KB  (%zu sent, %zu still pending)",
      ss.str().c_str(),
      static_cast<double>(sent_bytes) / 1024.0,
      sent_count, pending_bands_.size());
  }

  std::string input_topic_;
  std::string output_base_topic_;
  int haar_levels_;
  std::string compression_;
  std::string stream_id_;
  int max_bands_per_update_;
  double send_rate_hz_;

  rclcpp::Publisher<voxelcodec_msgs::msg::VoxelManifest>::SharedPtr manifest_pub_;
  std::vector<rclcpp::Publisher<voxelcodec_msgs::msg::VoxelChannel>::SharedPtr> band_pubs_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  std::vector<std::size_t> last_band_fingerprint_;
  // Bands whose fingerprint changed but haven't been sent yet, keyed by band index so the
  // lowest (coarsest/highest-priority) pending entry is always pending_bands_.begin().
  std::map<std::size_t, EncodedChannel> pending_bands_;
  // Latest known map geometry/timestamp, refreshed on every on_map() call, used by
  // send_pending_bands() regardless of which on_map() call queued the band it's sending.
  std_msgs::msg::Header latest_header_;
  Manifest latest_manifest_;
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
