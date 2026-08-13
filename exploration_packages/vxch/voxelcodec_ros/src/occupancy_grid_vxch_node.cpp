#include <algorithm>
#include <functional>
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

    if (haar_levels_ < 1 || haar_levels_ > 12) {
      throw std::runtime_error("haar_levels must be between 1 and 12");
    }

    const auto map_qos = rclcpp::QoS(1)
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);
    // Was best_effort()/volatile: a late-joining subscriber (a fresh viz
    // client, a robot reconnecting mid-mission) would never receive a band
    // unless it happened to be subscribed at the exact moment the encoder
    // republished it, since volatile keeps no history to replay. Matching
    // manifest's QoS means a late joiner gets the current value of every
    // band immediately instead of waiting on the next map change.
    const auto band_qos = map_qos;

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

    RCLCPP_INFO(
      get_logger(),
      "Encoding %s → %s/manifest + band_0..band_%d (haar_levels=%d, compression=%s)",
      input_topic_.c_str(), output_base_topic_.c_str(), haar_levels_,
      haar_levels_, compression_.c_str());
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

    const std_msgs::msg::Header header = msg->header;

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

    if (changed_indices.empty()) {
      RCLCPP_DEBUG(get_logger(), "all %zu bands unchanged — nothing sent", bands.size());
      return;
    }

    // Build a human-readable summary: "[0:1.2KB* 1:3.4KB* 2:0.8KB 3:- 4:- 5:-]"
    // '*' = changed/sent, '-' = suppressed (unchanged), bare size = sent
    {
      std::ostringstream ss;
      ss << "[";
      std::size_t total_bytes = 0;
      for (std::size_t k = 0; k < bands.size(); ++k) {
        if (k > 0) {ss << " ";}
        const bool changed = std::find(
          changed_indices.begin(), changed_indices.end(), k) != changed_indices.end();
        if (changed) {
          const double kb = static_cast<double>(bands[k].payload.size()) / 1024.0;
          ss << k << ":" << std::fixed;
          ss.precision(1);
          ss << kb << "KB";
          total_bytes += bands[k].payload.size();
        } else {
          ss << k << ":-";
        }
      }
      ss << "]";
      RCLCPP_INFO(
        get_logger(), "encode bands %s  total=%.1f KB  (%zu/%zu bands sent)",
        ss.str().c_str(),
        static_cast<double>(total_bytes) / 1024.0,
        changed_indices.size(), bands.size());
    }

    // Publish manifest before bands (TRANSIENT_LOCAL — reaches late-joining decoders).
    manifest_pub_->publish(manifest_to_msg(header, stream_id_, manifest));

    // Publish only changed bands, coarsest-first.
    for (std::size_t k : changed_indices) {
      band_pubs_[k]->publish(
        channel_to_msg(header, stream_id_, bands[k].descriptor, bands[k].payload));
    }
  }

  std::string input_topic_;
  std::string output_base_topic_;
  int haar_levels_;
  std::string compression_;
  std::string stream_id_;

  rclcpp::Publisher<voxelcodec_msgs::msg::VoxelManifest>::SharedPtr manifest_pub_;
  std::vector<rclcpp::Publisher<voxelcodec_msgs::msg::VoxelChannel>::SharedPtr> band_pubs_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  std::vector<std::size_t> last_band_fingerprint_;
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
