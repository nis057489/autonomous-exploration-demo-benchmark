#include <algorithm>
#include <memory>
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
      bands = make_haar_bands(values, haar_levels_, compression_);
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

    // Publish manifest first (TRANSIENT_LOCAL — reaches late-joining decoders)
    manifest_pub_->publish(manifest_to_msg(header, stream_id_, manifest));

    // Publish bands coarsest-first so smallest message departs the DDS queue first
    for (std::size_t k = 0; k < bands.size(); ++k) {
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
