#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include "voxelcodec_ros/ros_messages.hpp"
#include "voxelcodec_ros/tile_reconstructor.hpp"
#include "voxelcodec_ros/types.hpp"
#include "voxelcodec_msgs/msg/voxel_channel.hpp"
#include "voxelcodec_msgs/msg/voxel_manifest.hpp"

namespace voxelcodec_ros
{
namespace
{

// A tiled encoder can publish several different tiles' messages back-to-back
// on the SAME fixed band_k topic within one send tick (tile identity travels
// in the payload, not the topic). With a depth-1 queue, only the single latest
// sample published before this subscription's executor drains it would
// survive -- everything else is silently dropped at the DDS layer. Must match
// (or exceed) occupancy_grid_vxch_node's kBandQueueDepth.
constexpr std::size_t kBandQueueDepth = 64;

class VxchOccupancyGridNode : public rclcpp::Node
{
public:
  VxchOccupancyGridNode()
  : Node("vxch_occupancy_grid_node")
  {
    input_base_topic_ = declare_parameter<std::string>("input_base_topic", "/vxch/map");
    output_topic_ = declare_parameter<std::string>("output_topic", "");
    haar_levels_ = declare_parameter<int>("haar_levels", 4);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);

    if (output_topic_.empty()) {
      throw std::runtime_error("output_topic parameter is required");
    }
    if (haar_levels_ < 1 || haar_levels_ > 12) {
      throw std::runtime_error("haar_levels must be between 1 and 12");
    }

    reconstructor_ = std::make_unique<TileReconstructor>(haar_levels_);

    const auto map_qos = rclcpp::QoS(1)
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);
    const auto band_qos = rclcpp::QoS(kBandQueueDepth).best_effort();

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, map_qos);

    manifest_sub_ = create_subscription<voxelcodec_msgs::msg::VoxelManifest>(
      input_base_topic_ + "/manifest", map_qos,
      [this](voxelcodec_msgs::msg::VoxelManifest::ConstSharedPtr msg) {
        on_manifest(msg);
      });

    const int total_bands = haar_levels_ + 1;
    band_subs_.resize(static_cast<std::size_t>(total_bands));
    for (int k = 0; k < total_bands; ++k) {
      const std::string topic = input_base_topic_ + "/band_" + std::to_string(k);
      const int band_index = k;
      band_subs_[static_cast<std::size_t>(k)] =
        create_subscription<voxelcodec_msgs::msg::VoxelChannel>(
          topic, band_qos,
          [this, band_index](voxelcodec_msgs::msg::VoxelChannel::ConstSharedPtr msg) {
            on_band(band_index, msg);
          });
    }

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      [this]() {publish_map();});

    RCLCPP_INFO(
      get_logger(),
      "Decoding %s/band_0..band_%d → %s (haar_levels=%d, rate=%.1f Hz)",
      input_base_topic_.c_str(), haar_levels_, output_topic_.c_str(),
      haar_levels_, publish_rate_hz_);
  }

private:
  void on_manifest(const voxelcodec_msgs::msg::VoxelManifest::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const Manifest manifest = manifest_from_msg(*msg);
    const Stamp stamp{msg->header.stamp.sec, static_cast<std::uint32_t>(msg->header.stamp.nanosec)};
    if (!reconstructor_->ingest_manifest(manifest.metadata, stamp)) {
      RCLCPP_WARN(get_logger(), "Manifest missing grid_width/grid_height metadata");
    }
  }

  void on_band(int band_index, const voxelcodec_msgs::msg::VoxelChannel::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const ChannelDescriptor desc = descriptor_from_msg(msg->descriptor);
    const auto error = reconstructor_->ingest_band(band_index, desc, msg->payload);
    if (error.has_value()) {
      RCLCPP_WARN(get_logger(), "Failed to decode band %d: %s", band_index, error->c_str());
    }
  }

  void publish_map()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto reconstructed = reconstructor_->reconstruct();
    if (!reconstructed.has_value()) {
      return;
    }

    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp.sec = reconstructed->stamp.sec;
    out.header.stamp.nanosec = reconstructed->stamp.nanosec;
    out.header.frame_id = reconstructed->frame_id;
    out.info.map_load_time = out.header.stamp;
    out.info.resolution = reconstructed->resolution;
    out.info.width = reconstructed->width;
    out.info.height = reconstructed->height;
    out.info.origin.position.x = reconstructed->origin_x;
    out.info.origin.position.y = reconstructed->origin_y;
    out.info.origin.orientation.w = 1.0;
    out.data = std::move(reconstructed->data);

    map_pub_->publish(out);
  }

  std::string input_base_topic_;
  std::string output_topic_;
  int haar_levels_;
  double publish_rate_hz_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<voxelcodec_msgs::msg::VoxelManifest>::SharedPtr manifest_sub_;
  std::vector<rclcpp::Subscription<voxelcodec_msgs::msg::VoxelChannel>::SharedPtr> band_subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::unique_ptr<TileReconstructor> reconstructor_;
};

}  // namespace
}  // namespace voxelcodec_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<voxelcodec_ros::VxchOccupancyGridNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("vxch_occupancy_grid_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
