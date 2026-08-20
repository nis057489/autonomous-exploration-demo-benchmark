// Decompresses messages published by occupancy_grid_zstd_compress_node back
// into an OccupancyGrid. Deployed one instance per peer, on the incoming/
// side of the DDIL relay, so its output topic matches the vxch decoder's
// contract exactly (/{robot}/incoming/{peer}/map) -- downstream team-map
// fusion needs zero changes to accept either transport.
#include <cstdint>
#include <stdexcept>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <zstd.h>

namespace voxelcodec_ros
{
namespace
{

constexpr std::size_t kHeaderBytes = 8;

class OccupancyGridZstdDecompressNode : public rclcpp::Node
{
public:
  OccupancyGridZstdDecompressNode()
  : Node("occupancy_grid_zstd_decompress_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/zstd/map");
    output_topic_ = declare_parameter<std::string>("output_topic", "/map");

    const auto qos = rclcpp::QoS(1)
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, qos);
    sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      input_topic_, qos,
      std::bind(&OccupancyGridZstdDecompressNode::on_compressed, this, std::placeholders::_1));
  }

private:
  void on_compressed(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < kHeaderBytes) {
      RCLCPP_ERROR(get_logger(), "compressed message too short (%zu bytes)", msg->data.size());
      return;
    }
    std::size_t raw_size = 0;
    for (std::size_t i = 0; i < kHeaderBytes; ++i) {
      raw_size |= static_cast<std::size_t>(msg->data[i]) << (8 * i);
    }

    rclcpp::SerializedMessage serialized(raw_size);
    auto & rcl = serialized.get_rcl_serialized_message();
    const auto result = ZSTD_decompress(
      rcl.buffer, raw_size,
      msg->data.data() + kHeaderBytes, msg->data.size() - kHeaderBytes);
    if (ZSTD_isError(result)) {
      RCLCPP_ERROR(get_logger(), "zstd decompress failed: %s", ZSTD_getErrorName(result));
      return;
    }
    if (result != raw_size) {
      RCLCPP_ERROR(get_logger(), "decompressed size mismatch: expected %zu, got %zu",
        raw_size, result);
      return;
    }
    rcl.buffer_length = raw_size;

    auto out_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    deserializer_.deserialize_message(&serialized, out_msg.get());
    pub_->publish(*out_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> deserializer_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

}  // namespace
}  // namespace voxelcodec_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<voxelcodec_ros::OccupancyGridZstdDecompressNode>());
  rclcpp::shutdown();
  return 0;
}
