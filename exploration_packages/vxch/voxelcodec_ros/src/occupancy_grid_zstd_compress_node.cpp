// Third map-transport condition: identical to baseline (relays the raw
// OccupancyGrid, no wavelet/tiling/scheduling) except the serialized message
// is zstd-compressed before publishing, and decompressed back to an
// OccupancyGrid on the receiving end by occupancy_grid_zstd_decompress_node.
// Exists to isolate what generic zstd compression alone buys on top of
// baseline, independent of vxch's Haar/tiling encoding.
#include <cstdint>
#include <cstring>
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

// Compressed messages carry the original serialized size as an 8-byte
// little-endian header so the decompressor can size its output buffer
// without needing ZSTD_getFrameContentSize (which requires the frame
// content size flag, not guaranteed across zstd versions/settings).
constexpr std::size_t kHeaderBytes = 8;

class OccupancyGridZstdCompressNode : public rclcpp::Node
{
public:
  OccupancyGridZstdCompressNode()
  : Node("occupancy_grid_zstd_compress_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/map");
    output_topic_ = declare_parameter<std::string>("output_topic", "/zstd/map");
    compression_level_ = declare_parameter<int>("compression_level", 9);

    const auto map_qos = rclcpp::QoS(1)
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);
    const auto out_qos = rclcpp::QoS(1)
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);

    pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(output_topic_, out_qos);
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic_, map_qos,
      std::bind(&OccupancyGridZstdCompressNode::on_map, this, std::placeholders::_1));
  }

private:
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    rclcpp::SerializedMessage serialized;
    serializer_.serialize_message(msg.get(), &serialized);
    const auto & rcl = serialized.get_rcl_serialized_message();
    const std::size_t raw_size = rcl.buffer_length;

    std::vector<std::uint8_t> out(kHeaderBytes + ZSTD_compressBound(raw_size));
    for (std::size_t i = 0; i < kHeaderBytes; ++i) {
      out[i] = static_cast<std::uint8_t>((raw_size >> (8 * i)) & 0xFF);
    }
    const auto result = ZSTD_compress(
      out.data() + kHeaderBytes, out.size() - kHeaderBytes,
      rcl.buffer, raw_size, compression_level_);
    if (ZSTD_isError(result)) {
      RCLCPP_ERROR(get_logger(), "zstd compress failed: %s", ZSTD_getErrorName(result));
      return;
    }
    out.resize(kHeaderBytes + result);

    std_msgs::msg::UInt8MultiArray out_msg;
    out_msg.data = std::move(out);
    pub_->publish(out_msg);
  }

  std::string input_topic_;
  std::string output_topic_;
  int compression_level_;
  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_;
};

}  // namespace
}  // namespace voxelcodec_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<voxelcodec_ros::OccupancyGridZstdCompressNode>());
  rclcpp::shutdown();
  return 0;
}
