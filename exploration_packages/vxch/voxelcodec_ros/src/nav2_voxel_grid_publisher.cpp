#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nav2_msgs/msg/voxel_grid.hpp>
#include <nav2_voxel_grid/voxel_grid.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/nav2_voxel_grid_logic.hpp"
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{
namespace
{

class Nav2VoxelGridPublisher : public rclcpp::Node
{
public:
  Nav2VoxelGridPublisher()
  : Node("nav2_voxel_grid_publisher")
  {
    const auto file_path = declare_parameter<std::string>("file_path", "");
    const auto base_topic = declare_parameter<std::string>("base_topic", "/voxelcodec_nav2");
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const auto publish_period = declare_parameter<double>("publish_period", 1.0);
    const auto resolution_xy = declare_parameter<double>("xy_resolution", 1.0);
    const auto resolution_z = declare_parameter<double>("z_resolution", 0.0);
    const auto max_z_layers = declare_parameter<int>("max_z_layers", static_cast<int>(kNav2MaxZLayers));
    const auto publish_rviz_topics = declare_parameter<bool>("publish_rviz_topics", false);

    if (file_path.empty()) {
      throw std::runtime_error("file_path parameter is required");
    }
    if (publish_period <= 0.0) {
      throw std::runtime_error("publish_period must be positive");
    }
    if (resolution_xy <= 0.0) {
      throw std::runtime_error("xy_resolution must be positive");
    }
    if (max_z_layers <= 0 || max_z_layers > static_cast<int>(kNav2MaxZLayers)) {
      throw std::runtime_error("max_z_layers must be between 1 and 16");
    }

    const auto archive_bytes = read_binary_file(file_path);
    const auto archive = read_archive(archive_bytes);
    const auto decoded = decode_selected(archive, {kXChannel, kYChannel, kZChannel});
    const auto spec = build_grid_spec(
      decoded.at(kXChannel).values,
      decoded.at(kYChannel).values,
      decoded.at(kZChannel).values,
      resolution_xy,
      resolution_z,
      static_cast<std::uint32_t>(max_z_layers));
    grid_ = std::make_unique<nav2_voxel_grid::VoxelGrid>(spec.size_x, spec.size_y, spec.size_z);
    grid_->reset();
    message_ = build_voxel_grid_message(
      archive,
      resolution_xy,
      resolution_z,
      static_cast<std::uint32_t>(max_z_layers),
      frame_id_);
    std::vector<std::uint32_t> quantized_grid_data(message_.data.begin(), message_.data.end());
    std::copy(quantized_grid_data.begin(), quantized_grid_data.end(), grid_->getData());

    publisher_ = create_publisher<nav2_msgs::msg::VoxelGrid>(base_topic + "/voxel_grid", 10);
    if (publish_rviz_topics) {
      point_cloud_message_ = build_point_cloud_message(spec, quantized_grid_data, frame_id_);
      occupancy_grid_message_ = build_occupancy_grid_message(spec, quantized_grid_data, frame_id_);
      point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(base_topic + "/rviz/points", 10);
      occupancy_grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(base_topic + "/rviz/occupancy_grid", 10);
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period),
      std::bind(&Nav2VoxelGridPublisher::publish_message, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing nav2_msgs/msg/VoxelGrid %ux%ux%u on %s/voxel_grid",
      message_.size_x,
      message_.size_y,
      message_.size_z,
      base_topic.c_str());
  }

private:
  void publish_message()
  {
    message_.header.stamp = now();
    publisher_->publish(message_);
    if (point_cloud_publisher_) {
      point_cloud_message_.header.stamp = message_.header.stamp;
      point_cloud_publisher_->publish(point_cloud_message_);
    }
    if (occupancy_grid_publisher_) {
      occupancy_grid_message_.header.stamp = message_.header.stamp;
      occupancy_grid_message_.info.map_load_time = message_.header.stamp;
      occupancy_grid_publisher_->publish(occupancy_grid_message_);
    }
  }

  std::string frame_id_;
  std::unique_ptr<nav2_voxel_grid::VoxelGrid> grid_;
  nav2_msgs::msg::VoxelGrid message_;
  sensor_msgs::msg::PointCloud2 point_cloud_message_;
  nav_msgs::msg::OccupancyGrid occupancy_grid_message_;
  rclcpp::Publisher<nav2_msgs::msg::VoxelGrid>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace voxelcodec_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<voxelcodec_ros::Nav2VoxelGridPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("nav2_voxel_grid_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}