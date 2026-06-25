#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>
#include <nav2_voxel_grid/voxel_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{
namespace
{

constexpr char kXChannel[] = "x";
constexpr char kYChannel[] = "y";
constexpr char kZChannel[] = "z";
constexpr char kOccupancyChannel[] = "occupancy";
constexpr std::size_t kNav2MaxZLayers = 16;
constexpr std::uint32_t kNav2DisplayColor = 0xFFFF7F0EU;

struct GridSpec
{
  double origin_x{};
  double origin_y{};
  double origin_z{};
  double resolution_xy{1.0};
  double resolution_z{1.0};
  std::uint32_t size_x{1};
  std::uint32_t size_y{1};
  std::uint32_t size_z{1};
};

std::vector<std::uint8_t> read_binary_file(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("failed to open archive: " + path.string());
  }
  return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

double scalar_at(const ScalarBuffer & values, std::size_t index)
{
  return std::visit(
    [index](const auto & buffer) -> double {
      return static_cast<double>(buffer.at(index));
    }, values);
}

std::size_t scalar_count(const ScalarBuffer & values)
{
  return std::visit(
    [](const auto & buffer) -> std::size_t {
      return buffer.size();
    }, values);
}

bool occupied_at(const ScalarBuffer * values, std::size_t index)
{
  if (values == nullptr) {
    return true;
  }
  return scalar_at(*values, index) > 0.0;
}

std::uint32_t axis_size(double minimum, double maximum, double resolution)
{
  if (!(resolution > 0.0)) {
    throw std::runtime_error("grid resolution must be positive");
  }
  const double span = std::max(0.0, maximum - minimum);
  return static_cast<std::uint32_t>(std::floor(span / resolution + 1.0 + 1e-9));
}

GridSpec build_grid_spec(
  const ScalarBuffer & xs,
  const ScalarBuffer & ys,
  const ScalarBuffer & zs,
  double resolution_xy,
  double configured_resolution_z,
  std::uint32_t max_z_layers)
{
  const std::size_t count = scalar_count(xs);
  if (count == 0) {
    return GridSpec{};
  }

  double min_x = scalar_at(xs, 0);
  double max_x = min_x;
  double min_y = scalar_at(ys, 0);
  double max_y = min_y;
  double min_z = scalar_at(zs, 0);
  double max_z = min_z;

  for (std::size_t index = 1; index < count; ++index) {
    const double x = scalar_at(xs, index);
    const double y = scalar_at(ys, index);
    const double z = scalar_at(zs, index);
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
    min_z = std::min(min_z, z);
    max_z = std::max(max_z, z);
  }

  GridSpec spec;
  spec.origin_x = min_x;
  spec.origin_y = min_y;
  spec.origin_z = min_z;
  spec.resolution_xy = resolution_xy;
  spec.size_x = axis_size(min_x, max_x, resolution_xy);
  spec.size_y = axis_size(min_y, max_y, resolution_xy);

  if (configured_resolution_z > 0.0) {
    spec.resolution_z = configured_resolution_z;
    spec.size_z = axis_size(min_z, max_z, configured_resolution_z);
    if (spec.size_z > max_z_layers) {
      throw std::runtime_error(
              "configured z resolution produces " + std::to_string(spec.size_z) +
              " layers, but nav2_voxel_grid supports at most " + std::to_string(max_z_layers));
    }
  } else {
    const double span_z = std::max(0.0, max_z - min_z);
    if (span_z == 0.0 || max_z_layers <= 1) {
      spec.resolution_z = 1.0;
      spec.size_z = 1;
    } else {
      spec.resolution_z = std::max(span_z / static_cast<double>(max_z_layers - 1), 1e-9);
      spec.size_z = axis_size(min_z, max_z, spec.resolution_z);
      spec.size_z = std::min(spec.size_z, max_z_layers);
    }
  }

  return spec;
}

unsigned int coordinate_to_index(double value, double origin, double resolution, std::uint32_t size)
{
  if (size == 0) {
    throw std::runtime_error("invalid zero-sized grid axis");
  }

  const double scaled = std::floor(((value - origin) / resolution) + 1e-9);
  if (scaled <= 0.0) {
    return 0U;
  }
  const double max_index = static_cast<double>(size - 1);
  if (scaled >= max_index) {
    return static_cast<unsigned int>(size - 1);
  }
  return static_cast<unsigned int>(scaled);
}

nav2_msgs::msg::VoxelGrid build_voxel_grid_message(
  const Archive & archive,
  double resolution_xy,
  double configured_resolution_z,
  std::uint32_t max_z_layers,
  const std::string & frame_id)
{
  const auto decoded = decode_selected(archive, {kXChannel, kYChannel, kZChannel, kOccupancyChannel});
  const auto & xs = decoded.at(kXChannel).values;
  const auto & ys = decoded.at(kYChannel).values;
  const auto & zs = decoded.at(kZChannel).values;

  const std::size_t count = scalar_count(xs);
  if (scalar_count(ys) != count || scalar_count(zs) != count) {
    throw std::runtime_error("x, y, and z channel lengths do not match");
  }

  const ScalarBuffer * occupancy = nullptr;
  const auto occupancy_it = decoded.find(kOccupancyChannel);
  if (occupancy_it != decoded.end()) {
    if (scalar_count(occupancy_it->second.values) != count) {
      throw std::runtime_error("occupancy channel length does not match x/y/z channels");
    }
    occupancy = &occupancy_it->second.values;
  }

  const GridSpec spec = build_grid_spec(xs, ys, zs, resolution_xy, configured_resolution_z, max_z_layers);
  nav2_voxel_grid::VoxelGrid grid(spec.size_x, spec.size_y, spec.size_z);
  grid.reset();

  for (std::size_t index = 0; index < count; ++index) {
    if (!occupied_at(occupancy, index)) {
      continue;
    }

    const auto x_index = coordinate_to_index(scalar_at(xs, index), spec.origin_x, spec.resolution_xy, spec.size_x);
    const auto y_index = coordinate_to_index(scalar_at(ys, index), spec.origin_y, spec.resolution_xy, spec.size_y);
    const auto z_index = coordinate_to_index(scalar_at(zs, index), spec.origin_z, spec.resolution_z, spec.size_z);
    grid.markVoxel(x_index, y_index, z_index);
  }

  nav2_msgs::msg::VoxelGrid message;
  message.header.frame_id = frame_id;
  message.origin.x = static_cast<float>(spec.origin_x);
  message.origin.y = static_cast<float>(spec.origin_y);
  message.origin.z = static_cast<float>(spec.origin_z);
  message.resolutions.x = spec.resolution_xy;
  message.resolutions.y = spec.resolution_xy;
  message.resolutions.z = spec.resolution_z;
  message.size_x = spec.size_x;
  message.size_y = spec.size_y;
  message.size_z = spec.size_z;

  const auto cell_count = static_cast<std::size_t>(spec.size_x) * static_cast<std::size_t>(spec.size_y);
  const auto * data = grid.getData();
  message.data.assign(data, data + cell_count);
  return message;
}

sensor_msgs::msg::PointField make_point_field(
  const std::string & name,
  std::uint32_t offset,
  std::uint8_t datatype,
  std::uint32_t count)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = count;
  return field;
}

sensor_msgs::msg::PointCloud2 build_point_cloud_message(
  const GridSpec & spec,
  const std::vector<std::uint32_t> & grid_data,
  const std::string & frame_id)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = frame_id;
  message.height = 1;
  message.is_bigendian = false;
  message.is_dense = true;
  message.point_step = 20;
  message.fields = {
    make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1),
    make_point_field("rgba", 12, sensor_msgs::msg::PointField::UINT32, 1),
    make_point_field("intensity", 16, sensor_msgs::msg::PointField::FLOAT32, 1),
  };

  std::vector<std::uint8_t> data;
  for (std::uint32_t y_index = 0; y_index < spec.size_y; ++y_index) {
    for (std::uint32_t x_index = 0; x_index < spec.size_x; ++x_index) {
      for (std::uint32_t z_index = 0; z_index < spec.size_z; ++z_index) {
        if (
          nav2_voxel_grid::VoxelGrid::getVoxel(
            x_index,
            y_index,
            z_index,
            spec.size_x,
            spec.size_y,
            spec.size_z,
            grid_data.data()) != nav2_voxel_grid::MARKED)
        {
          continue;
        }

        const float x_value = static_cast<float>(spec.origin_x + spec.resolution_xy * static_cast<double>(x_index));
        const float y_value = static_cast<float>(spec.origin_y + spec.resolution_xy * static_cast<double>(y_index));
        const float z_value = static_cast<float>(spec.origin_z + spec.resolution_z * static_cast<double>(z_index));
        const float intensity = 1.0F;

        const auto * x_ptr = reinterpret_cast<const std::uint8_t *>(&x_value);
        data.insert(data.end(), x_ptr, x_ptr + sizeof(float));
        const auto * y_ptr = reinterpret_cast<const std::uint8_t *>(&y_value);
        data.insert(data.end(), y_ptr, y_ptr + sizeof(float));
        const auto * z_ptr = reinterpret_cast<const std::uint8_t *>(&z_value);
        data.insert(data.end(), z_ptr, z_ptr + sizeof(float));
        const auto * color_ptr = reinterpret_cast<const std::uint8_t *>(&kNav2DisplayColor);
        data.insert(data.end(), color_ptr, color_ptr + sizeof(std::uint32_t));
        const auto * intensity_ptr = reinterpret_cast<const std::uint8_t *>(&intensity);
        data.insert(data.end(), intensity_ptr, intensity_ptr + sizeof(float));
      }
    }
  }

  message.width = static_cast<std::uint32_t>(data.size() / message.point_step);
  message.row_step = message.width * message.point_step;
  message.data = std::move(data);
  return message;
}

nav_msgs::msg::OccupancyGrid build_occupancy_grid_message(
  const GridSpec & spec,
  const std::vector<std::uint32_t> & grid_data,
  const std::string & frame_id)
{
  nav_msgs::msg::OccupancyGrid message;
  message.header.frame_id = frame_id;
  message.info = nav_msgs::msg::MapMetaData();
  message.info.resolution = static_cast<float>(spec.resolution_xy);
  message.info.width = spec.size_x;
  message.info.height = spec.size_y;
  message.info.origin.position = geometry_msgs::msg::Point();
  message.info.origin.position.x = spec.origin_x;
  message.info.origin.position.y = spec.origin_y;
  message.info.origin.position.z = 0.0;
  message.info.origin.orientation = geometry_msgs::msg::Quaternion();
  message.info.origin.orientation.w = 1.0;
  message.data.assign(static_cast<std::size_t>(spec.size_x) * static_cast<std::size_t>(spec.size_y), 0);

  for (std::uint32_t y_index = 0; y_index < spec.size_y; ++y_index) {
    for (std::uint32_t x_index = 0; x_index < spec.size_x; ++x_index) {
      bool occupied = false;
      for (std::uint32_t z_index = 0; z_index < spec.size_z; ++z_index) {
        if (
          nav2_voxel_grid::VoxelGrid::getVoxel(
            x_index,
            y_index,
            z_index,
            spec.size_x,
            spec.size_y,
            spec.size_z,
            grid_data.data()) == nav2_voxel_grid::MARKED)
        {
          occupied = true;
          break;
        }
      }
      message.data[static_cast<std::size_t>(y_index) * spec.size_x + x_index] = occupied ? 100 : 0;
    }
  }

  return message;
}

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