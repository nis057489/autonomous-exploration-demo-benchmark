#pragma once

// Pure data-transformation logic behind nav2_voxel_grid_publisher.cpp's Node,
// pulled into a header so it's testable without an rclcpp::Node fixture --
// same reasoning as ddil_stale_epoch.hpp. Node-specific glue (parameter
// declaration, publishers, the wall timer) stays in the .cpp.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>
#include <nav2_voxel_grid/voxel_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{

inline constexpr char kXChannel[] = "x";
inline constexpr char kYChannel[] = "y";
inline constexpr char kZChannel[] = "z";
inline constexpr char kOccupancyChannel[] = "occupancy";
inline constexpr std::size_t kNav2MaxZLayers = 16;
inline constexpr std::uint32_t kNav2DisplayColor = 0xFFFF7F0EU;

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

inline std::vector<std::uint8_t> read_binary_file(const std::filesystem::path & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("failed to open archive: " + path.string());
  }
  return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

inline double scalar_at(const ScalarBuffer & values, std::size_t index)
{
  return std::visit(
    [index](const auto & buffer) -> double {
      return static_cast<double>(buffer.at(index));
    }, values);
}

inline std::size_t scalar_count(const ScalarBuffer & values)
{
  return std::visit(
    [](const auto & buffer) -> std::size_t {
      return buffer.size();
    }, values);
}

inline bool occupied_at(const ScalarBuffer * values, std::size_t index)
{
  if (values == nullptr) {
    return true;
  }
  return scalar_at(*values, index) > 0.0;
}

inline std::uint32_t axis_size(double minimum, double maximum, double resolution)
{
  if (!(resolution > 0.0)) {
    throw std::runtime_error("grid resolution must be positive");
  }
  const double span = std::max(0.0, maximum - minimum);
  return static_cast<std::uint32_t>(std::floor(span / resolution + 1.0 + 1e-9));
}

inline GridSpec build_grid_spec(
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

inline unsigned int coordinate_to_index(
  double value, double origin, double resolution, std::uint32_t size)
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

inline nav2_msgs::msg::VoxelGrid build_voxel_grid_message(
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

inline sensor_msgs::msg::PointField make_point_field(
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

inline sensor_msgs::msg::PointCloud2 build_point_cloud_message(
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

inline nav_msgs::msg::OccupancyGrid build_occupancy_grid_message(
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

}  // namespace voxelcodec_ros
