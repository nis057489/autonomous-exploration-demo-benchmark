#pragma once

#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

// Minimal on-disk format for an int8 occupancy grid (same value semantics as
// nav_msgs/OccupancyGrid.data: -1 = unknown, 0 = free, 100 = occupied), used
// so the C++ codec CLI and the Python GUI can both read/write it without
// pulling in ROS message serialization.
//
// Layout: magic "VXG1" (4 bytes) | width u32 LE | height u32 LE |
// resolution f32 LE | width*height bytes of int8 row-major data.
namespace vxch_test
{

struct SyntheticGrid
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  float resolution{0.05f};
  std::vector<std::int8_t> data;
};

inline constexpr char kGridMagic[4] = {'V', 'X', 'G', '1'};

inline void write_grid(const std::string & path, const SyntheticGrid & grid)
{
  if (grid.data.size() != static_cast<std::size_t>(grid.width) * grid.height) {
    throw std::runtime_error("write_grid: data size does not match width*height");
  }
  std::ofstream out(path, std::ios::binary);
  if (!out) {throw std::runtime_error("write_grid: could not open " + path);}
  out.write(kGridMagic, 4);
  out.write(reinterpret_cast<const char *>(&grid.width), 4);
  out.write(reinterpret_cast<const char *>(&grid.height), 4);
  out.write(reinterpret_cast<const char *>(&grid.resolution), 4);
  out.write(reinterpret_cast<const char *>(grid.data.data()),
    static_cast<std::streamsize>(grid.data.size()));
  if (!out) {throw std::runtime_error("write_grid: write failed for " + path);}
}

inline SyntheticGrid read_grid(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {throw std::runtime_error("read_grid: could not open " + path);}
  char magic[4];
  in.read(magic, 4);
  if (std::memcmp(magic, kGridMagic, 4) != 0) {
    throw std::runtime_error("read_grid: bad magic in " + path);
  }
  SyntheticGrid grid;
  in.read(reinterpret_cast<char *>(&grid.width), 4);
  in.read(reinterpret_cast<char *>(&grid.height), 4);
  in.read(reinterpret_cast<char *>(&grid.resolution), 4);
  const std::size_t n = static_cast<std::size_t>(grid.width) * grid.height;
  grid.data.resize(n);
  in.read(reinterpret_cast<char *>(grid.data.data()), static_cast<std::streamsize>(n));
  if (!in) {throw std::runtime_error("read_grid: truncated file " + path);}
  return grid;
}

}  // namespace vxch_test
