#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "voxelcodec_ros/nav2_voxel_grid_logic.hpp"

namespace
{

using voxelcodec_ros::ChannelDescriptor;
using voxelcodec_ros::EncodedChannel;
using voxelcodec_ros::Manifest;
using voxelcodec_ros::ScalarBuffer;

void push_f32le(std::vector<std::uint8_t> & out, float v)
{
  std::uint8_t bytes[4];
  std::memcpy(bytes, &v, 4);
  out.insert(out.end(), bytes, bytes + 4);
}

// Builds a minimal x/y/z(+occupancy) archive the way voxel_display.cpp's wvmp
// point-cloud channels are encoded: raw-le float32 for coordinates, raw-le
// uint8 for occupancy (occupied_at treats any value > 0 as occupied).
std::vector<std::uint8_t> build_point_archive(
  const std::vector<float> & xs, const std::vector<float> & ys, const std::vector<float> & zs,
  const std::vector<std::uint8_t> * occupancy = nullptr)
{
  const auto n = static_cast<std::uint32_t>(xs.size());
  auto float_channel = [&](const std::string & name, const std::vector<float> & values) {
      ChannelDescriptor desc;
      desc.name = name;
      desc.data_type = voxelcodec_ros::kDataTypeFloat32;
      desc.encoding = voxelcodec_ros::kEncodingRawLE;
      desc.compression = voxelcodec_ros::kCompressionNone;
      desc.element_count = n;
      std::vector<std::uint8_t> payload;
      for (const auto v : values) {push_f32le(payload, v);}
      return EncodedChannel{desc, payload};
    };

  Manifest manifest;
  std::vector<EncodedChannel> channels = {
    float_channel("x", xs), float_channel("y", ys), float_channel("z", zs)};

  if (occupancy != nullptr) {
    ChannelDescriptor desc;
    desc.name = "occupancy";
    desc.data_type = voxelcodec_ros::kDataTypeUint8;
    desc.encoding = voxelcodec_ros::kEncodingRawLE;
    desc.compression = voxelcodec_ros::kCompressionNone;
    desc.element_count = n;
    channels.push_back(EncodedChannel{desc, *occupancy});
  }

  for (const auto & c : channels) {manifest.channels.push_back(c.descriptor);}
  manifest.voxel_count = n;
  return voxelcodec_ros::encode_archive(manifest, channels);
}

}  // namespace

TEST(Nav2VoxelGridLogic, ReadBinaryFileRoundTrips)
{
  const std::string path = "/tmp/vxch_test_nav2_read_binary.bin";
  const std::vector<std::uint8_t> content{1, 2, 3, 250, 251, 252};
  {
    std::ofstream out(path, std::ios::binary);
    out.write(reinterpret_cast<const char *>(content.data()), static_cast<std::streamsize>(content.size()));
  }
  const auto read_back = voxelcodec_ros::read_binary_file(path);
  EXPECT_EQ(read_back, content);
  std::remove(path.c_str());
}

TEST(Nav2VoxelGridLogic, ReadBinaryFileMissingThrows)
{
  EXPECT_THROW(
    voxelcodec_ros::read_binary_file("/tmp/vxch_test_does_not_exist_12345.bin"),
    std::runtime_error);
}

TEST(Nav2VoxelGridLogic, ScalarHelpersAcrossTypes)
{
  // scalar_at/scalar_count's std::visit lambda is a separate instantiation
  // per ScalarBuffer alternative -- all six need direct exercise, not just
  // whichever types build_voxel_grid_message happens to use elsewhere.
  {
    const ScalarBuffer v = std::vector<std::uint8_t>{0, 5, 200};
    EXPECT_EQ(voxelcodec_ros::scalar_count(v), 3U);
    EXPECT_DOUBLE_EQ(voxelcodec_ros::scalar_at(v, 1), 5.0);
  }
  {
    const ScalarBuffer v = std::vector<std::uint16_t>{0, 500, 60000};
    EXPECT_EQ(voxelcodec_ros::scalar_count(v), 3U);
    EXPECT_DOUBLE_EQ(voxelcodec_ros::scalar_at(v, 1), 500.0);
  }
  {
    const ScalarBuffer v = std::vector<std::uint32_t>{0, 70000};
    EXPECT_EQ(voxelcodec_ros::scalar_count(v), 2U);
    EXPECT_DOUBLE_EQ(voxelcodec_ros::scalar_at(v, 1), 70000.0);
  }
  {
    const ScalarBuffer v = std::vector<std::uint64_t>{0, 10000000000ULL};
    EXPECT_EQ(voxelcodec_ros::scalar_count(v), 2U);
    EXPECT_DOUBLE_EQ(voxelcodec_ros::scalar_at(v, 1), 10000000000.0);
  }
  {
    const ScalarBuffer v = std::vector<float>{1.5F, -2.5F};
    EXPECT_EQ(voxelcodec_ros::scalar_count(v), 2U);
    EXPECT_DOUBLE_EQ(voxelcodec_ros::scalar_at(v, 1), -2.5);
  }
  {
    const ScalarBuffer v = std::vector<double>{1.25, -3.5};
    EXPECT_EQ(voxelcodec_ros::scalar_count(v), 2U);
    EXPECT_DOUBLE_EQ(voxelcodec_ros::scalar_at(v, 1), -3.5);
  }
}

TEST(Nav2VoxelGridLogic, OccupiedAtNullMeansAlwaysOccupied)
{
  EXPECT_TRUE(voxelcodec_ros::occupied_at(nullptr, 0));
}

TEST(Nav2VoxelGridLogic, OccupiedAtThresholdIsStrictlyPositive)
{
  const ScalarBuffer values = std::vector<std::uint8_t>{0, 1, 100};
  EXPECT_FALSE(voxelcodec_ros::occupied_at(&values, 0));
  EXPECT_TRUE(voxelcodec_ros::occupied_at(&values, 1));
  EXPECT_TRUE(voxelcodec_ros::occupied_at(&values, 2));
}

TEST(Nav2VoxelGridLogic, AxisSizeRejectsNonPositiveResolution)
{
  EXPECT_THROW(voxelcodec_ros::axis_size(0.0, 10.0, 0.0), std::runtime_error);
  EXPECT_THROW(voxelcodec_ros::axis_size(0.0, 10.0, -1.0), std::runtime_error);
}

TEST(Nav2VoxelGridLogic, AxisSizeComputesSpanOverResolution)
{
  // 10-unit span at 1.0 resolution -> 11 cells (fencepost: both ends inclusive).
  EXPECT_EQ(voxelcodec_ros::axis_size(0.0, 10.0, 1.0), 11U);
  EXPECT_EQ(voxelcodec_ros::axis_size(5.0, 5.0, 1.0), 1U);
}

TEST(Nav2VoxelGridLogic, BuildGridSpecEmptyReturnsDefault)
{
  const ScalarBuffer empty = std::vector<float>{};
  const auto spec = voxelcodec_ros::build_grid_spec(empty, empty, empty, 1.0, 0.0, 16);
  EXPECT_EQ(spec.size_x, 1U);
  EXPECT_EQ(spec.size_y, 1U);
  EXPECT_EQ(spec.size_z, 1U);
}

TEST(Nav2VoxelGridLogic, BuildGridSpecConfiguredZResolutionExceedingLimitThrows)
{
  const ScalarBuffer xs = std::vector<float>{0.0F, 1.0F};
  const ScalarBuffer ys = std::vector<float>{0.0F, 1.0F};
  const ScalarBuffer zs = std::vector<float>{0.0F, 100.0F};
  // z spans 100 units; a configured resolution of 1.0 needs 101 layers, way over max_z_layers=16.
  EXPECT_THROW(
    voxelcodec_ros::build_grid_spec(xs, ys, zs, 1.0, 1.0, 16), std::runtime_error);
}

TEST(Nav2VoxelGridLogic, BuildGridSpecZeroZSpanCollapsesToOneLayer)
{
  const ScalarBuffer xs = std::vector<float>{0.0F, 1.0F};
  const ScalarBuffer ys = std::vector<float>{0.0F, 1.0F};
  const ScalarBuffer zs = std::vector<float>{3.0F, 3.0F};  // all same z
  const auto spec = voxelcodec_ros::build_grid_spec(xs, ys, zs, 1.0, 0.0, 16);
  EXPECT_EQ(spec.size_z, 1U);
  EXPECT_DOUBLE_EQ(spec.resolution_z, 1.0);
}

TEST(Nav2VoxelGridLogic, BuildGridSpecMaxOneLayerCollapses)
{
  const ScalarBuffer xs = std::vector<float>{0.0F, 1.0F};
  const ScalarBuffer ys = std::vector<float>{0.0F, 1.0F};
  const ScalarBuffer zs = std::vector<float>{0.0F, 5.0F};
  const auto spec = voxelcodec_ros::build_grid_spec(xs, ys, zs, 1.0, 0.0, 1);
  EXPECT_EQ(spec.size_z, 1U);
}

TEST(Nav2VoxelGridLogic, BuildGridSpecAutoZResolutionFitsWithinMaxLayers)
{
  const ScalarBuffer xs = std::vector<float>{0.0F, 1.0F};
  const ScalarBuffer ys = std::vector<float>{0.0F, 1.0F};
  const ScalarBuffer zs = std::vector<float>{0.0F, 10.0F};
  const auto spec = voxelcodec_ros::build_grid_spec(xs, ys, zs, 1.0, 0.0, 8);
  EXPECT_LE(spec.size_z, 8U);
  EXPECT_GT(spec.resolution_z, 0.0);
}

TEST(Nav2VoxelGridLogic, CoordinateToIndexRejectsZeroSize)
{
  EXPECT_THROW(voxelcodec_ros::coordinate_to_index(1.0, 0.0, 1.0, 0), std::runtime_error);
}

TEST(Nav2VoxelGridLogic, CoordinateToIndexClampsBelowOrigin)
{
  EXPECT_EQ(voxelcodec_ros::coordinate_to_index(-5.0, 0.0, 1.0, 10), 0U);
}

TEST(Nav2VoxelGridLogic, CoordinateToIndexClampsAboveMax)
{
  EXPECT_EQ(voxelcodec_ros::coordinate_to_index(1000.0, 0.0, 1.0, 10), 9U);
}

TEST(Nav2VoxelGridLogic, CoordinateToIndexInterpolatesInRange)
{
  EXPECT_EQ(voxelcodec_ros::coordinate_to_index(3.0, 0.0, 1.0, 10), 3U);
}

TEST(Nav2VoxelGridLogic, MakePointFieldSetsAllFields)
{
  const auto field = voxelcodec_ros::make_point_field(
    "intensity", 16, sensor_msgs::msg::PointField::FLOAT32, 1);
  EXPECT_EQ(field.name, "intensity");
  EXPECT_EQ(field.offset, 16U);
  EXPECT_EQ(field.datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(field.count, 1U);
}

TEST(Nav2VoxelGridLogic, BuildVoxelGridMessageMarksOccupiedCells)
{
  // Four points at the corners of a 2x2x1 grid; occupancy is always required
  // by decode_selected's fixed channel list, so mark every point occupied.
  const std::vector<float> xs{0.0F, 1.0F, 0.0F, 1.0F};
  const std::vector<float> ys{0.0F, 0.0F, 1.0F, 1.0F};
  const std::vector<float> zs{0.0F, 0.0F, 0.0F, 0.0F};
  const std::vector<std::uint8_t> occupancy{1, 1, 1, 1};
  const auto archive_bytes = build_point_archive(xs, ys, zs, &occupancy);
  const auto archive = voxelcodec_ros::read_archive(archive_bytes);

  const auto msg = voxelcodec_ros::build_voxel_grid_message(archive, 1.0, 0.0, 16, "map");
  EXPECT_EQ(msg.size_x, 2U);
  EXPECT_EQ(msg.size_y, 2U);
  EXPECT_EQ(msg.size_z, 1U);
  EXPECT_EQ(msg.header.frame_id, "map");

  // VoxelGrid::reset() initializes every cell to UNKNOWN, not FREE -- a raw
  // nonzero packed value doesn't mean "marked" on its own, so every corner
  // must be checked through getVoxel() (over the whole packed array, the way
  // it indexes internally) the same way production code would.
  ASSERT_EQ(msg.data.size(), 4U);
  for (std::size_t y = 0; y < 2; ++y) {
    for (std::size_t x = 0; x < 2; ++x) {
      EXPECT_EQ(
        nav2_voxel_grid::VoxelGrid::getVoxel(
          static_cast<unsigned int>(x), static_cast<unsigned int>(y), 0, 2, 2, 1,
          msg.data.data()),
        nav2_voxel_grid::MARKED)
        << "corner (" << x << "," << y << ") not marked";
    }
  }
}

TEST(Nav2VoxelGridLogic, BuildVoxelGridMessageRespectsOccupancyChannel)
{
  const std::vector<float> xs{0.0F, 1.0F};
  const std::vector<float> ys{0.0F, 0.0F};
  const std::vector<float> zs{0.0F, 0.0F};
  const std::vector<std::uint8_t> occupancy{0, 1};  // only the second point is occupied
  const auto archive_bytes = build_point_archive(xs, ys, zs, &occupancy);
  const auto archive = voxelcodec_ros::read_archive(archive_bytes);

  const auto msg = voxelcodec_ros::build_voxel_grid_message(archive, 1.0, 0.0, 16, "map");
  ASSERT_EQ(msg.data.size(), 2U);
  // Column 0 (the unoccupied point) must stay UNKNOWN; column 1 must be MARKED.
  EXPECT_NE(
    nav2_voxel_grid::VoxelGrid::getVoxel(0, 0, 0, 2, 1, 1, msg.data.data()),
    nav2_voxel_grid::MARKED);
  EXPECT_EQ(
    nav2_voxel_grid::VoxelGrid::getVoxel(1, 0, 0, 2, 1, 1, msg.data.data()),
    nav2_voxel_grid::MARKED);
}

TEST(Nav2VoxelGridLogic, BuildVoxelGridMessageRejectsMismatchedLengths)
{
  Manifest manifest;
  std::vector<EncodedChannel> channels;
  auto add = [&](const std::string & name, int count) {
      ChannelDescriptor desc;
      desc.name = name;
      desc.data_type = voxelcodec_ros::kDataTypeFloat32;
      desc.encoding = voxelcodec_ros::kEncodingRawLE;
      desc.compression = voxelcodec_ros::kCompressionNone;
      desc.element_count = static_cast<std::uint32_t>(count);
      std::vector<std::uint8_t> payload;
      for (int i = 0; i < count; ++i) {push_f32le(payload, static_cast<float>(i));}
      manifest.channels.push_back(desc);
      channels.push_back(EncodedChannel{desc, payload});
    };
  add("x", 2);
  add("y", 2);
  add("z", 1);  // mismatched length -- this is what should trip the check

  // decode_selected demands every requested channel exist (including
  // "occupancy"), so without one it throws from inside decode_selected --
  // before ever reaching build_voxel_grid_message's own x/y/z length check.
  ChannelDescriptor occ_desc;
  occ_desc.name = "occupancy";
  occ_desc.data_type = voxelcodec_ros::kDataTypeUint8;
  occ_desc.encoding = voxelcodec_ros::kEncodingRawLE;
  occ_desc.compression = voxelcodec_ros::kCompressionNone;
  occ_desc.element_count = 2;
  manifest.channels.push_back(occ_desc);
  channels.push_back(EncodedChannel{occ_desc, {1, 1}});

  const auto archive_bytes = voxelcodec_ros::encode_archive(manifest, channels);
  const auto archive = voxelcodec_ros::read_archive(archive_bytes);
  EXPECT_THROW(
    voxelcodec_ros::build_voxel_grid_message(archive, 1.0, 0.0, 16, "map"), std::runtime_error);
}

TEST(Nav2VoxelGridLogic, BuildVoxelGridMessageRejectsMismatchedOccupancyLength)
{
  const std::vector<float> xs{0.0F, 1.0F};
  const std::vector<float> ys{0.0F, 0.0F};
  const std::vector<float> zs{0.0F, 0.0F};
  const std::vector<std::uint8_t> occupancy{1};  // wrong length (1 instead of 2)
  const auto archive_bytes = build_point_archive(xs, ys, zs, &occupancy);
  const auto archive = voxelcodec_ros::read_archive(archive_bytes);
  EXPECT_THROW(
    voxelcodec_ros::build_voxel_grid_message(archive, 1.0, 0.0, 16, "map"), std::runtime_error);
}

TEST(Nav2VoxelGridLogic, BuildPointCloudMessageEmitsOnePointPerMarkedVoxel)
{
  voxelcodec_ros::GridSpec spec;
  spec.size_x = 2;
  spec.size_y = 2;
  spec.size_z = 1;
  spec.resolution_xy = 1.0;
  spec.resolution_z = 1.0;

  nav2_voxel_grid::VoxelGrid grid(spec.size_x, spec.size_y, spec.size_z);
  grid.reset();
  grid.markVoxel(0, 0, 0);
  grid.markVoxel(1, 1, 0);

  const auto * data = grid.getData();
  const std::vector<std::uint32_t> grid_data(
    data, data + static_cast<std::size_t>(spec.size_x) * spec.size_y);

  const auto msg = voxelcodec_ros::build_point_cloud_message(spec, grid_data, "map");
  EXPECT_EQ(msg.width, 2U);
  EXPECT_EQ(msg.data.size(), 2U * msg.point_step);
  EXPECT_EQ(msg.fields.size(), 5U);
}

TEST(Nav2VoxelGridLogic, BuildOccupancyGridMessageMarksColumnsWithAnyOccupiedZ)
{
  voxelcodec_ros::GridSpec spec;
  spec.size_x = 2;
  spec.size_y = 1;
  spec.size_z = 2;
  spec.resolution_xy = 1.0;
  spec.resolution_z = 1.0;

  nav2_voxel_grid::VoxelGrid grid(spec.size_x, spec.size_y, spec.size_z);
  grid.reset();
  grid.markVoxel(0, 0, 1);  // column (0,0) occupied at the top z-layer only

  const auto * data = grid.getData();
  const std::vector<std::uint32_t> grid_data(
    data, data + static_cast<std::size_t>(spec.size_x) * spec.size_y);

  const auto msg = voxelcodec_ros::build_occupancy_grid_message(spec, grid_data, "map");
  ASSERT_EQ(msg.data.size(), 2U);
  EXPECT_EQ(msg.data[0], 100);  // column (0,0) -> occupied
  EXPECT_EQ(msg.data[1], 0);    // column (1,0) -> free
}
