#include <gtest/gtest.h>

#include "voxelcodec_ros/codec.hpp"

TEST(VoxelCodecRos, DecodeStructuredChannels)
{
  using voxelcodec_ros::ChannelDescriptor;
  using voxelcodec_ros::EncodedChannel;
  using voxelcodec_ros::Manifest;

  Manifest manifest;
  manifest.voxel_count = 4;
  manifest.channels = {
      ChannelDescriptor{"x", "position.x", "uint32", "delta-varint", "zstd", 4, 0, 0, 0, {}},
      ChannelDescriptor{"color", "color.rgba", "uint32", "palette", "zstd", 4, 0, 0, 0, {{"palette_values", "[4278190335,16711935,65535]"}, {"palette_index_type", "uint8"}}},
      ChannelDescriptor{"occupancy", "occupancy.probability", "float32", "constant", "zstd", 4, 0, 0, 0, {{"constant_value", "1"}}}};

  std::vector<EncodedChannel> channels = {
      {manifest.channels[0], {0x02, 0x02, 0x02, 0xf8, 0x03}},
      {manifest.channels[1], {0x00, 0x00, 0x01, 0x02}},
      {manifest.channels[2], {}}};

  const auto archive_bytes = voxelcodec_ros::encode_archive(manifest, channels);
  const auto archive = voxelcodec_ros::read_archive(archive_bytes);
  const auto decoded = voxelcodec_ros::decode_selected(archive, {"x", "color", "occupancy"});

  EXPECT_EQ(std::get<std::vector<std::uint32_t> >(decoded.at("x").values)[3], 255U);
  EXPECT_EQ(std::get<std::vector<std::uint32_t> >(decoded.at("color").values)[2], 16711935U);
  EXPECT_FLOAT_EQ(std::get<std::vector<float> >(decoded.at("occupancy").values)[1], 1.0F);
}

TEST(VoxelCodecRos, DecodeByteShuffleChannel)
{
  using voxelcodec_ros::ChannelDescriptor;

  const ChannelDescriptor descriptor{
      "detail_0", "node.detail.0", "uint32", "byte-shuffle", "none", 3, 0, 12, 12, {}};

  // Byte-shuffle layout for [0x11223344, 0xAABBCCDD, 0x01020304]
  // lane0: 44 DD 04
  // lane1: 33 CC 03
  // lane2: 22 BB 02
  // lane3: 11 AA 01
  const std::vector<std::uint8_t> payload{
      0x44, 0xDD, 0x04,
      0x33, 0xCC, 0x03,
      0x22, 0xBB, 0x02,
      0x11, 0xAA, 0x01};

  const auto decoded = voxelcodec_ros::decode_channel(descriptor, payload);
  const auto values = std::get<std::vector<std::uint32_t> >(decoded.values);

  ASSERT_EQ(values.size(), 3U);
  EXPECT_EQ(values[0], 0x11223344U);
  EXPECT_EQ(values[1], 0xAABBCCDDU);
  EXPECT_EQ(values[2], 0x01020304U);
}