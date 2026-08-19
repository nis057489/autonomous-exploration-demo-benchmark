#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "voxelcodec_ros/ros_messages.hpp"

namespace
{

using voxelcodec_ros::ChannelDescriptor;
using voxelcodec_ros::Manifest;

ChannelDescriptor make_descriptor()
{
  ChannelDescriptor descriptor;
  descriptor.name = "occupancy";
  descriptor.semantic = "occupancy.probability";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint32;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  descriptor.compression = voxelcodec_ros::kCompressionZstd;
  descriptor.element_count = 42;
  descriptor.payload_offset = 7;
  descriptor.compressed_size = 128;
  descriptor.uncompressed_size = 256;
  descriptor.metadata = {{"haar_levels", "3"}, {"haar_grid_width", "16"}};
  return descriptor;
}

}  // namespace

TEST(RosMessages, MetadataRoundTrips)
{
  const voxelcodec_ros::Metadata metadata = {{"a", "1"}, {"b", "2"}};
  const auto msg = voxelcodec_ros::metadata_to_msg(metadata);
  EXPECT_EQ(msg.size(), 2U);
  const auto round_tripped = voxelcodec_ros::metadata_from_msg(msg);
  EXPECT_EQ(round_tripped, metadata);
}

TEST(RosMessages, MetadataRoundTripsEmpty)
{
  const voxelcodec_ros::Metadata metadata;
  const auto msg = voxelcodec_ros::metadata_to_msg(metadata);
  EXPECT_TRUE(msg.empty());
  EXPECT_TRUE(voxelcodec_ros::metadata_from_msg(msg).empty());
}

TEST(RosMessages, DescriptorRoundTrips)
{
  const auto descriptor = make_descriptor();
  const auto msg = voxelcodec_ros::descriptor_to_msg(descriptor);
  const auto round_tripped = voxelcodec_ros::descriptor_from_msg(msg);

  EXPECT_EQ(round_tripped.name, descriptor.name);
  EXPECT_EQ(round_tripped.semantic, descriptor.semantic);
  EXPECT_EQ(round_tripped.data_type, descriptor.data_type);
  EXPECT_EQ(round_tripped.encoding, descriptor.encoding);
  EXPECT_EQ(round_tripped.compression, descriptor.compression);
  EXPECT_EQ(round_tripped.element_count, descriptor.element_count);
  EXPECT_EQ(round_tripped.payload_offset, descriptor.payload_offset);
  EXPECT_EQ(round_tripped.compressed_size, descriptor.compressed_size);
  EXPECT_EQ(round_tripped.uncompressed_size, descriptor.uncompressed_size);
  EXPECT_EQ(round_tripped.metadata, descriptor.metadata);
}

TEST(RosMessages, ManifestRoundTrips)
{
  Manifest manifest;
  manifest.format = voxelcodec_ros::kFormat;
  manifest.version = voxelcodec_ros::kArchiveVersion;
  manifest.voxel_count = 100;
  manifest.metadata = {{"session_id", "abc"}};
  manifest.channels = {make_descriptor(), make_descriptor()};
  manifest.channels[1].name = "color";

  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 5;
  header.stamp.nanosec = 500;

  const auto msg = voxelcodec_ros::manifest_to_msg(header, "stream-1", manifest);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.stream_id, "stream-1");
  // format/version/voxel_count/metadata/channels no longer live on the
  // message directly -- they're packed as compressed JSON in msg.payload
  // (see manifest_to_msg). Assert that instead of a field that no longer
  // exists on the wire; the round-trip checks below cover the content.
  EXPECT_EQ(msg.compression, voxelcodec_ros::kCompressionZstd);
  EXPECT_FALSE(msg.payload.empty());

  const auto round_tripped = voxelcodec_ros::manifest_from_msg(msg);
  EXPECT_EQ(round_tripped.format, manifest.format);
  EXPECT_EQ(round_tripped.version, manifest.version);
  EXPECT_EQ(round_tripped.voxel_count, manifest.voxel_count);
  EXPECT_EQ(round_tripped.metadata, manifest.metadata);
  ASSERT_EQ(round_tripped.channels.size(), 2U);
  EXPECT_EQ(round_tripped.channels[0].name, "occupancy");
  EXPECT_EQ(round_tripped.channels[1].name, "color");
}

TEST(RosMessages, ManifestRoundTripsNoChannels)
{
  const Manifest manifest;
  std_msgs::msg::Header header;
  const auto msg = voxelcodec_ros::manifest_to_msg(header, "s", manifest);
  EXPECT_TRUE(voxelcodec_ros::manifest_from_msg(msg).channels.empty());
}

TEST(RosMessages, ChannelToMsgCarriesPayloadAndDescriptor)
{
  const auto descriptor = make_descriptor();
  const std::vector<std::uint8_t> payload{1, 2, 3, 4, 5};
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  const auto msg = voxelcodec_ros::channel_to_msg(header, "s1", descriptor, payload);
  EXPECT_EQ(msg.header.frame_id, "odom");
  EXPECT_EQ(msg.stream_id, "s1");
  EXPECT_EQ(msg.payload, payload);
  EXPECT_EQ(msg.descriptor.name, descriptor.name);
}

TEST(RosMessages, ChannelTopicHandlesBaseTopicVariants)
{
  EXPECT_EQ(voxelcodec_ros::channel_topic("", "band_0"), "/channel/band_0");
  EXPECT_EQ(voxelcodec_ros::channel_topic("/", "band_0"), "/channel/band_0");
  EXPECT_EQ(voxelcodec_ros::channel_topic("/vxch/map/", "band_0"), "/vxch/map/channel/band_0");
  EXPECT_EQ(voxelcodec_ros::channel_topic("/vxch/map", "band_0"), "/vxch/map/channel/band_0");
}
