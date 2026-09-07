#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

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
  ASSERT_EQ(msg.channels.size(), 2U);

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
  EXPECT_TRUE(msg.channels.empty());
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


// --- split_tiles_into_batches ------------------------------------------------
// Bands travel BEST_EFFORT with no retransmission, so a batch large enough to
// IP-fragment turns one lost fragment into the loss of every tile it carried.
// These pin the invariants that keep one batch = one datagram = one loss unit.

namespace
{
voxelcodec_ros::TileBatchSpec make_spec()
{
  voxelcodec_ros::TileBatchSpec spec;
  spec.header.frame_id = "robot1/map";
  spec.stream_id = "map_stream";
  spec.band_index = 1;
  spec.haar_levels = 2;
  spec.haar_total_bands = 3;
  spec.varint_encoding = true;
  spec.compression = "zstd";
  spec.tile_size_cells = 60;
  return spec;
}

voxelcodec_msgs::msg::VoxelTilePayload make_tile(int row, int col, std::size_t payload_bytes)
{
  voxelcodec_msgs::msg::VoxelTilePayload tile;
  tile.tile_row = row;
  tile.tile_col = col;
  tile.tile_width = 60;
  tile.tile_height = 60;
  tile.element_count = static_cast<std::uint32_t>(payload_bytes);
  tile.payload.assign(payload_bytes, static_cast<std::uint8_t>(row & 0xFF));
  return tile;
}
}  // namespace

TEST(SplitTilesIntoBatches, EmptyInputProducesNoBatches)
{
  EXPECT_TRUE(voxelcodec_ros::split_tiles_into_batches(make_spec(), {}, 1300).empty());
}

TEST(SplitTilesIntoBatches, EverythingUnderTheCapStaysOneBatch)
{
  std::vector<voxelcodec_msgs::msg::VoxelTilePayload> tiles;
  for (int i = 0; i < 5; ++i) {tiles.push_back(make_tile(i, 0, 20));}

  const auto out = voxelcodec_ros::split_tiles_into_batches(make_spec(), tiles, 1300);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].tiles.size(), 5U);
}

TEST(SplitTilesIntoBatches, NoBatchExceedsTheCapAndEveryTileSurvivesInOrder)
{
  const auto spec = make_spec();
  std::vector<voxelcodec_msgs::msg::VoxelTilePayload> tiles;
  for (int i = 0; i < 60; ++i) {tiles.push_back(make_tile(i, i * 2, 40 + (i % 17)));}

  const std::size_t cap = 400;
  const auto out = voxelcodec_ros::split_tiles_into_batches(spec, tiles, cap);
  ASSERT_GT(out.size(), 1U) << "a cap this small must have forced a split";

  std::vector<std::pair<int, int>> seen;
  for (const auto & batch : out) {
    EXPECT_FALSE(batch.tiles.empty()) << "no empty batches";
    std::size_t est = voxelcodec_ros::estimated_batch_base_bytes(spec);
    for (const auto & tile : batch.tiles) {
      est += voxelcodec_ros::estimated_tile_bytes(tile);
      seen.emplace_back(tile.tile_row, tile.tile_col);
    }
    EXPECT_LE(est, cap) << "a batch outgrew the cap";
    // Stream constants must be replicated onto every batch, or the tiles in
    // the second and later ones arrive undecodable.
    EXPECT_EQ(batch.band_index, spec.band_index);
    EXPECT_EQ(batch.tile_size_cells, spec.tile_size_cells);
    EXPECT_EQ(batch.compression, spec.compression);
    EXPECT_EQ(batch.haar_levels, spec.haar_levels);
    EXPECT_EQ(batch.header.frame_id, spec.header.frame_id);
    EXPECT_EQ(batch.stream_id, spec.stream_id);
  }

  ASSERT_EQ(seen.size(), tiles.size()) << "tiles lost or duplicated by the split";
  for (std::size_t i = 0; i < tiles.size(); ++i) {
    EXPECT_EQ(seen[i].first, tiles[i].tile_row);
    EXPECT_EQ(seen[i].second, tiles[i].tile_col);
  }
}

TEST(SplitTilesIntoBatches, ZeroCapMeansUnlimited)
{
  std::vector<voxelcodec_msgs::msg::VoxelTilePayload> tiles;
  for (int i = 0; i < 200; ++i) {tiles.push_back(make_tile(i, 0, 100));}

  const auto out = voxelcodec_ros::split_tiles_into_batches(make_spec(), tiles, 0);
  ASSERT_EQ(out.size(), 1U);
  EXPECT_EQ(out[0].tiles.size(), 200U);
}

TEST(SplitTilesIntoBatches, ATileBiggerThanTheCapGoesOutAloneRatherThanBlockingOrEmptying)
{
  const auto spec = make_spec();
  std::vector<voxelcodec_msgs::msg::VoxelTilePayload> tiles{
    make_tile(0, 0, 10),
    make_tile(1, 0, 5000),   // exceeds any sane cap on its own; cannot be split
    make_tile(2, 0, 10),
  };

  const auto out = voxelcodec_ros::split_tiles_into_batches(spec, tiles, 1300);

  std::size_t total = 0;
  for (const auto & batch : out) {
    EXPECT_FALSE(batch.tiles.empty()) << "an over-cap tile must not emit an empty batch";
    total += batch.tiles.size();
  }
  EXPECT_EQ(total, 3U) << "the oversized tile must still be sent, not dropped";

  bool alone = false;
  for (const auto & batch : out) {
    if (batch.tiles.size() == 1U && batch.tiles[0].payload.size() == 5000U) {alone = true;}
  }
  EXPECT_TRUE(alone) << "the oversized tile should occupy a batch by itself";
}

TEST(SplitTilesIntoBatches, EstimatorIsNeverOptimisticAboutRealSerializedSize)
{
  // The cap only means anything if the estimate is never optimistic -- an
  // under-estimate is exactly what would push a real datagram past the MTU.
  const auto spec = make_spec();
  rclcpp::Serialization<voxelcodec_msgs::msg::VoxelTileBatch> ser;

  for (std::size_t payload = 0; payload <= 300; payload += 7) {
    std::vector<voxelcodec_msgs::msg::VoxelTilePayload> tiles{make_tile(3, 4, payload)};
    const auto batches = voxelcodec_ros::split_tiles_into_batches(spec, tiles, 0);
    ASSERT_EQ(batches.size(), 1U);

    std::size_t estimated = voxelcodec_ros::estimated_batch_base_bytes(spec);
    for (const auto & tile : batches[0].tiles) {
      estimated += voxelcodec_ros::estimated_tile_bytes(tile);
    }

    rclcpp::SerializedMessage serialized;
    ser.serialize_message(&batches[0], &serialized);
    // +4 for CDR's encapsulation header, which the estimate deliberately omits
    // (it is per-datagram, not per-tile, and the cap leaves MTU headroom).
    EXPECT_GE(estimated + 4, serialized.size())
      << "estimator UNDER-counted at payload=" << payload;
  }
}
