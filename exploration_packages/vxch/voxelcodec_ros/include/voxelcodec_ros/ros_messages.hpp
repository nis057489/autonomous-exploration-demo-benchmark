#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "std_msgs/msg/header.hpp"
#include "voxelcodec_msgs/msg/metadata_entry.hpp"
#include "voxelcodec_msgs/msg/voxel_channel.hpp"
#include "voxelcodec_msgs/msg/voxel_channel_descriptor.hpp"
#include "voxelcodec_msgs/msg/voxel_manifest.hpp"
#include "voxelcodec_msgs/msg/voxel_tile_batch.hpp"
#include "voxelcodec_msgs/msg/voxel_tile_payload.hpp"
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{

std::vector<voxelcodec_msgs::msg::MetadataEntry> metadata_to_msg(const Metadata & metadata);
Metadata metadata_from_msg(const std::vector<voxelcodec_msgs::msg::MetadataEntry> & metadata);

voxelcodec_msgs::msg::VoxelChannelDescriptor descriptor_to_msg(const ChannelDescriptor & descriptor);
ChannelDescriptor descriptor_from_msg(const voxelcodec_msgs::msg::VoxelChannelDescriptor & message);

voxelcodec_msgs::msg::VoxelManifest manifest_to_msg(
  const std_msgs::msg::Header & header,
  const std::string & stream_id,
  const Manifest & manifest);

Manifest manifest_from_msg(const voxelcodec_msgs::msg::VoxelManifest & message);

voxelcodec_msgs::msg::VoxelChannel channel_to_msg(
  const std_msgs::msg::Header & header,
  const std::string & stream_id,
  const ChannelDescriptor & descriptor,
  const std::vector<std::uint8_t> & payload);

std::string channel_topic(const std::string & base_topic, const std::string & channel_name);

// --- VoxelTileBatch glue -----------------------------------------------------
// The encoder packs each scheduled tile into typed VoxelTilePayload fields and
// the decoder unpacks them back into the ChannelDescriptor that
// TileReconstructor consumes. These two are exact inverses and live here
// together so that stays true -- a mismatch between them silently misplaces or
// fails to decode every tile, which no existing test would catch.

voxelcodec_msgs::msg::VoxelTilePayload tile_payload_to_msg(
  int tile_row, int tile_col,
  const ChannelDescriptor & descriptor,
  std::vector<std::uint8_t> payload);

ChannelDescriptor tile_payload_to_descriptor(
  const voxelcodec_msgs::msg::VoxelTileBatch & batch,
  const voxelcodec_msgs::msg::VoxelTilePayload & tile,
  int band_index);

// The stream-level constants shared by every tile in a batch -- everything a
// VoxelTileBatch carries except the tiles themselves.
struct TileBatchSpec
{
  std_msgs::msg::Header header;
  std::string stream_id;
  int band_index{0};
  int haar_levels{0};
  int haar_total_bands{0};
  bool varint_encoding{true};
  std::string compression;
  int tile_size_cells{0};
};

// Estimated CDR wire cost of one VoxelTilePayload entry: 6 int32 geometry
// fields + uint32 element_count + uint32 uncompressed_size (32 B), the
// payload's 4-byte length prefix,
// the bytes themselves, and up to 3 bytes of padding realigning the next
// entry. Deliberately an over-estimate -- overshooting splits a batch a little
// early, undershooting would push a datagram past the MTU, which is exactly
// what the cap exists to prevent.
std::size_t estimated_tile_bytes(const voxelcodec_msgs::msg::VoxelTilePayload & tile);

// Estimated fixed cost of a batch carrying no tiles: header stamp + frame_id,
// stream_id, the small codec constants, tile_size_cells, and the tiles array
// length prefix, each string carrying a 4-byte length and up to 3 of padding.
std::size_t estimated_batch_base_bytes(const TileBatchSpec & spec);

// Packs tiles into as few VoxelTileBatch messages as fit within
// max_batch_bytes of estimated serialized size, preserving order.
//
// Bands travel BEST_EFFORT with no retransmission, so a batch large enough to
// IP-fragment turns one lost fragment into the loss of every tile it carried.
// Keeping each batch inside the path MTU makes one batch one datagram, i.e.
// one loss unit. Splitting costs no bytes -- the same tiles go out in the same
// send tick, only divided differently.
//
// max_batch_bytes == 0 means unlimited (a single batch). A tile whose own
// estimated size exceeds the cap is emitted alone in an oversized batch: one
// tile's coefficients are indivisible here, so it cannot be split, but it also
// does not drag other tiles over the limit with it.
std::vector<voxelcodec_msgs::msg::VoxelTileBatch> split_tiles_into_batches(
  const TileBatchSpec & spec,
  std::vector<voxelcodec_msgs::msg::VoxelTilePayload> tiles,
  std::size_t max_batch_bytes);

}  // namespace voxelcodec_ros