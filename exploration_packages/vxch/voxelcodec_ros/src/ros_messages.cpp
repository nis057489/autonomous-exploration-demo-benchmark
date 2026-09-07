#include "voxelcodec_ros/ros_messages.hpp"

#include <limits>
#include <utility>

namespace voxelcodec_ros
{

std::vector<voxelcodec_msgs::msg::MetadataEntry> metadata_to_msg(const Metadata & metadata)
{
  std::vector<voxelcodec_msgs::msg::MetadataEntry> result;
  result.reserve(metadata.size());
  for (const auto & item : metadata) {
    voxelcodec_msgs::msg::MetadataEntry entry;
    entry.key = item.first;
    entry.value = item.second;
    result.push_back(std::move(entry));
  }
  return result;
}

Metadata metadata_from_msg(const std::vector<voxelcodec_msgs::msg::MetadataEntry> & metadata)
{
  Metadata result;
  for (const auto & entry : metadata) {
    result[entry.key] = entry.value;
  }
  return result;
}

voxelcodec_msgs::msg::VoxelChannelDescriptor descriptor_to_msg(const ChannelDescriptor & descriptor)
{
  voxelcodec_msgs::msg::VoxelChannelDescriptor message;
  message.name = descriptor.name;
  message.semantic = descriptor.semantic;
  message.data_type = descriptor.data_type;
  message.encoding = descriptor.encoding;
  message.compression = descriptor.compression;
  message.element_count = descriptor.element_count;
  message.payload_offset = descriptor.payload_offset;
  message.compressed_size = descriptor.compressed_size;
  message.uncompressed_size = descriptor.uncompressed_size;
  message.metadata = metadata_to_msg(descriptor.metadata);
  return message;
}

ChannelDescriptor descriptor_from_msg(const voxelcodec_msgs::msg::VoxelChannelDescriptor & message)
{
  ChannelDescriptor descriptor;
  descriptor.name = message.name;
  descriptor.semantic = message.semantic;
  descriptor.data_type = message.data_type;
  descriptor.encoding = message.encoding;
  descriptor.compression = message.compression;
  descriptor.element_count = message.element_count;
  descriptor.payload_offset = message.payload_offset;
  descriptor.compressed_size = message.compressed_size;
  descriptor.uncompressed_size = message.uncompressed_size;
  descriptor.metadata = metadata_from_msg(message.metadata);
  return descriptor;
}

voxelcodec_msgs::msg::VoxelManifest manifest_to_msg(
  const std_msgs::msg::Header & header,
  const std::string & stream_id,
  const Manifest & manifest)
{
  voxelcodec_msgs::msg::VoxelManifest message;
  message.header = header;
  message.stream_id = stream_id;
  message.format = manifest.format;
  message.version = manifest.version;
  message.voxel_count = manifest.voxel_count;
  message.metadata = metadata_to_msg(manifest.metadata);
  message.channels.reserve(manifest.channels.size());
  for (const auto & descriptor : manifest.channels) {
    message.channels.push_back(descriptor_to_msg(descriptor));
  }
  return message;
}

Manifest manifest_from_msg(const voxelcodec_msgs::msg::VoxelManifest & message)
{
  Manifest manifest;
  manifest.format = message.format;
  manifest.version = message.version;
  manifest.voxel_count = message.voxel_count;
  manifest.metadata = metadata_from_msg(message.metadata);
  manifest.channels.reserve(message.channels.size());
  for (const auto & descriptor : message.channels) {
    manifest.channels.push_back(descriptor_from_msg(descriptor));
  }
  return manifest;
}

voxelcodec_msgs::msg::VoxelChannel channel_to_msg(
  const std_msgs::msg::Header & header,
  const std::string & stream_id,
  const ChannelDescriptor & descriptor,
  const std::vector<std::uint8_t> & payload)
{
  voxelcodec_msgs::msg::VoxelChannel message;
  message.header = header;
  message.stream_id = stream_id;
  message.descriptor = descriptor_to_msg(descriptor);
  message.payload = payload;
  return message;
}

namespace
{
// Reads an int out of a descriptor's ASCII metadata map. The encoder side
// still receives tile geometry that way from TileScheduler; this is where it
// stops being strings and becomes typed wire fields.
int meta_int(const Metadata & metadata, const std::string & key, int fallback)
{
  auto it = metadata.find(key);
  if (it == metadata.end()) {
    return fallback;
  }
  try {
    return std::stoi(it->second);
  } catch (const std::exception &) {
    return fallback;
  }
}
}  // namespace

voxelcodec_msgs::msg::VoxelTilePayload tile_payload_to_msg(
  int tile_row, int tile_col,
  const ChannelDescriptor & descriptor,
  std::vector<std::uint8_t> payload)
{
  voxelcodec_msgs::msg::VoxelTilePayload tile;
  tile.tile_row = tile_row;
  tile.tile_col = tile_col;
  tile.tile_width = meta_int(descriptor.metadata, "tile_width", 0);
  tile.tile_height = meta_int(descriptor.metadata, "tile_height", 0);
  tile.tile_offset_row = meta_int(descriptor.metadata, "tile_offset_row", 0);
  tile.tile_offset_col = meta_int(descriptor.metadata, "tile_offset_col", 0);
  tile.element_count = descriptor.element_count;
  tile.uncompressed_size = static_cast<std::uint32_t>(descriptor.uncompressed_size);
  tile.payload = std::move(payload);
  return tile;
}

ChannelDescriptor tile_payload_to_descriptor(
  const voxelcodec_msgs::msg::VoxelTileBatch & batch,
  const voxelcodec_msgs::msg::VoxelTilePayload & tile,
  int band_index)
{
  ChannelDescriptor descriptor;
  descriptor.name = "band_" + std::to_string(band_index);
  descriptor.compression = batch.compression;
  descriptor.element_count = tile.element_count;
  // decompress_payload() sizes its output buffer from this; without it every
  // zstd decode fails with "Destination buffer is too small".
  descriptor.uncompressed_size = tile.uncompressed_size;
  descriptor.compressed_size = tile.payload.size();
  // Stream-level constants come off the batch, per-tile geometry off the tile.
  descriptor.metadata[kHaarVarintKey] = batch.varint_encoding ? "1" : "0";
  descriptor.metadata["tile_size_cells"] = std::to_string(batch.tile_size_cells);
  descriptor.metadata["tile_row"] = std::to_string(tile.tile_row);
  descriptor.metadata["tile_col"] = std::to_string(tile.tile_col);
  descriptor.metadata["tile_width"] = std::to_string(tile.tile_width);
  descriptor.metadata["tile_height"] = std::to_string(tile.tile_height);
  descriptor.metadata["tile_offset_row"] = std::to_string(tile.tile_offset_row);
  descriptor.metadata["tile_offset_col"] = std::to_string(tile.tile_offset_col);
  return descriptor;
}

std::size_t estimated_tile_bytes(const voxelcodec_msgs::msg::VoxelTilePayload & tile)
{
  return 32 + 4 + tile.payload.size() + 3;
}

std::size_t estimated_batch_base_bytes(const TileBatchSpec & spec)
{
  return 8 + (4 + spec.header.frame_id.size() + 3) +
         (4 + spec.stream_id.size() + 3) + 4 +
         (4 + spec.compression.size() + 3) + 4 + 4;
}

std::vector<voxelcodec_msgs::msg::VoxelTileBatch> split_tiles_into_batches(
  const TileBatchSpec & spec,
  std::vector<voxelcodec_msgs::msg::VoxelTilePayload> tiles,
  std::size_t max_batch_bytes)
{
  std::vector<voxelcodec_msgs::msg::VoxelTileBatch> out;
  if (tiles.empty()) {
    return out;
  }

  const auto make_batch = [&spec]() {
      voxelcodec_msgs::msg::VoxelTileBatch batch;
      batch.header = spec.header;
      batch.stream_id = spec.stream_id;
      batch.band_index = static_cast<std::uint8_t>(spec.band_index);
      batch.haar_levels = static_cast<std::uint8_t>(spec.haar_levels);
      batch.haar_total_bands = static_cast<std::uint8_t>(spec.haar_total_bands);
      batch.varint_encoding = spec.varint_encoding;
      batch.compression = spec.compression;
      batch.tile_size_cells = spec.tile_size_cells;
      return batch;
    };

  const std::size_t cap =
    (max_batch_bytes == 0) ? std::numeric_limits<std::size_t>::max() : max_batch_bytes;
  const std::size_t base = estimated_batch_base_bytes(spec);

  out.push_back(make_batch());
  std::size_t running = base;
  for (auto & tile : tiles) {
    const std::size_t tile_bytes = estimated_tile_bytes(tile);
    // Close the current batch only if it already holds something -- otherwise a
    // single over-cap tile would emit an empty batch ahead of itself.
    if (!out.back().tiles.empty() && running + tile_bytes > cap) {
      out.push_back(make_batch());
      running = base;
    }
    running += tile_bytes;
    out.back().tiles.push_back(std::move(tile));
  }
  return out;
}

std::string channel_topic(const std::string & base_topic, const std::string & channel_name)
{
  if (base_topic.empty() || base_topic == "/") {
    return "/channel/" + channel_name;
  }
  if (base_topic.back() == '/') {
    return base_topic + "channel/" + channel_name;
  }
  return base_topic + "/channel/" + channel_name;
}

}  // namespace voxelcodec_ros