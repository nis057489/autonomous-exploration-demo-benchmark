#include "voxelcodec_ros/ros_messages.hpp"

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