#pragma once

#include <string>
#include <vector>

#include "std_msgs/msg/header.hpp"
#include "voxelcodec_msgs/msg/metadata_entry.hpp"
#include "voxelcodec_msgs/msg/voxel_channel.hpp"
#include "voxelcodec_msgs/msg/voxel_channel_descriptor.hpp"
#include "voxelcodec_msgs/msg/voxel_manifest.hpp"
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

}  // namespace voxelcodec_ros