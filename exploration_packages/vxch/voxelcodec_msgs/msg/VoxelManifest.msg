std_msgs/Header header
string stream_id
# format/version/voxel_count/metadata/channels are packed as JSON and
# compressed below instead of native message fields -- see
# manifest_to_msg/manifest_from_msg in ros_messages.cpp. Mirrors
# VoxelChannel.msg's descriptor.compression/payload shape.
string compression
uint64 uncompressed_size
uint8[] payload