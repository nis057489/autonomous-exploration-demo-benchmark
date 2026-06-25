#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{

void validate_descriptor(const ChannelDescriptor & descriptor);
void validate_manifest(const Manifest & manifest);

std::vector<std::uint8_t> compress_payload(
  const std::string & compression,
  const std::vector<std::uint8_t> & raw_payload);

std::vector<std::uint8_t> decompress_payload(
  const ChannelDescriptor & descriptor,
  const std::vector<std::uint8_t> & compressed_payload);

std::vector<std::uint8_t> encode_archive(
  const Manifest & manifest,
  const std::vector<EncodedChannel> & channels);

Archive read_archive(const std::vector<std::uint8_t> & archive_bytes);

DecodedChannel decode_channel(
  const ChannelDescriptor & descriptor,
  const std::vector<std::uint8_t> & compressed_payload);

/// Decode a haar-wavelet channel with progressive band reconstruction.
/// @param max_bands Number of bands to reconstruct (1 = coarsest, 0 = full).
/// @return The decoded channel with partial reconstruction.
DecodedChannel decode_haar_progressive(
  const ChannelDescriptor & descriptor,
  const std::vector<std::uint8_t> & compressed_payload,
  int max_bands);

/// Query the maximum number of haar bands for a channel descriptor.
/// Returns 0 if the channel is not haar-wavelet encoded.
int haar_max_bands(const ChannelDescriptor & descriptor);

/// Reconstruct values from a pre-assembled int64 coefficient array.
/// Used by the VXCH decoder when bands arrive as separate messages and must be
/// assembled before calling the wavelet inverse.
/// @param coeffs   Assembled coefficient prefix (bands 0..max_bands-1 concatenated).
///                 Length must be >= band_boundaries[max_bands] where
///                 band_boundaries[k+1] = smooth_lens[levels-k].
/// @param original_len Full grid length (N) needed to compute smooth_lens.
/// @param levels   Number of Haar levels used during encoding.
/// @param max_bands Bands to reconstruct (1=coarsest only, levels+1=full).
/// @return Decoded values, length = band_boundaries[max_bands].
///         Caller is responsible for upsampling to original_len if max_bands < levels+1.
std::vector<std::uint32_t> reconstruct_haar_from_coeffs(
  const std::vector<std::int64_t> & coeffs,
  std::size_t original_len,
  int levels,
  int max_bands);

std::unordered_map<std::string, DecodedChannel> decode_selected(
  const Archive & archive,
  const std::vector<std::string> & channel_names);

}  // namespace voxelcodec_ros