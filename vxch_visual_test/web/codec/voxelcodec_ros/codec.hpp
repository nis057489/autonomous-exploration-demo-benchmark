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

/// Per-level dimensions of the LL (recursed) quadrant in the 2D Haar pyramid.
/// dims[0] = (width, height); dims[levels] = final coarsest LL dimensions.
struct HaarLevelDims
{
  std::size_t width;
  std::size_t height;
};

std::vector<HaarLevelDims> compute_haar_level_dims(
  std::size_t width, std::size_t height, int levels);

/// Per-band layout of the 2D Haar pyramid, coarsest-first (band 0 = final LL).
/// For band k>=1, (level_w, level_h) is the pre-transform size at the pass
/// this band's detail quadrants (HL,LH,HH) came from, and (new_w, new_h) is
/// that pass's LL size -- together these determine each quadrant's extent.
struct HaarBandLayout
{
  std::size_t element_count;
  std::size_t level_w;
  std::size_t level_h;
  std::size_t new_w;
  std::size_t new_h;
};

std::vector<HaarBandLayout> compute_haar_band_layout(
  std::size_t width, std::size_t height, int levels);

/// Result of a (possibly partial) 2D Haar reconstruction: row-major values of
/// size width*height, where width/height may be smaller than the original
/// grid if fewer than levels+1 bands were supplied (caller upsamples).
struct HaarReconstruction
{
  std::vector<std::uint32_t> values;
  std::size_t width;
  std::size_t height;
};

/// Reconstruct a 2D grid from per-band coefficient arrays, as received from
/// separate band messages (no pre-assembly into a flat buffer required).
/// @param band_coeffs band_coeffs[k] = zigzag-varint-decoded coefficients for
///                     band k, for k in [0, bands_received). Bands beyond
///                     bands_received are ignored.
/// @param width, height Full grid dimensions used during encoding.
/// @param levels        Number of Haar levels used during encoding.
/// @param bands_received Number of leading bands available (1=coarsest only,
///                        levels+1=full resolution).
HaarReconstruction reconstruct_haar_from_bands(
  const std::vector<std::vector<std::int64_t> > & band_coeffs,
  std::size_t width,
  std::size_t height,
  int levels,
  int bands_received);

std::unordered_map<std::string, DecodedChannel> decode_selected(
  const Archive & archive,
  const std::vector<std::string> & channel_names);

}  // namespace voxelcodec_ros