#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{

// Single forward Haar lifting step over coeffs[0..smooth_len).
// After: coeffs[0..new_smooth) = smooth values
//        coeffs[new_smooth..new_smooth + floor(smooth_len/2)) = detail values
// where new_smooth = (smooth_len + 1) / 2.
// Mirrors haar_inverse_level in codec.cpp (lifting scheme: d = b-a, s = a+(d>>1)).
inline void haar_forward_level(std::vector<std::int64_t> & coeffs, std::size_t smooth_len)
{
  if (smooth_len < 2) {
    return;
  }
  const std::size_t num_pairs = smooth_len / 2;
  const std::size_t new_smooth = (smooth_len + 1) / 2;

  std::vector<std::int64_t> smooth(new_smooth);
  std::vector<std::int64_t> detail(num_pairs);

  for (std::size_t i = 0; i < num_pairs; ++i) {
    const std::int64_t a = coeffs[2 * i];
    const std::int64_t b = coeffs[2 * i + 1];
    detail[i] = b - a;
    smooth[i] = a + (detail[i] >> 1);
  }
  if (smooth_len % 2 == 1) {
    smooth[new_smooth - 1] = coeffs[smooth_len - 1];
  }

  for (std::size_t i = 0; i < new_smooth; ++i) {
    coeffs[i] = smooth[i];
  }
  for (std::size_t i = 0; i < num_pairs; ++i) {
    coeffs[new_smooth + i] = detail[i];
  }
}

// Zigzag-varint encode a coefficient array.
// Mirrors zig_zag_decode + read_uvarint in codec.cpp.
inline std::vector<std::uint8_t> zigzag_varint_encode(const std::vector<std::int64_t> & coeffs)
{
  std::vector<std::uint8_t> out;
  out.reserve(coeffs.size() * 2);
  for (const std::int64_t v : coeffs) {
    // Zigzag encode: negative → odd, non-negative → even
    const std::uint64_t zz = (v >= 0)
      ? static_cast<std::uint64_t>(v) * 2
      : static_cast<std::uint64_t>(-(v + 1)) * 2 + 1;
    // Varint encode
    std::uint64_t u = zz;
    while (u >= 0x80U) {
      out.push_back(static_cast<std::uint8_t>((u & 0x7FU) | 0x80U));
      u >>= 7U;
    }
    out.push_back(static_cast<std::uint8_t>(u));
  }
  return out;
}

// Apply L levels of forward Haar to values, then split the coefficient array into L+1
// per-band EncodedChannels ordered from coarsest (band 0) to finest (band L).
//
// Band k spans coefficient positions [band_boundaries[k], band_boundaries[k+1]) where:
//   band_boundaries[0] = 0
//   band_boundaries[k+1] = smooth_lens[levels - k]   (matching decode_haar_progressive)
//
// Each EncodedChannel descriptor carries metadata so the decoder can reassemble:
//   haar_levels, haar_original_length, haar_band_index, haar_total_bands,
//   haar_cumulative_elements (= band_boundaries[k+1], i.e. how many coefficients are
//   needed from band 0 through k to call decode_haar_progressive with max_bands=k+1).
inline std::vector<EncodedChannel> make_haar_bands(
  const std::vector<std::uint32_t> & values,
  int levels,
  const std::string & compression)
{
  if (levels < 1) {
    throw std::runtime_error("haar levels must be >= 1");
  }

  const std::size_t N = values.size();
  if (N == 0) {
    throw std::runtime_error("make_haar_bands: empty values");
  }

  // Convert to int64 working array
  std::vector<std::int64_t> coeffs(values.begin(), values.end());

  // Smooth lengths per level: smooth_lens[0]=N, smooth_lens[k]=(smooth_lens[k-1]+1)/2
  std::vector<std::size_t> smooth_lens(static_cast<std::size_t>(levels + 1));
  smooth_lens[0] = N;
  for (int i = 1; i <= levels; ++i) {
    smooth_lens[static_cast<std::size_t>(i)] =
      (smooth_lens[static_cast<std::size_t>(i - 1)] + 1) / 2;
  }

  // Apply forward transform level by level
  for (int i = 0; i < levels; ++i) {
    haar_forward_level(coeffs, smooth_lens[static_cast<std::size_t>(i)]);
  }

  // Band boundaries matching decode_haar_progressive:
  // band_boundaries[k+1] = smooth_lens[levels - k]
  std::vector<std::size_t> bb(static_cast<std::size_t>(levels + 2), 0);
  for (int i = 0; i <= levels; ++i) {
    bb[static_cast<std::size_t>(i + 1)] = smooth_lens[static_cast<std::size_t>(levels - i)];
  }

  const int total_bands = levels + 1;
  std::vector<EncodedChannel> bands;
  bands.reserve(static_cast<std::size_t>(total_bands));

  for (int k = 0; k < total_bands; ++k) {
    const std::size_t band_start = bb[static_cast<std::size_t>(k)];
    const std::size_t band_end = bb[static_cast<std::size_t>(k + 1)];
    const std::size_t band_size = band_end - band_start;

    std::vector<std::int64_t> band_coeffs(
      coeffs.begin() + static_cast<std::ptrdiff_t>(band_start),
      coeffs.begin() + static_cast<std::ptrdiff_t>(band_end));

    std::vector<std::uint8_t> raw_payload = zigzag_varint_encode(band_coeffs);
    std::vector<std::uint8_t> payload = compress_payload(compression, raw_payload);

    ChannelDescriptor desc;
    desc.name = "band_" + std::to_string(k);
    desc.semantic = "occupancy_haar_band";
    desc.data_type = kDataTypeUint32;
    desc.encoding = kEncodingHaarWavelet;
    desc.compression = compression;
    desc.element_count = static_cast<std::uint32_t>(band_size);
    desc.uncompressed_size = static_cast<std::uint64_t>(raw_payload.size());
    desc.compressed_size = static_cast<std::uint64_t>(payload.size());
    desc.metadata[kHaarLevelsKey] = std::to_string(levels);
    desc.metadata[kHaarOriginalLengthKey] = std::to_string(N);
    desc.metadata["haar_band_index"] = std::to_string(k);
    desc.metadata["haar_total_bands"] = std::to_string(total_bands);
    desc.metadata["haar_cumulative_elements"] = std::to_string(band_end);

    EncodedChannel ec;
    ec.descriptor = std::move(desc);
    ec.payload = std::move(payload);
    bands.push_back(std::move(ec));
  }

  return bands;
}

}  // namespace voxelcodec_ros
