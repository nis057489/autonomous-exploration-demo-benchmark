#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/types.hpp"

namespace voxelcodec_ros
{

// Compatibility names for callers of the original codec API. The arithmetic
// and coefficient packing now live in the standalone WaveStream header.
using wavestream::haar_forward_level;
using wavestream::zigzag_varint_encode;
using wavestream::fixed_width_encode;
using wavestream::haar_forward_2d_level;
using wavestream::gather_detail_quadrants;
using wavestream::gather_ll;

// Apply L levels of a separable 2D forward Haar pyramid to a width x height grid,
// then split into L+1 per-band EncodedChannels ordered coarsest (band 0, the
// final LL quadrant) to finest (band L, the detail from the first pass on the
// full-resolution grid). Each detail band k (1..L) is the concatenated
// (HL,LH,HH) quadrants from pass (levels-k) -- see compute_haar_band_layout.
//
// Each EncodedChannel descriptor carries metadata so the decoder can
// reassemble: haar_levels, haar_original_length, haar_grid_width,
// haar_grid_height, haar_band_index, haar_total_bands. The grid-width/height
// keys are what let the decoder distinguish this 2D grid encoding from the
// legacy flat/1D haar-wavelet encoding still used by non-grid channels.
inline std::vector<EncodedChannel> make_haar_bands(
  const std::vector<std::uint32_t> & values,
  std::size_t width,
  std::size_t height,
  int levels,
  const std::string & compression,
  bool use_varint = true)
{
  const auto band_coeffs = wavestream::forward_transform(values, width, height, levels);
  const std::size_t N = values.size();
  const int total_bands = levels + 1;
  std::vector<EncodedChannel> bands;
  bands.reserve(static_cast<std::size_t>(total_bands));

  for (int k = 0; k < total_bands; ++k) {
    const auto & coeffs = band_coeffs[static_cast<std::size_t>(k)];

    // Coefficient energy used as a scheduling heuristic. Integer lifting
    // is not an orthonormal transform, so this is not exact squared error.
    double l2_energy = 0.0;
    for (const std::int64_t coeff : coeffs) {
      l2_energy += static_cast<double>(coeff) * static_cast<double>(coeff);
    }

    std::vector<std::uint8_t> raw_payload =
      use_varint ? zigzag_varint_encode(coeffs) : fixed_width_encode(coeffs);
    std::vector<std::uint8_t> payload = compress_payload(compression, raw_payload);

    ChannelDescriptor desc;
    desc.name = "band_" + std::to_string(k);
    desc.semantic = "occupancy_haar_band";
    desc.data_type = kDataTypeUint32;
    desc.encoding = kEncodingHaarWavelet;
    desc.compression = compression;
    desc.element_count = static_cast<std::uint32_t>(coeffs.size());
    desc.uncompressed_size = static_cast<std::uint64_t>(raw_payload.size());
    desc.compressed_size = static_cast<std::uint64_t>(payload.size());
    desc.metadata[kHaarLevelsKey] = std::to_string(levels);
    desc.metadata[kHaarOriginalLengthKey] = std::to_string(N);
    desc.metadata[kHaarGridWidthKey] = std::to_string(width);
    desc.metadata[kHaarGridHeightKey] = std::to_string(height);
    desc.metadata["haar_band_index"] = std::to_string(k);
    desc.metadata["haar_total_bands"] = std::to_string(total_bands);
    desc.metadata[kHaarVarintKey] = use_varint ? "1" : "0";

    EncodedChannel ec;
    ec.descriptor = std::move(desc);
    ec.payload = std::move(payload);
    ec.l2_energy = l2_energy;
    bands.push_back(std::move(ec));
  }

  return bands;
}

}  // namespace voxelcodec_ros
