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

// Fixed-width (int32 LE) coefficient encode -- the ablation alternative to
// zigzag_varint_encode, for isolating how much of vxch's bandwidth win comes
// from variable-length packing itself vs. the zstd compression layered on
// top of it (compression=none alone still leaves varint packing in place).
// Haar lifting keeps coefficient magnitude close to the embedded-occupancy
// input range (0..101, see occupancy_embedding.hpp) regardless of levels --
// it never explodes the way, say, a naive unbounded transform could -- so
// int32 has ample headroom;
// this doesn't defend against a pathological input that would overflow it,
// same as the rest of this codec's integer arithmetic.
inline std::vector<std::uint8_t> fixed_width_encode(const std::vector<std::int64_t> & coeffs)
{
  std::vector<std::uint8_t> out(coeffs.size() * 4);
  for (std::size_t i = 0; i < coeffs.size(); ++i) {
    const std::int32_t v = static_cast<std::int32_t>(coeffs[i]);
    std::memcpy(&out[i * 4], &v, 4);
  }
  return out;
}

// 2D forward Haar level: row pass then column pass (in that order -- this is
// what makes it 2D, vs. running haar_forward_level once over a flattened
// array), in place, on a tightly-packed stride-`stride` grid buffer's
// top-left w_i x h_i block. Mirrors haar_inverse_2d_level in codec.cpp,
// which must undo these two passes in reverse order.
inline void haar_forward_2d_level(
  std::vector<std::int64_t> & grid, std::size_t stride,
  std::size_t w_i, std::size_t h_i)
{
  std::vector<std::int64_t> row(w_i);
  for (std::size_t r = 0; r < h_i; ++r) {
    std::copy_n(grid.begin() + static_cast<std::ptrdiff_t>(r * stride), w_i, row.begin());
    haar_forward_level(row, w_i);
    std::copy(row.begin(), row.end(), grid.begin() + static_cast<std::ptrdiff_t>(r * stride));
  }
  std::vector<std::int64_t> col(h_i);
  for (std::size_t c = 0; c < w_i; ++c) {
    for (std::size_t r = 0; r < h_i; ++r) {col[r] = grid[r * stride + c];}
    haar_forward_level(col, h_i);
    for (std::size_t r = 0; r < h_i; ++r) {grid[r * stride + c] = col[r];}
  }
}

// Gather the (HL, LH, HH) detail quadrants of level (w_i,h_i)->(new_w,new_h)
// as one flat vector, in that fixed order, row-major within each quadrant.
// Mirrors scatter_detail_quadrants in codec.cpp.
inline std::vector<std::int64_t> gather_detail_quadrants(
  const std::vector<std::int64_t> & grid, std::size_t stride,
  std::size_t w_i, std::size_t h_i, std::size_t new_w, std::size_t new_h)
{
  std::vector<std::int64_t> out;
  out.reserve(new_h * (w_i - new_w) + (h_i - new_h) * new_w + (h_i - new_h) * (w_i - new_w));
  for (std::size_t r = 0; r < new_h; ++r) {
    for (std::size_t c = new_w; c < w_i; ++c) {out.push_back(grid[r * stride + c]);}
  }
  for (std::size_t r = new_h; r < h_i; ++r) {
    for (std::size_t c = 0; c < new_w; ++c) {out.push_back(grid[r * stride + c]);}
  }
  for (std::size_t r = new_h; r < h_i; ++r) {
    for (std::size_t c = new_w; c < w_i; ++c) {out.push_back(grid[r * stride + c]);}
  }
  return out;
}

inline std::vector<std::int64_t> gather_ll(
  const std::vector<std::int64_t> & grid, std::size_t stride,
  std::size_t w, std::size_t h)
{
  std::vector<std::int64_t> out;
  out.reserve(w * h);
  for (std::size_t r = 0; r < h; ++r) {
    for (std::size_t c = 0; c < w; ++c) {out.push_back(grid[r * stride + c]);}
  }
  return out;
}

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
  if (levels < 1) {
    throw std::runtime_error("haar levels must be >= 1");
  }
  if (width == 0 || height == 0) {
    throw std::runtime_error("make_haar_bands: empty grid dimensions");
  }
  const std::size_t N = width * height;
  if (values.size() != N) {
    throw std::runtime_error("make_haar_bands: values size does not match width*height");
  }

  std::vector<std::int64_t> grid(values.begin(), values.end());
  const auto dims = compute_haar_level_dims(width, height, levels);

  std::vector<std::vector<std::int64_t> > band_coeffs(static_cast<std::size_t>(levels) + 1);
  for (int i = 0; i < levels; ++i) {
    const auto & lvl = dims[static_cast<std::size_t>(i)];
    const auto & nxt = dims[static_cast<std::size_t>(i + 1)];
    haar_forward_2d_level(grid, width, lvl.width, lvl.height);
    const int k = levels - i;
    band_coeffs[static_cast<std::size_t>(k)] =
      gather_detail_quadrants(grid, width, lvl.width, lvl.height, nxt.width, nxt.height);
  }
  const auto & ll = dims[static_cast<std::size_t>(levels)];
  band_coeffs[0] = gather_ll(grid, width, ll.width, ll.height);

  const int total_bands = levels + 1;
  std::vector<EncodedChannel> bands;
  bands.reserve(static_cast<std::size_t>(total_bands));

  for (int k = 0; k < total_bands; ++k) {
    const auto & coeffs = band_coeffs[static_cast<std::size_t>(k)];

    // Σcoefficient² -- exact contribution of this band to squared
    // reconstruction error if withheld (Parseval's theorem; Haar is
    // orthonormal). double rather than int64 since it's only ever used as a
    // priority score, never decoded, so there's no reason to risk overflow.
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
