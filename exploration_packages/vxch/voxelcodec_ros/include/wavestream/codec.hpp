#pragma once

// WaveStream tile codec, C++17. Paper-facing entry points are at the end:
// encode_tile (Algorithm: Tile encoding) and decode_tile (Algorithm:
// Progressive tile reconstruction). All WaveStream definitions are inline;
// link libzstd. No ROS, JSON, archive, transport, or scheduler dependency.
//
// Paper -> code:
//   u -> phi(u) -> beta[0..L] -> p[0..L]       encode_tile
//   p[j] -> beta[j]                           decode_band
//   beta[0..k-1] -> inverse -> resize -> u_hat reconstruct_tile
//
// Geometry: row-major w*h cells; L includes the first decomposition.
// beta[0] is LL at level L, beta[j] is HL|LH|HH at level L-j+1.
// A prefix of k groups reconstructs level L-k+1. Odd boundaries retain
// the unpaired sample in LL; dimensions need not be multiples of 2^L.
// See wavestream/README.md and examples/paper_pipeline.cpp in the repository.
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>
#include <zstd.h>

namespace wavestream {
using Coefficients = std::vector<std::int64_t>;
using CoefficientGroups = std::vector<Coefficients>;
using Bytes = std::vector<std::uint8_t>;

struct HaarLevelDims
{
  std::size_t width;
  std::size_t height;
};

struct HaarBandLayout
{
  std::size_t element_count;
  std::size_t level_w;
  std::size_t level_h;
  std::size_t new_w;
  std::size_t new_h;
};

struct HaarReconstruction
{
  std::vector<std::uint32_t> values;
  std::size_t width;
  std::size_t height;
};

inline std::vector<HaarLevelDims> compute_haar_level_dims(
  std::size_t width, std::size_t height, int levels)
{
  if (levels < 1 || width == 0 || height == 0 ||
    width > std::numeric_limits<std::size_t>::max() / height) {
    throw std::runtime_error("invalid Haar dimensions or levels");
  }
  std::vector<HaarLevelDims> dims(static_cast<std::size_t>(levels) + 1);
  dims[0] = HaarLevelDims{width, height};
  for (int i = 1; i <= levels; ++i) {
    const auto & prev = dims[static_cast<std::size_t>(i - 1)];
    dims[static_cast<std::size_t>(i)] =
      HaarLevelDims{prev.width / 2 + prev.width % 2, prev.height / 2 + prev.height % 2};
  }
  return dims;
}

inline std::vector<HaarBandLayout> compute_haar_band_layout(
  std::size_t width, std::size_t height, int levels)
{
  const auto dims = compute_haar_level_dims(width, height, levels);
  std::vector<HaarBandLayout> layout(static_cast<std::size_t>(levels) + 1);

  const auto & ll = dims[static_cast<std::size_t>(levels)];
  layout[0] = HaarBandLayout{ll.width * ll.height, ll.width, ll.height, ll.width, ll.height};

  for (int k = 1; k <= levels; ++k) {
    const int i = levels - k;
    const auto & lvl = dims[static_cast<std::size_t>(i)];
    const auto & nxt = dims[static_cast<std::size_t>(i + 1)];
    const std::size_t hl = nxt.height * (lvl.width - nxt.width);
    const std::size_t lh = (lvl.height - nxt.height) * nxt.width;
    const std::size_t hh = (lvl.height - nxt.height) * (lvl.width - nxt.width);
    layout[static_cast<std::size_t>(k)] =
      HaarBandLayout{hl + lh + hh, lvl.width, lvl.height, nxt.width, nxt.height};
  }
  return layout;
}

// Equation (lifting): d=b-a, s=a+floor(d/2). Preserve odd trailing samples.
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

inline std::vector<std::uint8_t> fixed_width_encode(const std::vector<std::int64_t> & coeffs)
{
  std::vector<std::uint8_t> out(coeffs.size() * 4);
  for (std::size_t i = 0; i < coeffs.size(); ++i) {
    const std::int32_t v = static_cast<std::int32_t>(coeffs[i]);
    std::memcpy(&out[i * 4], &v, 4);
  }
  return out;
}

// One 2D decomposition: row lifting followed by column lifting.
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

// Equation (inverse lifting): a=s-floor(d/2), b=a+d.
inline void haar_inverse_level(std::vector<std::int64_t> &coeffs, std::size_t full_len, std::size_t smooth_len)
{
  if (full_len < 2)
  {
    return;
  }
  const std::size_t num_pairs = full_len / 2;
  std::vector<std::int64_t> smooth(coeffs.begin(), coeffs.begin() + static_cast<std::ptrdiff_t>(smooth_len));
  std::vector<std::int64_t> detail(
      coeffs.begin() + static_cast<std::ptrdiff_t>(smooth_len),
      coeffs.begin() + static_cast<std::ptrdiff_t>(smooth_len + num_pairs));
  for (std::size_t i = 0; i < num_pairs; ++i)
  {
    const std::int64_t a = smooth[i] - (detail[i] >> 1);
    const std::int64_t b = a + detail[i];
    coeffs[2 * i] = a;
    coeffs[2 * i + 1] = b;
  }
  if (full_len % 2 == 1)
  {
    coeffs[full_len - 1] = smooth[smooth_len - 1];
  }
}

// Undo the separable transform in reverse order: columns, then rows.
inline void haar_inverse_2d_level(
  std::vector<std::int64_t> & grid, std::size_t stride,
  std::size_t w_i, std::size_t h_i, std::size_t new_w, std::size_t new_h)
{
  std::vector<std::int64_t> col(h_i);
  for (std::size_t c = 0; c < w_i; ++c) {
    for (std::size_t r = 0; r < h_i; ++r) {col[r] = grid[r * stride + c];}
    haar_inverse_level(col, h_i, new_h);
    for (std::size_t r = 0; r < h_i; ++r) {grid[r * stride + c] = col[r];}
  }
  std::vector<std::int64_t> row(w_i);
  for (std::size_t r = 0; r < h_i; ++r) {
    std::copy_n(grid.begin() + static_cast<std::ptrdiff_t>(r * stride), w_i, row.begin());
    haar_inverse_level(row, w_i, new_w);
    std::copy(row.begin(), row.end(), grid.begin() + static_cast<std::ptrdiff_t>(r * stride));
  }
}

inline void scatter_detail_quadrants(
  std::vector<std::int64_t> & grid, std::size_t stride,
  std::size_t w_i, std::size_t h_i, std::size_t new_w, std::size_t new_h,
  const std::vector<std::int64_t> & flat)
{
  std::size_t idx = 0;
  for (std::size_t r = 0; r < new_h; ++r) {
    for (std::size_t c = new_w; c < w_i; ++c) {grid[r * stride + c] = flat[idx++];}
  }
  for (std::size_t r = new_h; r < h_i; ++r) {
    for (std::size_t c = 0; c < new_w; ++c) {grid[r * stride + c] = flat[idx++];}
  }
  for (std::size_t r = new_h; r < h_i; ++r) {
    for (std::size_t c = new_w; c < w_i; ++c) {grid[r * stride + c] = flat[idx++];}
  }
}

// Start from beta[0] and undo one level per additional detail group.
inline HaarReconstruction reconstruct_haar_2d_impl(
  const std::vector<std::vector<std::int64_t> > & band_coeffs,
  std::size_t width, std::size_t height, int levels, int bands_received)
{
  const auto dims = compute_haar_level_dims(width, height, levels);
  const auto layout = compute_haar_band_layout(width, height, levels);

  const int bands = std::max(1, std::min(bands_received, levels + 1));
  const int out_level = levels - bands + 1;

  if (band_coeffs.empty() ||
    band_coeffs[0].size() != layout[0].element_count)
  {
    throw std::runtime_error("reconstruct_haar_2d_impl: band 0 (LL) missing or wrong size");
  }

  std::size_t cur_level = static_cast<std::size_t>(levels);
  std::vector<std::int64_t> buf = band_coeffs[0];  // tightly packed dims[levels]

  while (static_cast<int>(cur_level) > out_level) {
    const std::size_t k = static_cast<std::size_t>(levels) - cur_level + 1;
    if (k >= band_coeffs.size() || band_coeffs[k].size() != layout[k].element_count) {
      throw std::runtime_error("reconstruct_haar_2d_impl: missing/wrong-size detail band");
    }
    const auto & lvl = dims[cur_level - 1];
    const auto & nxt = dims[cur_level];

    std::vector<std::int64_t> next_buf(lvl.width * lvl.height, 0);
    for (std::size_t r = 0; r < nxt.height; ++r) {
      std::copy_n(
        buf.begin() + static_cast<std::ptrdiff_t>(r * nxt.width), nxt.width,
        next_buf.begin() + static_cast<std::ptrdiff_t>(r * lvl.width));
    }
    scatter_detail_quadrants(
      next_buf, lvl.width, lvl.width, lvl.height, nxt.width, nxt.height, band_coeffs[k]);
    haar_inverse_2d_level(next_buf, lvl.width, lvl.width, lvl.height, nxt.width, nxt.height);

    buf = std::move(next_buf);
    --cur_level;
  }

  const auto & out_dims = dims[cur_level];
  std::vector<std::uint32_t> values(out_dims.width * out_dims.height);
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i] = static_cast<std::uint32_t>(buf[i] & 0xFFFFFFFFU);
  }
  return HaarReconstruction{std::move(values), out_dims.width, out_dims.height};
}

inline HaarReconstruction reconstruct_haar_from_bands(
  const std::vector<std::vector<std::int64_t> > & band_coeffs,
  std::size_t width,
  std::size_t height,
  int levels,
  int bands_received)
{
  if (levels < 1 || width == 0 || height == 0) {
    throw std::runtime_error("reconstruct_haar_from_bands: invalid levels or dimensions");
  }
  return reconstruct_haar_2d_impl(band_coeffs, width, height, levels, bands_received);
}

inline std::vector<std::int64_t> zigzag_varint_decode(
  const std::vector<std::uint8_t> & raw, std::size_t count)
{
  if (count > raw.size()) {throw std::runtime_error("truncated varint payload");}
  Coefficients out;
  out.reserve(count);
  std::size_t offset = 0;
  while (out.size() < count) {
    std::uint64_t value = 0;
    int shift = 0;
    while (true) {
      if (offset == raw.size()) {throw std::runtime_error("truncated varint");}
      const auto byte = raw[offset++];
      if (shift == 63 && byte > 1) {throw std::runtime_error("varint overflow");}
      value |= static_cast<std::uint64_t>(byte & 0x7FU) << shift;
      if ((byte & 0x80U) == 0) {break;}
      shift += 7;
    }
    const auto magnitude = static_cast<std::int64_t>(value >> 1U);
    out.push_back((value & 1U) ? -magnitude - 1 : magnitude);
  }
  if (offset != raw.size()) {throw std::runtime_error("trailing coefficient bytes");}
  return out;
}

inline std::vector<std::int64_t> fixed_width_decode(
  const std::vector<std::uint8_t> & raw, std::size_t count)
{
  if (count > raw.size() / 4 || raw.size() / 4 != count || raw.size() % 4 != 0)
  {throw std::runtime_error("invalid fixed-width payload size");}
  std::vector<std::int64_t> out(count);
  for (std::size_t i = 0; i < count; ++i) {
    std::int32_t v;
    std::memcpy(&v, &raw[i * 4], 4);
    out[i] = v;
  }
  return out;
}

inline std::vector<std::uint8_t> compress_payload(
    const std::string &compression,
    const std::vector<std::uint8_t> &raw_payload)
{
  if (compression != "none" && compression != "zstd") {
    throw std::runtime_error("unsupported compression: " + compression);
  }
  if (compression == "none")
  {
    return raw_payload;
  }

  std::vector<std::uint8_t> compressed(ZSTD_compressBound(raw_payload.size()));
  const auto result = ZSTD_compress(
      compressed.data(), compressed.size(), raw_payload.data(), raw_payload.size(), 9);
  if (ZSTD_isError(result))
  {
    throw std::runtime_error(ZSTD_getErrorName(result));
  }
  compressed.resize(result);
  return compressed;
}

inline std::vector<std::uint8_t> decompress_payload(
    const std::string &compression, std::size_t uncompressed_size,
    const std::vector<std::uint8_t> &compressed_payload)
{
  if (compression == "none")
  {
    return compressed_payload;
  }
  if (compression != "zstd")
  {
    throw std::runtime_error("unsupported compression: " + compression);
  }

  std::vector<std::uint8_t> raw(uncompressed_size);
  const auto result = ZSTD_decompress(
      raw.data(), raw.size(), compressed_payload.data(), compressed_payload.size());
  if (ZSTD_isError(result))
  {
    throw std::runtime_error(ZSTD_getErrorName(result));
  }
  if (result != uncompressed_size)
  {
    throw std::runtime_error("decompressed payload size mismatch");
  }
  return raw;
}

// Paper: L row/column lifting steps, recursing only on LL. Group 0 is
// the final LL; group j contains HL|LH|HH from level L-j+1.
inline CoefficientGroups forward_transform(
  const std::vector<std::uint32_t> & values,
  std::size_t width, std::size_t height, int levels)
{
  if (levels < 1) {
    throw std::runtime_error("haar levels must be >= 1");
  }
  if (width == 0 || height == 0 || width > std::numeric_limits<std::size_t>::max() / height) {
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

  return band_coeffs;
}

// Paper: phi(v) = v+1, including unknown (-1) -> 0.
inline std::uint32_t occupancy_to_embedded(std::int8_t v)
{
  return static_cast<std::uint32_t>(static_cast<int>(v) + 1);
}

inline std::int8_t embedded_to_occupancy(std::uint32_t e)
{
  // Interpret the reconstructed uint32 as a signed value before clamping;
  // inconsistent incremental bands can produce values outside [0,101].
  const std::int64_t value = e <= 0x7FFFFFFFU ? static_cast<std::int64_t>(e) :
    static_cast<std::int64_t>(e) - 0x100000000LL;
  return static_cast<std::int8_t>(std::max<std::int64_t>(-1,
    std::min<std::int64_t>(100, value - 1)));
}

// A group is independently decodable. These fields belong in the transport's
// metadata; this struct does not prescribe a wire envelope.
struct EncodedBand
{
  Bytes payload;
  std::size_t coefficient_count{0};
  std::size_t uncompressed_size{0};
};

struct EncodedTile
{
  std::size_t width{0};
  std::size_t height{0};
  int haar_levels{0};
  std::vector<EncodedBand> bands;  // band 0 = LL, bands 1..L = coarse-to-fine details
};

// Paper: ZigZag -> Varint -> Zstd, independently for each beta^(j).
inline EncodedBand encode_band(const Coefficients & coefficients)
{
  const auto raw = zigzag_varint_encode(coefficients);
  return {compress_payload("zstd", raw), coefficients.size(), raw.size()};
}

inline Coefficients decode_band(const EncodedBand & band)
{
  return zigzag_varint_decode(
    decompress_payload("zstd", band.uncompressed_size, band.payload),
    band.coefficient_count);
}

// Paper Algorithm: Tile encoding. L counts the initial decomposition too.
// Input is a row-major tile of occupancy values (-1, 0..100).
inline EncodedTile encode_tile(const std::vector<std::int8_t> & tile,
  std::size_t width, std::size_t height, int haar_levels)
{
  std::vector<std::uint32_t> embedded;
  embedded.reserve(tile.size());
  for (const auto v : tile) {
    if (v < -1 || v > 100) {throw std::invalid_argument("invalid occupancy value");}
    embedded.push_back(occupancy_to_embedded(v));
  }
  const auto beta = forward_transform(embedded, width, height, haar_levels);
  EncodedTile encoded{width, height, haar_levels, {}};
  encoded.bands.reserve(beta.size());
  for (const auto & group : beta) {encoded.bands.push_back(encode_band(group));}
  return encoded;
}

// Paper: nearest-neighbour resampling to the tile extent, then phi^-1.
inline std::vector<std::int8_t> upsample_occupancy(
  const HaarReconstruction & approximation, std::size_t width, std::size_t height)
{
  if (width == 0 || height == 0 || width > std::numeric_limits<std::size_t>::max() / height ||
    approximation.width == 0 || approximation.height == 0 ||
    approximation.width > width || approximation.height > height ||
    approximation.values.size() != approximation.width * approximation.height)
  {throw std::invalid_argument("invalid reconstruction dimensions");}
  std::vector<std::int8_t> tile(width * height);
  for (std::size_t r = 0; r < height; ++r) {
    for (std::size_t c = 0; c < width; ++c) {
      tile[r * width + c] = embedded_to_occupancy(approximation.values[
        (r * approximation.height / height) * approximation.width +
        c * approximation.width / width]);
    }
  }
  return tile;
}

// Paper Algorithm: Progressive tile reconstruction. beta contains a contiguous
// prefix starting at band 0, decoded independently as messages arrive. Cache
// ownership, geometry invalidation and old-state fallback belong to the caller.
inline std::vector<std::int8_t> reconstruct_tile(const CoefficientGroups & beta,
  std::size_t width, std::size_t height, int haar_levels)
{
  if (haar_levels < 1 || beta.empty() || beta.size() > static_cast<std::size_t>(haar_levels) + 1)
  {throw std::invalid_argument("expected a nonempty coarse-to-fine prefix");}
  return upsample_occupancy(reconstruct_haar_from_bands(
    beta, width, height, haar_levels, static_cast<int>(beta.size())), width, height);
}

// Convenience for a self-contained encoded tile: decode its first k groups.
// k=1 gives the coarsest tile; k=L+1 gives the full tile. No implicit k=0 mode.
inline std::vector<std::int8_t> decode_tile(const EncodedTile & tile, std::size_t k)
{
  if (tile.haar_levels < 1 || k == 0 || k > tile.bands.size() ||
    k > static_cast<std::size_t>(tile.haar_levels) + 1)
  {throw std::invalid_argument("invalid prefix length");}
  const auto layout = compute_haar_band_layout(tile.width, tile.height, tile.haar_levels);
  CoefficientGroups beta;
  beta.reserve(k);
  for (std::size_t j = 0; j < k; ++j) {
    if (tile.bands[j].coefficient_count != layout[j].element_count)
    {throw std::invalid_argument("band coefficient count does not match tile geometry");}
    beta.push_back(decode_band(tile.bands[j]));
  }
  return reconstruct_tile(beta, tile.width, tile.height, tile.haar_levels);
}
}  // namespace wavestream
