#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "voxelcodec_ros/codec.hpp"
#include "voxelcodec_ros/haar_forward.hpp"

namespace
{

using voxelcodec_ros::ChannelDescriptor;
using voxelcodec_ros::EncodedChannel;
using voxelcodec_ros::HaarReconstruction;

// Concatenate raw (uncompressed) per-band payloads in band order into one
// monolithic byte stream -- this is exactly the layout decode_haar_wavelet_channel/
// decode_haar_progressive's 2D branch expects for the archive-style single-channel
// encoding (each band was independently zigzag-varint-encoded with no padding, so
// back-to-back concatenation is unambiguous to re-parse sequentially).
std::vector<std::uint8_t> concat_band_payloads(
  const std::vector<EncodedChannel> & bands, int count)
{
  std::vector<std::uint8_t> out;
  for (int k = 0; k < count; ++k) {
    const auto & p = bands[static_cast<std::size_t>(k)].payload;
    out.insert(out.end(), p.begin(), p.end());
  }
  return out;
}

ChannelDescriptor build_grid_descriptor(
  std::size_t width, std::size_t height, int levels)
{
  ChannelDescriptor desc;
  desc.name = "occupancy";
  desc.data_type = voxelcodec_ros::kDataTypeUint32;
  desc.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  desc.compression = voxelcodec_ros::kCompressionNone;
  desc.metadata[voxelcodec_ros::kHaarLevelsKey] = std::to_string(levels);
  desc.metadata[voxelcodec_ros::kHaarOriginalLengthKey] = std::to_string(width * height);
  desc.metadata[voxelcodec_ros::kHaarGridWidthKey] = std::to_string(width);
  desc.metadata[voxelcodec_ros::kHaarGridHeightKey] = std::to_string(height);
  return desc;
}

std::vector<std::uint32_t> make_grid(
  std::size_t width, std::size_t height,
  const std::function<std::uint32_t(std::size_t, std::size_t)> & f)
{
  std::vector<std::uint32_t> values(width * height);
  for (std::size_t r = 0; r < height; ++r) {
    for (std::size_t c = 0; c < width; ++c) {
      values[r * width + c] = f(c, r);
    }
  }
  return values;
}

// The streaming per-band consumers (occupancy_grid_vxch_node's decoder counterpart,
// vxch_occupancy_grid_node, and vxch_visual_test's CLI) each keep their own local copy
// of these two decode functions rather than sharing one from the library -- see
// zigzag_varint_decode's own comment in vxch_occupancy_grid_node.cpp. Mirrored here so
// tests can drive the exact same decompress -> decode -> reconstruct_haar_from_bands
// path production code uses, instead of the archive-style decode_channel/
// decode_haar_progressive path below (which -- unlike this one -- doesn't consult
// kHaarVarintKey and always assumes varint packing).
std::vector<std::int64_t> zigzag_varint_decode(
  const std::vector<std::uint8_t> & raw, std::size_t count)
{
  std::vector<std::int64_t> out;
  out.reserve(count);
  std::size_t offset = 0;
  while (out.size() < count) {
    std::uint64_t value = 0;
    int shift = 0;
    while (offset < raw.size()) {
      const std::uint8_t byte = raw[offset++];
      value |= static_cast<std::uint64_t>(byte & 0x7FU) << shift;
      if ((byte & 0x80U) == 0) {break;}
      shift += 7;
      if (shift >= 64) {throw std::runtime_error("varint overflow");}
    }
    const std::int64_t decoded = (value & 1U)
      ? -static_cast<std::int64_t>((value >> 1U) + 1U)
      : static_cast<std::int64_t>(value >> 1U);
    out.push_back(decoded);
  }
  return out;
}

std::vector<std::int64_t> fixed_width_decode(
  const std::vector<std::uint8_t> & raw, std::size_t count)
{
  std::vector<std::int64_t> out(count);
  for (std::size_t i = 0; i < count; ++i) {
    std::int32_t v;
    std::memcpy(&v, &raw[i * 4], 4);
    out[i] = v;
  }
  return out;
}

// decode_channel() (unlike decode_haar_progressive()) validates
// uncompressed_size against the actual payload length -- fill it in from the
// payload we're about to decode (compression=none, so it's payload.size()).
voxelcodec_ros::DecodedChannel decode_full(
  ChannelDescriptor descriptor, const std::vector<std::uint8_t> & payload)
{
  descriptor.uncompressed_size = payload.size();
  return voxelcodec_ros::decode_channel(descriptor, payload);
}

// Little-endian byte packers for building raw-le payloads by hand, mirroring
// codec.cpp's own store_u16/store_u64 byte-by-byte approach (rather than
// memcpy) so these tests don't quietly depend on the host being
// little-endian for the multi-byte integer packing.
void push_u16le(std::vector<std::uint8_t> & out, std::uint16_t v)
{
  out.push_back(static_cast<std::uint8_t>(v & 0xffU));
  out.push_back(static_cast<std::uint8_t>((v >> 8U) & 0xffU));
}

void push_u32le(std::vector<std::uint8_t> & out, std::uint32_t v)
{
  for (int i = 0; i < 4; ++i) {out.push_back(static_cast<std::uint8_t>((v >> (i * 8U)) & 0xffU));}
}

void push_u64le(std::vector<std::uint8_t> & out, std::uint64_t v)
{
  for (int i = 0; i < 8; ++i) {out.push_back(static_cast<std::uint8_t>((v >> (i * 8U)) & 0xffU));}
}

void push_f32le(std::vector<std::uint8_t> & out, float v)
{
  std::uint32_t bits;
  std::memcpy(&bits, &v, sizeof(bits));
  push_u32le(out, bits);
}

void push_f64le(std::vector<std::uint8_t> & out, double v)
{
  std::uint64_t bits;
  std::memcpy(&bits, &v, sizeof(bits));
  push_u64le(out, bits);
}

}  // namespace

TEST(VoxelCodecRos, DecodeStructuredChannels)
{
  using voxelcodec_ros::ChannelDescriptor;
  using voxelcodec_ros::EncodedChannel;
  using voxelcodec_ros::Manifest;

  Manifest manifest;
  manifest.voxel_count = 4;
  manifest.channels = {
      ChannelDescriptor{"x", "position.x", "uint32", "delta-varint", "zstd", 4, 0, 0, 0, {}},
      ChannelDescriptor{"color", "color.rgba", "uint32", "palette", "zstd", 4, 0, 0, 0, {{"palette_values", "[4278190335,16711935,65535]"}, {"palette_index_type", "uint8"}}},
      ChannelDescriptor{"occupancy", "occupancy.probability", "float32", "constant", "zstd", 4, 0, 0, 0, {{"constant_value", "1"}}}};

  std::vector<EncodedChannel> channels = {
      {manifest.channels[0], {0x02, 0x02, 0x02, 0xf8, 0x03}},
      {manifest.channels[1], {0x00, 0x00, 0x01, 0x02}},
      {manifest.channels[2], {}}};

  const auto archive_bytes = voxelcodec_ros::encode_archive(manifest, channels);
  const auto archive = voxelcodec_ros::read_archive(archive_bytes);
  const auto decoded = voxelcodec_ros::decode_selected(archive, {"x", "color", "occupancy"});

  EXPECT_EQ(std::get<std::vector<std::uint32_t> >(decoded.at("x").values)[3], 255U);
  EXPECT_EQ(std::get<std::vector<std::uint32_t> >(decoded.at("color").values)[2], 16711935U);
  EXPECT_FLOAT_EQ(std::get<std::vector<float> >(decoded.at("occupancy").values)[1], 1.0F);
}

TEST(VoxelCodecRos, DecodeByteShuffleChannel)
{
  using voxelcodec_ros::ChannelDescriptor;

  const ChannelDescriptor descriptor{
      "detail_0", "node.detail.0", "uint32", "byte-shuffle", "none", 3, 0, 12, 12, {}};

  // Byte-shuffle layout for [0x11223344, 0xAABBCCDD, 0x01020304]
  // lane0: 44 DD 04
  // lane1: 33 CC 03
  // lane2: 22 BB 02
  // lane3: 11 AA 01
  const std::vector<std::uint8_t> payload{
      0x44, 0xDD, 0x04,
      0x33, 0xCC, 0x03,
      0x22, 0xBB, 0x02,
      0x11, 0xAA, 0x01};

  const auto decoded = decode_full(descriptor, payload);
  const auto values = std::get<std::vector<std::uint32_t> >(decoded.values);

  ASSERT_EQ(values.size(), 3U);
  EXPECT_EQ(values[0], 0x11223344U);
  EXPECT_EQ(values[1], 0xAABBCCDDU);
  EXPECT_EQ(values[2], 0x01020304U);
}

TEST(VoxelCodecRos, HaarWavelet2dFullRoundTripExact)
{
  constexpr std::size_t W = 64, H = 48;
  constexpr int levels = 4;
  const auto values = make_grid(W, H, [](std::size_t c, std::size_t r) {
      return static_cast<std::uint32_t>((c * 7 + r * 13) % 101);
    });

  const auto bands = voxelcodec_ros::make_haar_bands(values, W, H, levels, "none");
  ASSERT_EQ(bands.size(), static_cast<std::size_t>(levels + 1));

  const auto payload = concat_band_payloads(bands, levels + 1);
  const auto descriptor = build_grid_descriptor(W, H, levels);
  const auto decoded = decode_full(descriptor, payload);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);

  ASSERT_EQ(out.size(), values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(out[i], values[i]) << "mismatch at index " << i;
  }
}

TEST(VoxelCodecRos, HaarWavelet2dOddDimensionsRoundTrip)
{
  constexpr std::size_t W = 7, H = 7;
  constexpr int levels = 3;
  const auto values = make_grid(W, H, [](std::size_t c, std::size_t r) {
      return static_cast<std::uint32_t>(c * 3 + r);
    });

  const auto bands = voxelcodec_ros::make_haar_bands(values, W, H, levels, "none");
  const auto payload = concat_band_payloads(bands, levels + 1);
  const auto descriptor = build_grid_descriptor(W, H, levels);
  const auto decoded = decode_full(descriptor, payload);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);

  ASSERT_EQ(out.size(), values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(out[i], values[i]);
  }
}

TEST(VoxelCodecRos, HaarWavelet2dAsymmetricDimensionsRoundTrip)
{
  // H freezes at 1 well before W stops halving -- exercises independent
  // per-axis degeneracy in compute_haar_level_dims/compute_haar_band_layout.
  constexpr std::size_t W = 200, H = 15;
  constexpr int levels = 5;
  const auto values = make_grid(W, H, [](std::size_t c, std::size_t r) {
      return static_cast<std::uint32_t>((c + r * 17) % 53);
    });

  const auto layout = voxelcodec_ros::compute_haar_band_layout(W, H, levels);
  // Band 1 = detail from the last forward pass (coarsest detail): H is
  // already frozen there, so LH/HH (which depend on height shrinking) must
  // be empty while HL (width-only detail) is not.
  const auto dims = voxelcodec_ros::compute_haar_level_dims(W, H, levels);
  EXPECT_EQ(dims[static_cast<std::size_t>(levels)].height, 1U);
  EXPECT_GT(layout[1].element_count, 0U);

  const auto bands = voxelcodec_ros::make_haar_bands(values, W, H, levels, "none");
  const auto payload = concat_band_payloads(bands, levels + 1);
  const auto descriptor = build_grid_descriptor(W, H, levels);
  const auto decoded = decode_full(descriptor, payload);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);

  ASSERT_EQ(out.size(), values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(out[i], values[i]);
  }
}

TEST(VoxelCodecRos, HaarWavelet2dCoarseReconstructionPreservesVerticalEdge)
{
  // The bug this redesign fixes: the old 1D-flattened transform smeared a
  // clean 2D edge into row-dependent streaks under partial-band decode. A
  // correct 2D pyramid must keep the edge at roughly the same column in
  // every row of the coarse (band-0-only) reconstruction.
  constexpr std::size_t W = 64, H = 64;
  constexpr int levels = 4;
  const auto values = make_grid(W, H, [](std::size_t c, std::size_t) {
      return c < W / 2 ? 100U : 0U;
    });

  const auto bands = voxelcodec_ros::make_haar_bands(values, W, H, levels, "none");
  // decode_haar_progressive's archive-style API expects the full concatenated
  // payload (like the pre-redesign 1D code); max_bands=1 limits how much of
  // it gets inverse-transformed, not how much payload is provided.
  const auto payload = concat_band_payloads(bands, levels + 1);
  const auto descriptor = build_grid_descriptor(W, H, levels);
  const auto decoded = voxelcodec_ros::decode_haar_progressive(descriptor, payload, 1);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);

  const auto coarse_dims =
    voxelcodec_ros::compute_haar_level_dims(W, H, levels)[static_cast<std::size_t>(levels)];
  const std::size_t w_prime = coarse_dims.width;
  const std::size_t h_prime = coarse_dims.height;
  ASSERT_EQ(out.size(), w_prime * h_prime);
  ASSERT_GT(w_prime, 1U);

  // Every row's high->low transition should land at the same column --
  // no row-dependent streaking.
  std::vector<std::size_t> transition_col(h_prime);
  for (std::size_t r = 0; r < h_prime; ++r) {
    std::size_t t = w_prime;
    for (std::size_t c = 0; c < w_prime; ++c) {
      if (out[r * w_prime + c] < 50U) {t = c; break;}
    }
    transition_col[r] = t;
  }
  for (std::size_t r = 1; r < h_prime; ++r) {
    EXPECT_EQ(transition_col[r], transition_col[0])
      << "row " << r << " transition column differs from row 0 -- streak artifact";
  }
  // And it should land roughly at the midpoint, not at an arbitrary offset.
  EXPECT_NEAR(
    static_cast<double>(transition_col[0]), static_cast<double>(w_prime) / 2.0,
    static_cast<double>(w_prime) / 2.0 + 1.0);
}

TEST(VoxelCodecRos, HaarWavelet2dCheckerboardStaysTwoDimensional)
{
  // A block checkerboard reconstructed at low bands should still alternate
  // on BOTH axes, not degenerate into stripes along one axis (which would
  // indicate a regression to a horizontal-only transform). max_bands=2 with
  // levels=3 downsamples by 4x (dims[2] of a 32x32 grid is 8x8), so the
  // checkerboard block size (8) must exceed the downsample factor (4) for
  // the pattern to survive averaging -- otherwise every output cell would
  // legitimately average to a constant, which isn't a codec bug.
  constexpr std::size_t W = 32, H = 32;
  constexpr int levels = 3;
  const auto values = make_grid(W, H, [](std::size_t c, std::size_t r) {
      return ((c / 8 + r / 8) % 2 == 0) ? 100U : 0U;
    });

  const auto bands = voxelcodec_ros::make_haar_bands(values, W, H, levels, "none");
  const auto payload = concat_band_payloads(bands, levels + 1);
  const auto descriptor = build_grid_descriptor(W, H, levels);
  const auto decoded = voxelcodec_ros::decode_haar_progressive(descriptor, payload, 2);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
  const std::size_t n = decoded.descriptor.element_count;
  ASSERT_GT(n, 0U);
  const std::size_t side = static_cast<std::size_t>(std::sqrt(static_cast<double>(n)));
  ASSERT_GE(side, 2U);

  // Both a horizontal and a vertical neighbor pair must show variation
  // somewhere in the grid -- a horizontal-only (1D) transform would produce
  // rows that are internally constant (no horizontal variation) or columns
  // that are identical to their neighbor (no vertical variation).
  bool horizontal_variation = false;
  bool vertical_variation = false;
  for (std::size_t r = 0; r < side && !(horizontal_variation && vertical_variation); ++r) {
    for (std::size_t c = 0; c + 1 < side; ++c) {
      if (out[r * side + c] != out[r * side + c + 1]) {horizontal_variation = true;}
    }
  }
  for (std::size_t c = 0; c < side && !(horizontal_variation && vertical_variation); ++c) {
    for (std::size_t r = 0; r + 1 < side; ++r) {
      if (out[r * side + c] != out[(r + 1) * side + c]) {vertical_variation = true;}
    }
  }
  EXPECT_TRUE(horizontal_variation);
  EXPECT_TRUE(vertical_variation);
}

TEST(VoxelCodecRos, HaarWavelet2dDegenerateBandsAreEmptyAndSafe)
{
  // 200 takes exactly 8 halvings to reach 1 (200/2^8=0.78, dims[8]=(1,1)), so
  // levels must exceed 8 for both axes to already be frozen at a pass boundary
  // -- that's what makes a detail band's quadrants (and therefore its element
  // count) collapse to genuinely 0, not just small.
  constexpr std::size_t W = 200, H = 15;
  constexpr int levels = 9;
  const auto values = make_grid(W, H, [](std::size_t c, std::size_t r) {
      return static_cast<std::uint32_t>((c + r) % 17);
    });

  const auto bands = voxelcodec_ros::make_haar_bands(values, W, H, levels, "none");
  ASSERT_EQ(bands.size(), static_cast<std::size_t>(levels + 1));
  bool saw_empty_band = false;
  for (const auto & b : bands) {
    if (b.descriptor.element_count == 0) {saw_empty_band = true;}
  }
  EXPECT_TRUE(saw_empty_band);

  const auto payload = concat_band_payloads(bands, levels + 1);
  const auto descriptor = build_grid_descriptor(W, H, levels);
  const auto decoded = decode_full(descriptor, payload);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
  ASSERT_EQ(out.size(), values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(out[i], values[i]);
  }
}

TEST(VoxelCodecRos, HaarWavelet2dPartialThenCompleteBands)
{
  constexpr std::size_t W = 40, H = 30;
  constexpr int levels = 3;
  const auto values = make_grid(W, H, [](std::size_t c, std::size_t r) {
      return static_cast<std::uint32_t>((c * 5 + r * 3) % 47);
    });

  const auto bands = voxelcodec_ros::make_haar_bands(values, W, H, levels, "none");
  const auto descriptor = build_grid_descriptor(W, H, levels);
  const auto dims = voxelcodec_ros::compute_haar_level_dims(W, H, levels);

  // Partial: use only bands 0-1 of the reconstruction (max_bands=2).
  {
    const auto payload = concat_band_payloads(bands, levels + 1);
    const auto decoded = voxelcodec_ros::decode_haar_progressive(descriptor, payload, 2);
    const auto out_level = dims[static_cast<std::size_t>(levels - 1)];
    EXPECT_EQ(decoded.descriptor.element_count, out_level.width * out_level.height);
  }

  // Complete: all bands, exact round trip.
  {
    const auto payload = concat_band_payloads(bands, levels + 1);
    const auto decoded = decode_full(descriptor, payload);
    const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
    ASSERT_EQ(out.size(), values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
      EXPECT_EQ(out[i], values[i]);
    }
  }
}

TEST(VoxelCodecRos, HaarWaveletLegacy1dPathUnaffectedByGridDispatch)
{
  // A descriptor with no grid-width/height metadata must still decode via
  // the legacy flat/1D path (this is what keeps voxel_display.cpp's wvmp
  // point-cloud channels working unchanged).
  constexpr int levels = 2;
  const std::vector<std::uint32_t> values = {1, 2, 3, 4, 5, 6, 7, 8};

  std::vector<std::int64_t> coeffs(values.begin(), values.end());
  std::vector<std::size_t> smooth_lens(levels + 1);
  smooth_lens[0] = values.size();
  for (int i = 1; i <= levels; ++i) {smooth_lens[i] = (smooth_lens[i - 1] + 1) / 2;}
  for (int i = 0; i < levels; ++i) {
    voxelcodec_ros::haar_forward_level(coeffs, smooth_lens[i]);
  }
  const auto payload = voxelcodec_ros::zigzag_varint_encode(coeffs);

  ChannelDescriptor descriptor;
  descriptor.name = "wvmp_x";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint32;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  descriptor.compression = voxelcodec_ros::kCompressionNone;
  descriptor.metadata[voxelcodec_ros::kHaarLevelsKey] = std::to_string(levels);
  descriptor.metadata[voxelcodec_ros::kHaarOriginalLengthKey] = std::to_string(values.size());
  // Deliberately no kHaarGridWidthKey/kHaarGridHeightKey.

  const auto decoded = decode_full(descriptor, payload);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);

  ASSERT_EQ(out.size(), values.size());
  for (std::size_t i = 0; i < values.size(); ++i) {
    EXPECT_EQ(out[i], values[i]);
  }
}

TEST(VoxelCodecRos, HaarAblationMatrixRoundTripExact)
{
  // Formalizes the compression x varint_encoding ablation matrix manually verified
  // via vxch_visual_test/vxch_cli: every combination must reconstruct bit-exact
  // through the same decompress -> decode -> reconstruct_haar_from_bands path
  // occupancy_grid_vxch_node / vxch_occupancy_grid_node use for real band traffic
  // (as opposed to the archive-style decode_channel/decode_haar_progressive path
  // exercised elsewhere in this file, which doesn't know about kHaarVarintKey).
  constexpr std::size_t W = 48, H = 36;
  constexpr int levels = 3;
  const auto values = make_grid(W, H, [](std::size_t c, std::size_t r) {
      return static_cast<std::uint32_t>((c * 5 + r * 3) % 47);
    });

  for (const char * compression :
    {voxelcodec_ros::kCompressionNone, voxelcodec_ros::kCompressionZstd})
  {
    for (const bool use_varint : {true, false}) {
      SCOPED_TRACE(
        std::string("compression=") + compression +
        " varint=" + (use_varint ? "true" : "false"));

      const auto bands =
        voxelcodec_ros::make_haar_bands(values, W, H, levels, compression, use_varint);
      ASSERT_EQ(bands.size(), static_cast<std::size_t>(levels + 1));

      std::vector<std::vector<std::int64_t> > band_coeffs(bands.size());
      for (std::size_t k = 0; k < bands.size(); ++k) {
        const auto & desc = bands[k].descriptor;
        EXPECT_EQ(desc.metadata.at(voxelcodec_ros::kHaarVarintKey), use_varint ? "1" : "0");
        const auto raw = voxelcodec_ros::decompress_payload(desc, bands[k].payload);
        band_coeffs[k] = use_varint ?
          zigzag_varint_decode(raw, desc.element_count) :
          fixed_width_decode(raw, desc.element_count);
      }

      const auto recon =
        voxelcodec_ros::reconstruct_haar_from_bands(band_coeffs, W, H, levels, levels + 1);
      ASSERT_EQ(recon.width, W);
      ASSERT_EQ(recon.height, H);
      for (std::size_t i = 0; i < values.size(); ++i) {
        EXPECT_EQ(recon.values[i], values[i]) << "mismatch at index " << i;
      }
    }
  }
}

TEST(VoxelCodecRos, HaarVarintPackingSmallerThanFixedWidthForSmallCoefficients)
{
  // Small-magnitude coefficients (typical for a mostly-flat occupancy region) should
  // pack to fewer bytes under varint than the flat 4 bytes/coefficient fixed-width
  // encoding always uses -- the whole reason varint packing exists as a bandwidth win
  // independent of zstd (see occupancy_grid_vxch_node's varint_encoding parameter).
  constexpr std::size_t W = 32, H = 32;
  constexpr int levels = 2;
  const auto values = make_grid(W, H, [](std::size_t, std::size_t) {return 0U;});

  const auto varint_bands = voxelcodec_ros::make_haar_bands(values, W, H, levels, "none", true);
  const auto fixed_bands = voxelcodec_ros::make_haar_bands(values, W, H, levels, "none", false);

  std::size_t varint_total = 0, fixed_total = 0, total_elements = 0;
  for (const auto & b : varint_bands) {varint_total += b.payload.size();}
  for (const auto & b : fixed_bands) {
    fixed_total += b.payload.size();
    total_elements += b.descriptor.element_count;
  }

  EXPECT_LT(varint_total, fixed_total);
  // Fixed-width is exactly 4 bytes per coefficient, always -- no data-dependent variation.
  EXPECT_EQ(fixed_total, total_elements * 4);
}

// ── raw-le: the one encoding with no other test coverage at all ───────────

TEST(VoxelCodecRos, DecodeRawLeAllDataTypes)
{
  {
    SCOPED_TRACE("uint8");
    const std::vector<std::uint8_t> values{0, 1, 200, 255};
    ChannelDescriptor descriptor{
        "v", "", voxelcodec_ros::kDataTypeUint8, voxelcodec_ros::kEncodingRawLE,
        voxelcodec_ros::kCompressionNone, 4, 0, 0, 0, {}};
    const auto decoded = decode_full(descriptor, values);
    const auto out = std::get<std::vector<std::uint8_t> >(decoded.values);
    EXPECT_EQ(out, values);
  }
  {
    SCOPED_TRACE("uint16");
    const std::vector<std::uint16_t> values{0, 1, 500, 65535};
    std::vector<std::uint8_t> raw;
    for (const auto v : values) {push_u16le(raw, v);}
    ChannelDescriptor descriptor{
        "v", "", voxelcodec_ros::kDataTypeUint16, voxelcodec_ros::kEncodingRawLE,
        voxelcodec_ros::kCompressionNone, 4, 0, 0, 0, {}};
    const auto decoded = decode_full(descriptor, raw);
    const auto out = std::get<std::vector<std::uint16_t> >(decoded.values);
    EXPECT_EQ(out, values);
  }
  {
    SCOPED_TRACE("uint32");
    const std::vector<std::uint32_t> values{0, 1, 100000, 4294967295U};
    std::vector<std::uint8_t> raw;
    for (const auto v : values) {push_u32le(raw, v);}
    ChannelDescriptor descriptor{
        "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
        voxelcodec_ros::kCompressionNone, 4, 0, 0, 0, {}};
    const auto decoded = decode_full(descriptor, raw);
    const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
    EXPECT_EQ(out, values);
  }
  {
    SCOPED_TRACE("uint64");
    const std::vector<std::uint64_t> values{
        0, 1, 10000000000ULL, 18446744073709551615ULL};
    std::vector<std::uint8_t> raw;
    for (const auto v : values) {push_u64le(raw, v);}
    ChannelDescriptor descriptor{
        "v", "", voxelcodec_ros::kDataTypeUint64, voxelcodec_ros::kEncodingRawLE,
        voxelcodec_ros::kCompressionNone, 4, 0, 0, 0, {}};
    const auto decoded = decode_full(descriptor, raw);
    const auto out = std::get<std::vector<std::uint64_t> >(decoded.values);
    EXPECT_EQ(out, values);
  }
  {
    SCOPED_TRACE("float32");
    const std::vector<float> values{0.0F, 1.5F, -2.25F, 3.14159F};
    std::vector<std::uint8_t> raw;
    for (const auto v : values) {push_f32le(raw, v);}
    ChannelDescriptor descriptor{
        "v", "", voxelcodec_ros::kDataTypeFloat32, voxelcodec_ros::kEncodingRawLE,
        voxelcodec_ros::kCompressionNone, 4, 0, 0, 0, {}};
    const auto decoded = decode_full(descriptor, raw);
    const auto out = std::get<std::vector<float> >(decoded.values);
    ASSERT_EQ(out.size(), values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {EXPECT_FLOAT_EQ(out[i], values[i]);}
  }
  {
    SCOPED_TRACE("float64");
    const std::vector<double> values{0.0, 1.5, -2.25, 2.718281828};
    std::vector<std::uint8_t> raw;
    for (const auto v : values) {push_f64le(raw, v);}
    ChannelDescriptor descriptor{
        "v", "", voxelcodec_ros::kDataTypeFloat64, voxelcodec_ros::kEncodingRawLE,
        voxelcodec_ros::kCompressionNone, 4, 0, 0, 0, {}};
    const auto decoded = decode_full(descriptor, raw);
    const auto out = std::get<std::vector<double> >(decoded.values);
    ASSERT_EQ(out.size(), values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {EXPECT_DOUBLE_EQ(out[i], values[i]);}
  }
}

TEST(VoxelCodecRos, DecodeRawLeAlignmentMismatchThrows)
{
  // 3 bytes doesn't divide evenly by uint16's 2-byte width.
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint16, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  const std::vector<std::uint8_t> raw{0x01, 0x02, 0x03};
  EXPECT_THROW(decode_full(descriptor, raw), std::runtime_error);
}

// ── validate_descriptor / validate_manifest direct throw paths ────────────

TEST(VoxelCodecRos, ValidateDescriptorRejectsUnsupportedDataType)
{
  ChannelDescriptor descriptor{
      "v", "", "uint128", voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  EXPECT_THROW(voxelcodec_ros::validate_descriptor(descriptor), std::runtime_error);
}

TEST(VoxelCodecRos, ValidateDescriptorRejectsUnsupportedEncoding)
{
  ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, "run-length",
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  EXPECT_THROW(voxelcodec_ros::validate_descriptor(descriptor), std::runtime_error);
}

TEST(VoxelCodecRos, ValidateDescriptorRejectsUnsupportedCompression)
{
  ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      "lz4", 1, 0, 0, 0, {}};
  EXPECT_THROW(voxelcodec_ros::validate_descriptor(descriptor), std::runtime_error);
}

TEST(VoxelCodecRos, ValidateDescriptorRejectsEmptyName)
{
  ChannelDescriptor descriptor{
      "", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  EXPECT_THROW(voxelcodec_ros::validate_descriptor(descriptor), std::runtime_error);
}

TEST(VoxelCodecRos, ValidateManifestRejectsEmptyFormat)
{
  voxelcodec_ros::Manifest manifest;
  manifest.format = "";
  EXPECT_THROW(voxelcodec_ros::validate_manifest(manifest), std::runtime_error);
}

TEST(VoxelCodecRos, ValidateManifestRejectsDuplicateChannelNames)
{
  voxelcodec_ros::Manifest manifest;
  const ChannelDescriptor d{
      "dup", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  manifest.channels = {d, d};
  EXPECT_THROW(voxelcodec_ros::validate_manifest(manifest), std::runtime_error);
}

TEST(VoxelCodecRos, CompressPayloadRejectsUnsupportedCompression)
{
  EXPECT_THROW(
    voxelcodec_ros::compress_payload("lz4", {1, 2, 3}), std::runtime_error);
}

// ── constant channel: remaining data types + missing-metadata throw ───────

TEST(VoxelCodecRos, DecodeConstantChannelAllDataTypes)
{
  auto build = [](const char * data_type, const std::string & value) {
    ChannelDescriptor descriptor{
        "v", "", data_type, voxelcodec_ros::kEncodingConstant,
        voxelcodec_ros::kCompressionNone, 3, 0, 0, 0,
        {{voxelcodec_ros::kConstantValueKey, value}}};
    return decode_full(descriptor, {});
  };
  {
    const auto decoded = build(voxelcodec_ros::kDataTypeUint8, "7");
    const auto out = std::get<std::vector<std::uint8_t> >(decoded.values);
    EXPECT_EQ(out, (std::vector<std::uint8_t>{7, 7, 7}));
  }
  {
    const auto decoded = build(voxelcodec_ros::kDataTypeUint16, "1000");
    const auto out = std::get<std::vector<std::uint16_t> >(decoded.values);
    EXPECT_EQ(out, (std::vector<std::uint16_t>{1000, 1000, 1000}));
  }
  {
    const auto decoded = build(voxelcodec_ros::kDataTypeUint32, "70000");
    const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
    EXPECT_EQ(out, (std::vector<std::uint32_t>{70000, 70000, 70000}));
  }
  {
    const auto decoded = build(voxelcodec_ros::kDataTypeUint64, "10000000000");
    const auto out = std::get<std::vector<std::uint64_t> >(decoded.values);
    EXPECT_EQ(out, (std::vector<std::uint64_t>{10000000000ULL, 10000000000ULL, 10000000000ULL}));
  }
  {
    const auto decoded = build(voxelcodec_ros::kDataTypeFloat64, "2.5");
    const auto out = std::get<std::vector<double> >(decoded.values);
    ASSERT_EQ(out.size(), 3U);
    EXPECT_DOUBLE_EQ(out[0], 2.5);
  }
}

TEST(VoxelCodecRos, DecodeConstantChannelMissingMetadataThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingConstant,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  EXPECT_THROW(decode_full(descriptor, {}), std::runtime_error);
}

// ── palette: remaining index types + all its error paths ──────────────────

TEST(VoxelCodecRos, DecodePaletteChannelUint16Indices)
{
  ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingPalette,
      voxelcodec_ros::kCompressionNone, 2, 0, 0, 0,
      {{voxelcodec_ros::kPaletteValuesKey, "[10,20,30]"},
       {voxelcodec_ros::kPaletteIndexTypeKey, voxelcodec_ros::kDataTypeUint16}}};
  std::vector<std::uint8_t> raw;
  push_u16le(raw, 2);
  push_u16le(raw, 0);
  const auto decoded = decode_full(descriptor, raw);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
  EXPECT_EQ(out, (std::vector<std::uint32_t>{30, 10}));
}

TEST(VoxelCodecRos, DecodePaletteChannelUint32Indices)
{
  ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingPalette,
      voxelcodec_ros::kCompressionNone, 2, 0, 0, 0,
      {{voxelcodec_ros::kPaletteValuesKey, "[10,20,30]"},
       {voxelcodec_ros::kPaletteIndexTypeKey, voxelcodec_ros::kDataTypeUint32}}};
  std::vector<std::uint8_t> raw;
  push_u32le(raw, 1);
  push_u32le(raw, 2);
  const auto decoded = decode_full(descriptor, raw);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
  EXPECT_EQ(out, (std::vector<std::uint32_t>{20, 30}));
}

TEST(VoxelCodecRos, DecodePaletteChannelWrongDataTypeThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint8, voxelcodec_ros::kEncodingPalette,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0,
      {{voxelcodec_ros::kPaletteValuesKey, "[10]"},
       {voxelcodec_ros::kPaletteIndexTypeKey, voxelcodec_ros::kDataTypeUint8}}};
  EXPECT_THROW(decode_full(descriptor, {0}), std::runtime_error);
}

TEST(VoxelCodecRos, DecodePaletteChannelMissingPaletteValuesThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingPalette,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0,
      {{voxelcodec_ros::kPaletteIndexTypeKey, voxelcodec_ros::kDataTypeUint8}}};
  EXPECT_THROW(decode_full(descriptor, {0}), std::runtime_error);
}

TEST(VoxelCodecRos, DecodePaletteChannelMissingIndexTypeThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingPalette,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0,
      {{voxelcodec_ros::kPaletteValuesKey, "[10]"}}};
  EXPECT_THROW(decode_full(descriptor, {0}), std::runtime_error);
}

TEST(VoxelCodecRos, DecodePaletteChannelUnsupportedIndexTypeThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingPalette,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0,
      {{voxelcodec_ros::kPaletteValuesKey, "[10]"},
       {voxelcodec_ros::kPaletteIndexTypeKey, "uint64"}}};
  EXPECT_THROW(decode_full(descriptor, {0}), std::runtime_error);
}

TEST(VoxelCodecRos, DecodePaletteChannelIndexOutOfRangeThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingPalette,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0,
      {{voxelcodec_ros::kPaletteValuesKey, "[10,20]"},
       {voxelcodec_ros::kPaletteIndexTypeKey, voxelcodec_ros::kDataTypeUint8}}};
  EXPECT_THROW(decode_full(descriptor, {5}), std::runtime_error);
}

// ── delta-varint: error paths (happy path covered by DecodeStructuredChannels) ─

TEST(VoxelCodecRos, DecodeDeltaVarintWrongDataTypeThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint8, voxelcodec_ros::kEncodingDeltaVarint,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  EXPECT_THROW(decode_full(descriptor, {0}), std::runtime_error);
}

TEST(VoxelCodecRos, DecodeDeltaVarintNegativeValueThrows)
{
  // Zigzag-decoding the single varint byte 0x01 yields -1 as the very first
  // delta, so the running sum goes negative immediately.
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingDeltaVarint,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  EXPECT_THROW(decode_full(descriptor, {0x01}), std::runtime_error);
}

TEST(VoxelCodecRos, DecodeDeltaVarintTrailingBytesThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingDeltaVarint,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  // One value's worth of varint (0x00 -> delta 0) plus an unconsumed extra byte.
  EXPECT_THROW(decode_full(descriptor, {0x00, 0x00}), std::runtime_error);
}

TEST(VoxelCodecRos, ReadUvarintOverflowThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingDeltaVarint,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  // 10 continuation bytes push the accumulated shift past 64 bits.
  const std::vector<std::uint8_t> raw(10, 0x80);
  EXPECT_THROW(decode_full(descriptor, raw), std::runtime_error);
}

TEST(VoxelCodecRos, ReadUvarintTruncatedThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingDeltaVarint,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  // Continuation bit set on the last (and only) byte -- buffer ends mid-varint.
  EXPECT_THROW(decode_full(descriptor, {0x80}), std::runtime_error);
}

// ── byte-shuffle: remaining error paths ────────────────────────────────────

TEST(VoxelCodecRos, DecodeByteShuffleWrongDataTypeThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint8, voxelcodec_ros::kEncodingByteShuffle,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  EXPECT_THROW(decode_full(descriptor, {0}), std::runtime_error);
}

TEST(VoxelCodecRos, DecodeByteShufflePayloadSizeMismatchThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingByteShuffle,
      voxelcodec_ros::kCompressionNone, 3, 0, 0, 0, {}};
  // 3 elements need 12 bytes; give it 11.
  EXPECT_THROW(decode_full(descriptor, std::vector<std::uint8_t>(11, 0)), std::runtime_error);
}

// ── 1D haar-wavelet: zero-levels passthrough + validation paths ───────────

TEST(VoxelCodecRos, DecodeHaarWavelet1dZeroLevelsPassthrough)
{
  // haar_levels absent (defaults to 0) means no transform was ever applied --
  // the coefficients are the raw values, just zigzag-varint packed.
  const std::vector<std::uint32_t> values{5, 300, 0, 42};
  std::vector<std::int64_t> coeffs(values.begin(), values.end());
  const auto payload = voxelcodec_ros::zigzag_varint_encode(coeffs);

  ChannelDescriptor descriptor;
  descriptor.name = "v";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint32;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  descriptor.compression = voxelcodec_ros::kCompressionNone;
  descriptor.metadata[voxelcodec_ros::kHaarOriginalLengthKey] = std::to_string(values.size());
  // Deliberately no kHaarLevelsKey and no grid keys -> legacy 1D, levels=0.

  const auto decoded = decode_full(descriptor, payload);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
  EXPECT_EQ(out, values);
}

TEST(VoxelCodecRos, DecodeHaarWavelet1dWrongDataTypeThrows)
{
  ChannelDescriptor descriptor;
  descriptor.name = "v";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint8;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  descriptor.compression = voxelcodec_ros::kCompressionNone;
  EXPECT_THROW(decode_full(descriptor, {}), std::runtime_error);
}

// ── 2D haar-wavelet (non-progressive) validation paths ─────────────────────

TEST(VoxelCodecRos, DecodeHaarWavelet2dWrongDataTypeThrows)
{
  auto descriptor = build_grid_descriptor(4, 4, 2);
  descriptor.data_type = voxelcodec_ros::kDataTypeUint8;
  EXPECT_THROW(decode_full(descriptor, {}), std::runtime_error);
}

TEST(VoxelCodecRos, DecodeHaarWavelet2dMissingLevelsThrows)
{
  auto descriptor = build_grid_descriptor(4, 4, 2);
  descriptor.metadata.erase(voxelcodec_ros::kHaarLevelsKey);
  EXPECT_THROW(decode_full(descriptor, {}), std::runtime_error);
}

// ── 2D haar-wavelet progressive: missing-levels path ───────────────────────

TEST(VoxelCodecRos, DecodeHaarProgressive2dMissingLevelsThrows)
{
  auto descriptor = build_grid_descriptor(4, 4, 2);
  descriptor.metadata.erase(voxelcodec_ros::kHaarLevelsKey);
  EXPECT_THROW(
    voxelcodec_ros::decode_haar_progressive(descriptor, {}, 1), std::runtime_error);
}

// ── 1D haar-wavelet progressive: entirely untested via decode_haar_progressive
// before this, since every other progressive test uses a 2D grid descriptor ──

TEST(VoxelCodecRos, DecodeHaarProgressive1dFullAndPartial)
{
  constexpr int levels = 3;
  const std::vector<std::uint32_t> values{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

  std::vector<std::int64_t> coeffs(values.begin(), values.end());
  std::vector<std::size_t> smooth_lens(levels + 1);
  smooth_lens[0] = values.size();
  for (int i = 1; i <= levels; ++i) {smooth_lens[i] = (smooth_lens[i - 1] + 1) / 2;}
  for (int i = 0; i < levels; ++i) {
    voxelcodec_ros::haar_forward_level(coeffs, smooth_lens[i]);
  }
  const auto payload = voxelcodec_ros::zigzag_varint_encode(coeffs);

  ChannelDescriptor descriptor;
  descriptor.name = "wvmp_x";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint32;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  descriptor.compression = voxelcodec_ros::kCompressionNone;
  descriptor.metadata[voxelcodec_ros::kHaarLevelsKey] = std::to_string(levels);
  descriptor.metadata[voxelcodec_ros::kHaarOriginalLengthKey] = std::to_string(values.size());

  // max_bands=0 takes the "full decode" branch.
  {
    const auto decoded = voxelcodec_ros::decode_haar_progressive(descriptor, payload, 0);
    const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
    EXPECT_EQ(out, values);
  }
  // max_bands beyond levels+1 also takes the "full decode" branch.
  {
    const auto decoded = voxelcodec_ros::decode_haar_progressive(descriptor, payload, 99);
    const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
    EXPECT_EQ(out, values);
  }
  // A genuine partial band count exercises the coarse-reconstruction branch.
  {
    const auto decoded = voxelcodec_ros::decode_haar_progressive(descriptor, payload, 2);
    EXPECT_LT(decoded.descriptor.element_count, values.size());
    EXPECT_GT(decoded.descriptor.element_count, 0U);
  }
  // max_bands=1 is the coarsest possible reconstruction.
  {
    const auto decoded = voxelcodec_ros::decode_haar_progressive(descriptor, payload, 1);
    EXPECT_EQ(decoded.descriptor.element_count, smooth_lens[levels]);
  }
}

// ── decompress_payload direct error paths ──────────────────────────────────

TEST(VoxelCodecRos, DecompressPayloadUnsupportedCompressionThrows)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      "lz4", 1, 0, 0, 4, {}};
  EXPECT_THROW(
    voxelcodec_ros::decompress_payload(descriptor, {0, 0, 0, 0}), std::runtime_error);
}

TEST(VoxelCodecRos, DecompressPayloadCorruptZstdThrows)
{
  ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionZstd, 1, 0, 0, 4, {}};
  const std::vector<std::uint8_t> garbage{0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};
  EXPECT_THROW(voxelcodec_ros::decompress_payload(descriptor, garbage), std::runtime_error);
}

TEST(VoxelCodecRos, DecompressPayloadSizeMismatchThrows)
{
  const std::vector<std::uint8_t> raw_payload{1, 2, 3, 4, 5, 6, 7, 8};
  const auto compressed =
    voxelcodec_ros::compress_payload(voxelcodec_ros::kCompressionZstd, raw_payload);

  ChannelDescriptor descriptor;
  descriptor.name = "v";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint32;
  descriptor.encoding = voxelcodec_ros::kEncodingRawLE;
  descriptor.compression = voxelcodec_ros::kCompressionZstd;
  // Larger (not smaller) than the real size: a smaller declared size would
  // make the destination buffer too small and fail inside ZSTD_decompress
  // itself (the ZSTD_isError branch), never reaching the size-mismatch check.
  descriptor.uncompressed_size = raw_payload.size() + 1;

  EXPECT_THROW(voxelcodec_ros::decompress_payload(descriptor, compressed), std::runtime_error);
}

// ── encode_archive error paths ──────────────────────────────────────────────

TEST(VoxelCodecRos, EncodeArchiveChannelCountMismatchThrows)
{
  voxelcodec_ros::Manifest manifest;
  manifest.channels = {ChannelDescriptor{
      "a", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 0, 0, 0, 0, {}}};
  EXPECT_THROW(voxelcodec_ros::encode_archive(manifest, {}), std::runtime_error);
}

TEST(VoxelCodecRos, EncodeArchiveChannelOrderingMismatchThrows)
{
  voxelcodec_ros::Manifest manifest;
  const ChannelDescriptor a{
      "a", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 0, 0, 0, 0, {}};
  manifest.channels = {a};
  EncodedChannel wrong_name{a, {}};
  wrong_name.descriptor.name = "not-a";
  EXPECT_THROW(voxelcodec_ros::encode_archive(manifest, {wrong_name}), std::runtime_error);
}

TEST(VoxelCodecRos, EncodeDecodeArchiveRoundTripsManifestMetadata)
{
  // Exercises manifest_from_json's "top-level metadata present" branch --
  // every other archive test leaves manifest.metadata empty.
  voxelcodec_ros::Manifest manifest;
  manifest.metadata["session_id"] = "abc123";
  const ChannelDescriptor a{
      "a", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 0, 0, 0, 0, {}};
  manifest.channels = {a};

  const auto archive_bytes = voxelcodec_ros::encode_archive(manifest, {{a, {}}});
  const auto archive = voxelcodec_ros::read_archive(archive_bytes);
  ASSERT_EQ(archive.manifest.metadata.count("session_id"), 1U);
  EXPECT_EQ(archive.manifest.metadata.at("session_id"), "abc123");
}

// ── read_archive error paths ────────────────────────────────────────────────

namespace
{
std::vector<std::uint8_t> build_minimal_archive()
{
  voxelcodec_ros::Manifest manifest;
  const ChannelDescriptor a{
      "a", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  manifest.channels = {a};
  return voxelcodec_ros::encode_archive(manifest, {{a, {1, 2, 3, 4}}});
}
}  // namespace

TEST(VoxelCodecRos, ReadArchiveTooSmallThrows)
{
  const std::vector<std::uint8_t> tiny(5, 0);
  EXPECT_THROW(voxelcodec_ros::read_archive(tiny), std::runtime_error);
}

TEST(VoxelCodecRos, ReadArchiveInvalidMagicThrows)
{
  auto archive_bytes = build_minimal_archive();
  archive_bytes[0] ^= 0xFF;
  EXPECT_THROW(voxelcodec_ros::read_archive(archive_bytes), std::runtime_error);
}

TEST(VoxelCodecRos, ReadArchiveUnsupportedVersionThrows)
{
  auto archive_bytes = build_minimal_archive();
  archive_bytes[4] = 0xFF;
  archive_bytes[5] = 0xFF;
  EXPECT_THROW(voxelcodec_ros::read_archive(archive_bytes), std::runtime_error);
}

TEST(VoxelCodecRos, ReadArchiveManifestLengthOutsideArchiveThrows)
{
  auto archive_bytes = build_minimal_archive();
  // Large enough to blow past the archive's actual size, but far short of
  // UINT64_MAX -- kHeaderSize + manifest_length must not itself wrap back
  // under archive_bytes.size(), which would skip the intended check entirely.
  for (int i = 0; i < 4; ++i) {archive_bytes[8 + i] = 0xFF;}
  for (int i = 4; i < 8; ++i) {archive_bytes[8 + i] = 0x00;}
  EXPECT_THROW(voxelcodec_ros::read_archive(archive_bytes), std::runtime_error);
}

TEST(VoxelCodecRos, ReadArchiveTreatsNonObjectChannelMetadataAsEmpty)
{
  // metadata_from_json's is_object() guard: every other archive test only
  // ever produces well-formed metadata (it's always built from a real
  // Metadata map via encode_archive), so this manually crafts a manifest
  // where a channel's "metadata" field is a JSON string instead of an
  // object -- the only way to exercise this defensive parse path.
  const std::string manifest_json =
    "{\"format\":\"voxel-channel-archive\",\"version\":1,\"voxel_count\":0,"
    "\"channels\":[{\"name\":\"a\",\"semantic\":\"\",\"data_type\":\"uint32\","
    "\"encoding\":\"raw-le\",\"compression\":\"none\",\"element_count\":0,"
    "\"payload_offset\":0,\"compressed_size\":0,\"uncompressed_size\":0,"
    "\"metadata\":\"not-an-object\"}]}";

  std::vector<std::uint8_t> archive_bytes;
  archive_bytes.insert(
    archive_bytes.end(), voxelcodec_ros::kArchiveMagic, voxelcodec_ros::kArchiveMagic + 4);
  push_u16le(archive_bytes, voxelcodec_ros::kArchiveVersion);
  push_u16le(archive_bytes, 0);
  push_u64le(archive_bytes, manifest_json.size());
  archive_bytes.insert(archive_bytes.end(), manifest_json.begin(), manifest_json.end());

  const auto archive = voxelcodec_ros::read_archive(archive_bytes);
  ASSERT_EQ(archive.manifest.channels.size(), 1U);
  EXPECT_TRUE(archive.manifest.channels[0].metadata.empty());
}

TEST(VoxelCodecRos, ReadArchiveChannelPayloadOutsideArchiveThrows)
{
  auto archive_bytes = build_minimal_archive();
  archive_bytes.pop_back();  // last channel's declared payload no longer fits
  EXPECT_THROW(voxelcodec_ros::read_archive(archive_bytes), std::runtime_error);
}

// ── decode_channel: remaining direct error path ─────────────────────────────

TEST(VoxelCodecRos, DecodeChannelUncompressedSizeMismatchThrows)
{
  ChannelDescriptor descriptor;
  descriptor.name = "v";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint32;
  descriptor.encoding = voxelcodec_ros::kEncodingRawLE;
  descriptor.compression = voxelcodec_ros::kCompressionNone;
  descriptor.uncompressed_size = 999;  // doesn't match the payload below
  EXPECT_THROW(
    voxelcodec_ros::decode_channel(descriptor, {1, 2, 3, 4}), std::runtime_error);
}

// ── decode_selected: skip-unrequested / missing-payload / not-found paths ──

TEST(VoxelCodecRos, DecodeSelectedSkipsUnrequestedChannels)
{
  voxelcodec_ros::Manifest manifest;
  const ChannelDescriptor a{
      "a", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  const ChannelDescriptor b{
      "b", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  manifest.channels = {a, b};
  const auto archive_bytes =
    voxelcodec_ros::encode_archive(manifest, {{a, {1, 0, 0, 0}}, {b, {2, 0, 0, 0}}});
  const auto archive = voxelcodec_ros::read_archive(archive_bytes);

  const auto decoded = voxelcodec_ros::decode_selected(archive, {"a"});
  EXPECT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded.count("a"), 1U);
  EXPECT_EQ(decoded.count("b"), 0U);
}

TEST(VoxelCodecRos, DecodeSelectedMissingPayloadThrows)
{
  voxelcodec_ros::Archive archive;
  archive.manifest.channels = {ChannelDescriptor{
      "a", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}}};
  // archive.payloads deliberately left empty.
  EXPECT_THROW(
    voxelcodec_ros::decode_selected(archive, {"a"}), std::runtime_error);
}

TEST(VoxelCodecRos, DecodeSelectedRequestedChannelNotFoundThrows)
{
  voxelcodec_ros::Archive archive;
  archive.manifest.channels = {ChannelDescriptor{
      "a", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}}};
  archive.payloads["a"] = {1, 0, 0, 0};
  EXPECT_THROW(
    voxelcodec_ros::decode_selected(archive, {"does-not-exist"}), std::runtime_error);
}

// ── haar_max_bands ───────────────────────────────────────────────────────────

TEST(VoxelCodecRos, HaarMaxBandsReturnsZeroForNonHaarEncoding)
{
  const ChannelDescriptor descriptor{
      "v", "", voxelcodec_ros::kDataTypeUint32, voxelcodec_ros::kEncodingRawLE,
      voxelcodec_ros::kCompressionNone, 1, 0, 0, 0, {}};
  EXPECT_EQ(voxelcodec_ros::haar_max_bands(descriptor), 0);
}

TEST(VoxelCodecRos, HaarMaxBandsReturnsLevelsPlusOne)
{
  ChannelDescriptor descriptor;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  descriptor.metadata[voxelcodec_ros::kHaarLevelsKey] = "3";
  EXPECT_EQ(voxelcodec_ros::haar_max_bands(descriptor), 4);
}

TEST(VoxelCodecRos, HaarMaxBandsDefaultsToZeroLevelsWhenMetadataMissing)
{
  ChannelDescriptor descriptor;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  EXPECT_EQ(voxelcodec_ros::haar_max_bands(descriptor), 1);
}

// ── decode_haar_progressive dispatch ─────────────────────────────────────────

TEST(VoxelCodecRos, DecodeHaarProgressiveNonHaarDelegatesToDecodeChannel)
{
  ChannelDescriptor descriptor;
  descriptor.name = "v";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint32;
  descriptor.encoding = voxelcodec_ros::kEncodingRawLE;
  descriptor.compression = voxelcodec_ros::kCompressionNone;
  descriptor.uncompressed_size = 4;
  const std::vector<std::uint8_t> payload{9, 0, 0, 0};

  const auto decoded = voxelcodec_ros::decode_haar_progressive(descriptor, payload, 1);
  const auto out = std::get<std::vector<std::uint32_t> >(decoded.values);
  EXPECT_EQ(out, (std::vector<std::uint32_t>{9}));
}

TEST(VoxelCodecRos, DecodeHaarProgressiveWrongDataTypeThrows)
{
  ChannelDescriptor descriptor;
  descriptor.name = "v";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint8;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  descriptor.compression = voxelcodec_ros::kCompressionNone;
  EXPECT_THROW(
    voxelcodec_ros::decode_haar_progressive(descriptor, {}, 1), std::runtime_error);
}

// ── compute_haar_level_dims / reconstruct_haar_from_bands: direct validation ─

TEST(VoxelCodecRos, ComputeHaarLevelDimsRejectsInvalidLevels)
{
  EXPECT_THROW(voxelcodec_ros::compute_haar_level_dims(4, 4, 0), std::runtime_error);
}

TEST(VoxelCodecRos, ReconstructHaarFromBandsRejectsInvalidLevelsOrDimensions)
{
  EXPECT_THROW(
    voxelcodec_ros::reconstruct_haar_from_bands({}, 4, 4, 0, 1), std::runtime_error);
  EXPECT_THROW(
    voxelcodec_ros::reconstruct_haar_from_bands({}, 0, 4, 2, 1), std::runtime_error);
  EXPECT_THROW(
    voxelcodec_ros::reconstruct_haar_from_bands({}, 4, 0, 2, 1), std::runtime_error);
}

TEST(VoxelCodecRos, ReconstructHaarFromBandsRejectsMissingOrWrongSizeLlBand)
{
  EXPECT_THROW(
    voxelcodec_ros::reconstruct_haar_from_bands({}, 8, 8, 2, 3), std::runtime_error);

  const auto layout = voxelcodec_ros::compute_haar_band_layout(8, 8, 2);
  std::vector<std::vector<std::int64_t> > bad_ll = {
    std::vector<std::int64_t>(layout[0].element_count + 1, 0)};
  EXPECT_THROW(
    voxelcodec_ros::reconstruct_haar_from_bands(bad_ll, 8, 8, 2, 3), std::runtime_error);
}

TEST(VoxelCodecRos, ReconstructHaarFromBandsRejectsMissingDetailBand)
{
  const auto layout = voxelcodec_ros::compute_haar_band_layout(8, 8, 2);
  // Band 0 (LL) and band 1 present and correctly sized; band 2 missing
  // entirely, but bands_received=3 asks for the full reconstruction.
  std::vector<std::vector<std::int64_t> > bands = {
    std::vector<std::int64_t>(layout[0].element_count, 0),
    std::vector<std::int64_t>(layout[1].element_count, 0)};
  EXPECT_THROW(
    voxelcodec_ros::reconstruct_haar_from_bands(bands, 8, 8, 2, 3), std::runtime_error);
}

// ── haar_forward.hpp: make_haar_bands validation paths ─────────────────────

TEST(VoxelCodecRos, MakeHaarBandsRejectsInvalidLevels)
{
  const std::vector<std::uint32_t> values(16, 0);
  EXPECT_THROW(
    voxelcodec_ros::make_haar_bands(values, 4, 4, 0, "none"), std::runtime_error);
}

TEST(VoxelCodecRos, MakeHaarBandsRejectsEmptyGridDimensions)
{
  const std::vector<std::uint32_t> values;
  EXPECT_THROW(
    voxelcodec_ros::make_haar_bands(values, 0, 4, 1, "none"), std::runtime_error);
  EXPECT_THROW(
    voxelcodec_ros::make_haar_bands(values, 4, 0, 1, "none"), std::runtime_error);
}

TEST(VoxelCodecRos, MakeHaarBandsRejectsValuesSizeMismatch)
{
  const std::vector<std::uint32_t> values(15, 0);  // 4x4 needs 16
  EXPECT_THROW(
    voxelcodec_ros::make_haar_bands(values, 4, 4, 1, "none"), std::runtime_error);
}