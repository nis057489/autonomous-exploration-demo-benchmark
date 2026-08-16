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