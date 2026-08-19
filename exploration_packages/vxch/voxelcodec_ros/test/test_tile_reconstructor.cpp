#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "voxelcodec_ros/haar_forward.hpp"
#include "voxelcodec_ros/tile_reconstructor.hpp"

namespace
{

using voxelcodec_ros::ChannelDescriptor;
using voxelcodec_ros::EncodedChannel;
using voxelcodec_ros::Metadata;
using voxelcodec_ros::Stamp;
using voxelcodec_ros::TileReconstructor;

// Mirrors occupancy_grid_vxch_node's shift_to_uint32 (kept private to that
// header's TileScheduler, so duplicated here for building encoder-side test
// fixtures) -- the inverse of unshift_from_uint32 under test.
std::uint32_t shift_to_uint32(std::int8_t v)
{
  return static_cast<std::uint32_t>(static_cast<int>(v) + 1);
}

Metadata manifest_metadata(
  std::uint32_t width, std::uint32_t height, int tile_size_cells, double resolution = 1.0,
  const std::string & frame_id = "map", double origin_x = 0.0, double origin_y = 0.0)
{
  return {
    {"grid_width", std::to_string(width)},
    {"grid_height", std::to_string(height)},
    {"tile_size_cells", std::to_string(tile_size_cells)},
    {"resolution", std::to_string(resolution)},
    {"origin_x", std::to_string(origin_x)},
    {"origin_y", std::to_string(origin_y)},
    {"frame_id", frame_id},
  };
}

// Encodes a tile the same way occupancy_grid_vxch_node does: shift to
// uint32, run make_haar_bands, tag each band's descriptor with tile
// placement metadata.
std::vector<EncodedChannel> encode_tile(
  const std::vector<std::int8_t> & occupancy, int width, int height, int levels,
  int tile_row, int tile_col, int tile_size_cells)
{
  std::vector<std::uint32_t> values(occupancy.size());
  for (std::size_t i = 0; i < occupancy.size(); ++i) {values[i] = shift_to_uint32(occupancy[i]);}
  auto bands = voxelcodec_ros::make_haar_bands(
    values, static_cast<std::size_t>(width), static_cast<std::size_t>(height), levels, "none");
  for (auto & band : bands) {
    band.descriptor.metadata["tile_row"] = std::to_string(tile_row);
    band.descriptor.metadata["tile_col"] = std::to_string(tile_col);
    band.descriptor.metadata["tile_width"] = std::to_string(width);
    band.descriptor.metadata["tile_height"] = std::to_string(height);
    band.descriptor.metadata["tile_size_cells"] = std::to_string(tile_size_cells);
  }
  return bands;
}

std::vector<std::int8_t> make_occupancy(int w, int h, int seed = 0)
{
  std::vector<std::int8_t> values(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i] = static_cast<std::int8_t>((static_cast<int>(i) * 5 + seed * 11) % 100 - 1);
  }
  return values;
}

}  // namespace

TEST(TileReconstructor, UnshiftFromUint32RoundTripsShiftToUint32Range)
{
  EXPECT_EQ(voxelcodec_ros::unshift_from_uint32(0), -1);
  EXPECT_EQ(voxelcodec_ros::unshift_from_uint32(1), 0);
  EXPECT_EQ(voxelcodec_ros::unshift_from_uint32(101), 100);
}

TEST(TileReconstructor, UnshiftFromUint32ClampsOutOfRangeValues)
{
  // Values beyond the encoder's legitimate [0,101] range (e.g. from a
  // corrupted/foreign payload) must still clamp into OccupancyGrid's valid
  // [-1,100] range rather than wrapping or producing garbage.
  EXPECT_EQ(voxelcodec_ros::unshift_from_uint32(5000), 100);
}

TEST(TileReconstructor, IngestManifestRejectsMissingGridDimensions)
{
  TileReconstructor reconstructor(2);
  EXPECT_FALSE(reconstructor.ingest_manifest({{"resolution", "0.1"}}, Stamp{}));
  EXPECT_FALSE(reconstructor.geometry().have_manifest);
}

TEST(TileReconstructor, IngestManifestParsesGeometry)
{
  TileReconstructor reconstructor(2);
  const bool ok = reconstructor.ingest_manifest(manifest_metadata(8, 8, 4, 0.5, "odom"), Stamp{7, 42});
  EXPECT_TRUE(ok);
  const auto & geom = reconstructor.geometry();
  EXPECT_TRUE(geom.have_manifest);
  EXPECT_EQ(geom.grid_width, 8U);
  EXPECT_EQ(geom.grid_height, 8U);
  EXPECT_EQ(geom.tile_size_cells, 4);
  EXPECT_FLOAT_EQ(geom.grid_resolution, 0.5F);
  EXPECT_EQ(geom.frame_id, "odom");
  EXPECT_EQ(geom.manifest_stamp.sec, 7);
  EXPECT_EQ(geom.manifest_stamp.nanosec, 42U);
}

TEST(TileReconstructor, ReconstructReturnsNulloptWithoutManifest)
{
  TileReconstructor reconstructor(2);
  EXPECT_FALSE(reconstructor.reconstruct().has_value());
}

TEST(TileReconstructor, ReconstructReturnsNulloptWithNoBandsReceived)
{
  TileReconstructor reconstructor(2);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{});
  EXPECT_FALSE(reconstructor.reconstruct().has_value());
}

TEST(TileReconstructor, IngestBandBeforeManifestIsIgnored)
{
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  const auto occupancy = make_occupancy(4, 4);
  const auto bands = encode_tile(occupancy, 4, 4, levels, 0, 0, 4);

  // No manifest ingested yet -- ingest_band must no-op, not crash or record state.
  const auto error = reconstructor.ingest_band(0, bands[0].descriptor, bands[0].payload);
  EXPECT_FALSE(error.has_value());

  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{});
  EXPECT_FALSE(reconstructor.reconstruct().has_value());  // band was never actually recorded
}

TEST(TileReconstructor, FullBandRoundTripReconstructsExactGrid)
{
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{3, 500});

  const auto occupancy = make_occupancy(4, 4);
  const auto bands = encode_tile(occupancy, 4, 4, levels, 0, 0, 4);
  for (std::size_t k = 0; k < bands.size(); ++k) {
    const auto error = reconstructor.ingest_band(
      static_cast<int>(k), bands[k].descriptor, bands[k].payload);
    EXPECT_FALSE(error.has_value()) << "band " << k;
  }

  const auto grid = reconstructor.reconstruct();
  ASSERT_TRUE(grid.has_value());
  EXPECT_EQ(grid->width, 4U);
  EXPECT_EQ(grid->height, 4U);
  EXPECT_EQ(grid->stamp.sec, 3);
  EXPECT_EQ(grid->stamp.nanosec, 500U);
  ASSERT_EQ(grid->data.size(), occupancy.size());
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    EXPECT_EQ(grid->data[i], occupancy[i]) << "cell " << i;
  }
}

TEST(TileReconstructor, PartialBandsProduceUpsampledCoarseReconstruction)
{
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{});

  const auto occupancy = make_occupancy(4, 4);
  const auto bands = encode_tile(occupancy, 4, 4, levels, 0, 0, 4);
  // Only the coarsest band (band 0) arrives.
  const auto error = reconstructor.ingest_band(0, bands[0].descriptor, bands[0].payload);
  EXPECT_FALSE(error.has_value());

  const auto grid = reconstructor.reconstruct();
  ASSERT_TRUE(grid.has_value());
  // Still upsampled to full grid dimensions -- just blurrier content.
  EXPECT_EQ(grid->width, 4U);
  EXPECT_EQ(grid->height, 4U);
  EXPECT_EQ(grid->data.size(), 16U);
}

TEST(TileReconstructor, NonContiguousBandsStopAtFirstGap)
{
  // Band 1 arriving without band 0 must not count toward "bands received"
  // (reconstruction requires a contiguous coarsest-first prefix) -- receiving
  // it should leave the tile exactly as un-reconstructable as having nothing.
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{});
  const auto occupancy = make_occupancy(4, 4);
  const auto bands = encode_tile(occupancy, 4, 4, levels, 0, 0, 4);

  reconstructor.ingest_band(1, bands[1].descriptor, bands[1].payload);
  EXPECT_FALSE(reconstructor.reconstruct().has_value());
}

TEST(TileReconstructor, IngestBandDecodeFailureReturnsErrorWithoutThrowing)
{
  TileReconstructor reconstructor(2);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{});

  ChannelDescriptor descriptor;
  descriptor.name = "band_0";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint32;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  descriptor.compression = voxelcodec_ros::kCompressionZstd;
  descriptor.uncompressed_size = 4;
  // Deliberately missing kHaarEncodingKey metadata, and not valid zstd
  // either -- decompress_payload should throw first, caught internally.
  const std::vector<std::uint8_t> garbage{0xDE, 0xAD, 0xBE, 0xEF};

  const auto error = reconstructor.ingest_band(0, descriptor, garbage);
  ASSERT_TRUE(error.has_value());
  EXPECT_FALSE(error->empty());
}

TEST(TileReconstructor, ManifestWidthHeightGrowthPreservesAccumulatedTiles)
{
  // Grid growth alone (origin + tile_size_cells unchanged) must NOT drop
  // already-decoded tile state -- tile_row/tile_col are pure array-index
  // offsets, so growing grid_width/grid_height doesn't change what world
  // location an existing tile's data belongs to. Regression test for a bug
  // where production SLAM's routine grid growth (every few seconds, origin
  // usually unchanged) wiped a peer's entire already-known map.
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{1, 0});
  const auto occupancy = make_occupancy(4, 4);
  const auto bands = encode_tile(occupancy, 4, 4, levels, 0, 0, 4);
  for (std::size_t k = 0; k < bands.size(); ++k) {
    reconstructor.ingest_band(static_cast<int>(k), bands[k].descriptor, bands[k].payload);
  }
  ASSERT_TRUE(reconstructor.reconstruct().has_value());

  // Grid grew (e.g. SLAM explored new area); origin and tile_size_cells unchanged.
  reconstructor.ingest_manifest(manifest_metadata(8, 8, 4), Stamp{2, 0});

  const auto grid = reconstructor.reconstruct();
  ASSERT_TRUE(grid.has_value());
  EXPECT_EQ(grid->width, 8U);
  EXPECT_EQ(grid->height, 8U);
  // Original tile's decoded content must still be present at its same offset.
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      EXPECT_EQ(
        grid->data[static_cast<std::size_t>(r) * 8 + static_cast<std::size_t>(c)],
        occupancy[static_cast<std::size_t>(r) * 4 + static_cast<std::size_t>(c)])
        << "cell (" << r << "," << c << ")";
    }
  }
  // The newly-added region (never covered by any tile) reads as unknown.
  EXPECT_EQ(grid->data[4], -1);          // row 0, col 4
  EXPECT_EQ(grid->data[4 * 8 + 0], -1);  // row 4, col 0
}

TEST(TileReconstructor, ManifestOriginChangeClearsAccumulatedTiles)
{
  // An actual origin shift reindexes what world location array position
  // (0,0) corresponds to -- existing tile_row/tile_col keys no longer mean
  // the same place, so accumulated tile state must still be dropped.
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(
    manifest_metadata(4, 4, 4, 1.0, "map", 0.0, 0.0), Stamp{1, 0});
  const auto occupancy = make_occupancy(4, 4);
  const auto bands = encode_tile(occupancy, 4, 4, levels, 0, 0, 4);
  for (std::size_t k = 0; k < bands.size(); ++k) {
    reconstructor.ingest_band(static_cast<int>(k), bands[k].descriptor, bands[k].payload);
  }
  ASSERT_TRUE(reconstructor.reconstruct().has_value());

  // Same grid dimensions and tile_size_cells -- only origin moved.
  reconstructor.ingest_manifest(
    manifest_metadata(4, 4, 4, 1.0, "map", 0.5, 0.0), Stamp{2, 0});
  EXPECT_FALSE(reconstructor.reconstruct().has_value());
}

TEST(TileReconstructor, ManifestTileSizeChangeClearsAccumulatedTiles)
{
  // Same dimensions/origin -- only tile_size_cells changes (e.g. a resolution
  // rescale) -- this genuinely invalidates tile_row/tile_col addressing and
  // must still clear, unlike plain width/height growth above.
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(8, 8, 4), Stamp{1, 0});
  const auto occupancy = make_occupancy(4, 4);
  const auto bands = encode_tile(occupancy, 4, 4, levels, 0, 0, 4);
  for (std::size_t k = 0; k < bands.size(); ++k) {
    reconstructor.ingest_band(static_cast<int>(k), bands[k].descriptor, bands[k].payload);
  }
  ASSERT_TRUE(reconstructor.reconstruct().has_value());

  reconstructor.ingest_manifest(manifest_metadata(8, 8, 2), Stamp{2, 0});
  EXPECT_FALSE(reconstructor.reconstruct().has_value());
}

TEST(TileReconstructor, ManifestStampOnlyChangeKeepsAccumulatedTiles)
{
  // Bands are sent progressively (not necessarily under the same manifest
  // stamp that queued them) -- a fresher manifest stamp with IDENTICAL
  // geometry must not discard already-decoded bands.
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{1, 0});
  const auto occupancy = make_occupancy(4, 4);
  const auto bands = encode_tile(occupancy, 4, 4, levels, 0, 0, 4);
  for (std::size_t k = 0; k < bands.size(); ++k) {
    reconstructor.ingest_band(static_cast<int>(k), bands[k].descriptor, bands[k].payload);
  }
  ASSERT_TRUE(reconstructor.reconstruct().has_value());

  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{2, 0});
  EXPECT_TRUE(reconstructor.reconstruct().has_value());
}

TEST(TileReconstructor, IngestBandFixedWidthEncodingRoundTrips)
{
  // Every other round-trip test uses make_haar_bands' default varint packing
  // -- this is the only test exercising fixed_width_decode (the ablation
  // counterpart, selected via kHaarEncodingKey="fixed_width").
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{});

  const auto occupancy = make_occupancy(4, 4, /*seed=*/1);
  std::vector<std::uint32_t> values(occupancy.size());
  for (std::size_t i = 0; i < occupancy.size(); ++i) {values[i] = shift_to_uint32(occupancy[i]);}
  auto bands = voxelcodec_ros::make_haar_bands(
    values, 4, 4, levels, "none", voxelcodec_ros::kHaarEncodingFixedWidth);
  for (auto & band : bands) {
    band.descriptor.metadata["tile_row"] = "0";
    band.descriptor.metadata["tile_col"] = "0";
    band.descriptor.metadata["tile_width"] = "4";
    band.descriptor.metadata["tile_height"] = "4";
    band.descriptor.metadata["tile_size_cells"] = "4";
  }

  for (std::size_t k = 0; k < bands.size(); ++k) {
    const auto error = reconstructor.ingest_band(
      static_cast<int>(k), bands[k].descriptor, bands[k].payload);
    EXPECT_FALSE(error.has_value()) << "band " << k;
  }

  const auto grid = reconstructor.reconstruct();
  ASSERT_TRUE(grid.has_value());
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    EXPECT_EQ(grid->data[i], occupancy[i]) << "cell " << i;
  }
}

TEST(TileReconstructor, IngestBandSparseRleEncodingRoundTrips)
{
  // Mirrors IngestBandFixedWidthEncodingRoundTrips above, for the 3rd
  // packing scheme -- validates the actual production TileReconstructor::
  // ingest_band 3-way branch on kHaarEncodingKey="sparse_rle", not just
  // sparse_rle_encode/decode in isolation.
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{});

  const auto occupancy = make_occupancy(4, 4, /*seed=*/1);
  std::vector<std::uint32_t> values(occupancy.size());
  for (std::size_t i = 0; i < occupancy.size(); ++i) {values[i] = shift_to_uint32(occupancy[i]);}
  auto bands = voxelcodec_ros::make_haar_bands(
    values, 4, 4, levels, "none", voxelcodec_ros::kHaarEncodingSparseRle);
  for (auto & band : bands) {
    band.descriptor.metadata["tile_row"] = "0";
    band.descriptor.metadata["tile_col"] = "0";
    band.descriptor.metadata["tile_width"] = "4";
    band.descriptor.metadata["tile_height"] = "4";
    band.descriptor.metadata["tile_size_cells"] = "4";
  }

  for (std::size_t k = 0; k < bands.size(); ++k) {
    const auto error = reconstructor.ingest_band(
      static_cast<int>(k), bands[k].descriptor, bands[k].payload);
    EXPECT_FALSE(error.has_value()) << "band " << k;
  }

  const auto grid = reconstructor.reconstruct();
  ASSERT_TRUE(grid.has_value());
  for (std::size_t i = 0; i < occupancy.size(); ++i) {
    EXPECT_EQ(grid->data[i], occupancy[i]) << "cell " << i;
  }
}

TEST(TileReconstructor, IngestBandRejectsOutOfRangeBandIndex)
{
  constexpr int levels = 2;  // 3 total bands: 0, 1, 2
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{});
  const auto occupancy = make_occupancy(4, 4);
  const auto bands = encode_tile(occupancy, 4, 4, levels, 0, 0, 4);

  // Band index 99 doesn't exist for a 3-band tile -- must be a silent no-op,
  // not an out-of-bounds access.
  const auto error = reconstructor.ingest_band(99, bands[0].descriptor, bands[0].payload);
  EXPECT_FALSE(error.has_value());
  EXPECT_FALSE(reconstructor.reconstruct().has_value());
}

TEST(TileReconstructor, ReconstructSkipsTileWhenBandSizeMismatchCausesThrow)
{
  // Craft a band 0 whose declared element_count doesn't match the real LL
  // layout size for a 4x4/levels=2 tile -- ingest_band's own decode succeeds
  // (it just trusts element_count), but reconstruct_haar_from_bands must
  // reject the resulting wrong-sized coefficient vector. reconstruct() should
  // catch that and simply skip the tile, not propagate the exception.
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(4, 4, 4), Stamp{});

  ChannelDescriptor descriptor;
  descriptor.name = "band_0";
  descriptor.data_type = voxelcodec_ros::kDataTypeUint32;
  descriptor.encoding = voxelcodec_ros::kEncodingHaarWavelet;
  descriptor.compression = voxelcodec_ros::kCompressionNone;
  descriptor.metadata["tile_row"] = "0";
  descriptor.metadata["tile_col"] = "0";
  descriptor.metadata["tile_width"] = "4";
  descriptor.metadata["tile_height"] = "4";
  descriptor.metadata["tile_size_cells"] = "4";
  descriptor.metadata[voxelcodec_ros::kHaarEncodingKey] = voxelcodec_ros::kHaarEncodingVarint;

  // A 4x4 grid at levels=2 halves to 1x1 (LL band 0 needs exactly 1
  // coefficient) -- claim 5 instead, so the decoded vector's size disagrees
  // with reconstruct_haar_from_bands' expected layout.
  descriptor.element_count = 5;
  const auto payload = voxelcodec_ros::zigzag_varint_encode(std::vector<std::int64_t>(5, 0));

  const auto error = reconstructor.ingest_band(0, descriptor, payload);
  EXPECT_FALSE(error.has_value());  // decode itself succeeds -- only reconstruct() detects the mismatch
  EXPECT_FALSE(reconstructor.reconstruct().has_value());
}

TEST(TileReconstructor, MultiTileReconstructionPlacesEachTileCorrectly)
{
  // Two side-by-side 4x4 tiles forming an 8x4 grid; tile (0,0) all "free" (0),
  // tile (0,1) all "occupied" (100) -- reconstruction must place each tile's
  // content at its own offset, not overwrite or swap them.
  constexpr int levels = 2;
  TileReconstructor reconstructor(levels);
  reconstructor.ingest_manifest(manifest_metadata(8, 4, 4), Stamp{});

  const std::vector<std::int8_t> left_tile(16, 0);
  const std::vector<std::int8_t> right_tile(16, 100);
  const auto left_bands = encode_tile(left_tile, 4, 4, levels, 0, 0, 4);
  const auto right_bands = encode_tile(right_tile, 4, 4, levels, 0, 1, 4);
  for (std::size_t k = 0; k < left_bands.size(); ++k) {
    reconstructor.ingest_band(static_cast<int>(k), left_bands[k].descriptor, left_bands[k].payload);
    reconstructor.ingest_band(
      static_cast<int>(k), right_bands[k].descriptor, right_bands[k].payload);
  }

  const auto grid = reconstructor.reconstruct();
  ASSERT_TRUE(grid.has_value());
  ASSERT_EQ(grid->data.size(), 32U);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      EXPECT_EQ(grid->data[static_cast<std::size_t>(r * 8 + c)], 0) << "left tile (" << r << "," << c << ")";
      EXPECT_EQ(grid->data[static_cast<std::size_t>(r * 8 + 4 + c)], 100)
        << "right tile (" << r << "," << c << ")";
    }
  }
}
