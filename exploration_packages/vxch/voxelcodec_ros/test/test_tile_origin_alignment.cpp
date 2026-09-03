#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "voxelcodec_ros/tile_reconstructor.hpp"
#include "voxelcodec_ros/tile_scheduler.hpp"

namespace
{

using voxelcodec_ros::Metadata;
using voxelcodec_ros::Stamp;
using voxelcodec_ros::TileReconstructor;
using voxelcodec_ros::TileScheduler;

// Drives the REAL sender (TileScheduler::ingest_grid) into the REAL receiver
// (TileReconstructor) and returns the reconstructed grid, so the round trip
// exercises the exact metadata the production encoder actually emits.
std::vector<std::int8_t> round_trip(
  const std::vector<std::int8_t> & grid, int w, int h, double resolution,
  double origin_x, double origin_y, int levels, double tile_size_m)
{
  TileScheduler scheduler(tile_size_m, levels, "none", true, "smart");
  scheduler.ingest_grid(grid, w, h, resolution, origin_x, origin_y);

  TileReconstructor reconstructor(levels);
  const Metadata manifest{
    {"grid_width", std::to_string(w)},
    {"grid_height", std::to_string(h)},
    {"tile_size_cells", std::to_string(scheduler.tile_size_cells())},
    {"resolution", std::to_string(resolution)},
    {"origin_x", std::to_string(origin_x)},
    {"origin_y", std::to_string(origin_y)},
    {"frame_id", "map"},
  };
  EXPECT_TRUE(reconstructor.ingest_manifest(manifest, Stamp{}));

  // Drain every pending band so the receiver has a COMPLETE picture -- any
  // difference left after this is a placement/coverage bug, not progressive
  // transmission still being in flight.
  for (int i = 0; i < 100 && scheduler.has_pending(); ++i) {
    for (auto & item : scheduler.take_pending_bands(100, -1)) {
      const auto err =
        reconstructor.ingest_band(item.band_index, item.channel.descriptor, item.channel.payload);
      EXPECT_FALSE(err.has_value()) << *err;
    }
  }

  const auto out = reconstructor.reconstruct();
  EXPECT_TRUE(out.has_value());
  return out->data;
}

std::vector<std::int8_t> make_grid(int w, int h)
{
  std::vector<std::int8_t> v(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  for (std::size_t i = 0; i < v.size(); ++i) {
    v[i] = static_cast<std::int8_t>(i % 100);
  }
  return v;
}

}  // namespace

// Baseline: origin sits exactly on the tile lattice, so every tile is full-size.
TEST(OriginAlignment, TileAlignedOriginRoundTripsExactly)
{
  const int w = 16, h = 16;
  const auto grid = make_grid(w, h);
  // resolution 1.0, tile_size_m 4.0 -> tile_size_cells 4; origin 8.0 is a
  // multiple of 4 cells, so all spans are full tiles.
  const auto out = round_trip(grid, w, h, 1.0, 8.0, 8.0, 2, 4.0);
  EXPECT_EQ(out, grid);
}

// The real case: SLAM origins move continuously and do NOT snap to the tile
// lattice, so the first span on each axis is a PARTIAL tile.
TEST(OriginAlignment, MisalignedOriginRoundTripsExactly)
{
  const int w = 16, h = 16;
  const auto grid = make_grid(w, h);
  // origin 9.0 cells = lattice boundary 8 + 1: first column/row span is a
  // 3-cell partial tile, the rest are full 4-cell tiles.
  const auto out = round_trip(grid, w, h, 1.0, 9.0, 9.0, 2, 4.0);

  int mismatches = 0;
  int first_bad_col = -1, first_bad_row = -1;
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) {
      const std::size_t i = static_cast<std::size_t>(r) * w + c;
      if (out[i] != grid[i]) {
        ++mismatches;
        if (first_bad_col < 0) {first_bad_row = r; first_bad_col = c;}
      }
    }
  }
  EXPECT_EQ(mismatches, 0)
    << "first mismatch at row " << first_bad_row << " col " << first_bad_col
    << " (of " << w * h << " cells)";
}

// Sweep every misalignment phase against the tile lattice, on both axes and
// at a few grid sizes -- the edge band only appears for particular
// origin-vs-tile-boundary phases, so a single origin can pass by luck.
TEST(OriginAlignment, EveryOriginPhaseRoundTripsExactly)
{
  for (const int w : {13, 16, 21}) {
    for (const int h : {12, 16, 19}) {
      const auto grid = make_grid(w, h);
      for (int phase_x = 0; phase_x < 4; ++phase_x) {
        for (int phase_y = 0; phase_y < 4; ++phase_y) {
          const auto out = round_trip(
            grid, w, h, 1.0, 8.0 + phase_x, 8.0 + phase_y, 2, 4.0);
          EXPECT_EQ(out, grid)
            << "w=" << w << " h=" << h
            << " phase_x=" << phase_x << " phase_y=" << phase_y;
        }
      }
    }
  }
}

// Negative origins exercise the floor_div path (tile indices go negative),
// where truncating division would land tiles a whole tile off.
TEST(OriginAlignment, NegativeMisalignedOriginRoundTripsExactly)
{
  const int w = 16, h = 16;
  const auto grid = make_grid(w, h);
  for (int phase = 0; phase < 4; ++phase) {
    const auto out = round_trip(grid, w, h, 1.0, -10.0 + phase, -7.0 - phase, 2, 4.0);
    EXPECT_EQ(out, grid) << "phase=" << phase;
  }
}

// The scenario that actually produced the artifact: the map grows leftward
// (origin decreases) between ingests while the tile lattice stays fixed, so a
// tile's clipped extent and in-tile offset change even though the terrain
// under it does not.
TEST(OriginAlignment, OriginShiftBetweenIngestsStillReconstructsExactly)
{
  const int levels = 2;
  const double resolution = 1.0, tile_size_m = 4.0;
  const int w = 16, h = 16;
  const auto grid = make_grid(w, h);

  TileScheduler scheduler(tile_size_m, levels, "none", true, "smart");
  TileReconstructor reconstructor(levels);

  // Same content, origin sliding left one cell at a time.
  for (int step = 0; step < 4; ++step) {
    const double origin_x = 9.0 - step;
    const double origin_y = 9.0;
    scheduler.ingest_grid(grid, w, h, resolution, origin_x, origin_y);

    const Metadata manifest{
      {"grid_width", std::to_string(w)},
      {"grid_height", std::to_string(h)},
      {"tile_size_cells", std::to_string(scheduler.tile_size_cells())},
      {"resolution", std::to_string(resolution)},
      {"origin_x", std::to_string(origin_x)},
      {"origin_y", std::to_string(origin_y)},
      {"frame_id", "map"},
    };
    ASSERT_TRUE(reconstructor.ingest_manifest(manifest, Stamp{}));

    for (int i = 0; i < 100 && scheduler.has_pending(); ++i) {
      for (auto & item : scheduler.take_pending_bands(100, -1)) {
        reconstructor.ingest_band(
          item.band_index, item.channel.descriptor, item.channel.payload);
      }
    }

    const auto out = reconstructor.reconstruct();
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->data, grid) << "step=" << step << " origin_x=" << origin_x;
  }
}
