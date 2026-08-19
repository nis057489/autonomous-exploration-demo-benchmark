#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <set>
#include <string>
#include <vector>

#include "voxelcodec_ros/tile_scheduler.hpp"

namespace
{

using voxelcodec_ros::TileKey;
using voxelcodec_ros::TileScheduler;

// A flat grid where cell value cycles through a small pattern -- gives
// make_haar_bands something non-degenerate to encode without needing real
// occupancy-grid semantics.
std::vector<std::int8_t> make_grid(int w, int h, int seed = 0)
{
  std::vector<std::int8_t> values(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  for (std::size_t i = 0; i < values.size(); ++i) {
    values[i] = static_cast<std::int8_t>(
      (static_cast<int>(i) * 7 + seed * 13) % 100 - 1);  // stays in [-1, 98]
  }
  return values;
}

}  // namespace

TEST(TileScheduler, ShiftToUint32)
{
  EXPECT_EQ(voxelcodec_ros::shift_to_uint32(-1), 0U);
  EXPECT_EQ(voxelcodec_ros::shift_to_uint32(0), 1U);
  EXPECT_EQ(voxelcodec_ros::shift_to_uint32(100), 101U);
}

TEST(TileScheduler, ComputeTileGeomFullTile)
{
  const auto geom = voxelcodec_ros::compute_tile_geom(0, 0, 4, 8, 8);
  EXPECT_EQ(geom.row0, 0);
  EXPECT_EQ(geom.col0, 0);
  EXPECT_EQ(geom.width, 4);
  EXPECT_EQ(geom.height, 4);
}

TEST(TileScheduler, ComputeTileGeomClampsAtGridEdge)
{
  // A 10-cell-wide grid with tile_size_cells=4: tile column 2 starts at x=8,
  // but only 2 columns remain before the grid edge.
  const auto geom = voxelcodec_ros::compute_tile_geom(0, 2, 4, 10, 10);
  EXPECT_EQ(geom.col0, 8);
  EXPECT_EQ(geom.width, 2);
  EXPECT_EQ(geom.height, 4);
}

TEST(TileScheduler, IngestGridRejectsSizeMismatch)
{
  TileScheduler scheduler(4.0, 2, "none", true, "smart");
  const std::vector<std::int8_t> values(15, 0);  // 4x4 grid needs 16
  EXPECT_THROW(scheduler.ingest_grid(values, 4, 4, 1.0), std::runtime_error);
}

TEST(TileScheduler, IngestGridFirstCallQueuesChangedBands)
{
  // tile_size_m=4, resolution=1 -> tile_size_cells=4; a 4x4 grid is exactly one tile.
  TileScheduler scheduler(4.0, 2, "none", true, "smart");
  const auto result = scheduler.ingest_grid(make_grid(4, 4), 4, 4, 1.0);
  EXPECT_GT(result.total_changed, 0U);
  EXPECT_EQ(result.queued_tiles, 1U);
  EXPECT_TRUE(result.tile_errors.empty());
  EXPECT_TRUE(scheduler.has_pending());
}

TEST(TileScheduler, SmartModeSkipsUnchangedContentOnReingest)
{
  TileScheduler scheduler(4.0, 2, "none", true, "smart");
  const auto grid = make_grid(4, 4);
  const auto first = scheduler.ingest_grid(grid, 4, 4, 1.0);
  ASSERT_GT(first.total_changed, 0U);

  // Drain everything queued from the first ingest before re-ingesting the
  // identical grid, so any leftover pending bands can't masquerade as "still
  // changed" -- the assertion below is specifically about ingest_grid's own
  // fingerprint check, not about undrained backlog.
  scheduler.take_pending_bands(/*max_bands_per_update=*/100, /*max_tiles_per_update=*/-1);

  const auto second = scheduler.ingest_grid(grid, 4, 4, 1.0);
  EXPECT_EQ(second.total_changed, 0U);
  EXPECT_FALSE(scheduler.has_pending());
}

TEST(TileScheduler, SimpleModeAlwaysRequeuesEverything)
{
  TileScheduler scheduler(4.0, 2, "none", true, "simple");
  const auto grid = make_grid(4, 4);
  const auto first = scheduler.ingest_grid(grid, 4, 4, 1.0);
  scheduler.take_pending_bands(100, -1);

  const auto second = scheduler.ingest_grid(grid, 4, 4, 1.0);
  // Simple mode never consults fingerprints -- identical content still re-queues.
  EXPECT_EQ(second.total_changed, first.total_changed);
}

TEST(TileScheduler, TilesAreFingerprintedIndependently)
{
  // 8x4 grid, tile_size_cells=4 -> two side-by-side tiles: (0,0) and (0,1).
  TileScheduler scheduler(4.0, 1, "none", true, "smart");
  auto grid = make_grid(8, 4);
  scheduler.ingest_grid(grid, 8, 4, 1.0);
  scheduler.take_pending_bands(100, -1);  // drain both tiles fully

  // Only perturb tile (0,1)'s cells (columns 4..7).
  for (int r = 0; r < 4; ++r) {
    grid[static_cast<std::size_t>(r * 8 + 4)] = static_cast<std::int8_t>(
      grid[static_cast<std::size_t>(r * 8 + 4)] == 0 ? 50 : 0);
  }
  const auto result = scheduler.ingest_grid(grid, 8, 4, 1.0);
  EXPECT_GT(result.total_changed, 0U);
  EXPECT_EQ(result.queued_tiles, 1U);  // only the perturbed tile re-queued

  const auto scheduled = scheduler.take_pending_bands(100, -1);
  std::set<TileKey> changed_tiles;
  for (const auto & item : scheduled) {changed_tiles.insert(item.tile);}
  EXPECT_EQ(changed_tiles.size(), 1U);
  EXPECT_EQ(*changed_tiles.begin(), (TileKey{0, 1}));
}

TEST(TileScheduler, ResolutionChangeResetsPendingState)
{
  TileScheduler scheduler(4.0, 2, "none", true, "smart");
  scheduler.ingest_grid(make_grid(4, 4), 4, 4, 1.0);
  ASSERT_TRUE(scheduler.has_pending());
  EXPECT_EQ(scheduler.tile_size_cells(), 4);

  // Halving the resolution doubles tile_size_cells (same physical tile_size_m),
  // which invalidates the old tile partition -- old backlog must be dropped,
  // not carried over as if it still applied to the new partition.
  scheduler.ingest_grid(make_grid(4, 4), 4, 4, 0.5);
  EXPECT_EQ(scheduler.tile_size_cells(), 8);
}

TEST(TileScheduler, TakePendingBandsOnEmptySchedulerReturnsEmpty)
{
  TileScheduler scheduler(4.0, 2, "none", true, "smart");
  const auto scheduled = scheduler.take_pending_bands(1, -1);
  EXPECT_TRUE(scheduled.empty());
}

TEST(TileScheduler, TakePendingBandsRespectsMaxBandsPerUpdate)
{
  // levels=2 -> 3 bands per tile; cap at 1 band per call.
  TileScheduler scheduler(4.0, 2, "none", true, "smart");
  scheduler.ingest_grid(make_grid(4, 4), 4, 4, 1.0);

  const auto first_take = scheduler.take_pending_bands(1, -1);
  EXPECT_EQ(first_take.size(), 1U);
  EXPECT_TRUE(scheduler.has_pending());  // 2 bands still remain

  const auto second_take = scheduler.take_pending_bands(1, -1);
  EXPECT_EQ(second_take.size(), 1U);
  EXPECT_TRUE(scheduler.has_pending());

  const auto third_take = scheduler.take_pending_bands(1, -1);
  EXPECT_EQ(third_take.size(), 1U);
  EXPECT_FALSE(scheduler.has_pending());  // all 3 bands drained
}

TEST(TileScheduler, TakePendingBandsRespectsMaxTilesPerUpdate)
{
  // 12x4 grid, tile_size_cells=4 -> three tiles side by side.
  TileScheduler scheduler(4.0, 1, "none", true, "smart");
  scheduler.ingest_grid(make_grid(12, 4), 12, 4, 1.0);
  ASSERT_EQ(scheduler.queued_tile_count(), 3U);

  const auto scheduled = scheduler.take_pending_bands(100, /*max_tiles_per_update=*/2);
  std::set<TileKey> tiles_serviced;
  for (const auto & item : scheduled) {tiles_serviced.insert(item.tile);}
  EXPECT_EQ(tiles_serviced.size(), 2U);
  // The third tile is still queued, untouched, for a future tick.
  EXPECT_EQ(scheduler.queued_tile_count(), 1U);
}

TEST(TileScheduler, SmartModePrefersNeverSentBandOverRecentlySentOne)
{
  // levels=2 -> bands 0,1,2 all pending after the first ingest.
  TileScheduler scheduler(4.0, 2, "none", true, "smart");
  auto grid = make_grid(4, 4);
  scheduler.ingest_grid(grid, 4, 4, 1.0);

  const auto first = scheduler.take_pending_bands(1, -1);
  ASSERT_EQ(first.size(), 1U);
  EXPECT_EQ(first[0].band_index, 0);  // coarsest-first when everything ties at "never sent"

  // Re-ingest identical content: band 0 was already sent, but bands 1 and 2
  // are still sitting in the backlog from before (never popped). A "simple"
  // scheduler would blindly hand out band 0 again since it's still index 0;
  // "smart" must prefer whichever of the still-pending bands has never had a turn.
  scheduler.ingest_grid(grid, 4, 4, 1.0);
  const auto second = scheduler.take_pending_bands(1, -1);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(second[0].band_index, 1);
}

TEST(TileScheduler, SimpleModeIgnoresSendRecencyAndStaysCoarsestFirst)
{
  TileScheduler scheduler(4.0, 2, "none", true, "simple");
  auto grid = make_grid(4, 4);
  scheduler.ingest_grid(grid, 4, 4, 1.0);

  const auto first = scheduler.take_pending_bands(1, -1);
  ASSERT_EQ(first.size(), 1U);
  EXPECT_EQ(first[0].band_index, 0);

  // Simple mode re-queues band 0 again unconditionally (no fingerprint check),
  // and with no recency tracking it must come out first again, not band 1.
  scheduler.ingest_grid(grid, 4, 4, 1.0);
  const auto second = scheduler.take_pending_bands(1, -1);
  ASSERT_EQ(second.size(), 1U);
  EXPECT_EQ(second[0].band_index, 0);
}

TEST(TileScheduler, IngestGridRecordsPerTileEncodingErrorsWithoutThrowing)
{
  // haar_levels=0 makes every tile's make_haar_bands() call throw ("haar
  // levels must be >= 1") -- TileScheduler doesn't validate haar_levels
  // itself (the Node wrapper does, before construction), so this is the only
  // way to exercise the catch-and-continue path from the public API.
  TileScheduler scheduler(4.0, 0, "none", true, "smart");
  const auto result = scheduler.ingest_grid(make_grid(4, 4), 4, 4, 1.0);
  EXPECT_EQ(result.total_changed, 0U);
  ASSERT_EQ(result.tile_errors.size(), 1U);
  EXPECT_NE(result.tile_errors[0].find("tile (0,0)"), std::string::npos);
  EXPECT_FALSE(scheduler.has_pending());
}

TEST(TileScheduler, IngestGridZeroSizeGridIsANoOp)
{
  TileScheduler scheduler(4.0, 2, "none", true, "smart");
  const auto result = scheduler.ingest_grid({}, 0, 0, 1.0);
  EXPECT_EQ(result.total_changed, 0U);
  EXPECT_FALSE(scheduler.has_pending());
}
