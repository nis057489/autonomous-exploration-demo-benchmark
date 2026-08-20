#include <gtest/gtest.h>

#include <cstdint>
#include <functional>
#include <set>
#include <string>
#include <vector>

#include "voxelcodec_ros/occupancy_embedding.hpp"
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

TEST(TileScheduler, OccupancyEmbeddingIsABijectionOverAllLegalValues)
{
  std::set<std::uint32_t> seen;
  for (int v = -1; v <= 100; ++v) {
    const auto e = voxelcodec_ros::occupancy_to_embedded(static_cast<std::int8_t>(v));
    EXPECT_TRUE(seen.insert(e).second) << "collision at v=" << v << " -> " << e;
    EXPECT_EQ(voxelcodec_ros::embedded_to_occupancy(e), v) << "round-trip failed for v=" << v;
  }
}

TEST(TileScheduler, OccupancyEmbeddingPlacesUnknownAtTheMidpointNotAdjacentToFree)
{
  // Unknown must sit roughly equidistant from both known extremes, unlike
  // the old v+1 mapping (unknown=0, free=1, occupied=101), where
  // unknown->free was a magnitude-1 step and unknown->occupied was
  // magnitude-101 -- a ~100x mismatch that made a coefficient-energy-based
  // priority score treat newly-discovered free space as far less
  // informative than an occupied-state flip, for no principled reason.
  const auto unknown = voxelcodec_ros::occupancy_to_embedded(-1);
  const auto free = voxelcodec_ros::occupancy_to_embedded(0);
  const auto occupied = voxelcodec_ros::occupancy_to_embedded(100);
  const auto dist_to_free = unknown > free ? unknown - free : free - unknown;
  const auto dist_to_occupied = unknown > occupied ? unknown - occupied : occupied - unknown;
  EXPECT_LE(
    dist_to_free > dist_to_occupied ? dist_to_free - dist_to_occupied :
    dist_to_occupied - dist_to_free,
    2U);
}

TEST(TileScheduler, FloorDivRoundsTowardNegativeInfinity)
{
  EXPECT_EQ(voxelcodec_ros::floor_div(7, 4), 1);
  EXPECT_EQ(voxelcodec_ros::floor_div(-1, 4), -1);
  EXPECT_EQ(voxelcodec_ros::floor_div(-4, 4), -1);
  EXPECT_EQ(voxelcodec_ros::floor_div(-5, 4), -2);
  EXPECT_EQ(voxelcodec_ros::floor_div(0, 4), 0);
}

TEST(TileScheduler, ComputeAxisTileSpansOriginAlignedFullTile)
{
  // origin_cell=0, extent=8, tile_size_cells=4 -> local index 0 sits exactly
  // on a world-tile boundary, so this is the pre-world-anchoring case: two
  // full 4-wide spans, tile indices 0 and 1.
  const auto spans = voxelcodec_ros::compute_axis_tile_spans(0, 8, 4);
  ASSERT_EQ(spans.size(), 2U);
  EXPECT_EQ(spans[0].tile_index, 0);
  EXPECT_EQ(spans[0].local_start, 0);
  EXPECT_EQ(spans[0].length, 4);
  EXPECT_EQ(spans[1].tile_index, 1);
  EXPECT_EQ(spans[1].local_start, 4);
  EXPECT_EQ(spans[1].length, 4);
}

TEST(TileScheduler, ComputeAxisTileSpansClampsAtExtentEdge)
{
  // A 10-cell extent with tile_size_cells=4: last span is clipped to 2 cells,
  // same clipping shape as the old compute_tile_geom's grid-edge behavior.
  const auto spans = voxelcodec_ros::compute_axis_tile_spans(0, 10, 4);
  ASSERT_EQ(spans.size(), 3U);
  EXPECT_EQ(spans[2].tile_index, 2);
  EXPECT_EQ(spans[2].local_start, 8);
  EXPECT_EQ(spans[2].length, 2);
}

TEST(TileScheduler, ComputeAxisTileSpansNonAlignedOriginClipsFirstAndLastSpan)
{
  // origin_cell=2 -> local index 0 sits 2 cells into world tile 0 (which
  // spans world cells [0,4)), so the array's own first 2 local cells are a
  // partial tile, then a full tile, then whatever's left.
  const auto spans = voxelcodec_ros::compute_axis_tile_spans(2, 10, 4);
  ASSERT_EQ(spans.size(), 3U);
  EXPECT_EQ(spans[0].tile_index, 0);
  EXPECT_EQ(spans[0].local_start, 0);
  EXPECT_EQ(spans[0].length, 2);  // world cells [2,4) -- clipped at the tile's own start
  EXPECT_EQ(spans[1].tile_index, 1);
  EXPECT_EQ(spans[1].local_start, 2);
  EXPECT_EQ(spans[1].length, 4);  // world cells [4,8) -- full tile
  EXPECT_EQ(spans[2].tile_index, 2);
  EXPECT_EQ(spans[2].local_start, 6);
  EXPECT_EQ(spans[2].length, 4);  // world cells [8,12), array only has 4 more cells
}

TEST(TileScheduler, ComputeAxisTileSpansNegativeOriginCellProducesNegativeTileIndices)
{
  // origin_cell=-3, tile_size_cells=4 -> local 0 is world cell -3, which
  // floor-divides into tile -1 (covering world cells [-4,0)) -- local cells
  // 0,1,2 are world cells -3,-2,-1 (3 cells left in that tile), then local 3
  // is world cell 0, the start of tile 0.
  const auto spans = voxelcodec_ros::compute_axis_tile_spans(-3, 6, 4);
  ASSERT_EQ(spans.size(), 2U);
  EXPECT_EQ(spans[0].tile_index, -1);
  EXPECT_EQ(spans[0].local_start, 0);
  EXPECT_EQ(spans[0].length, 3);  // world cells [-3,0)
  EXPECT_EQ(spans[1].tile_index, 0);
  EXPECT_EQ(spans[1].local_start, 3);
  EXPECT_EQ(spans[1].length, 3);  // world cells [0,3), extent runs out at local 6
}

TEST(TileScheduler, ComputeAxisTileSpansZeroExtentOrTileSizeIsEmpty)
{
  EXPECT_TRUE(voxelcodec_ros::compute_axis_tile_spans(0, 0, 4).empty());
  EXPECT_TRUE(voxelcodec_ros::compute_axis_tile_spans(0, 8, 0).empty());
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

TEST(TileScheduler, GridGrowthWithOriginShiftKeepsUnchangedTileFingerprintStable)
{
  // Simulates a real SLAM grid-growth event: the array extends backward in
  // -X by exactly one tile-width, so origin_x moves from 0.0 to -4.0 and
  // grid_w grows from 4 to 8 -- the SAME physical content that used to sit
  // at local columns [0,4) now sits at local columns [4,8). Before world-
  // anchored tiling, this was a real bug on the ENCODER side too, not just
  // the receiver: under pure local-array indexing, the relocated content
  // would land under a "new" tile key with no prior fingerprint (first-time
  // full resend of data the receiver already has) while the genuinely-new
  // columns landed under the OLD tile 0's key (spuriously flagged "changed"
  // relative to a fingerprint that was never really for this content). World
  // anchoring fixes both: only the genuinely new tile gets queued.
  TileScheduler scheduler(4.0, 2, "none", true, "smart");
  const auto tile_content = make_grid(4, 4);
  const auto first = scheduler.ingest_grid(tile_content, 4, 4, /*resolution=*/1.0, 0.0, 0.0);
  ASSERT_GT(first.total_changed, 0U);
  scheduler.take_pending_bands(100, -1);  // drain fully

  std::vector<std::int8_t> grown(8 * 4, -1);  // new columns [0,4) unknown
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      grown[static_cast<std::size_t>(r * 8 + (c + 4))] =
        tile_content[static_cast<std::size_t>(r * 4 + c)];
    }
  }
  const auto second = scheduler.ingest_grid(grown, 8, 4, 1.0, -4.0, 0.0);
  // Only the genuinely new (unknown-filled) tile should be queued -- the
  // relocated original content must be recognized as unchanged.
  EXPECT_EQ(second.queued_tiles, 1U);

  const auto scheduled = scheduler.take_pending_bands(1000, -1);
  for (const auto & item : scheduled) {
    EXPECT_EQ(item.tile, (TileKey{0, -1})) << "unexpected tile got re-queued";
  }
}

TEST(TileScheduler, NonTileAlignedOriginProducesNegativeAndClippedTileKeys)
{
  // origin_x=-2.0 at resolution 1.0 (tile_size_cells=4) means local column 0
  // sits at world cell -2 -- inside world tile -1 (covering world cells
  // [-4,0)), not tile 0. An 8-wide grid then spans world tiles -1, 0, and
  // (partially) 1.
  TileScheduler scheduler(4.0, 1, "none", true, "smart");
  const auto result = scheduler.ingest_grid(make_grid(8, 4), 8, 4, 1.0, -2.0, 0.0);
  EXPECT_EQ(result.queued_tiles, 3U);

  const auto scheduled = scheduler.take_pending_bands(1000, -1);
  std::set<TileKey> tiles_seen;
  for (const auto & item : scheduled) {tiles_seen.insert(item.tile);}
  ASSERT_EQ(tiles_seen.size(), 3U);
  EXPECT_TRUE(tiles_seen.count(TileKey{0, -1}));
  EXPECT_TRUE(tiles_seen.count(TileKey{0, 0}));
  EXPECT_TRUE(tiles_seen.count(TileKey{0, 1}));
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

TEST(TileScheduler, RdModePrefersHigherEnergyTileOverFifoOrder)
{
  // 8x4 grid, tile_size_cells=4 -> tiles (0,0) and (0,1), inserted into
  // tile_queue_ in that order (row-major scan), so round-robin/FIFO would
  // hand out (0,0) first. Perturb (0,0) with a tiny 1-cell delta and (0,1)
  // with a much larger whole-column delta -- rd mode must pick (0,1) first
  // despite it being LATER in insertion order, since only score, not queue
  // position, decides.
  TileScheduler scheduler(4.0, 1, "none", true, "rd");
  auto grid = make_grid(8, 4);
  scheduler.ingest_grid(grid, 8, 4, 1.0);
  scheduler.take_pending_bands(1000, -1);  // drain fully

  grid[0] = static_cast<std::int8_t>(grid[0] == 0 ? 1 : 0);  // tile (0,0): tiny delta
  for (int r = 0; r < 4; ++r) {  // tile (0,1): whole column flips to the opposite extreme
    grid[static_cast<std::size_t>(r * 8 + 4)] = 100;
  }
  scheduler.ingest_grid(grid, 8, 4, 1.0);

  const auto scheduled = scheduler.take_pending_bands(1, 1);
  ASSERT_EQ(scheduled.size(), 1U);
  EXPECT_EQ(scheduled[0].tile, (TileKey{0, 1}));
}

TEST(TileScheduler, RdModeRespectsMaxBandsAndMaxTilesCaps)
{
  // 12x4 grid, tile_size_cells=4 -> three tiles; levels=1 -> 2 bands/tile.
  TileScheduler scheduler(4.0, 1, "none", true, "rd");
  scheduler.ingest_grid(make_grid(12, 4), 12, 4, 1.0);

  const auto scheduled = scheduler.take_pending_bands(/*max_bands_per_update=*/1, /*max_tiles_per_update=*/2);
  std::set<TileKey> tiles_serviced;
  for (const auto & item : scheduled) {tiles_serviced.insert(item.tile);}
  EXPECT_EQ(scheduled.size(), 2U);       // 1 band/tile x 2 tiles
  EXPECT_EQ(tiles_serviced.size(), 2U);  // never more than max_tiles_per_update distinct tiles
  EXPECT_TRUE(scheduler.has_pending());  // third tile, and each serviced tile's 2nd band, remain
}

TEST(TileScheduler, RdModeAgingLetsAWaitingLowScoreBandEventuallyWinOverFreshHighScoreOnes)
{
  // Same starvation shape as ddil_proxy_logic.hpp's BandQueue bug this
  // mirrors: a "hot" tile that keeps producing fresh, higher-score content
  // every tick must not be able to indefinitely preempt a "cold" tile's
  // single band that's just sitting there waiting.
  TileScheduler scheduler(4.0, 1, "none", true, "rd");
  auto grid = make_grid(8, 4);
  scheduler.ingest_grid(grid, 8, 4, 1.0);
  scheduler.take_pending_bands(1000, -1);  // drain fully

  // Cold tile (0,0): one whole-column delta, queued once, then left alone.
  for (int r = 0; r < 4; ++r) {
    grid[static_cast<std::size_t>(r * 8)] = 80;
  }
  scheduler.ingest_grid(grid, 8, 4, 1.0);

  bool cold_tile_won = false;
  bool toggle = false;
  for (int tick = 0; tick < 200 && !cold_tile_won; ++tick) {
    // Hot tile (0,1): re-perturbed to a slightly larger delta every tick, so
    // its raw score is always somewhat higher than the cold tile's -- the
    // only thing that can let the cold tile through is aging.
    toggle = !toggle;
    for (int r = 0; r < 4; ++r) {
      grid[static_cast<std::size_t>(r * 8 + 4)] = toggle ? 100 : 0;
    }
    scheduler.ingest_grid(grid, 8, 4, 1.0);

    const auto scheduled = scheduler.take_pending_bands(1, 1);
    ASSERT_EQ(scheduled.size(), 1U);
    if (scheduled[0].tile == (TileKey{0, 0})) {
      cold_tile_won = true;
    }
  }
  EXPECT_TRUE(cold_tile_won) << "aging never let the waiting low-score band win";
}
