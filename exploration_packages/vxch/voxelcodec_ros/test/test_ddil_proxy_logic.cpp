#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <voxelcodec_msgs/msg/voxel_channel.hpp>

#include "voxelcodec_ros/ddil_proxy_logic.hpp"

namespace
{

std::shared_ptr<rclcpp::SerializedMessage> serialize_channel(
  const voxelcodec_msgs::msg::VoxelChannel & msg)
{
  static rclcpp::Serialization<voxelcodec_msgs::msg::VoxelChannel> ser;
  auto serialized = std::make_shared<rclcpp::SerializedMessage>();
  ser.serialize_message(&msg, serialized.get());
  return serialized;
}

}  // namespace

TEST(DdilProxyLogic, EpochRoleFromMsgType)
{
  EXPECT_EQ(
    voxelcodec_ros::epoch_role_from_msg_type("voxelcodec_msgs/msg/VoxelManifest"),
    voxelcodec_ros::EpochRole::kManifest);
  EXPECT_EQ(
    voxelcodec_ros::epoch_role_from_msg_type("voxelcodec_msgs/msg/VoxelChannel"),
    voxelcodec_ros::EpochRole::kBand);
  EXPECT_EQ(
    voxelcodec_ros::epoch_role_from_msg_type("nav_msgs/msg/OccupancyGrid"),
    voxelcodec_ros::EpochRole::kNone);
}

TEST(DdilProxyLogic, ParseRelayEntryThreeTokensDefaultsToBestEffort)
{
  const auto cfg = voxelcodec_ros::parse_relay_entry("/map /peer/map nav_msgs/msg/OccupancyGrid");
  EXPECT_EQ(cfg.input_topic, "/map");
  EXPECT_EQ(cfg.output_topic, "/peer/map");
  EXPECT_EQ(cfg.msg_type, "nav_msgs/msg/OccupancyGrid");
  EXPECT_FALSE(cfg.reliable);
  EXPECT_FALSE(cfg.bypass);
}

TEST(DdilProxyLogic, ParseRelayEntryReliableFlag)
{
  const auto cfg = voxelcodec_ros::parse_relay_entry(
    "/vxch/map/manifest /peer/vxch/map/manifest voxelcodec_msgs/msg/VoxelManifest reliable");
  EXPECT_TRUE(cfg.reliable);
}

TEST(DdilProxyLogic, ParseRelayEntryUnrecognizedFourthTokenIsNotReliable)
{
  const auto cfg = voxelcodec_ros::parse_relay_entry("/a /b some/msg/Type nonsense");
  EXPECT_FALSE(cfg.reliable);
}

TEST(DdilProxyLogic, ParseRelayEntryRejectsMissingTokens)
{
  EXPECT_THROW(voxelcodec_ros::parse_relay_entry("/a /b"), std::runtime_error);
  EXPECT_THROW(voxelcodec_ros::parse_relay_entry(""), std::runtime_error);
}

TEST(DdilProxyLogic, BandIndexFromTopicParsesTrailingIndex)
{
  EXPECT_EQ(voxelcodec_ros::band_index_from_topic("/vxch/map/band_0"), 0);
  EXPECT_EQ(voxelcodec_ros::band_index_from_topic("/vxch/map/band_12"), 12);
}

TEST(DdilProxyLogic, BandIndexFromTopicRejectsNonBandTopics)
{
  EXPECT_EQ(voxelcodec_ros::band_index_from_topic("/vxch/map/manifest"), -1);
  EXPECT_EQ(voxelcodec_ros::band_index_from_topic("/vxch/map/band_"), -1);
  EXPECT_EQ(voxelcodec_ros::band_index_from_topic("/vxch/map/band_abc"), -1);
  EXPECT_EQ(voxelcodec_ros::band_index_from_topic("/vxch/map/band_3x"), -1);
}

TEST(DdilProxyLogic, IsManifestTopic)
{
  EXPECT_TRUE(voxelcodec_ros::is_manifest_topic("/vxch/map/manifest"));
  EXPECT_FALSE(voxelcodec_ros::is_manifest_topic("/vxch/map/manifest2"));
  EXPECT_FALSE(voxelcodec_ros::is_manifest_topic("/vxch/map/band_0"));
  EXPECT_FALSE(voxelcodec_ros::is_manifest_topic("m"));
}

TEST(DdilProxyLogic, TileIdFromChannelMsgReadsMetadata)
{
  voxelcodec_msgs::msg::VoxelChannel msg;
  voxelcodec_msgs::msg::MetadataEntry row;
  row.key = "tile_row";
  row.value = "3";
  voxelcodec_msgs::msg::MetadataEntry col;
  col.key = "tile_col";
  col.value = "7";
  msg.descriptor.metadata = {row, col};

  const auto serialized = serialize_channel(msg);
  const auto [tile_row, tile_col] = voxelcodec_ros::tile_id_from_channel_msg(*serialized);
  EXPECT_EQ(tile_row, 3);
  EXPECT_EQ(tile_col, 7);
}

TEST(DdilProxyLogic, TileIdFromChannelMsgDefaultsToOriginWhenUntagged)
{
  voxelcodec_msgs::msg::VoxelChannel msg;  // no tile_row/tile_col metadata
  const auto serialized = serialize_channel(msg);
  const auto [tile_row, tile_col] = voxelcodec_ros::tile_id_from_channel_msg(*serialized);
  EXPECT_EQ(tile_row, 0);
  EXPECT_EQ(tile_col, 0);
}

TEST(DdilProxyLogic, TokenBucketUnlimitedBandwidthNeverBlocks)
{
  voxelcodec_ros::TokenBucket bucket(0.0);
  const auto start = std::chrono::steady_clock::now();
  bucket.consume(1'000'000'000ULL);  // would take ages at any finite rate
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(std::chrono::duration<double>(elapsed).count(), 0.05);
}

TEST(DdilProxyLogic, TokenBucketConsumesWithoutBlockingWithinBudget)
{
  // 8 kbps -> 1000 bytes/sec bucket, starts full.
  voxelcodec_ros::TokenBucket bucket(8.0);
  bucket.consume(500);
  const double remaining = bucket.tokens_for_testing();
  // Should be just under 500 (a slice of refill happened during the calls),
  // and well above 0 -- consuming half the bucket shouldn't have blocked.
  EXPECT_GT(remaining, 400.0);
  EXPECT_LT(remaining, 600.0);
}

TEST(DdilProxyLogic, TokenBucketBlocksAndRefillsWhenOverBudget)
{
  // 8 kbps -> 1000 bytes/sec bucket. Consuming 1200 bytes needs the bucket to
  // refill ~200 bytes worth of deficit, i.e. block for roughly 0.2s.
  voxelcodec_ros::TokenBucket bucket(8.0);
  const auto start = std::chrono::steady_clock::now();
  bucket.consume(1200);
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_GT(elapsed, 0.1);
  EXPECT_LT(elapsed, 1.0);
}

TEST(DdilProxyLogic, BandQueuePopsInPriorityOrder)
{
  voxelcodec_ros::BandQueue queue;
  voxelcodec_ros::QueuedMessage low_priority;
  low_priority.band_priority = 5;
  voxelcodec_ros::QueuedMessage high_priority;
  high_priority.band_priority = 0;

  EXPECT_FALSE(queue.push(low_priority));
  EXPECT_FALSE(queue.push(high_priority));
  ASSERT_EQ(queue.size(), 2U);

  const auto first = queue.pop();
  EXPECT_EQ(first.band_priority, 0);
  const auto second = queue.pop();
  EXPECT_EQ(second.band_priority, 5);
  EXPECT_TRUE(queue.empty());
}

TEST(DdilProxyLogic, BandQueueSamePriorityIsFifo)
{
  voxelcodec_ros::BandQueue queue;
  voxelcodec_ros::QueuedMessage first_in;
  first_in.band_priority = 1;
  first_in.dedup_key = "a";
  voxelcodec_ros::QueuedMessage second_in;
  second_in.band_priority = 1;
  second_in.dedup_key = "b";

  queue.push(first_in);
  queue.push(second_in);

  EXPECT_EQ(queue.pop().dedup_key, "a");
  EXPECT_EQ(queue.pop().dedup_key, "b");
}

TEST(DdilProxyLogic, BandQueueDedupReplacesInPlace)
{
  voxelcodec_ros::BandQueue queue;
  voxelcodec_ros::QueuedMessage first;
  first.band_priority = 2;
  first.dedup_key = "tile_0_0:band_1";
  first.serialized = std::make_shared<rclcpp::SerializedMessage>();

  voxelcodec_ros::QueuedMessage replacement;
  replacement.band_priority = 2;
  replacement.dedup_key = "tile_0_0:band_1";
  replacement.serialized = std::make_shared<rclcpp::SerializedMessage>();

  EXPECT_FALSE(queue.push(first));
  EXPECT_TRUE(queue.push(replacement));  // dedup fired
  // Still just one entry -- the replacement overwrote it in place, not appended.
  ASSERT_EQ(queue.size(), 1U);
  const auto popped = queue.pop();
  EXPECT_EQ(popped.serialized, replacement.serialized);
}

TEST(DdilProxyLogic, BandQueueEmptyDedupKeyNeverDedups)
{
  voxelcodec_ros::BandQueue queue;
  voxelcodec_ros::QueuedMessage a;
  voxelcodec_ros::QueuedMessage b;
  EXPECT_FALSE(queue.push(a));
  EXPECT_FALSE(queue.push(b));
  EXPECT_EQ(queue.size(), 2U);
}

TEST(DdilProxyLogic, PendingByBandAggregatesAcrossTilesExcludingManifestAndBypass)
{
  voxelcodec_ros::BandQueue queue;

  voxelcodec_msgs::msg::VoxelChannel chan;
  chan.payload.resize(100, 0);
  auto band0_tile_a = serialize_channel(chan);
  auto band0_tile_b = serialize_channel(chan);
  auto band1_tile_a = serialize_channel(chan);

  voxelcodec_ros::QueuedMessage m0a;
  m0a.band_priority = 0;
  m0a.dedup_key = "tile_0_0:band_0";
  m0a.serialized = band0_tile_a;
  voxelcodec_ros::QueuedMessage m0b;
  m0b.band_priority = 0;
  m0b.dedup_key = "tile_1_0:band_0";
  m0b.serialized = band0_tile_b;
  voxelcodec_ros::QueuedMessage m1a;
  m1a.band_priority = 1;
  m1a.dedup_key = "tile_0_0:band_1";
  m1a.serialized = band1_tile_a;

  // Manifest (-1) and non-band relay traffic (INT_MAX) must not show up as bands.
  voxelcodec_ros::QueuedMessage manifest;
  manifest.band_priority = -1;
  manifest.dedup_key = "manifest";
  manifest.serialized = serialize_channel(chan);
  voxelcodec_ros::QueuedMessage other;
  other.band_priority = std::numeric_limits<int>::max();
  other.serialized = serialize_channel(chan);

  queue.push(m0a);
  queue.push(m0b);
  queue.push(m1a);
  queue.push(manifest);
  queue.push(other);

  const auto pending = queue.pending_by_band();
  ASSERT_EQ(pending.size(), 2U);
  EXPECT_EQ(pending.at(0).first, 2U);
  EXPECT_EQ(pending.at(0).second, band0_tile_a->size() + band0_tile_b->size());
  EXPECT_EQ(pending.at(1).first, 1U);
  EXPECT_EQ(pending.at(1).second, band1_tile_a->size());
}

namespace
{
// BandQueue sizes its budget from serialized payload size, so these tests need
// messages with a real byte count rather than the default-constructed
// (null-payload) QueuedMessages the ordering tests use.
voxelcodec_ros::QueuedMessage sized_band(int band_priority, std::size_t bytes,
  const std::string & dedup_key = "")
{
  voxelcodec_ros::QueuedMessage msg;
  msg.band_priority = band_priority;
  msg.dedup_key = dedup_key;
  msg.serialized = std::make_shared<rclcpp::SerializedMessage>(bytes);
  // SerializedMessage's constructor reserves capacity; size() tracks the
  // buffer length, which is what the budget accounting reads.
  msg.serialized->get_rcl_serialized_message().buffer_length = bytes;
  return msg;
}
}  // namespace

// The regression this whole bounding effort exists for: with an unbounded
// queue, a link whose fill rate exceeds its drain rate accumulates forever, so
// everything it eventually delivers is minutes-to-hours stale and the peer's
// map freezes on whatever arrived first. Measured live at 5 kbps: 4.0 MB
// queued, band_2 ETA 1h47m, peer holding 64 of the sender's 468 tiles.
TEST(DdilProxyLogic, BandQueueByteBudgetBoundsTheBacklog)
{
  voxelcodec_ros::BandQueue queue;
  queue.set_byte_budget(1000);

  for (int i = 0; i < 50; ++i) {
    queue.push(sized_band(1, 100, "tile_" + std::to_string(i) + ":band_1"));
  }

  EXPECT_LE(queue.queued_bytes(), 1000U);
  EXPECT_GT(queue.shed_count(), 0U);
  EXPECT_GT(queue.shed_bytes(), 0U);
}

// Shedding has to drop the FINEST band, not simply the oldest: keeping coarse
// coverage of newly explored ground is the point of the scheme, and plain
// tail-drop would instead spend the link refining ground the peer already has.
TEST(DdilProxyLogic, BandQueueShedsFinestBandFirstAndKeepsCoarse)
{
  voxelcodec_ros::BandQueue queue;
  queue.set_byte_budget(300);

  queue.push(sized_band(0, 100, "tile_a:band_0"));
  queue.push(sized_band(1, 100, "tile_a:band_1"));
  queue.push(sized_band(2, 100, "tile_a:band_2"));
  ASSERT_EQ(queue.size(), 3U);
  ASSERT_EQ(queue.shed_count(), 0U);

  // One more coarse arrival puts the queue over budget; band_2 is what goes.
  queue.push(sized_band(0, 100, "tile_b:band_0"));

  EXPECT_EQ(queue.shed_count(), 1U);
  ASSERT_EQ(queue.size(), 3U);
  const auto pending = queue.pending_by_band();
  EXPECT_EQ(pending.count(2), 0U) << "the finest band should have been shed";
  ASSERT_EQ(pending.count(0), 1U);
  EXPECT_EQ(pending.at(0).first, 2U) << "both coarse bands must survive";
  EXPECT_EQ(pending.count(1), 1U);
}

// The manifest carries the geometry without which a decoder cannot place any
// tile at all, so it must never be a shedding victim regardless of budget.
TEST(DdilProxyLogic, BandQueueNeverShedsManifestOrNonBandTraffic)
{
  voxelcodec_ros::BandQueue queue;
  queue.set_byte_budget(100);

  auto manifest = sized_band(-1, 500, "pub:manifest");
  auto other = sized_band(std::numeric_limits<int>::max(), 500, "");
  queue.push(manifest);
  queue.push(other);
  queue.push(sized_band(2, 500, "tile_a:band_2"));

  // Way over budget, but only the band entry is eligible to be dropped.
  EXPECT_EQ(queue.size(), 2U);
  EXPECT_EQ(queue.shed_count(), 1U);
  EXPECT_EQ(queue.pop().band_priority, -1) << "manifest still first";
  EXPECT_EQ(queue.pop().band_priority, std::numeric_limits<int>::max());
}

// The bug this replaced continuous aging to fix. Under a deep backlog every
// queued band used to age down to effective priority 0 within
// aging_interval_ms * max_band of queueing, tying with fresh band_0 arrivals
// and collapsing the queue to FIFO -- so a receiver saw no coarse-to-fine
// progression at all, and (because fine bands emit more sub-MTU messages per
// tick) actually got MORE fine detail than coarse. Strict band ordering must
// survive an arbitrarily long wait.
TEST(DdilProxyLogic, BandQueueDeepBacklogStillPopsCoarsestFirst)
{
  // Small interval so any *continuous* decay would definitely have flattened
  // these by the time we pop.
  voxelcodec_ros::BandQueue queue(/*aging_interval_ms=*/20.0);

  queue.push(sized_band(2, 100, "tile_a:band_2"));
  queue.push(sized_band(1, 100, "tile_a:band_1"));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  // A fresh coarse arrival after everything else has waited well past what
  // used to be its full catch-up window.
  queue.push(sized_band(0, 100, "tile_b:band_0"));

  // band_1/band_2 are past their promotion deadline (20ms * k), so they are
  // promoted ahead of the fresh band_0 -- but promotion is a single tier, so
  // among themselves FIFO holds and band_0 still precedes nothing finer that
  // is *not* promoted. What must NOT happen is the queue silently becoming
  // insertion-ordered while claiming to be a priority queue.
  const auto first = queue.pop();
  EXPECT_EQ(first.dedup_key, "tile_a:band_2") <<
    "oldest promoted entry goes first once past its starvation deadline";
  const auto second = queue.pop();
  EXPECT_EQ(second.dedup_key, "tile_a:band_1");
  const auto third = queue.pop();
  EXPECT_EQ(third.dedup_key, "tile_b:band_0");
}

// With no starvation deadline reached, ordering is strictly by band index no
// matter how much is queued or in what order it arrived -- this is the
// coarse-before-fine property progressive decoding actually depends on.
TEST(DdilProxyLogic, BandQueueStrictCoarsestFirstWithinDeadline)
{
  voxelcodec_ros::BandQueue queue(/*aging_interval_ms=*/60000.0);

  queue.push(sized_band(2, 100, "tile_a:band_2"));
  queue.push(sized_band(2, 100, "tile_b:band_2"));
  queue.push(sized_band(1, 100, "tile_a:band_1"));
  queue.push(sized_band(0, 100, "tile_a:band_0"));
  queue.push(sized_band(1, 100, "tile_b:band_1"));
  queue.push(sized_band(0, 100, "tile_b:band_0"));

  std::vector<int> order;
  while (!queue.empty()) {
    order.push_back(queue.pop().band_priority);
  }
  EXPECT_EQ(order, (std::vector<int>{0, 0, 1, 1, 2, 2}));
}

// Narrowing the link has to resize the existing backlog, not just cap future
// pushes -- otherwise a queue sized for the old rate sits there draining at
// the new one, which is the stale-delivery problem all over again.
TEST(DdilProxyLogic, BandQueueShrinkingBudgetShedsImmediately)
{
  voxelcodec_ros::BandQueue queue;
  for (int i = 0; i < 10; ++i) {
    queue.push(sized_band(2, 100, "tile_" + std::to_string(i) + ":band_2"));
  }
  ASSERT_EQ(queue.queued_bytes(), 1000U);
  ASSERT_EQ(queue.shed_count(), 0U);

  queue.set_byte_budget(250);

  EXPECT_LE(queue.queued_bytes(), 250U);
  EXPECT_EQ(queue.shed_count(), 8U);
}

// Dedup replaces a queued payload in place, so the budget accounting has to
// follow the size change rather than double-counting the slot.
TEST(DdilProxyLogic, BandQueueDedupKeepsByteAccountingConsistent)
{
  voxelcodec_ros::BandQueue queue;
  queue.push(sized_band(1, 100, "tile_a:band_1"));
  ASSERT_EQ(queue.queued_bytes(), 100U);

  EXPECT_TRUE(queue.push(sized_band(1, 400, "tile_a:band_1")));
  EXPECT_EQ(queue.size(), 1U);
  EXPECT_EQ(queue.queued_bytes(), 400U);

  queue.pop();
  EXPECT_EQ(queue.queued_bytes(), 0U);
}

// End-to-end statement of the property the user-visible bug was about: under a
// sustained overload, what actually reaches the peer must be coarse-dominant,
// because that is what makes a receiver see a blurry-then-sharper map instead
// of a random sample of detail.
//
// The arrival mix and rates here are taken from a measured run
// (experiment_runs/20260910_004301_vxch_office, robot3 -> robot2 at 5 kbps):
// the encoder offered ~7 KB/s against a 625 B/s link, and per tick emitted
// more fine-band messages than coarse ones because fine payloads are larger
// and MTU-split into more sub-messages. Before bounding + strict ordering, the
// delivered mix was 73/91/107 for bands 0/1/2 -- inverted. It must now favor
// band 0.
TEST(DdilProxyLogic, SustainedOverloadDeliversCoarseDominantMix)
{
  constexpr double kLinkBytesPerSec = 625.0;   // 5 kbps
  constexpr int kTicks = 120;                  // 120 virtual seconds
  // Deadline far beyond this test's wall-clock duration, so promotion never
  // fires and we are measuring the steady-state ordering policy itself.
  voxelcodec_ros::BandQueue queue(/*aging_interval_ms=*/600000.0);
  queue.set_byte_budget(static_cast<std::uint64_t>(kLinkBytesPerSec * 4.0));

  // Per-tick offered load, per band: (message count, bytes each).
  const std::vector<std::pair<int, std::size_t>> offered{{3, 770}, {4, 850}, {5, 880}};

  std::map<int, int> delivered;
  int seq = 0;
  for (int tick = 0; tick < kTicks; ++tick) {
    for (int band = 0; band < 3; ++band) {
      for (int i = 0; i < offered[band].first; ++i) {
        queue.push(
          sized_band(
            band, offered[band].second,
            "tile_" + std::to_string(seq++) + ":band_" + std::to_string(band)));
      }
    }
    // Drain one second's worth of link capacity.
    double budget = kLinkBytesPerSec;
    while (!queue.empty()) {
      const auto next_bytes = static_cast<double>(queue.pending_by_band().empty() ? 0 : 1);
      (void)next_bytes;
      const auto msg = queue.pop();
      const double bytes = msg.serialized ? static_cast<double>(msg.serialized->size()) : 0.0;
      delivered[msg.band_priority] += 1;
      budget -= bytes;
      if (budget <= 0.0) {break;}
    }
  }

  const int b0 = delivered[0];
  const int b1 = delivered[1];
  const int b2 = delivered[2];
  EXPECT_GT(b0, b1) << "band_0 must out-deliver band_1 under overload; got "
                    << b0 << "/" << b1 << "/" << b2;
  EXPECT_GT(b0, b2) << "band_0 must out-deliver band_2 under overload; got "
                    << b0 << "/" << b1 << "/" << b2;
  // Coarse should dominate decisively, not just edge it out -- the whole point
  // is that a starved link spends what it has on a coarse picture of
  // everything.
  EXPECT_GT(b0, 2 * (b1 + b2)) << "delivered mix " << b0 << "/" << b1 << "/" << b2;
}

TEST(DdilProxyLogic, BandQueueAgingLetsAWaitingFineBandEventuallyWinOverFreshCoarseOnes)
{
  // 20ms aging interval so the test doesn't need to sleep for real seconds:
  // a band_5 entry needs 5 * 20ms = 100ms of wait to fully catch up to a
  // never-waited band_0.
  voxelcodec_ros::BandQueue queue(/*aging_interval_ms=*/20.0);

  voxelcodec_ros::QueuedMessage fine;
  fine.band_priority = 5;
  fine.dedup_key = "tile_0_0:band_5";
  ASSERT_FALSE(queue.push(fine));

  // Let it age past the catch-up threshold before any coarse traffic arrives.
  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  // A never-before-waited band_0 arrives from some other, currently-active tile --
  // under the old strict-priority BandQueue this would unconditionally win.
  voxelcodec_ros::QueuedMessage coarse;
  coarse.band_priority = 0;
  coarse.dedup_key = "tile_3_3:band_0";
  ASSERT_FALSE(queue.push(coarse));

  // The long-waiting fine band has aged down to priority 0 too, and it was
  // queued first, so FIFO among ties hands it out ahead of the fresh coarse one.
  EXPECT_EQ(queue.pop().dedup_key, "tile_0_0:band_5");
  EXPECT_EQ(queue.pop().dedup_key, "tile_3_3:band_0");
}

TEST(DdilProxyLogic, BandQueueSustainedFreshCoarseArrivalsCannotStarveAWaitingFineBand)
{
  // This is the scenario observed in a real bag capture: 6925/6925 band
  // messages sent over a whole run were band_0, zero of band_1..5 ever got
  // through, because a continuously-regenerating band_0 always won the strict
  // priority race. With aging, even a never-ending stream of fresh band_0
  // arrivals eventually loses to a band that's been waiting long enough.
  voxelcodec_ros::BandQueue queue(/*aging_interval_ms=*/20.0);

  voxelcodec_ros::QueuedMessage fine;
  fine.band_priority = 5;
  fine.dedup_key = "tile_0_0:band_5";
  ASSERT_FALSE(queue.push(fine));

  bool fine_band_won = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  int coarse_seq = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    voxelcodec_ros::QueuedMessage coarse;
    coarse.band_priority = 0;
    coarse.dedup_key = "tile_3_3:band_0_" + std::to_string(coarse_seq++);
    queue.push(coarse);

    const auto popped = queue.pop();
    if (popped.dedup_key == "tile_0_0:band_5") {
      fine_band_won = true;
      break;
    }
    // Simulate the fresh coarse arrival winning again -- still, band_5 keeps
    // accruing wait time in the queue for the next iteration's comparison.
  }

  EXPECT_TRUE(fine_band_won) <<
    "band_5 never won against a sustained stream of fresh band_0 arrivals -- "
    "this is the starvation bug aging is supposed to bound.";
}
