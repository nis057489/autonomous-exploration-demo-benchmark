#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

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
