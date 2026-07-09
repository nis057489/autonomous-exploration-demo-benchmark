#include <gtest/gtest.h>

#include <optional>

#include "voxelcodec_ros/ddil_stale_epoch.hpp"

using voxelcodec_ros::Stamp;
using voxelcodec_ros::should_drop_as_stale;
using voxelcodec_ros::stamp_less;

TEST(DdilStaleEpoch, NoPriorManifestNeverDrops)
{
  EXPECT_FALSE(should_drop_as_stale(Stamp{0, 0}, std::nullopt));
  EXPECT_FALSE(should_drop_as_stale(Stamp{123, 456}, std::nullopt));
}

TEST(DdilStaleEpoch, ExactMatchNeverDrops)
{
  const Stamp s{100, 500};
  EXPECT_FALSE(should_drop_as_stale(s, std::optional<Stamp>(s)));
}

TEST(DdilStaleEpoch, OlderSameSecDrops)
{
  const Stamp band{100, 100};
  const Stamp manifest{100, 200};
  EXPECT_TRUE(should_drop_as_stale(band, std::optional<Stamp>(manifest)));
}

TEST(DdilStaleEpoch, OlderSmallerSecDrops)
{
  const Stamp band{99, 999999999};
  const Stamp manifest{100, 0};
  EXPECT_TRUE(should_drop_as_stale(band, std::optional<Stamp>(manifest)));
}

TEST(DdilStaleEpoch, SecNanosecCarryCase)
{
  // Larger nanosec does NOT mean newer once sec differs -- this is the case
  // a naive "compare nanosec whenever it looks bigger" implementation gets
  // wrong.
  const Stamp band{100, 999999999};
  const Stamp manifest{101, 0};
  EXPECT_TRUE(stamp_less(band, manifest));
  EXPECT_TRUE(should_drop_as_stale(band, std::optional<Stamp>(manifest)));
}

TEST(DdilStaleEpoch, NewerSecNeverDrops)
{
  // Guards against regressing to an equality-based check: a band from a
  // newer epoch than any manifest sent so far must always be forwarded.
  const Stamp band{101, 0};
  const Stamp manifest{100, 999999999};
  EXPECT_FALSE(should_drop_as_stale(band, std::optional<Stamp>(manifest)));
}

TEST(DdilStaleEpoch, SameSecLargerNanosecNeverDrops)
{
  const Stamp band{100, 200};
  const Stamp manifest{100, 100};
  EXPECT_FALSE(should_drop_as_stale(band, std::optional<Stamp>(manifest)));
}

TEST(DdilStaleEpoch, ZeroStampBothPresentNeverDrops)
{
  // {0,0} is a real, reachable sim-time timestamp, not a sentinel -- proves
  // std::optional (not a {0,0} sentinel) is what represents "no manifest
  // sent yet".
  const Stamp zero{0, 0};
  EXPECT_FALSE(should_drop_as_stale(zero, std::optional<Stamp>(zero)));
}

TEST(DdilStaleEpoch, NanosecBoundaryValues)
{
  const Stamp band{5, 999999999};
  const Stamp manifest{5, 999999999};
  EXPECT_FALSE(should_drop_as_stale(band, std::optional<Stamp>(manifest)));

  const Stamp band_older{5, 999999998};
  EXPECT_TRUE(should_drop_as_stale(band_older, std::optional<Stamp>(manifest)));
}
