#include <gtest/gtest.h>

#include <optional>

#include "voxelcodec_ros/tile_send_scheduler.hpp"

using voxelcodec_ros::tile_resend_eligible;

TEST(TileSendScheduler, NeverSentIsAlwaysEligible)
{
  EXPECT_TRUE(tile_resend_eligible(std::nullopt, 0.0, 8.0));
  EXPECT_TRUE(tile_resend_eligible(std::nullopt, 1000.0, 8.0));
}

TEST(TileSendScheduler, WithinIntervalNotEligible)
{
  const std::optional<double> last_sent(100.0);
  EXPECT_FALSE(tile_resend_eligible(last_sent, 100.0, 8.0));
  EXPECT_FALSE(tile_resend_eligible(last_sent, 107.9, 8.0));
}

TEST(TileSendScheduler, ExactlyAtIntervalIsEligible)
{
  const std::optional<double> last_sent(100.0);
  EXPECT_TRUE(tile_resend_eligible(last_sent, 108.0, 8.0));
}

TEST(TileSendScheduler, PastIntervalIsEligible)
{
  const std::optional<double> last_sent(100.0);
  EXPECT_TRUE(tile_resend_eligible(last_sent, 500.0, 8.0));
}

TEST(TileSendScheduler, ZeroIntervalAlwaysEligible)
{
  const std::optional<double> last_sent(100.0);
  EXPECT_TRUE(tile_resend_eligible(last_sent, 100.0, 0.0));
}
