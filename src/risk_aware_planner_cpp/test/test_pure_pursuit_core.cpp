#include <cmath>

#include "gtest/gtest.h"
#include "risk_aware_planner_cpp/pure_pursuit_core.hpp"

namespace rap = risk_aware_planner_cpp;

TEST(PurePursuitCore, DynamicLookaheadIsBounded)
{
  rap::PurePursuitConfig config;
  EXPECT_DOUBLE_EQ(0.70, rap::computeDynamicLookahead(config, 0.0));
  EXPECT_DOUBLE_EQ(1.20, rap::computeDynamicLookahead(config, 10.0));
}

TEST(PurePursuitCore, StraightTargetProducesForwardMotion)
{
  rap::PurePursuitConfig config;
  const auto command = rap::computePurePursuitCommand(config, 1.0, 0.0, 4.0);
  EXPECT_TRUE(command.valid);
  EXPECT_FALSE(command.aligning);
  EXPECT_GT(command.linear, 0.0);
  EXPECT_DOUBLE_EQ(0.0, command.angular);
}

TEST(PurePursuitCore, TurnDirectionMatchesTargetSide)
{
  rap::PurePursuitConfig config;
  const auto left = rap::computePurePursuitCommand(config, 1.0, 0.5, 4.0);
  const auto right = rap::computePurePursuitCommand(config, 1.0, -0.5, 4.0);
  EXPECT_GT(left.angular, 0.0);
  EXPECT_LT(right.angular, 0.0);
  EXPECT_NEAR(left.linear, right.linear, 1.0e-12);
}

TEST(PurePursuitCore, TargetBehindRotatesWithoutDriving)
{
  rap::PurePursuitConfig config;
  const auto command = rap::computePurePursuitCommand(config, -1.0, 0.2, 4.0);
  EXPECT_TRUE(command.valid);
  EXPECT_TRUE(command.aligning);
  EXPECT_DOUBLE_EQ(0.0, command.linear);
  EXPECT_NE(0.0, command.angular);
}

TEST(PurePursuitCore, GoalToleranceProducesStop)
{
  rap::PurePursuitConfig config;
  const auto command = rap::computePurePursuitCommand(config, 0.2, 0.0, 0.2);
  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(0.0, command.linear);
  EXPECT_DOUBLE_EQ(0.0, command.angular);
}

TEST(PurePursuitCore, DegenerateTargetIsRejected)
{
  rap::PurePursuitConfig config;
  const auto command = rap::computePurePursuitCommand(config, 0.0, 0.0, 4.0);
  EXPECT_FALSE(command.valid);
}
