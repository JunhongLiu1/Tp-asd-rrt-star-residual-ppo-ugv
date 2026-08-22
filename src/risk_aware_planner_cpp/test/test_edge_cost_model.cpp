#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "risk_aware_planner_cpp/edge_cost_model.hpp"

namespace risk_aware_planner_cpp
{

namespace
{

EdgeCostWeights weights(bool include_time = true)
{
  return EdgeCostWeights{0.2, 0.4, 0.4, 0.25, include_time};
}

}  // namespace

TEST(EdgeCostModel, TerrainRadiationAndTimeAreIndividuallyMonotonic)
{
  const EdgeCostComponents baseline{0.1, 0.1, 0.1, 0.1};
  const double baseline_cost = combineEdgeCost(baseline, weights());

  EdgeCostComponents terrain = baseline;
  terrain.terrain = 0.2;
  EXPECT_GT(combineEdgeCost(terrain, weights()), baseline_cost);

  EdgeCostComponents radiation = baseline;
  radiation.radiation = 0.2;
  EXPECT_GT(combineEdgeCost(radiation, weights()), baseline_cost);

  EdgeCostComponents time = baseline;
  time.traversal_time = 0.2;
  EXPECT_GT(combineEdgeCost(time, weights()), baseline_cost);
}

TEST(EdgeCostModel, DisabledTimePenaltyRemovesOnlyTimeContribution)
{
  const EdgeCostComponents fast{0.1, 0.2, 0.3, 0.1};
  EdgeCostComponents slow = fast;
  slow.traversal_time = 0.8;

  EXPECT_LT(combineEdgeCost(fast, weights()),
    combineEdgeCost(slow, weights()));
  EXPECT_DOUBLE_EQ(combineEdgeCost(fast, weights(false)),
    combineEdgeCost(slow, weights(false)));
}

TEST(EdgeCostModel, InvalidComponentsFailClosed)
{
  EdgeCostComponents invalid{0.1, 0.2, 0.3, 0.4};
  invalid.radiation = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(std::isinf(combineEdgeCost(invalid, weights())));
}

}  // namespace risk_aware_planner_cpp
