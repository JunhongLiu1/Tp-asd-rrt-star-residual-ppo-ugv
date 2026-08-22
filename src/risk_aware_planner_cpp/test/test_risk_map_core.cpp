#include <algorithm>
#include <gtest/gtest.h>
#include "risk_aware_planner_cpp/risk_map_core.hpp"
namespace risk_aware_planner_cpp
{
RiskMapGeometry geometry() {return RiskMapGeometry{10, 10, 1.0, 0.0, 0.0, 0.0};}
TEST(RiskMapCore, VersionsIncreaseAndNeverReset)
{
  RiskMapCore map;
  EXPECT_TRUE(map.resetGeometry(geometry()));
  const auto initial = map.version();
  EXPECT_TRUE(map.applyMeasurement(5.0, 5.0, 12.5, 1.0));
  EXPECT_GT(map.version(), initial);
  const auto measured = map.version();
  auto changed = geometry(); changed.width = 11;
  EXPECT_TRUE(map.resetGeometry(changed));
  EXPECT_GT(map.version(), measured);
}
TEST(RiskMapCore, SameSequenceIsDeterministicAndContinuous)
{
  RiskMapCore a, b; a.resetGeometry(geometry()); b.resetGeometry(geometry());
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(a.applyMeasurement(5.0, 5.0, 8.123 + i, i + 1.0));
    ASSERT_TRUE(b.applyMeasurement(5.0, 5.0, 8.123 + i, i + 1.0));
  }
  EXPECT_EQ(a.version(), b.version());
  EXPECT_EQ(a.doseRate(), b.doseRate());
  EXPECT_GT(*std::max_element(a.doseRate().begin(), a.doseRate().end()), 8.0F);
}
TEST(RiskMapCore, RejectsOutOfOrderAndInvalidMeasurements)
{
  RiskMapCore map; map.resetGeometry(geometry());
  ASSERT_TRUE(map.applyMeasurement(5.0, 5.0, 1.0, 2.0));
  const auto version = map.version();
  EXPECT_FALSE(map.applyMeasurement(5.0, 5.0, 2.0, 1.0));
  EXPECT_FALSE(map.applyMeasurement(5.0, 5.0, -1.0, 3.0));
  EXPECT_EQ(map.version(), version);
}
TEST(RiskMapCore, TracksCoverageAndConfidenceDecay)
{
  RiskMapConfig config; config.confidence_decay_per_sec = 1.0;
  RiskMapCore map(config); map.resetGeometry(geometry());
  ASSERT_TRUE(map.applyMeasurement(5.0, 5.0, 1.0, 1.0));
  const auto before = map.confidence()[55];
  map.decayTo(2.0);
  EXPECT_GT(map.coverage(), 0.0);
  EXPECT_LT(map.confidence()[55], before);
}
}  // namespace risk_aware_planner_cpp
