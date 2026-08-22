#include <limits>
#include <gtest/gtest.h>
#include "risk_aware_planner_cpp/safety_gate_core.hpp"

namespace risk_aware_planner_cpp
{
SafetyGateInput validInput()
{
  SafetyGateInput input;
  input.enabled = true;
  input.estop = false;
  input.command_finite = true;
  input.navigation_tracking = true;
  input.path_valid = true;
  input.path_version_valid = true;
  input.command_age_sec = input.odom_age_sec = input.map_age_sec = 0.01;
  input.path_age_sec = input.state_age_sec = input.estop_age_sec = 0.01;
  input.linear = 0.1;
  input.angular = 0.2;
  return input;
}

TEST(SafetyGateCore, ValidFreshInputsPass)
{
  const auto out = evaluateSafetyGate(SafetyGateConfig(), validInput(), 0.0, 0.0, 1.0);
  EXPECT_TRUE(out.allowed);
  EXPECT_DOUBLE_EQ(out.linear, 0.1);
}

TEST(SafetyGateCore, EveryStaleRequiredInputStops)
{
  SafetyGateConfig config;
  for (int field = 0; field < 5; ++field) {
    auto input = validInput();
    if (field == 0) input.command_age_sec = 10.0;
    if (field == 1) input.odom_age_sec = 10.0;
    if (field == 2) input.map_age_sec = 10.0;
    if (field == 3) input.state_age_sec = 10.0;
    if (field == 4) input.estop_age_sec = 10.0;
    const auto out = evaluateSafetyGate(config, input, 0.1, 0.1, 0.05);
    EXPECT_FALSE(out.allowed) << field;
    EXPECT_DOUBLE_EQ(out.linear, 0.0) << field;
  }
}

TEST(SafetyGateCore, RejectsNonFiniteAndInvalidVersion)
{
  auto input = validInput();
  input.linear = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(evaluateSafetyGate(SafetyGateConfig(), input, 0.0, 0.0, 0.1).allowed);
  input = validInput();
  input.path_version_valid = false;
  EXPECT_FALSE(evaluateSafetyGate(SafetyGateConfig(), input, 0.0, 0.0, 0.1).allowed);
}

TEST(SafetyGateCore, AppliesSpeedAndAccelerationLimits)
{
  auto input = validInput();
  input.linear = 9.0;
  input.angular = -9.0;
  const auto out = evaluateSafetyGate(SafetyGateConfig(), input, 0.0, 0.0, 0.1);
  EXPECT_TRUE(out.allowed);
  EXPECT_TRUE(out.saturated);
  EXPECT_NEAR(out.linear, 0.03, 1e-12);
  EXPECT_NEAR(out.angular, -0.15, 1e-12);
}
}  // namespace risk_aware_planner_cpp
