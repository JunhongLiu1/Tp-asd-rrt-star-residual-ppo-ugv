#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "risk_aware_planner_cpp/pid_core.hpp"

using risk_aware_planner_cpp::PidConfig;
using risk_aware_planner_cpp::PidController;

TEST(PidCore, ProportionalResponseUsesTrackingError)
{
  PidConfig config;
  config.kp = 2.0;
  config.integral_min = -1.0;
  config.integral_max = 1.0;
  config.output_min = -10.0;
  config.output_max = 10.0;
  PidController controller(config);
  const auto result = controller.update(1.0, 0.25, 0.1);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.error, 0.75);
  EXPECT_DOUBLE_EQ(result.output, 1.5);
}

TEST(PidCore, IntegralAndOutputAreBoundedWithAntiWindup)
{
  PidConfig config;
  config.ki = 1.0;
  config.integral_min = -0.5;
  config.integral_max = 0.5;
  config.output_min = -0.2;
  config.output_max = 0.2;
  PidController controller(config);
  for (int i = 0; i < 100; ++i) {
    const auto result = controller.update(1.0, 0.0, 0.1);
    EXPECT_LE(result.output, 0.2);
  }
  EXPECT_LE(controller.integralState(), 0.2 + 1.0e-12);
}

TEST(PidCore, RuntimeActuatorLimitParticipatesInAntiWindup)
{
  PidConfig config;
  config.kp = 1.0;
  config.ki = 1.0;
  config.integral_min = -10.0;
  config.integral_max = 10.0;
  config.output_min = -1.0;
  config.output_max = 1.0;
  PidController controller(config);

  // A 0.8 feed-forward command with a 1.0 actuator limit leaves only 0.2
  // positive correction. The PID must not integrate behind that outer limit.
  for (int i = 0; i < 100; ++i) {
    const auto result = controller.update(0.8, 0.0, 0.1, -0.8, 0.2);
    ASSERT_TRUE(result.valid);
    EXPECT_LE(result.output, 0.2);
    EXPECT_TRUE(result.saturated);
  }
  EXPECT_DOUBLE_EQ(controller.integralState(), 0.0);

  // Opposite error must still be able to move the controller away from the
  // upper limit.
  const auto recovering = controller.update(0.8, 1.0, 0.1, -0.8, 0.2);
  ASSERT_TRUE(recovering.valid);
  EXPECT_LT(recovering.output, 0.0);
}

TEST(PidCore, RuntimeLimitsCannotWidenConfiguredLimits)
{
  PidConfig config;
  config.kp = 10.0;
  config.integral_min = -1.0;
  config.integral_max = 1.0;
  config.output_min = -0.25;
  config.output_max = 0.25;
  PidController controller(config);

  const auto result = controller.update(1.0, 0.0, 0.1, -10.0, 10.0);
  ASSERT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(result.output, 0.25);
  EXPECT_TRUE(result.saturated);
}

TEST(PidCore, InvalidRuntimeLimitsResetState)
{
  PidConfig config;
  config.ki = 1.0;
  config.integral_min = -1.0;
  config.integral_max = 1.0;
  config.output_min = -1.0;
  config.output_max = 1.0;
  PidController controller(config);
  ASSERT_TRUE(controller.update(1.0, 0.0, 0.1).valid);
  ASSERT_GT(controller.integralState(), 0.0);

  const auto invalid = controller.update(1.0, 0.0, 0.1, 1.0, -1.0);
  EXPECT_FALSE(invalid.valid);
  EXPECT_DOUBLE_EQ(controller.integralState(), 0.0);
}

TEST(PidCore, DerivativeUsesErrorChange)
{
  PidConfig config;
  config.kd = 0.5;
  config.integral_min = -1.0;
  config.integral_max = 1.0;
  config.output_min = -10.0;
  config.output_max = 10.0;
  PidController controller(config);
  EXPECT_DOUBLE_EQ(controller.update(1.0, 0.0, 0.1).derivative, 0.0);
  EXPECT_NEAR(controller.update(1.0, 0.5, 0.1).derivative, -2.5, 1.0e-12);
}

TEST(PidCore, InvalidInputResetsState)
{
  PidConfig config;
  config.ki = 1.0;
  config.integral_min = -1.0;
  config.integral_max = 1.0;
  config.output_min = -10.0;
  config.output_max = 10.0;
  PidController controller(config);
  ASSERT_TRUE(controller.update(1.0, 0.0, 0.1).valid);
  EXPECT_GT(controller.integralState(), 0.0);
  const auto invalid = controller.update(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1);
  EXPECT_FALSE(invalid.valid);
  EXPECT_DOUBLE_EQ(controller.integralState(), 0.0);
}

TEST(PidCore, RejectsInvalidConfiguration)
{
  PidConfig config;
  config.kp = -1.0;
  EXPECT_THROW(PidController controller(config), std::invalid_argument);
}
