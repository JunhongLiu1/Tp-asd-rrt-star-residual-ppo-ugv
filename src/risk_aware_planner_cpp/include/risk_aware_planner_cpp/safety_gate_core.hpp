#ifndef RISK_AWARE_PLANNER_CPP__SAFETY_GATE_CORE_HPP_
#define RISK_AWARE_PLANNER_CPP__SAFETY_GATE_CORE_HPP_

#include <string>

namespace risk_aware_planner_cpp
{

struct SafetyGateConfig
{
  double command_timeout_sec{0.5};
  double odom_timeout_sec{1.0};
  double map_timeout_sec{5.0};
  double path_timeout_sec{0.0};
  double state_timeout_sec{1.0};
  double estop_timeout_sec{1.0};
  double max_linear_speed{0.2};
  double max_angular_speed{0.6};
  double max_linear_accel{0.3};
  double max_angular_accel{1.5};
  bool allow_reverse{false};
};

struct SafetyGateInput
{
  bool enabled{false};
  bool estop{true};
  bool command_finite{false};
  bool navigation_tracking{false};
  bool path_valid{false};
  bool path_version_valid{false};
  double command_age_sec{-1.0};
  double odom_age_sec{-1.0};
  double map_age_sec{-1.0};
  double path_age_sec{-1.0};
  double state_age_sec{-1.0};
  double estop_age_sec{-1.0};
  double linear{0.0};
  double angular{0.0};
};

struct SafetyGateOutput
{
  double linear{0.0};
  double angular{0.0};
  bool allowed{false};
  bool saturated{false};
  std::string reason;
};

SafetyGateOutput evaluateSafetyGate(
  const SafetyGateConfig & config,
  const SafetyGateInput & input,
  double previous_linear,
  double previous_angular,
  double dt);

}  // namespace risk_aware_planner_cpp

#endif
