#include "risk_aware_planner_cpp/safety_gate_core.hpp"

#include <algorithm>
#include <cmath>

namespace risk_aware_planner_cpp
{
namespace
{
bool fresh(double age, double timeout)
{
  return age >= 0.0 && (timeout <= 0.0 || age <= timeout);
}

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double approach(double current, double target, double delta)
{
  return target > current ? std::min(target, current + delta) :
         std::max(target, current - delta);
}
}  // namespace

SafetyGateOutput evaluateSafetyGate(
  const SafetyGateConfig & config, const SafetyGateInput & input,
  double previous_linear, double previous_angular, double dt)
{
  SafetyGateOutput output;
  auto reject = [&output](const char * reason) {
      output.reason = reason;
      return output;
    };
  if (!input.enabled) {return reject("DISABLED");}
  if (!fresh(input.estop_age_sec, config.estop_timeout_sec)) {return reject("STALE_ESTOP");}
  if (input.estop) {return reject("E_STOP_ACTIVE");}
  if (!input.navigation_tracking) {return reject("NAVIGATION_NOT_TRACKING");}
  if (!input.path_valid) {return reject("INVALID_PATH");}
  if (!input.path_version_valid) {return reject("INVALID_PATH_VERSION");}
  if (!fresh(input.state_age_sec, config.state_timeout_sec)) {return reject("STALE_STATE");}
  if (!fresh(input.path_age_sec, config.path_timeout_sec)) {return reject("STALE_PATH");}
  if (!fresh(input.odom_age_sec, config.odom_timeout_sec)) {return reject("STALE_ODOM");}
  if (!fresh(input.map_age_sec, config.map_timeout_sec)) {return reject("STALE_MAP");}
  if (!fresh(input.command_age_sec, config.command_timeout_sec)) {return reject("STALE_COMMAND");}
  if (!input.command_finite || !std::isfinite(input.linear) || !std::isfinite(input.angular)) {
    return reject("INVALID_COMMAND");
  }
  const double target_linear = clamp(
    input.linear, config.allow_reverse ? -config.max_linear_speed : 0.0,
    config.max_linear_speed);
  const double target_angular = clamp(
    input.angular, -config.max_angular_speed, config.max_angular_speed);
  output.saturated = target_linear != input.linear || target_angular != input.angular;
  output.linear = approach(previous_linear, target_linear, config.max_linear_accel * std::max(0.0, dt));
  output.angular = approach(previous_angular, target_angular, config.max_angular_accel * std::max(0.0, dt));
  output.allowed = true;
  output.reason = output.saturated ? "ACTIVE_SATURATED" : "ACTIVE";
  return output;
}
}  // namespace risk_aware_planner_cpp
