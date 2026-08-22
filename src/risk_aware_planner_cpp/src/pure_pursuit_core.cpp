#include "risk_aware_planner_cpp/pure_pursuit_core.hpp"

#include <algorithm>
#include <cmath>

namespace risk_aware_planner_cpp
{
namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

}  // namespace

double computeDynamicLookahead(
  const PurePursuitConfig & config,
  double measured_linear_speed)
{
  return clamp(
    config.base_lookahead +
    config.lookahead_time * std::abs(measured_linear_speed),
    config.min_lookahead,
    config.max_lookahead);
}

PurePursuitCommand computePurePursuitCommand(
  const PurePursuitConfig & config,
  double target_x_robot,
  double target_y_robot,
  double goal_distance)
{
  PurePursuitCommand command;
  const double target_distance_sq =
    target_x_robot * target_x_robot +
    target_y_robot * target_y_robot;

  if (!std::isfinite(target_distance_sq) || target_distance_sq <= 1.0e-8 ||
    !std::isfinite(goal_distance))
  {
    return command;
  }

  command.valid = true;
  if (goal_distance <= config.goal_tolerance) {
    return command;
  }

  if (target_x_robot <= 0.0) {
    command.aligning = true;
    const double target_bearing = std::atan2(target_y_robot, target_x_robot);
    command.angular = clamp(
      config.rotate_in_place_angular_gain * target_bearing,
      -config.max_angular_speed,
      config.max_angular_speed);
    return command;
  }

  command.curvature = 2.0 * target_y_robot / target_distance_sq;
  const double curvature_factor = 1.0 /
    (1.0 + config.curvature_speed_gain * std::abs(command.curvature));
  const double goal_factor = clamp(
    (goal_distance - config.goal_tolerance) /
    (config.goal_slowdown_distance - config.goal_tolerance),
    0.0, 1.0);
  const double tracking_speed = std::max(
    config.minimum_tracking_speed,
    config.max_linear_speed * curvature_factor);

  command.linear = clamp(
    tracking_speed * goal_factor, 0.0, config.max_linear_speed);
  command.angular = clamp(
    command.linear * command.curvature,
    -config.max_angular_speed,
    config.max_angular_speed);
  return command;
}

}  // namespace risk_aware_planner_cpp
