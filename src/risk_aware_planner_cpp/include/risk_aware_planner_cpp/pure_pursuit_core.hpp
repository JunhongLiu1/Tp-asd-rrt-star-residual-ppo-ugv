#ifndef RISK_AWARE_PLANNER_CPP__PURE_PURSUIT_CORE_HPP_
#define RISK_AWARE_PLANNER_CPP__PURE_PURSUIT_CORE_HPP_

namespace risk_aware_planner_cpp
{

struct PurePursuitConfig
{
  double base_lookahead{0.70};
  double min_lookahead{0.45};
  double max_lookahead{1.20};
  double lookahead_time{1.50};
  double goal_tolerance{0.35};
  double goal_slowdown_distance{1.50};
  double max_linear_speed{0.20};
  double max_angular_speed{0.80};
  double minimum_tracking_speed{0.04};
  double curvature_speed_gain{1.50};
  double rotate_in_place_angular_gain{1.50};
};

struct PurePursuitCommand
{
  bool valid{false};
  bool aligning{false};
  double linear{0.0};
  double angular{0.0};
  double curvature{0.0};
};

double computeDynamicLookahead(
  const PurePursuitConfig & config,
  double measured_linear_speed);

PurePursuitCommand computePurePursuitCommand(
  const PurePursuitConfig & config,
  double target_x_robot,
  double target_y_robot,
  double goal_distance);

}  // namespace risk_aware_planner_cpp

#endif  // RISK_AWARE_PLANNER_CPP__PURE_PURSUIT_CORE_HPP_
