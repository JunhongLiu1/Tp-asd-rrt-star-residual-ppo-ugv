#ifndef RISK_AWARE_PLANNER_CPP__EDGE_COST_MODEL_HPP_
#define RISK_AWARE_PLANNER_CPP__EDGE_COST_MODEL_HPP_

#include <cmath>
#include <limits>

namespace risk_aware_planner_cpp
{

struct EdgeCostComponents
{
  double distance{0.0};
  double terrain{0.0};
  double radiation{0.0};
  double traversal_time{0.0};
};

struct EdgeCostWeights
{
  double distance{0.0};
  double terrain{0.0};
  double radiation{0.0};
  double traversal_time{0.0};
  bool include_time_penalty{true};
};

inline double combineEdgeCost(
  const EdgeCostComponents & components,
  const EdgeCostWeights & weights)
{
  const double values[] = {
    components.distance, components.terrain, components.radiation,
    components.traversal_time, weights.distance, weights.terrain,
    weights.radiation, weights.traversal_time,
  };
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      return std::numeric_limits<double>::infinity();
    }
  }
  double total = weights.distance * components.distance +
    weights.terrain * components.terrain +
    weights.radiation * components.radiation;
  if (weights.include_time_penalty) {
    total += weights.traversal_time * components.traversal_time;
  }
  return total;
}

}  // namespace risk_aware_planner_cpp

#endif  // RISK_AWARE_PLANNER_CPP__EDGE_COST_MODEL_HPP_
