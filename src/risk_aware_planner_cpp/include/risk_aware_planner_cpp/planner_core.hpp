#ifndef RISK_AWARE_PLANNER_CPP__PLANNER_CORE_HPP_
#define RISK_AWARE_PLANNER_CPP__PLANNER_CORE_HPP_

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace risk_aware_planner_cpp
{

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct Bounds
{
  double min_x{-5.0};
  double max_x{5.0};
  double min_y{-5.0};
  double max_y{5.0};
};

enum class FailureCode
{
  SUCCESS,
  MAP_UNAVAILABLE,
  STALE_MAP,
  INVALID_START,
  INVALID_GOAL,
  NO_FEASIBLE_PATH,
  TIMEOUT,
  CANCELLED,
  PATH_VALIDATION_FAILED,
};

std::string toString(FailureCode code);

struct PlannerConfig
{
  double step_size{0.45};
  double search_radius{1.0};
  double goal_radius{0.5};
  std::size_t max_iterations{1800};
  double planning_timeout_sec{2.0};
  double goal_sample_rate{0.12};
  unsigned int random_seed{31};
  bool stop_on_first_feasible{false};
  bool enable_adaptive_sampling{true};
  std::size_t sampling_region_rows{8};
  std::size_t sampling_region_cols{8};
  double risk_sample_rate{0.35};
  double min_uniform_sample_rate{0.15};
  double min_goal_sample_rate{0.05};
  double max_goal_sample_rate{0.45};
  double min_risk_sample_rate{0.10};
  double max_risk_sample_rate{0.70};
  double low_risk_exponent{3.0};
  double region_goal_distance_gain{0.25};
  double gradient_epsilon{0.25};
  double random_direction_weight{0.55};
  double goal_attraction_weight{0.25};
  double risk_repulsion_weight{0.35};
  std::size_t adaptation_interval{100};
  std::size_t stagnation_iterations{100};
  double adaptation_step{0.05};
};

struct SamplingProbabilities
{
  double uniform{0.53};
  double goal{0.12};
  double risk{0.35};
};

struct TreeNode
{
  Point point;
  TreeNode * parent{nullptr};
  double cost{0.0};
  std::vector<TreeNode *> children;
};

struct PlanningResult
{
  bool success{false};
  std::vector<Point> path;
  double cost{std::numeric_limits<double>::infinity()};
  FailureCode failure_code{FailureCode::NO_FEASIBLE_PATH};
  double planning_time_sec{0.0};
  std::size_t iterations{0};
  std::size_t node_count{0};
  std::size_t uniform_sample_count{0};
  std::size_t goal_sample_count{0};
  std::size_t risk_sample_count{0};
  std::size_t guided_extension_count{0};
  std::size_t adaptation_count{0};
  // -1 means that no feasible path was found.
  double first_feasible_time_sec{-1.0};
  std::string message;
};

class PlannerCoreTestAccess;

class PlannerCore
{
public:
  using EdgeCostFn = std::function<double(const Point &, const Point &)>;
  using PointValidityFn = std::function<bool(const Point &)>;
  using CancelCheckFn = std::function<bool()>;
  using ClockFn = std::function<double()>;
  // Return normalized combined terrain+radiation risk in [0, 1].
  using SamplingRiskFn = std::function<double(const Point &)>;

  PlannerCore(
    EdgeCostFn edge_cost,
    Bounds bounds,
    PlannerConfig config = PlannerConfig(),
    PointValidityFn point_validity = PointValidityFn(),
    ClockFn clock = ClockFn(),
    SamplingRiskFn sampling_risk = SamplingRiskFn());

  PlanningResult plan(
    const Point & start,
    const Point & goal,
    CancelCheckFn cancel_check = CancelCheckFn());

private:
  friend class PlannerCoreTestAccess;

  static double distance(const Point & first, const Point & second);
  static bool isFinitePoint(const Point & point);
  static bool samePoint(const Point & first, const Point & second);

  bool validPoint(const Point & point) const;
  bool safeEdgeCost(
    const Point & first,
    const Point & second,
    double & cost) const;
  Point sample(const Point & goal);
  Point adaptiveSample(const Point & goal);
  bool guidedTarget(
    const TreeNode & source, const Point & sampled, const Point & goal,
    Point & target);
  bool safeRisk(const Point & point, double & risk) const;
  void buildSamplingRegions(const Point & goal);
  void adaptProbabilities(bool stagnating, double progress);
  void normalizeProbabilities();
  TreeNode * nearest(const Point & point) const;
  bool steer(
    const TreeNode & source,
    const Point & target,
    Point & result) const;
  std::vector<TreeNode *> nearby(const TreeNode & node) const;

  static bool isAncestor(const TreeNode * ancestor, const TreeNode * node);
  static void attach(TreeNode * parent, TreeNode * child);
  static void detach(TreeNode * parent, TreeNode * child);
  static void reparent(
    TreeNode * node,
    TreeNode * new_parent,
    double new_cost);

  PlanningResult makeResult(
    double started,
    bool success,
    std::vector<Point> path,
    double cost,
    FailureCode failure_code,
    std::size_t iterations,
    double first_feasible_time_sec,
    const std::string & message) const;

  double elapsed(double started) const;

  EdgeCostFn edge_cost_;
  Bounds bounds_;
  PlannerConfig config_;
  PointValidityFn point_validity_;
  ClockFn clock_;
  SamplingRiskFn sampling_risk_;
  struct SamplingRegion
  {
    Bounds bounds;
    double weight{0.0};
  };
  std::vector<SamplingRegion> sampling_regions_;
  SamplingProbabilities sampling_probabilities_;
  std::size_t uniform_sample_count_{0};
  std::size_t goal_sample_count_{0};
  std::size_t risk_sample_count_{0};
  std::size_t guided_extension_count_{0};
  std::size_t adaptation_count_{0};
  std::vector<std::unique_ptr<TreeNode>> nodes_;
  std::mt19937 random_generator_;
};

}  // namespace risk_aware_planner_cpp

#endif  // RISK_AWARE_PLANNER_CPP__PLANNER_CORE_HPP_
