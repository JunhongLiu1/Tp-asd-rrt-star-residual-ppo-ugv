#include "risk_aware_planner_cpp/planner_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>

namespace risk_aware_planner_cpp
{

namespace
{

double steadyClockSeconds()
{
  const auto now = std::chrono::steady_clock::now();
  const auto epoch = now.time_since_epoch();
  return std::chrono::duration<double>(epoch).count();
}

}  // namespace

std::string toString(FailureCode code)
{
  switch (code) {
    case FailureCode::SUCCESS:
      return "SUCCESS";
    case FailureCode::MAP_UNAVAILABLE:
      return "MAP_UNAVAILABLE";
    case FailureCode::STALE_MAP:
      return "STALE_MAP";
    case FailureCode::INVALID_START:
      return "INVALID_START";
    case FailureCode::INVALID_GOAL:
      return "INVALID_GOAL";
    case FailureCode::NO_FEASIBLE_PATH:
      return "NO_FEASIBLE_PATH";
    case FailureCode::TIMEOUT:
      return "TIMEOUT";
    case FailureCode::CANCELLED:
      return "CANCELLED";
    case FailureCode::PATH_VALIDATION_FAILED:
      return "PATH_VALIDATION_FAILED";
  }
  return "UNKNOWN";
}

PlannerCore::PlannerCore(
  EdgeCostFn edge_cost,
  Bounds bounds,
  PlannerConfig config,
  PointValidityFn point_validity,
  ClockFn clock,
  SamplingRiskFn sampling_risk)
: edge_cost_(std::move(edge_cost)),
  bounds_(bounds),
  config_(config),
  point_validity_(std::move(point_validity)),
  clock_(std::move(clock)),
  sampling_risk_(std::move(sampling_risk)),
  random_generator_(config.random_seed)
{
  if (!edge_cost_) {
    throw std::invalid_argument("edge_cost callback must be provided");
  }
  if (!std::isfinite(bounds_.min_x) || !std::isfinite(bounds_.max_x) ||
    !std::isfinite(bounds_.min_y) || !std::isfinite(bounds_.max_y) ||
    bounds_.min_x >= bounds_.max_x || bounds_.min_y >= bounds_.max_y)
  {
    throw std::invalid_argument("bounds must be finite and ordered");
  }
  if (!std::isfinite(config_.step_size) || config_.step_size <= 0.0 ||
    !std::isfinite(config_.search_radius) || config_.search_radius <= 0.0 ||
    !std::isfinite(config_.goal_radius) || config_.goal_radius <= 0.0)
  {
    throw std::invalid_argument("geometric planner parameters must be positive");
  }
  if (config_.max_iterations == 0) {
    throw std::invalid_argument("max_iterations must be positive");
  }
  if (!std::isfinite(config_.planning_timeout_sec) ||
    config_.planning_timeout_sec < 0.0)
  {
    throw std::invalid_argument("planning_timeout_sec must be non-negative");
  }
  if (!std::isfinite(config_.goal_sample_rate) ||
    config_.goal_sample_rate < 0.0 || config_.goal_sample_rate > 1.0)
  {
    throw std::invalid_argument("goal_sample_rate must be inside [0, 1]");
  }
  if (config_.enable_adaptive_sampling) {
    const double adaptive_values[] = {
      config_.risk_sample_rate, config_.min_uniform_sample_rate,
      config_.min_goal_sample_rate, config_.max_goal_sample_rate,
      config_.min_risk_sample_rate, config_.max_risk_sample_rate,
      config_.low_risk_exponent, config_.region_goal_distance_gain,
      config_.gradient_epsilon, config_.random_direction_weight,
      config_.goal_attraction_weight, config_.risk_repulsion_weight,
      config_.adaptation_step,
    };
    for (double value : adaptive_values) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument(
                "adaptive sampling parameters must be finite and non-negative");
      }
    }
    if (config_.sampling_region_rows == 0 ||
      config_.sampling_region_cols == 0 ||
      config_.sampling_region_rows > 256 ||
      config_.sampling_region_cols > 256 ||
      config_.sampling_region_rows * config_.sampling_region_cols > 4096 ||
      config_.adaptation_interval == 0 ||
      config_.stagnation_iterations == 0 || config_.gradient_epsilon <= 0.0 ||
      config_.low_risk_exponent <= 0.0 ||
      config_.min_uniform_sample_rate >= 1.0 ||
      config_.min_goal_sample_rate > config_.max_goal_sample_rate ||
      config_.min_risk_sample_rate > config_.max_risk_sample_rate ||
      config_.min_uniform_sample_rate + config_.min_goal_sample_rate +
      config_.min_risk_sample_rate > 1.0 ||
      config_.goal_sample_rate + config_.risk_sample_rate > 1.0 ||
      config_.random_direction_weight + config_.goal_attraction_weight +
      config_.risk_repulsion_weight <= 0.0)
    {
      throw std::invalid_argument("adaptive sampling configuration is invalid");
    }
  }
  if (!clock_) {
    clock_ = steadyClockSeconds;
  }
  if (!point_validity_) {
    point_validity_ = [](const Point &) {return true;};
  }
}

double PlannerCore::distance(const Point & first, const Point & second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

bool PlannerCore::isFinitePoint(const Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool PlannerCore::samePoint(const Point & first, const Point & second)
{
  return distance(first, second) <= 1.0e-12;
}

bool PlannerCore::validPoint(const Point & point) const
{
  return isFinitePoint(point) && point_validity_(point);
}

bool PlannerCore::safeEdgeCost(
  const Point & first,
  const Point & second,
  double & cost) const
{
  cost = edge_cost_(first, second);
  return std::isfinite(cost) && cost >= 0.0;
}

Point PlannerCore::sample(const Point & goal)
{
  std::uniform_real_distribution<double> probability(0.0, 1.0);
  if (probability(random_generator_) < config_.goal_sample_rate) {
    return goal;
  }

  std::uniform_real_distribution<double> x_distribution(
    bounds_.min_x, bounds_.max_x);
  std::uniform_real_distribution<double> y_distribution(
    bounds_.min_y, bounds_.max_y);
  return Point{x_distribution(random_generator_), y_distribution(random_generator_)};
}

bool PlannerCore::safeRisk(const Point & point, double & risk) const
{
  if (!sampling_risk_) {
    return false;
  }
  try {
    risk = sampling_risk_(point);
  } catch (...) {
    return false;
  }
  return std::isfinite(risk) && risk >= 0.0 && risk <= 1.0;
}

void PlannerCore::buildSamplingRegions(const Point & goal)
{
  sampling_regions_.clear();
  if (!sampling_risk_) {
    return;
  }
  const double width = (bounds_.max_x - bounds_.min_x) /
    static_cast<double>(config_.sampling_region_cols);
  const double height = (bounds_.max_y - bounds_.min_y) /
    static_cast<double>(config_.sampling_region_rows);
  for (std::size_t row = 0; row < config_.sampling_region_rows; ++row) {
    for (std::size_t col = 0; col < config_.sampling_region_cols; ++col) {
      Bounds region{
        bounds_.min_x + static_cast<double>(col) * width,
        bounds_.min_x + static_cast<double>(col + 1) * width,
        bounds_.min_y + static_cast<double>(row) * height,
        bounds_.min_y + static_cast<double>(row + 1) * height,
      };
      const Point center{
        0.5 * (region.min_x + region.max_x),
        0.5 * (region.min_y + region.max_y),
      };
      double risk = 0.0;
      if (!safeRisk(center, risk)) {
        continue;
      }
      const double low_risk = std::max(
        0.05, std::pow(1.0 - risk, config_.low_risk_exponent));
      const double goal_weight = 1.0 /
        (1.0 + config_.region_goal_distance_gain * distance(center, goal));
      sampling_regions_.push_back(
        SamplingRegion{region, low_risk * goal_weight});
    }
  }
}

void PlannerCore::normalizeProbabilities()
{
  double goal = std::max(
    config_.min_goal_sample_rate,
    std::min(config_.max_goal_sample_rate, sampling_probabilities_.goal));
  double risk = std::max(
    config_.min_risk_sample_rate,
    std::min(config_.max_risk_sample_rate, sampling_probabilities_.risk));
  const double available = 1.0 - config_.min_uniform_sample_rate;
  if (goal + risk > available) {
    double excess = goal + risk - available;
    const double risk_reduction = std::min(
      excess, risk - config_.min_risk_sample_rate);
    risk -= risk_reduction;
    excess -= risk_reduction;
    goal -= std::min(
      excess, goal - config_.min_goal_sample_rate);
  }
  sampling_probabilities_.goal = goal;
  sampling_probabilities_.risk = risk;
  sampling_probabilities_.uniform = 1.0 - goal - risk;
}

void PlannerCore::adaptProbabilities(bool stagnating, double progress)
{
  progress = std::max(0.0, std::min(1.0, progress));
  if (stagnating) {
    sampling_probabilities_.uniform += config_.adaptation_step;
    sampling_probabilities_.goal += 0.5 * config_.adaptation_step;
    sampling_probabilities_.risk -= 1.5 * config_.adaptation_step;
  } else {
    sampling_probabilities_.goal += config_.adaptation_step * progress;
    sampling_probabilities_.risk += config_.adaptation_step * (1.0 - progress);
  }
  normalizeProbabilities();
  ++adaptation_count_;
}

Point PlannerCore::adaptiveSample(const Point & goal)
{
  std::uniform_real_distribution<double> probability(0.0, 1.0);
  const double choice = probability(random_generator_);
  if (choice < sampling_probabilities_.goal) {
    ++goal_sample_count_;
    return goal;
  }
  if (choice < sampling_probabilities_.goal +
    sampling_probabilities_.risk && !sampling_regions_.empty())
  {
    double total_weight = 0.0;
    for (const auto & region : sampling_regions_) {
      total_weight += region.weight;
    }
    if (std::isfinite(total_weight) && total_weight > 0.0) {
      std::uniform_real_distribution<double> select(0.0, total_weight);
      const double threshold = select(random_generator_);
      double cumulative = 0.0;
      const SamplingRegion * selected = &sampling_regions_.back();
      for (const auto & region : sampling_regions_) {
        cumulative += region.weight;
        if (cumulative >= threshold) {
          selected = &region;
          break;
        }
      }
      std::uniform_real_distribution<double> x(
        selected->bounds.min_x, selected->bounds.max_x);
      std::uniform_real_distribution<double> y(
        selected->bounds.min_y, selected->bounds.max_y);
      ++risk_sample_count_;
      return Point{x(random_generator_), y(random_generator_)};
    }
  }
  std::uniform_real_distribution<double> x(bounds_.min_x, bounds_.max_x);
  std::uniform_real_distribution<double> y(bounds_.min_y, bounds_.max_y);
  ++uniform_sample_count_;
  return Point{x(random_generator_), y(random_generator_)};
}

bool PlannerCore::guidedTarget(
  const TreeNode & source, const Point & sampled, const Point & goal,
  Point & target)
{
  const auto unit = [](double x, double y, double & output_x, double & output_y) {
      const double length = std::hypot(x, y);
      if (length <= 1.0e-12) {
        output_x = 0.0;
        output_y = 0.0;
        return;
      }
      output_x = x / length;
      output_y = y / length;
    };
  double random_x = 0.0;
  double random_y = 0.0;
  double goal_x = 0.0;
  double goal_y = 0.0;
  unit(sampled.x - source.point.x, sampled.y - source.point.y,
    random_x, random_y);
  unit(goal.x - source.point.x, goal.y - source.point.y, goal_x, goal_y);
  double repulsive_x = 0.0;
  double repulsive_y = 0.0;
  const double epsilon = config_.gradient_epsilon;
  double plus_x = 0.0;
  double minus_x = 0.0;
  double plus_y = 0.0;
  double minus_y = 0.0;
  if (safeRisk(Point{source.point.x + epsilon, source.point.y}, plus_x) &&
    safeRisk(Point{source.point.x - epsilon, source.point.y}, minus_x) &&
    safeRisk(Point{source.point.x, source.point.y + epsilon}, plus_y) &&
    safeRisk(Point{source.point.x, source.point.y - epsilon}, minus_y))
  {
    unit(-(plus_x - minus_x), -(plus_y - minus_y),
      repulsive_x, repulsive_y);
  }
  const double x = config_.random_direction_weight * random_x +
    config_.goal_attraction_weight * goal_x +
    config_.risk_repulsion_weight * repulsive_x;
  const double y = config_.random_direction_weight * random_y +
    config_.goal_attraction_weight * goal_y +
    config_.risk_repulsion_weight * repulsive_y;
  double direction_x = 0.0;
  double direction_y = 0.0;
  unit(x, y, direction_x, direction_y);
  if (std::hypot(direction_x, direction_y) <= 1.0e-12) {
    target = sampled;
    return false;
  }
  target = Point{
    source.point.x + config_.step_size * direction_x,
    source.point.y + config_.step_size * direction_y,
  };
  ++guided_extension_count_;
  return true;
}

TreeNode * PlannerCore::nearest(const Point & point) const
{
  if (nodes_.empty()) {
    return nullptr;
  }

  TreeNode * result = nodes_.front().get();
  double best_distance = distance(result->point, point);
  for (const auto & candidate : nodes_) {
    const double candidate_distance = distance(candidate->point, point);
    if (candidate_distance < best_distance) {
      best_distance = candidate_distance;
      result = candidate.get();
    }
  }
  return result;
}

bool PlannerCore::steer(
  const TreeNode & source,
  const Point & target,
  Point & result) const
{
  const double length = distance(source.point, target);
  if (length <= 1.0e-12) {
    return false;
  }
  if (length <= config_.step_size) {
    result = target;
    return true;
  }

  const double ratio = config_.step_size / length;
  result = Point{
    source.point.x + ratio * (target.x - source.point.x),
    source.point.y + ratio * (target.y - source.point.y),
  };
  return true;
}

std::vector<TreeNode *> PlannerCore::nearby(const TreeNode & node) const
{
  std::vector<TreeNode *> result;
  for (const auto & candidate : nodes_) {
    if (distance(candidate->point, node.point) <= config_.search_radius) {
      result.push_back(candidate.get());
    }
  }
  return result;
}

bool PlannerCore::isAncestor(
  const TreeNode * ancestor,
  const TreeNode * node)
{
  TreeNode * current = node == nullptr ? nullptr : node->parent;
  while (current != nullptr) {
    if (current == ancestor) {
      return true;
    }
    current = current->parent;
  }
  return false;
}

void PlannerCore::attach(TreeNode * parent, TreeNode * child)
{
  if (parent == nullptr || child == nullptr) {
    throw std::invalid_argument("tree parent and child cannot be null");
  }
  if (std::find(parent->children.begin(), parent->children.end(), child) ==
    parent->children.end())
  {
    parent->children.push_back(child);
  }
  child->parent = parent;
}

void PlannerCore::detach(TreeNode * parent, TreeNode * child)
{
  if (parent == nullptr || child == nullptr) {
    return;
  }
  parent->children.erase(
    std::remove(parent->children.begin(), parent->children.end(), child),
    parent->children.end());
}

void PlannerCore::reparent(
  TreeNode * node,
  TreeNode * new_parent,
  double new_cost)
{
  if (node == nullptr || new_parent == nullptr) {
    throw std::invalid_argument("tree node and new parent cannot be null");
  }
  if (node == new_parent || isAncestor(node, new_parent)) {
    return;
  }

  TreeNode * old_parent = node->parent;
  if (old_parent == new_parent) {
    node->cost = new_cost;
    return;
  }

  detach(old_parent, node);
  const double delta = new_cost - node->cost;
  attach(new_parent, node);
  node->cost = new_cost;

  std::vector<TreeNode *> pending(node->children.begin(), node->children.end());
  while (!pending.empty()) {
    TreeNode * child = pending.back();
    pending.pop_back();
    child->cost += delta;
    pending.insert(pending.end(), child->children.begin(), child->children.end());
  }
}

double PlannerCore::elapsed(double started) const
{
  return std::max(0.0, clock_() - started);
}

PlanningResult PlannerCore::makeResult(
  double started,
  bool success,
  std::vector<Point> path,
  double cost,
  FailureCode failure_code,
  std::size_t iterations,
  double first_feasible_time_sec,
  const std::string & message) const
{
  PlanningResult result;
  result.success = success;
  result.path = std::move(path);
  result.cost = success ? cost : std::numeric_limits<double>::infinity();
  result.failure_code = failure_code;
  result.planning_time_sec = elapsed(started);
  result.iterations = iterations;
  result.node_count = nodes_.size();
  result.uniform_sample_count = uniform_sample_count_;
  result.goal_sample_count = goal_sample_count_;
  result.risk_sample_count = risk_sample_count_;
  result.guided_extension_count = guided_extension_count_;
  result.adaptation_count = adaptation_count_;
  result.first_feasible_time_sec = first_feasible_time_sec;
  result.message = message;
  return result;
}

PlanningResult PlannerCore::plan(
  const Point & start,
  const Point & goal,
  CancelCheckFn cancel_check)
{
  const double started = clock_();
  nodes_.clear();
  random_generator_.seed(config_.random_seed);
  uniform_sample_count_ = 0;
  goal_sample_count_ = 0;
  risk_sample_count_ = 0;
  guided_extension_count_ = 0;
  adaptation_count_ = 0;
  sampling_probabilities_ = SamplingProbabilities{
    1.0 - config_.goal_sample_rate - config_.risk_sample_rate,
    config_.goal_sample_rate,
    config_.risk_sample_rate,
  };
  if (config_.enable_adaptive_sampling) {
    normalizeProbabilities();
    buildSamplingRegions(goal);
  } else {
    sampling_regions_.clear();
  }
  if (!cancel_check) {
    cancel_check = []() {return false;};
  }

  if (!isFinitePoint(start) || !validPoint(start)) {
    return makeResult(
      started, false, {}, std::numeric_limits<double>::infinity(),
      FailureCode::INVALID_START, 0, -1.0,
      "Start point is not finite or traversable.");
  }
  if (!isFinitePoint(goal) || !validPoint(goal)) {
    return makeResult(
      started, false, {}, std::numeric_limits<double>::infinity(),
      FailureCode::INVALID_GOAL, 0, -1.0,
      "Goal point is not finite or traversable.");
  }
  if (config_.planning_timeout_sec > 0.0 &&
    elapsed(started) >= config_.planning_timeout_sec)
  {
    return makeResult(
      started, false, {}, std::numeric_limits<double>::infinity(),
      FailureCode::TIMEOUT, 0, -1.0,
      "Planning time budget expired before search started.");
  }

  auto start_node = std::unique_ptr<TreeNode>(new TreeNode());
  start_node->point = start;
  nodes_.push_back(std::move(start_node));
  if (samePoint(start, goal)) {
    return makeResult(
      started, true, {start}, 0.0, FailureCode::SUCCESS, 0, 0.0,
      "Start and goal are identical.");
  }

  TreeNode * best_goal_parent = nullptr;
  double best_goal_cost = std::numeric_limits<double>::infinity();
  double first_feasible_time = -1.0;
  bool timed_out = false;
  std::size_t iterations = 0;
  std::size_t consecutive_failures = 0;
  std::size_t since_goal_improvement = 0;
  double closest_goal_distance = distance(start, goal);

  for (iterations = 1; iterations <= config_.max_iterations; ++iterations) {
    if (cancel_check()) {
      return makeResult(
        started, false, {}, std::numeric_limits<double>::infinity(),
        FailureCode::CANCELLED, iterations - 1, first_feasible_time,
        "Planning was cancelled before a validated result was returned.");
    }
    if (config_.planning_timeout_sec > 0.0 &&
      elapsed(started) >= config_.planning_timeout_sec)
    {
      timed_out = true;
      break;
    }

    ++since_goal_improvement;
    if (config_.enable_adaptive_sampling && iterations > 1 &&
      (iterations - 1) % config_.adaptation_interval == 0)
    {
      const bool stagnating =
        consecutive_failures >= config_.stagnation_iterations ||
        since_goal_improvement >= config_.stagnation_iterations;
      adaptProbabilities(
        stagnating,
        static_cast<double>(iterations - 1) /
        static_cast<double>(config_.max_iterations));
    }

    Point random_point;
    if (config_.enable_adaptive_sampling) {
      random_point = adaptiveSample(goal);
    } else {
      random_point = sample(goal);
      if (samePoint(random_point, goal)) {
        ++goal_sample_count_;
      } else {
        ++uniform_sample_count_;
      }
    }
    TreeNode * nearest_node = nearest(random_point);
    if (nearest_node == nullptr) {
      break;
    }

    Point steer_target = random_point;
    if (config_.enable_adaptive_sampling) {
      guidedTarget(*nearest_node, random_point, goal, steer_target);
    }
    Point new_point;
    if (!steer(*nearest_node, steer_target, new_point) ||
      !validPoint(new_point))
    {
      ++consecutive_failures;
      continue;
    }

    double edge_cost = 0.0;
    if (!safeEdgeCost(nearest_node->point, new_point, edge_cost)) {
      ++consecutive_failures;
      continue;
    }

    auto new_node = std::unique_ptr<TreeNode>(new TreeNode());
    new_node->point = new_point;
    new_node->cost = nearest_node->cost + edge_cost;
    std::vector<TreeNode *> near_nodes = nearby(*new_node);
    TreeNode * best_parent = nearest_node;
    double best_cost = new_node->cost;

    for (TreeNode * candidate : near_nodes) {
      double candidate_edge = 0.0;
      if (!safeEdgeCost(candidate->point, new_point, candidate_edge)) {
        continue;
      }
      const double candidate_cost = candidate->cost + candidate_edge;
      if (candidate_cost < best_cost) {
        best_parent = candidate;
        best_cost = candidate_cost;
      }
    }

    new_node->cost = best_cost;
    attach(best_parent, new_node.get());
    TreeNode * new_node_ptr = new_node.get();
    nodes_.push_back(std::move(new_node));
    consecutive_failures = 0;
    const double new_goal_distance = distance(new_point, goal);
    if (new_goal_distance + 1.0e-12 < closest_goal_distance) {
      closest_goal_distance = new_goal_distance;
      since_goal_improvement = 0;
    }

    for (TreeNode * candidate : near_nodes) {
      if (candidate == new_node_ptr || candidate == best_parent ||
        isAncestor(candidate, new_node_ptr))
      {
        continue;
      }
      double candidate_edge = 0.0;
      if (!safeEdgeCost(new_point, candidate->point, candidate_edge)) {
        continue;
      }
      const double candidate_cost = new_node_ptr->cost + candidate_edge;
      if (candidate_cost + 1.0e-12 < candidate->cost) {
        reparent(candidate, new_node_ptr, candidate_cost);
      }
    }

    if (distance(new_point, goal) > config_.goal_radius) {
      continue;
    }
    double goal_edge = 0.0;
    if (!safeEdgeCost(new_point, goal, goal_edge)) {
      continue;
    }
    const double candidate_goal_cost = new_node_ptr->cost + goal_edge;
    if (candidate_goal_cost + 1.0e-12 >= best_goal_cost) {
      continue;
    }

    best_goal_parent = new_node_ptr;
    best_goal_cost = candidate_goal_cost;
    if (first_feasible_time < 0.0) {
      first_feasible_time = elapsed(started);
    }
    if (config_.stop_on_first_feasible) {
      break;
    }
  }

  if (best_goal_parent == nullptr) {
    if (config_.planning_timeout_sec > 0.0 &&
      elapsed(started) >= config_.planning_timeout_sec)
    {
      timed_out = true;
    }
    const FailureCode failure = timed_out ? FailureCode::TIMEOUT :
      FailureCode::NO_FEASIBLE_PATH;
    const std::string message = timed_out ?
      "Planning time budget expired before a feasible path was found." :
      "Search finished without a feasible path to the goal.";
    return makeResult(
      started, false, {}, std::numeric_limits<double>::infinity(), failure,
      iterations, first_feasible_time, message);
  }

  std::vector<Point> path;
  path.push_back(goal);
  TreeNode * current = best_goal_parent;
  while (current != nullptr) {
    path.push_back(current->point);
    current = current->parent;
  }
  std::reverse(path.begin(), path.end());

  return makeResult(
    started, true, std::move(path), best_goal_cost, FailureCode::SUCCESS,
    iterations, first_feasible_time, "Validated feasible path found.");
}

}  // namespace risk_aware_planner_cpp
