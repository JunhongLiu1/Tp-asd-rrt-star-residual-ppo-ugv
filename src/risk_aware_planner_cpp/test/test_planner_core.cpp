#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "risk_aware_planner_cpp/planner_core.hpp"

namespace risk_aware_planner_cpp
{

class PlannerCoreTestAccess
{
public:
  static void reparent(TreeNode * node, TreeNode * parent, double cost)
  {
    PlannerCore::reparent(node, parent, cost);
  }

  static void attach(TreeNode * parent, TreeNode * child)
  {
    PlannerCore::attach(parent, child);
  }

  static void prepare(PlannerCore & planner, const Point & goal)
  {
    planner.random_generator_.seed(planner.config_.random_seed);
    planner.sampling_probabilities_ = SamplingProbabilities{
      1.0 - planner.config_.goal_sample_rate -
      planner.config_.risk_sample_rate,
      planner.config_.goal_sample_rate,
      planner.config_.risk_sample_rate,
    };
    planner.normalizeProbabilities();
    planner.buildSamplingRegions(goal);
  }

  static Point adaptiveSample(PlannerCore & planner, const Point & goal)
  {
    return planner.adaptiveSample(goal);
  }

  static Point legacySample(PlannerCore & planner, const Point & goal)
  {
    return planner.sample(goal);
  }

  static bool guide(
    PlannerCore & planner, const Point & source, const Point & sampled,
    const Point & goal, Point & target)
  {
    TreeNode node;
    node.point = source;
    return planner.guidedTarget(node, sampled, goal, target);
  }

  static SamplingProbabilities probabilities(const PlannerCore & planner)
  {
    return planner.sampling_probabilities_;
  }

  static void adapt(PlannerCore & planner, bool stagnating, double progress)
  {
    planner.adaptProbabilities(stagnating, progress);
  }
};

namespace
{

double euclideanCost(const Point & first, const Point & second)
{
  return std::hypot(second.x - first.x, second.y - first.y);
}

PlannerConfig testConfig()
{
  PlannerConfig config;
  config.step_size = 0.4;
  config.search_radius = 0.8;
  config.goal_radius = 0.45;
  config.max_iterations = 600;
  config.planning_timeout_sec = 0.0;
  config.goal_sample_rate = 0.2;
  config.random_seed = 31;
  config.stop_on_first_feasible = true;
  return config;
}

}  // namespace

TEST(PlannerCore, OpenSpaceReturnsValidatedPath)
{
  PlannerCore planner(
    euclideanCost,
    Bounds{-2.0, 2.0, -2.0, 2.0},
    testConfig());

  const PlanningResult result = planner.plan(Point{-1.5, -1.5}, Point{1.5, 1.5});

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.failure_code, FailureCode::SUCCESS);
  ASSERT_FALSE(result.path.empty());
  EXPECT_DOUBLE_EQ(result.path.front().x, -1.5);
  EXPECT_DOUBLE_EQ(result.path.front().y, -1.5);
  EXPECT_DOUBLE_EQ(result.path.back().x, 1.5);
  EXPECT_DOUBLE_EQ(result.path.back().y, 1.5);
  EXPECT_TRUE(std::isfinite(result.cost));
  EXPECT_GE(result.first_feasible_time_sec, 0.0);
}

TEST(PlannerCore, InvalidGoalDoesNotUseFallbackStart)
{
  PlannerCore planner(
    euclideanCost,
    Bounds{-2.0, 2.0, -2.0, 2.0},
    testConfig(),
    [](const Point & point) {
      return !(point.x == 1.5 && point.y == 1.5);
    });

  const PlanningResult result = planner.plan(Point{-1.5, -1.5}, Point{1.5, 1.5});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, FailureCode::INVALID_GOAL);
  EXPECT_TRUE(result.path.empty());
}

TEST(PlannerCore, InfeasibleEdgesReturnFailure)
{
  PlannerCore planner(
    [](const Point &, const Point &) {
      return std::numeric_limits<double>::infinity();
    },
    Bounds{-2.0, 2.0, -2.0, 2.0},
    testConfig());

  const PlanningResult result = planner.plan(Point{-1.5, -1.5}, Point{1.5, 1.5});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, FailureCode::NO_FEASIBLE_PATH);
  EXPECT_TRUE(result.path.empty());
}

TEST(PlannerCore, CancellationReturnsFailure)
{
  PlannerCore planner(
    euclideanCost,
    Bounds{-2.0, 2.0, -2.0, 2.0},
    testConfig());
  int checks = 0;

  const PlanningResult result = planner.plan(
    Point{-1.5, -1.5},
    Point{1.5, 1.5},
    [&checks]() {
      ++checks;
      return checks >= 4;
    });

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, FailureCode::CANCELLED);
}

TEST(PlannerCore, TimeoutReturnsFailure)
{
  double fake_time = 0.0;
  PlannerConfig config = testConfig();
  config.planning_timeout_sec = 0.0005;

  PlannerCore planner(
    euclideanCost,
    Bounds{-2.0, 2.0, -2.0, 2.0},
    config,
    PlannerCore::PointValidityFn(),
    [&fake_time]() {
      fake_time += 0.001;
      return fake_time;
    });

  const PlanningResult result = planner.plan(Point{-1.5, -1.5}, Point{1.5, 1.5});

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.failure_code, FailureCode::TIMEOUT);
}

TEST(PlannerCore, ReparentPropagatesCostToAllDescendants)
{
  TreeNode root;
  TreeNode old_parent;
  TreeNode new_parent;
  TreeNode child;
  TreeNode grandchild;
  root.cost = 0.0;
  old_parent.cost = 4.0;
  new_parent.cost = 1.0;
  child.cost = 6.0;
  grandchild.cost = 9.0;
  PlannerCoreTestAccess::attach(&root, &old_parent);
  PlannerCoreTestAccess::attach(&root, &new_parent);
  PlannerCoreTestAccess::attach(&old_parent, &child);
  PlannerCoreTestAccess::attach(&child, &grandchild);

  PlannerCoreTestAccess::reparent(&old_parent, &new_parent, 2.0);

  EXPECT_EQ(old_parent.parent, &new_parent);
  EXPECT_DOUBLE_EQ(old_parent.cost, 2.0);
  EXPECT_DOUBLE_EQ(child.cost, 4.0);
  EXPECT_DOUBLE_EQ(grandchild.cost, 7.0);
}

TEST(PlannerCore, SameSeedProducesIdenticalResult)
{
  PlannerCore first(euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, testConfig());
  PlannerCore second(euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, testConfig());
  const PlanningResult a = first.plan(Point{-1.5, -1.5}, Point{1.5, 1.5});
  const PlanningResult b = second.plan(Point{-1.5, -1.5}, Point{1.5, 1.5});
  ASSERT_EQ(a.success, b.success);
  ASSERT_EQ(a.path.size(), b.path.size());
  EXPECT_DOUBLE_EQ(a.cost, b.cost);
  EXPECT_EQ(a.iterations, b.iterations);
  for (std::size_t i = 0; i < a.path.size(); ++i) {
    EXPECT_DOUBLE_EQ(a.path[i].x, b.path[i].x);
    EXPECT_DOUBLE_EQ(a.path[i].y, b.path[i].y);
  }
}

TEST(PlannerCore, RiskRegionsBiasSamplesTowardLowRisk)
{
  PlannerConfig config = testConfig();
  config.goal_sample_rate = 0.0;
  config.risk_sample_rate = 1.0;
  config.min_uniform_sample_rate = 0.0;
  config.min_goal_sample_rate = 0.0;
  config.max_goal_sample_rate = 1.0;
  config.min_risk_sample_rate = 0.0;
  config.max_risk_sample_rate = 1.0;
  PlannerCore planner(
    euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, config,
    PlannerCore::PointValidityFn(), PlannerCore::ClockFn(),
    [](const Point & point) {return point.x < 0.0 ? 0.0 : 0.95;});
  PlannerCoreTestAccess::prepare(planner, Point{0.0, 0.0});
  std::size_t low_risk = 0;
  for (std::size_t index = 0; index < 1000; ++index) {
    if (PlannerCoreTestAccess::adaptiveSample(
        planner, Point{0.0, 0.0}).x < 0.0)
    {
      ++low_risk;
    }
  }
  EXPECT_GT(low_risk, 850U);
}

TEST(PlannerCore, RiskGradientRepelsInCorrectHorizontalDirection)
{
  PlannerConfig config = testConfig();
  config.random_direction_weight = 0.0;
  config.goal_attraction_weight = 0.0;
  config.risk_repulsion_weight = 1.0;
  PlannerCore increasing(
    euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, config,
    PlannerCore::PointValidityFn(), PlannerCore::ClockFn(),
    [](const Point & point) {return 0.5 + 0.1 * point.x;});
  Point left;
  ASSERT_TRUE(PlannerCoreTestAccess::guide(
      increasing, Point{0.0, 0.0}, Point{0.0, 1.0},
      Point{1.0, 0.0}, left));
  EXPECT_LT(left.x, 0.0);

  PlannerCore decreasing(
    euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, config,
    PlannerCore::PointValidityFn(), PlannerCore::ClockFn(),
    [](const Point & point) {return 0.5 - 0.1 * point.x;});
  Point right;
  ASSERT_TRUE(PlannerCoreTestAccess::guide(
      decreasing, Point{0.0, 0.0}, Point{0.0, 1.0},
      Point{1.0, 0.0}, right));
  EXPECT_GT(right.x, 0.0);
}

TEST(PlannerCore, GoalAttractionControlsGuidedDirection)
{
  PlannerConfig config = testConfig();
  config.random_direction_weight = 0.0;
  config.goal_attraction_weight = 1.0;
  config.risk_repulsion_weight = 0.0;
  PlannerCore planner(
    euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, config);
  Point target;
  ASSERT_TRUE(PlannerCoreTestAccess::guide(
      planner, Point{0.0, 0.0}, Point{-1.0, 0.0},
      Point{1.0, 0.0}, target));
  EXPECT_GT(target.x, 0.0);
  EXPECT_NEAR(target.y, 0.0, 1.0e-12);
}

TEST(PlannerCore, StagnationChangesOnlyLegalProbabilities)
{
  PlannerCore planner(
    euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, testConfig());
  PlannerCoreTestAccess::prepare(planner, Point{1.0, 1.0});
  const SamplingProbabilities before =
    PlannerCoreTestAccess::probabilities(planner);
  PlannerCoreTestAccess::adapt(planner, true, 0.5);
  const SamplingProbabilities after =
    PlannerCoreTestAccess::probabilities(planner);
  EXPECT_GT(after.uniform, before.uniform);
  EXPECT_NEAR(after.uniform + after.goal + after.risk, 1.0, 1.0e-12);
  EXPECT_GE(after.uniform, testConfig().min_uniform_sample_rate);
  EXPECT_GE(after.goal, testConfig().min_goal_sample_rate);
  EXPECT_GE(after.risk, testConfig().min_risk_sample_rate);
  EXPECT_LE(after.goal, testConfig().max_goal_sample_rate);
  EXPECT_LE(after.risk, testConfig().max_risk_sample_rate);
}

TEST(PlannerCore, AdaptiveSamplingIsDeterministicForSameSeed)
{
  const auto risk = [](const Point & point) {
      return std::min(1.0, std::abs(point.x) / 2.0);
    };
  PlannerCore first(
    euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, testConfig(),
    PlannerCore::PointValidityFn(), PlannerCore::ClockFn(), risk);
  PlannerCore second(
    euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, testConfig(),
    PlannerCore::PointValidityFn(), PlannerCore::ClockFn(), risk);
  PlannerCoreTestAccess::prepare(first, Point{1.0, 1.0});
  PlannerCoreTestAccess::prepare(second, Point{1.0, 1.0});
  for (std::size_t index = 0; index < 100; ++index) {
    const Point a = PlannerCoreTestAccess::adaptiveSample(
      first, Point{1.0, 1.0});
    const Point b = PlannerCoreTestAccess::adaptiveSample(
      second, Point{1.0, 1.0});
    EXPECT_DOUBLE_EQ(a.x, b.x);
    EXPECT_DOUBLE_EQ(a.y, b.y);
  }
}

TEST(PlannerCore, DisabledModePreservesLegacyDrawsAndNeverReadsRisk)
{
  PlannerConfig config = testConfig();
  config.enable_adaptive_sampling = false;
  std::size_t risk_calls = 0;
  PlannerCore planner(
    euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, config,
    PlannerCore::PointValidityFn(), PlannerCore::ClockFn(),
    [&risk_calls](const Point &) {
      ++risk_calls;
      return 0.0;
    });
  std::mt19937 reference(config.random_seed);
  std::uniform_real_distribution<double> probability(0.0, 1.0);
  std::uniform_real_distribution<double> coordinate(-2.0, 2.0);
  for (std::size_t index = 0; index < 100; ++index) {
    Point expected{1.0, 1.0};
    if (probability(reference) >= config.goal_sample_rate) {
      expected = Point{coordinate(reference), coordinate(reference)};
    }
    const Point actual = PlannerCoreTestAccess::legacySample(
      planner, Point{1.0, 1.0});
    EXPECT_DOUBLE_EQ(actual.x, expected.x);
    EXPECT_DOUBLE_EQ(actual.y, expected.y);
  }
  const PlanningResult result = planner.plan(
    Point{-1.5, -1.5}, Point{1.5, 1.5});
  EXPECT_EQ(risk_calls, 0U);
  EXPECT_EQ(result.risk_sample_count, 0U);
  EXPECT_EQ(result.guided_extension_count, 0U);
  EXPECT_EQ(result.adaptation_count, 0U);
}

TEST(PlannerCore, InvalidRiskCallbackFallsBackToUniformSampling)
{
  PlannerConfig config = testConfig();
  config.goal_sample_rate = 0.0;
  config.risk_sample_rate = 1.0;
  config.min_uniform_sample_rate = 0.0;
  config.min_goal_sample_rate = 0.0;
  config.max_goal_sample_rate = 1.0;
  config.min_risk_sample_rate = 0.0;
  config.max_risk_sample_rate = 1.0;
  PlannerCore planner(
    euclideanCost, Bounds{-2.0, 2.0, -2.0, 2.0}, config,
    PlannerCore::PointValidityFn(), PlannerCore::ClockFn(),
    [](const Point &) {return std::numeric_limits<double>::quiet_NaN();});
  PlannerCoreTestAccess::prepare(planner, Point{1.0, 1.0});
  for (std::size_t index = 0; index < 100; ++index) {
    const Point point = PlannerCoreTestAccess::adaptiveSample(
      planner, Point{1.0, 1.0});
    EXPECT_TRUE(std::isfinite(point.x));
    EXPECT_TRUE(std::isfinite(point.y));
    EXPECT_GE(point.x, -2.0);
    EXPECT_LE(point.x, 2.0);
  }
}

}  // namespace risk_aware_planner_cpp
