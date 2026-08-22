#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "radiation_interfaces/action/plan_risk_aware_path.hpp"
#include "radiation_interfaces/msg/risk_map.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "risk_aware_planner_cpp/edge_cost_model.hpp"
#include "risk_aware_planner_cpp/planner_core.hpp"

namespace risk_aware_planner_cpp
{

namespace
{

struct GridValue
{
  bool valid{false};
  double value{0.0};
};

struct MapSnapshot
{
  nav_msgs::msg::OccupancyGrid terrain;
  nav_msgs::msg::OccupancyGrid radiation;
  nav_msgs::msg::OccupancyGrid traversability;
  bool has_traversability{false};
  std::vector<float> dose_rate_usv_h;
  std::vector<float> confidence;
  std::uint64_t risk_map_version{0U};
};

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool worldToIndex(
  const nav_msgs::msg::OccupancyGrid & grid,
  const Point & point,
  std::size_t & index)
{
  const double resolution = static_cast<double>(grid.info.resolution);
  const std::size_t width = static_cast<std::size_t>(grid.info.width);
  const std::size_t height = static_cast<std::size_t>(grid.info.height);
  if (resolution <= 0.0 || width == 0U || height == 0U ||
    grid.data.size() != width * height)
  {
    return false;
  }

  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const double origin_yaw = yawFromQuaternion(grid.info.origin.orientation);
  const double dx = point.x - origin_x;
  const double dy = point.y - origin_y;

  // Apply the inverse planar origin rotation before converting to a cell.
  const double local_x = std::cos(origin_yaw) * dx +
    std::sin(origin_yaw) * dy;
  const double local_y = -std::sin(origin_yaw) * dx +
    std::cos(origin_yaw) * dy;

  const long column = static_cast<long>(std::floor(local_x / resolution));
  const long row = static_cast<long>(std::floor(local_y / resolution));
  if (column < 0L || row < 0L ||
    column >= static_cast<long>(width) || row >= static_cast<long>(height))
  {
    return false;
  }

  index = static_cast<std::size_t>(row) * width +
    static_cast<std::size_t>(column);
  return true;
}

GridValue readGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  const Point & point)
{
  std::size_t index = 0U;
  if (!worldToIndex(grid, point, index)) {
    return {};
  }
  const int raw_value = static_cast<int>(grid.data[index]);
  if (raw_value < 0) {
    return {};
  }
  return GridValue{true, static_cast<double>(raw_value)};
}

bool sameGeometry(
  const nav_msgs::msg::OccupancyGrid & first,
  const nav_msgs::msg::OccupancyGrid & second)
{
  const double tolerance = 1.0e-9;
  return first.info.width == second.info.width &&
    first.info.height == second.info.height &&
    std::abs(first.info.resolution - second.info.resolution) <= tolerance &&
    std::abs(first.info.origin.position.x - second.info.origin.position.x) <=
    tolerance &&
    std::abs(first.info.origin.position.y - second.info.origin.position.y) <=
    tolerance;
}

bool validGridGeometry(const nav_msgs::msg::OccupancyGrid & grid)
{
  const std::size_t width = static_cast<std::size_t>(grid.info.width);
  const std::size_t height = static_cast<std::size_t>(grid.info.height);
  return grid.info.resolution > 0.0 && width > 0U && height > 0U &&
    grid.data.size() == width * height;
}

double pathLength(const std::vector<Point> & path)
{
  double length = 0.0;
  for (std::size_t index = 1U; index < path.size(); ++index) {
    length += std::hypot(
      path[index].x - path[index - 1U].x,
      path[index].y - path[index - 1U].y);
  }
  return length;
}

}  // namespace

class PlannerNode final : public rclcpp::Node
{
public:
  using PlanAction = radiation_interfaces::action::PlanRiskAwarePath;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PlanAction>;
  PlannerNode()
  : Node("tp_asd_rrt_star_planner_cpp")
  {
    declare_parameter("terrain_topic", "/terrain_impedance_map");
    declare_parameter("radiation_topic", "/radiation_map");
    declare_parameter("traversability_topic", "/terrain_traversability_mask");
    declare_parameter("odom_topic", "/odometry/filtered");
    declare_parameter("goal_topic", "/goal_pose");
    declare_parameter("path_topic", "/tp_asd_rrt_star_cpp_path");
    declare_parameter("status_topic", "/tp_asd_rrt_star_cpp_status");
    declare_parameter("map_frame", "map");
    declare_parameter("require_traversability_map", false);
    declare_parameter("map_timeout_sec", 5.0);
    declare_parameter("odom_timeout_sec", 1.0);
    declare_parameter("odom_tf_timeout_sec", 0.2);
    declare_parameter("replan_rate_hz", 1.0);
    declare_parameter("replan_min_distance_m", 0.25);
    declare_parameter("replan_min_interval_sec", 1.0);
    declare_parameter("replan_on_map_update", true);

    declare_parameter("step_size", 0.45);
    declare_parameter("search_radius", 1.0);
    declare_parameter("goal_radius", 0.5);
    declare_parameter("max_iterations", 1800);
    declare_parameter("planning_timeout_sec", 2.0);
    declare_parameter("goal_sample_rate", 0.12);
    declare_parameter("random_seed", 31);
    declare_parameter("stop_on_first_feasible", false);
    declare_parameter("enable_adaptive_sampling", true);
    declare_parameter("sampling_region_rows", 8);
    declare_parameter("sampling_region_cols", 8);
    declare_parameter("risk_sample_rate", 0.35);
    declare_parameter("min_uniform_sample_rate", 0.15);
    declare_parameter("min_goal_sample_rate", 0.05);
    declare_parameter("max_goal_sample_rate", 0.45);
    declare_parameter("min_risk_sample_rate", 0.10);
    declare_parameter("max_risk_sample_rate", 0.70);
    declare_parameter("low_risk_exponent", 3.0);
    declare_parameter("region_goal_distance_gain", 0.25);
    declare_parameter("gradient_epsilon", 0.25);
    declare_parameter("random_direction_weight", 0.55);
    declare_parameter("goal_attraction_weight", 0.25);
    declare_parameter("risk_repulsion_weight", 0.35);
    declare_parameter("adaptation_interval", 100);
    declare_parameter("stagnation_iterations", 100);
    declare_parameter("adaptation_step", 0.05);

    declare_parameter("terrain_input_max", 100.0);
    declare_parameter("radiation_input_max", 100.0);
    declare_parameter("unknown_radiation_value", 25.0);
    declare_parameter("radiation_hard_threshold", 100.0);
    declare_parameter("terrain_hard_threshold", 100.0);
    declare_parameter("traversability_threshold", 50.0);
    declare_parameter("edge_sample_resolution", 0.2);
    declare_parameter("reference_length_m", 35.53606325105835);
    declare_parameter("reference_time_s", 444.20079063822936);
    declare_parameter("radiation_reference_usv_h", 8.0);
    declare_parameter("radiation_dose_hard_threshold_usv_h", 8.0);
    declare_parameter("unknown_dose_rate_usv_h", 0.25);
    declare_parameter("nominal_speed_m_s", 0.08);
    declare_parameter("minimum_speed_factor", 0.3);
    declare_parameter("terrain_speed_penalty_gain", 0.7);
    declare_parameter("distance_weight", 0.2);
    declare_parameter("terrain_weight", 0.4);
    declare_parameter("radiation_weight", 0.4);
    declare_parameter("time_penalty_lambda", 0.25);
    declare_parameter("include_time_penalty", true);

    terrain_topic_ = get_parameter("terrain_topic").as_string();
    radiation_topic_ = get_parameter("radiation_topic").as_string();
    traversability_topic_ = get_parameter("traversability_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    goal_topic_ = get_parameter("goal_topic").as_string();
    path_topic_ = get_parameter("path_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    require_traversability_map_ =
      get_parameter("require_traversability_map").as_bool();
    map_timeout_sec_ = get_parameter("map_timeout_sec").as_double();
    odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();
    odom_tf_timeout_sec_ =
      get_parameter("odom_tf_timeout_sec").as_double();
    replan_rate_hz_ = get_parameter("replan_rate_hz").as_double();
    replan_min_distance_m_ =
      get_parameter("replan_min_distance_m").as_double();
    replan_min_interval_sec_ =
      get_parameter("replan_min_interval_sec").as_double();
    replan_on_map_update_ =
      get_parameter("replan_on_map_update").as_bool();

    config_.step_size = get_parameter("step_size").as_double();
    config_.search_radius = get_parameter("search_radius").as_double();
    config_.goal_radius = get_parameter("goal_radius").as_double();
    config_.max_iterations = static_cast<std::size_t>(
      get_parameter("max_iterations").as_int());
    config_.planning_timeout_sec =
      get_parameter("planning_timeout_sec").as_double();
    config_.goal_sample_rate =
      get_parameter("goal_sample_rate").as_double();
    config_.random_seed = static_cast<unsigned int>(
      get_parameter("random_seed").as_int());
    config_.stop_on_first_feasible =
      get_parameter("stop_on_first_feasible").as_bool();
    config_.enable_adaptive_sampling =
      get_parameter("enable_adaptive_sampling").as_bool();
    const auto sampling_region_rows =
      get_parameter("sampling_region_rows").as_int();
    const auto sampling_region_cols =
      get_parameter("sampling_region_cols").as_int();
    const auto adaptation_interval =
      get_parameter("adaptation_interval").as_int();
    const auto stagnation_iterations =
      get_parameter("stagnation_iterations").as_int();
    if (config_.enable_adaptive_sampling &&
      (sampling_region_rows <= 0 || sampling_region_cols <= 0 ||
      adaptation_interval <= 0 || stagnation_iterations <= 0))
    {
      throw std::invalid_argument(
              "adaptive sampling integer parameters must be positive");
    }
    config_.sampling_region_rows =
      static_cast<std::size_t>(sampling_region_rows);
    config_.sampling_region_cols =
      static_cast<std::size_t>(sampling_region_cols);
    config_.risk_sample_rate = get_parameter("risk_sample_rate").as_double();
    config_.min_uniform_sample_rate =
      get_parameter("min_uniform_sample_rate").as_double();
    config_.min_goal_sample_rate =
      get_parameter("min_goal_sample_rate").as_double();
    config_.max_goal_sample_rate =
      get_parameter("max_goal_sample_rate").as_double();
    config_.min_risk_sample_rate =
      get_parameter("min_risk_sample_rate").as_double();
    config_.max_risk_sample_rate =
      get_parameter("max_risk_sample_rate").as_double();
    config_.low_risk_exponent =
      get_parameter("low_risk_exponent").as_double();
    config_.region_goal_distance_gain =
      get_parameter("region_goal_distance_gain").as_double();
    config_.gradient_epsilon =
      get_parameter("gradient_epsilon").as_double();
    config_.random_direction_weight =
      get_parameter("random_direction_weight").as_double();
    config_.goal_attraction_weight =
      get_parameter("goal_attraction_weight").as_double();
    config_.risk_repulsion_weight =
      get_parameter("risk_repulsion_weight").as_double();
    config_.adaptation_interval =
      static_cast<std::size_t>(adaptation_interval);
    config_.stagnation_iterations =
      static_cast<std::size_t>(stagnation_iterations);
    config_.adaptation_step =
      get_parameter("adaptation_step").as_double();

    terrain_input_max_ = get_parameter("terrain_input_max").as_double();
    radiation_input_max_ = get_parameter("radiation_input_max").as_double();
    unknown_radiation_value_ =
      get_parameter("unknown_radiation_value").as_double();
    radiation_hard_threshold_ =
      get_parameter("radiation_hard_threshold").as_double();
    terrain_hard_threshold_ =
      get_parameter("terrain_hard_threshold").as_double();
    traversability_threshold_ =
      get_parameter("traversability_threshold").as_double();
    edge_sample_resolution_ =
      get_parameter("edge_sample_resolution").as_double();
    reference_length_m_ = get_parameter("reference_length_m").as_double();
    reference_time_s_ = get_parameter("reference_time_s").as_double();
    radiation_reference_usv_h_ =
      get_parameter("radiation_reference_usv_h").as_double();
    radiation_dose_hard_threshold_usv_h_ =
      get_parameter("radiation_dose_hard_threshold_usv_h").as_double();
    unknown_dose_rate_usv_h_ = get_parameter("unknown_dose_rate_usv_h").as_double();
    nominal_speed_m_s_ = get_parameter("nominal_speed_m_s").as_double();
    minimum_speed_factor_ =
      get_parameter("minimum_speed_factor").as_double();
    terrain_speed_penalty_gain_ =
      get_parameter("terrain_speed_penalty_gain").as_double();
    distance_weight_ = get_parameter("distance_weight").as_double();
    terrain_weight_ = get_parameter("terrain_weight").as_double();
    radiation_weight_ = get_parameter("radiation_weight").as_double();
    time_penalty_lambda_ =
      get_parameter("time_penalty_lambda").as_double();
    include_time_penalty_ =
      get_parameter("include_time_penalty").as_bool();

    validateParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    terrain_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      terrain_topic_, map_qos,
      std::bind(&PlannerNode::terrainCallback, this, std::placeholders::_1));
    radiation_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      radiation_topic_, map_qos,
      std::bind(&PlannerNode::radiationCallback, this, std::placeholders::_1));
    continuous_risk_sub_ = create_subscription<radiation_interfaces::msg::RiskMap>(
      "/risk_map/continuous", map_qos,
      std::bind(&PlannerNode::continuousRiskCallback, this, std::placeholders::_1));
    traversability_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      traversability_topic_, map_qos,
      std::bind(
        &PlannerNode::traversabilityCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS(10),
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    action_server_ = rclcpp_action::create_server<PlanAction>(
      this, "plan_risk_aware_path",
      std::bind(&PlannerNode::handleActionGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PlannerNode::handleActionCancel, this, std::placeholders::_1),
      std::bind(&PlannerNode::handleActionAccepted, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    const auto replan_period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / replan_rate_hz_));
    replan_timer_ = create_wall_timer(
      replan_period, std::bind(&PlannerNode::replanTimer, this));

    RCLCPP_INFO(get_logger(), "C++ TP-ASD planner node started.");
    RCLCPP_INFO(get_logger(), "Goal topic: %s", goal_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Path topic: %s", path_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Odom topic: %s; transforming odometry into frame: %s",
      odom_topic_.c_str(), map_frame_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Online replanning: %.2f Hz, map_update=%s, min_distance=%.2f m",
      replan_rate_hz_, replan_on_map_update_ ? "enabled" : "disabled",
      replan_min_distance_m_);
  }

private:
  rclcpp_action::GoalResponse handleActionGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PlanAction::Goal> goal)
  {
    if (!goal || !std::isfinite(goal->goal.pose.position.x) ||
      !std::isfinite(goal->goal.pose.position.y) ||
      (!goal->goal.header.frame_id.empty() && goal->goal.header.frame_id != map_frame_)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleActionCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleActionAccepted(const std::shared_ptr<GoalHandle> handle)
  {
    if (active_action_goal_ && active_action_goal_->is_active()) {
      auto superseded = std::make_shared<PlanAction::Result>();
      superseded->success = false;
      superseded->failure_code = "CANCELLED";
      superseded->message = "superseded by a newer planning request";
      active_action_goal_->abort(superseded);
    }
    active_action_goal_ = handle;
    const auto goal = handle->get_goal();
    goal_point_ = Point{goal->goal.pose.position.x, goal->goal.pose.position.y};
    goal_active_ = true;
    replan_requested_ = true;
    ++goal_generation_;
  }

  void finishAction(
    const PlanningResult & result, const nav_msgs::msg::Path & path,
    std::uint64_t risk_version, double risk_age)
  {
    if (!active_action_goal_) return;
    auto output = std::make_shared<PlanAction::Result>();
    output->success = result.success;
    output->failure_code = toString(result.failure_code);
    output->message = result.message;
    output->path = path;
    output->risk_map_version = risk_version;
    output->risk_map_age_sec = risk_age;
    output->planning_time_sec = result.planning_time_sec;
    output->iterations = result.iterations;
    if (active_action_goal_->is_canceling() || result.failure_code == FailureCode::CANCELLED) {
      active_action_goal_->canceled(output);
    } else if (result.success) active_action_goal_->succeed(output);
    else active_action_goal_->abort(output);
    active_action_goal_.reset();
  }
  void validateParameters() const
  {
    const double values[] = {
      map_timeout_sec_, odom_timeout_sec_, terrain_input_max_,
      radiation_input_max_, unknown_radiation_value_, radiation_hard_threshold_,
      terrain_hard_threshold_, traversability_threshold_,
      odom_tf_timeout_sec_, replan_rate_hz_, replan_min_distance_m_,
      replan_min_interval_sec_, edge_sample_resolution_, reference_length_m_,
      reference_time_s_,
      radiation_reference_usv_h_, nominal_speed_m_s_, minimum_speed_factor_,
      terrain_speed_penalty_gain_, distance_weight_, terrain_weight_,
      radiation_weight_, time_penalty_lambda_,
    };
    for (double value : values) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("planner parameter must be finite");
      }
    }
    if (map_timeout_sec_ < 0.0 || odom_timeout_sec_ < 0.0 ||
      odom_tf_timeout_sec_ <= 0.0 || replan_rate_hz_ <= 0.0 ||
      replan_min_distance_m_ < 0.0 ||
      replan_min_interval_sec_ < 0.0 ||
      terrain_input_max_ <= 0.0 || radiation_input_max_ <= 0.0 ||
      unknown_radiation_value_ < 0.0 ||
      unknown_radiation_value_ > radiation_input_max_ ||
      radiation_hard_threshold_ <= 0.0 || terrain_hard_threshold_ <= 0.0 ||
      edge_sample_resolution_ <= 0.0 || reference_length_m_ <= 0.0 ||
      reference_time_s_ <= 0.0 || radiation_reference_usv_h_ <= 0.0 ||
      nominal_speed_m_s_ <= 0.0 || minimum_speed_factor_ <= 0.0 ||
      minimum_speed_factor_ > 1.0 || terrain_speed_penalty_gain_ < 0.0 ||
      distance_weight_ < 0.0 || terrain_weight_ < 0.0 ||
      radiation_weight_ < 0.0 || time_penalty_lambda_ < 0.0)
    {
      throw std::invalid_argument("planner parameter is outside its valid range");
    }
  }

  void terrainCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (!message) {
      return;
    }
    terrain_map_ = *message;
    terrain_received_sec_ = get_clock()->now().seconds();
    has_terrain_map_ = true;
    ++map_generation_;
    if (goal_active_ && replan_on_map_update_) {
      replan_requested_ = true;
    }
  }

  void radiationCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (!message) {
      return;
    }
    radiation_map_ = *message;
    radiation_received_sec_ = get_clock()->now().seconds();
    has_radiation_map_ = true;
    ++map_generation_;
    if (goal_active_ && replan_on_map_update_) {
      replan_requested_ = true;
    }
  }

  void continuousRiskCallback(
    const radiation_interfaces::msg::RiskMap::SharedPtr message)
  {
    if (!message || message->dose_rate_usv_h.size() != message->confidence.size() ||
      message->dose_rate_usv_h.size() !=
      static_cast<std::size_t>(message->info.width) * message->info.height) return;
    continuous_risk_map_ = *message;
    risk_map_version_ = message->version;
    continuous_risk_received_sec_ = get_clock()->now().seconds();
    has_continuous_risk_map_ = true;
    if (goal_active_ && risk_map_version_ != planned_risk_map_version_) replan_requested_ = true;
  }

  void traversabilityCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (!message) {
      return;
    }
    traversability_map_ = *message;
    traversability_received_sec_ = get_clock()->now().seconds();
    has_traversability_map_ = true;
    ++map_generation_;
    if (goal_active_ && replan_on_map_update_) {
      replan_requested_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!message) {
      return;
    }

    if (message->header.frame_id.empty()) {
      if (!odom_tf_warning_logged_) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring odometry with an empty header.frame_id.");
        odom_tf_warning_logged_ = true;
      }
      return;
    }

    geometry_msgs::msg::PoseStamped source_pose;
    source_pose.header = message->header;
    source_pose.pose = message->pose.pose;

    geometry_msgs::msg::PoseStamped map_pose;
    try {
      if (source_pose.header.frame_id == map_frame_) {
        map_pose = source_pose;
      } else {
        map_pose = tf_buffer_->transform(
          source_pose,
          map_frame_,
          tf2::durationFromSec(odom_tf_timeout_sec_));
      }
    } catch (const tf2::TransformException & exception) {
      if (!odom_tf_warning_logged_) {
        RCLCPP_WARN(
          get_logger(),
          "Could not transform odometry from '%s' to '%s': %s",
          source_pose.header.frame_id.c_str(), map_frame_.c_str(),
          exception.what());
        odom_tf_warning_logged_ = true;
      }
      return;
    }

    odometry_ = *message;
    odometry_.header = map_pose.header;
    odometry_.header.frame_id = map_frame_;
    odometry_.pose.pose = map_pose.pose;
    odom_received_sec_ = get_clock()->now().seconds();
    has_odom_ = true;

    if (odom_tf_warning_logged_) {
      RCLCPP_INFO(
        get_logger(), "Odometry transform to '%s' is available again.",
        map_frame_.c_str());
      odom_tf_warning_logged_ = false;
    }

    if (goal_active_ && has_last_plan_start_ &&
      std::hypot(
      map_pose.pose.position.x - last_plan_start_.x,
      map_pose.pose.position.y - last_plan_start_.y) >=
      replan_min_distance_m_)
    {
      replan_requested_ = true;
    }
  }

  bool isFresh(double received_sec, double timeout_sec)
  {
    if (received_sec < 0.0) {
      return false;
    }
    const double age = std::max(
      0.0, get_clock()->now().seconds() - received_sec);
    return timeout_sec <= 0.0 || age <= timeout_sec;
  }

  bool inputsReady()
  {
    if (!has_terrain_map_ || !has_radiation_map_ || !has_continuous_risk_map_ || !has_odom_) {
      return false;
    }
    if (!isFresh(terrain_received_sec_, map_timeout_sec_) ||
      !isFresh(radiation_received_sec_, map_timeout_sec_) ||
      !isFresh(continuous_risk_received_sec_, map_timeout_sec_) ||
      !isFresh(odom_received_sec_, odom_timeout_sec_))
    {
      return false;
    }
    if (require_traversability_map_ &&
      (!has_traversability_map_ ||
      !isFresh(traversability_received_sec_, map_timeout_sec_)))
    {
      return false;
    }
    if (!validGridGeometry(terrain_map_) ||
      !validGridGeometry(radiation_map_))
    {
      return false;
    }
    if (require_traversability_map_ &&
      !validGridGeometry(traversability_map_))
    {
      return false;
    }
    if (!sameGeometry(terrain_map_, radiation_map_)) {
      return false;
    }
    if (require_traversability_map_ &&
      !sameGeometry(terrain_map_, traversability_map_))
    {
      return false;
    }
    return true;
  }

  Point currentMapPoint() const
  {
    const auto & pose = odometry_.pose.pose;
    return Point{pose.position.x, pose.position.y};
  }

  static Bounds mapBounds(const nav_msgs::msg::OccupancyGrid & map)
  {
    const double resolution = static_cast<double>(map.info.resolution);
    return Bounds{
      map.info.origin.position.x,
      map.info.origin.position.x +
      static_cast<double>(map.info.width) * resolution,
      map.info.origin.position.y,
      map.info.origin.position.y +
      static_cast<double>(map.info.height) * resolution,
    };
  }

  bool validPoint(const MapSnapshot & maps, const Point & point) const
  {
    const GridValue terrain = readGrid(maps.terrain, point);
    if (!terrain.valid || terrain.value >= terrain_hard_threshold_) {
      return false;
    }
    if (maps.has_traversability) {
      const GridValue traversability =
        readGrid(maps.traversability, point);
      if (!traversability.valid ||
        traversability.value < traversability_threshold_)
      {
        return false;
      }
    }
    std::size_t index = 0U;
    if (!worldToIndex(maps.radiation, point, index)) return false;
    const double dose = index < maps.confidence.size() && maps.confidence[index] > 0.0F ?
      maps.dose_rate_usv_h[index] : unknown_dose_rate_usv_h_;
    return std::isfinite(dose) && dose < radiation_dose_hard_threshold_usv_h_;
  }

  double samplingRisk(const MapSnapshot & maps, const Point & point) const
  {
    const GridValue terrain = readGrid(maps.terrain, point);
    const GridValue radiation = readGrid(maps.radiation, point);
    if (!terrain.valid || !radiation.valid) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double terrain_risk = clamp01(terrain.value / terrain_input_max_);
    const double radiation_risk = clamp01(
      radiation.value / radiation_input_max_);
    return 0.5 * terrain_risk + 0.5 * radiation_risk;
  }

  double edgeCost(
    const MapSnapshot & maps,
    const Point & first,
    const Point & second) const
  {
    const double length = std::hypot(second.x - first.x, second.y - first.y);
    if (length <= 1.0e-12) {
      return validPoint(maps, first) ? 0.0 :
        std::numeric_limits<double>::infinity();
    }

    const int steps = std::max(
      1,
      static_cast<int>(std::ceil(length / edge_sample_resolution_)));
    const double sub_distance = length / static_cast<double>(steps);
    double total_cost = 0.0;

    for (int index = 0; index < steps; ++index) {
      const double ratio = (static_cast<double>(index) + 0.5) /
        static_cast<double>(steps);
      const Point sample{
        first.x + ratio * (second.x - first.x),
        first.y + ratio * (second.y - first.y),
      };

      if (!validPoint(maps, sample)) {
        return std::numeric_limits<double>::infinity();
      }

      const double terrain = clamp01(
        readGrid(maps.terrain, sample).value / terrain_input_max_);
      std::size_t risk_index = 0U;
      worldToIndex(maps.radiation, sample, risk_index);
      const double dose_rate = risk_index < maps.confidence.size() &&
        maps.confidence[risk_index] > 0.0F ? maps.dose_rate_usv_h[risk_index] :
        unknown_dose_rate_usv_h_;
      const double radiation = dose_rate / radiation_reference_usv_h_;
      const double speed_factor = std::max(
        minimum_speed_factor_,
        1.0 - terrain_speed_penalty_gain_ * terrain);
      const double speed = nominal_speed_m_s_ * speed_factor;
      const double traversal_time = sub_distance / speed;
      const double distance_term = sub_distance / reference_length_m_;
      const double terrain_term = terrain * distance_term;
      const double predicted_dose_usv = dose_rate * traversal_time / 3600.0;
      const double radiation_term = predicted_dose_usv /
        (radiation_reference_usv_h_ * reference_time_s_ / 3600.0);
      const double time_term = traversal_time / reference_time_s_;

      total_cost += combineEdgeCost(
        EdgeCostComponents{
          distance_term, terrain_term, radiation_term, time_term},
        EdgeCostWeights{
          distance_weight_, terrain_weight_, radiation_weight_,
          time_penalty_lambda_, include_time_penalty_});
    }
    return total_cost;
  }

  void publishStatus(
    const std::string & code,
    const std::string & message)
  {
    std_msgs::msg::String status;
    std::ostringstream text;
    text << code << ": " << message;
    status.data = text.str();
    if (status.data == last_status_) {
      return;
    }
    status_pub_->publish(status);
    last_status_ = status.data;
  }

  void goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!message) {
      return;
    }

    if (!message->header.frame_id.empty() &&
      message->header.frame_id != map_frame_)
    {
      publishStatus(
        "INVALID_GOAL",
        "goal frame is not the configured map frame");
      RCLCPP_WARN(
        get_logger(),
        "Ignoring goal in frame '%s'; expected '%s'.",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    if (!std::isfinite(message->pose.position.x) ||
      !std::isfinite(message->pose.position.y))
    {
      publishStatus("INVALID_GOAL", "goal position is not finite");
      return;
    }

    goal_point_ = Point{
      message->pose.position.x,
      message->pose.position.y,
    };
    goal_active_ = true;
    replan_requested_ = true;
    ++goal_generation_;

    std::ostringstream goal_text;
    goal_text << "x=" << goal_point_.x << ", y=" << goal_point_.y;
    publishStatus("GOAL_ACCEPTED", goal_text.str());

    RCLCPP_INFO(
      get_logger(),
      "Accepted online goal (%.3f, %.3f).",
      goal_point_.x, goal_point_.y);
  }

  void replanTimer()
  {
    if (!goal_active_) {
      return;
    }

    if (!inputsReady()) {
      publishStatus(
        "WAITING_INPUTS",
        "terrain/radiation/odom input is missing, stale, or inconsistent");
      return;
    }

    const Point start = currentMapPoint();
    const double now = get_clock()->now().seconds();
    const bool map_changed =
      map_generation_ != planned_map_generation_;
    const bool interval_elapsed =
      last_plan_sec_ < 0.0 ||
      now - last_plan_sec_ >= replan_min_interval_sec_;
    const bool moved_enough =
      !has_last_plan_start_ ||
      std::hypot(
      start.x - last_plan_start_.x,
      start.y - last_plan_start_.y) >= replan_min_distance_m_;

    const bool should_replan =
      replan_requested_ ||
      (replan_on_map_update_ && map_changed) ||
      (interval_elapsed && moved_enough);

    if (!should_replan) {
      return;
    }

    replan_requested_ = false;
    publishStatus("PLANNING", "online replan in progress");

    const auto maps = std::make_shared<MapSnapshot>();
    maps->terrain = terrain_map_;
    maps->radiation = radiation_map_;
    maps->dose_rate_usv_h = continuous_risk_map_.dose_rate_usv_h;
    maps->confidence = continuous_risk_map_.confidence;
    maps->risk_map_version = continuous_risk_map_.version;
    maps->has_traversability =
      require_traversability_map_ && has_traversability_map_;

    if (maps->has_traversability) {
      maps->traversability = traversability_map_;
    }

    const Bounds bounds = mapBounds(maps->terrain);

    PlannerCore planner(
      [this, maps](const Point & first, const Point & second) {
        return edgeCost(*maps, first, second);
      },
      bounds,
      config_,
      [this, maps](const Point & point) {
        return validPoint(*maps, point);
      },
      PlannerCore::ClockFn(),
      [this, maps](const Point & point) {
        return samplingRisk(*maps, point);
      });

    RCLCPP_INFO(
      get_logger(),
      "Online replan #%llu from (%.3f, %.3f) to (%.3f, %.3f).",
      static_cast<unsigned long long>(goal_generation_),
      start.x, start.y, goal_point_.x, goal_point_.y);

    const PlanningResult result = planner.plan(
      start, goal_point_, [this]() {
        return active_action_goal_ && active_action_goal_->is_canceling();
      });

    last_plan_sec_ = get_clock()->now().seconds();
    planned_map_generation_ = map_generation_;
    planned_risk_map_version_ = maps->risk_map_version;

    if (result.success && risk_map_version_ != maps->risk_map_version) {
      replan_requested_ = true;
      publishStatus("STALE_MAP", "risk map changed while planning; result rejected");
      return;
    }

    if (!result.success) {
      replan_requested_ = true;

      std::ostringstream failure_text;
      failure_text << result.message
                   << ", planning_time_sec=" << result.planning_time_sec
                   << ", iterations=" << result.iterations
                   << ", samples_uniform=" << result.uniform_sample_count
                   << ", samples_goal=" << result.goal_sample_count
                   << ", samples_risk=" << result.risk_sample_count
                   << ", guided=" << result.guided_extension_count
                   << ", adaptations=" << result.adaptation_count;

      publishStatus(
        toString(result.failure_code),
        failure_text.str());

      RCLCPP_WARN(
        get_logger(),
        "Online replanning failed: %s (%s).",
        toString(result.failure_code).c_str(),
        result.message.c_str());
      finishAction(result, nav_msgs::msg::Path(), maps->risk_map_version,
        std::max(0.0, get_clock()->now().seconds() - continuous_risk_received_sec_));
      return;
    }

    nav_msgs::msg::Path path;
    path.header.stamp = get_clock()->now();
    path.header.frame_id = map_frame_;

    for (const Point & point : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }

    path_pub_->publish(path);
    finishAction(result, path, maps->risk_map_version,
      std::max(0.0, get_clock()->now().seconds() - continuous_risk_received_sec_));

    last_plan_start_ = start;
    has_last_plan_start_ = true;

    std::ostringstream success_text;
    success_text << "path_points=" << result.path.size()
                 << ", path_length_m=" << pathLength(result.path)
                 << ", cost=" << result.cost
                 << ", planning_time_sec=" << result.planning_time_sec
                 << ", iterations=" << result.iterations
                 << ", samples_uniform=" << result.uniform_sample_count
                 << ", samples_goal=" << result.goal_sample_count
                 << ", samples_risk=" << result.risk_sample_count
                 << ", guided=" << result.guided_extension_count
                 << ", adaptations=" << result.adaptation_count
                 << ", map_generation=" << map_generation_;
    success_text << ", risk_map_version=" << maps->risk_map_version
                 << ", risk_map_age_sec="
                 << std::max(0.0, get_clock()->now().seconds() - continuous_risk_received_sec_);

    publishStatus("SUCCESS", success_text.str());

    RCLCPP_INFO(
      get_logger(),
      "Published online path with %zu points.",
      result.path.size());
  }

  std::string terrain_topic_;
  std::string radiation_topic_;
  std::string traversability_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string status_topic_;
  std::string map_frame_;

  bool require_traversability_map_{false};
  double map_timeout_sec_{5.0};
  double odom_timeout_sec_{1.0};
  double odom_tf_timeout_sec_{0.2};
  double replan_rate_hz_{1.0};
  double replan_min_distance_m_{0.25};
  double replan_min_interval_sec_{1.0};
  double terrain_input_max_{100.0};
  double radiation_input_max_{100.0};
  double unknown_radiation_value_{25.0};
  double radiation_hard_threshold_{100.0};
  double terrain_hard_threshold_{100.0};
  double traversability_threshold_{50.0};
  double edge_sample_resolution_{0.2};
  double reference_length_m_{35.53606325105835};
  double reference_time_s_{444.20079063822936};
  double radiation_reference_usv_h_{8.0};
  double radiation_dose_hard_threshold_usv_h_{8.0};
  double unknown_dose_rate_usv_h_{0.25};
  double nominal_speed_m_s_{0.08};
  double minimum_speed_factor_{0.3};
  double terrain_speed_penalty_gain_{0.7};
  double distance_weight_{0.2};
  double terrain_weight_{0.4};
  double radiation_weight_{0.4};
  double time_penalty_lambda_{0.25};
  bool include_time_penalty_{true};
  bool replan_on_map_update_{true};

  PlannerConfig config_;
  nav_msgs::msg::OccupancyGrid terrain_map_;
  nav_msgs::msg::OccupancyGrid radiation_map_;
  nav_msgs::msg::OccupancyGrid traversability_map_;
  nav_msgs::msg::Odometry odometry_;
  radiation_interfaces::msg::RiskMap continuous_risk_map_;
  bool has_terrain_map_{false};
  bool has_radiation_map_{false};
  bool has_traversability_map_{false};
  bool has_odom_{false};
  bool has_continuous_risk_map_{false};
  bool goal_active_{false};
  bool replan_requested_{false};
  bool has_last_plan_start_{false};
  Point goal_point_;
  Point last_plan_start_;
  std::uint64_t goal_generation_{0U};
  std::uint64_t map_generation_{0U};
  std::uint64_t planned_map_generation_{0U};
  std::uint64_t risk_map_version_{0U};
  std::uint64_t planned_risk_map_version_{0U};
  double terrain_received_sec_{-1.0};
  double radiation_received_sec_{-1.0};
  double continuous_risk_received_sec_{-1.0};
  double traversability_received_sec_{-1.0};
  double odom_received_sec_{-1.0};
  double last_plan_sec_{-1.0};
  bool odom_tf_warning_logged_{false};
  std::string last_status_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr radiation_sub_;
  rclcpp::Subscription<radiation_interfaces::msg::RiskMap>::SharedPtr continuous_risk_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
  rclcpp_action::Server<PlanAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandle> active_action_goal_;
};

}  // namespace risk_aware_planner_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<risk_aware_planner_cpp::PlannerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
