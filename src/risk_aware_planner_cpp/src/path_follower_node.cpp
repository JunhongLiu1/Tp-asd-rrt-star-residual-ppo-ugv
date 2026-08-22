#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "radiation_interfaces/msg/control_metrics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "risk_aware_planner_cpp/pid_core.hpp"
#include "risk_aware_planner_cpp/pure_pursuit_core.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace risk_aware_planner_cpp
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double siny_cosp = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cosy_cosp = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double distance2d(
  const geometry_msgs::msg::Point & first,
  const geometry_msgs::msg::Point & second)
{
  return std::hypot(first.x - second.x, first.y - second.y);
}

struct VelocityCommandResult
{
  geometry_msgs::msg::Twist twist;
  bool valid{true};
  bool saturated{false};
};

}  // namespace

class PathFollowerNode final : public rclcpp::Node
{
public:
  PathFollowerNode()
  : Node("tp_asd_rrt_star_path_follower_cpp")
  {
    declare_parameter("path_topic", "/tp_asd_rrt_star_cpp_path");
    declare_parameter("odom_topic", "/odometry/filtered");
    declare_parameter("cmd_vel_topic", "/control/base_cmd");
    declare_parameter("e_stop_topic", "/e_stop");
    declare_parameter(
      "status_topic", "/tp_asd_rrt_star_cpp_follower_status");
    declare_parameter("map_frame", "map");

    // Safety gate: motion is disabled unless explicitly enabled at launch.
    declare_parameter("enable_motion", false);
    declare_parameter("control_rate_hz", 10.0);
    declare_parameter("lookahead_distance", 0.70);
    declare_parameter("min_lookahead_distance", 0.45);
    declare_parameter("max_lookahead_distance", 1.20);
    declare_parameter("lookahead_time", 1.50);
    declare_parameter("goal_tolerance", 0.35);
    declare_parameter("goal_slowdown_distance", 1.50);
    declare_parameter("max_path_deviation", 1.50);
    declare_parameter("path_timeout_sec", 0.0);
    declare_parameter("odom_timeout_sec", 1.0);
    declare_parameter("tf_timeout_sec", 0.2);
    declare_parameter("max_linear_speed", 0.20);
    declare_parameter("max_angular_speed", 0.80);
    declare_parameter("minimum_tracking_speed", 0.04);
    declare_parameter("curvature_speed_gain", 1.50);
    declare_parameter("rotate_in_place_angular_gain", 1.50);
    declare_parameter("terrain_topic", "/terrain_impedance_map");
    declare_parameter("terrain_speed_gain", 0.70);
    declare_parameter("metrics_topic", "/control/pure_pursuit_metrics");
    declare_parameter("enable_velocity_pid", true);
    declare_parameter("pid_max_dt_sec", 0.50);
    declare_parameter("linear_pid.kp", 0.80);
    declare_parameter("linear_pid.ki", 0.10);
    declare_parameter("linear_pid.kd", 0.0);
    declare_parameter("linear_pid.integral_limit", 0.30);
    declare_parameter("linear_pid.correction_limit", 0.10);
    declare_parameter("angular_pid.kp", 0.50);
    declare_parameter("angular_pid.ki", 0.05);
    declare_parameter("angular_pid.kd", 0.0);
    declare_parameter("angular_pid.integral_limit", 0.50);
    declare_parameter("angular_pid.correction_limit", 0.30);

    path_topic_ = get_parameter("path_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    e_stop_topic_ = get_parameter("e_stop_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    enable_motion_ = get_parameter("enable_motion").as_bool();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    lookahead_distance_ =
      get_parameter("lookahead_distance").as_double();
    min_lookahead_distance_ =
      get_parameter("min_lookahead_distance").as_double();
    max_lookahead_distance_ =
      get_parameter("max_lookahead_distance").as_double();
    lookahead_time_ = get_parameter("lookahead_time").as_double();
    goal_tolerance_ = get_parameter("goal_tolerance").as_double();
    goal_slowdown_distance_ =
      get_parameter("goal_slowdown_distance").as_double();
    max_path_deviation_ =
      get_parameter("max_path_deviation").as_double();
    path_timeout_sec_ = get_parameter("path_timeout_sec").as_double();
    odom_timeout_sec_ = get_parameter("odom_timeout_sec").as_double();
    tf_timeout_sec_ = get_parameter("tf_timeout_sec").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    minimum_tracking_speed_ =
      get_parameter("minimum_tracking_speed").as_double();
    curvature_speed_gain_ =
      get_parameter("curvature_speed_gain").as_double();
    rotate_in_place_angular_gain_ =
      get_parameter("rotate_in_place_angular_gain").as_double();
    terrain_speed_gain_ = get_parameter("terrain_speed_gain").as_double();
    enable_velocity_pid_ = get_parameter("enable_velocity_pid").as_bool();
    pid_max_dt_sec_ = get_parameter("pid_max_dt_sec").as_double();

    PidConfig linear_pid_config;
    linear_pid_config.kp = get_parameter("linear_pid.kp").as_double();
    linear_pid_config.ki = get_parameter("linear_pid.ki").as_double();
    linear_pid_config.kd = get_parameter("linear_pid.kd").as_double();
    const double linear_integral_limit =
      get_parameter("linear_pid.integral_limit").as_double();
    const double linear_correction_limit =
      get_parameter("linear_pid.correction_limit").as_double();
    linear_pid_config.integral_min = -linear_integral_limit;
    linear_pid_config.integral_max = linear_integral_limit;
    linear_pid_config.output_min = -linear_correction_limit;
    linear_pid_config.output_max = linear_correction_limit;
    linear_pid_.setConfig(linear_pid_config);

    PidConfig angular_pid_config;
    angular_pid_config.kp = get_parameter("angular_pid.kp").as_double();
    angular_pid_config.ki = get_parameter("angular_pid.ki").as_double();
    angular_pid_config.kd = get_parameter("angular_pid.kd").as_double();
    const double angular_integral_limit =
      get_parameter("angular_pid.integral_limit").as_double();
    const double angular_correction_limit =
      get_parameter("angular_pid.correction_limit").as_double();
    angular_pid_config.integral_min = -angular_integral_limit;
    angular_pid_config.integral_max = angular_integral_limit;
    angular_pid_config.output_min = -angular_correction_limit;
    angular_pid_config.output_max = angular_correction_limit;
    angular_pid_.setConfig(angular_pid_config);

    validateParameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      std::bind(
        &PathFollowerNode::pathCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      std::bind(
        &PathFollowerNode::odomCallback, this, std::placeholders::_1));
    rclcpp::QoS e_stop_qos(10);
    e_stop_qos.best_effort();
    e_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      e_stop_topic_, e_stop_qos,
      std::bind(
        &PathFollowerNode::eStopCallback, this, std::placeholders::_1));
    terrain_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      get_parameter("terrain_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr message) {
        if (message) {terrain_map_ = *message; has_terrain_ = true;}
      });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      status_topic_, 10);
    metrics_pub_ = create_publisher<radiation_interfaces::msg::ControlMetrics>(
      get_parameter("metrics_topic").as_string(), 10);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
    control_timer_ = create_wall_timer(
      period, std::bind(&PathFollowerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "C++ TP-ASD path follower started.");
    RCLCPP_INFO(get_logger(), "Path topic: %s", path_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Odom topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Cmd_vel topic: %s", cmd_vel_topic_.c_str());
    RCLCPP_INFO(get_logger(), "E-stop topic: %s", e_stop_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Motion safety gate: %s",
      enable_motion_ ? "ENABLED" : "DISABLED (publishing zero commands)");
    RCLCPP_INFO(
      get_logger(), "Velocity PID: %s",
      enable_velocity_pid_ ? "ENABLED" : "DISABLED");
  }

private:
  void validateParameters() const
  {
    const double values[] = {
      control_rate_hz_, lookahead_distance_, min_lookahead_distance_,
      max_lookahead_distance_, lookahead_time_, goal_tolerance_,
      goal_slowdown_distance_, max_path_deviation_, path_timeout_sec_,
      odom_timeout_sec_, tf_timeout_sec_, max_linear_speed_,
      max_angular_speed_, minimum_tracking_speed_, curvature_speed_gain_,
      rotate_in_place_angular_gain_, terrain_speed_gain_, pid_max_dt_sec_};
    const bool all_finite = std::all_of(
      std::begin(values), std::end(values),
      [](double value) {return std::isfinite(value);});
    if (!all_finite || control_rate_hz_ <= 0.0 || lookahead_distance_ <= 0.0 ||
      min_lookahead_distance_ <= 0.0 ||
      max_lookahead_distance_ < min_lookahead_distance_ ||
      lookahead_time_ < 0.0 || goal_tolerance_ <= 0.0 ||
      goal_slowdown_distance_ <= goal_tolerance_ ||
      max_path_deviation_ <= 0.0 || path_timeout_sec_ < 0.0 ||
      odom_timeout_sec_ <= 0.0 || tf_timeout_sec_ <= 0.0 ||
      max_linear_speed_ < 0.0 || max_angular_speed_ < 0.0 ||
      minimum_tracking_speed_ < 0.0 ||
      minimum_tracking_speed_ > max_linear_speed_ ||
      curvature_speed_gain_ < 0.0 || rotate_in_place_angular_gain_ <= 0.0 ||
      terrain_speed_gain_ < 0.0 ||
      pid_max_dt_sec_ < 1.0 / control_rate_hz_)
    {
      throw std::invalid_argument(
        "path follower parameter is outside its valid range");
    }
  }

  bool isFresh(double received_sec, double timeout_sec) 
  {
    if (received_sec < 0.0) {
      return false;
    }
    const double age = std::max(
      0.0, get_clock()->now().seconds() - received_sec);
    return age <= timeout_sec;
  }

  void publishStatus(const std::string & status_text)
  {
    if (status_text == last_status_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = status_text;
    status_pub_->publish(message);
    last_status_ = status_text;
  }

  void resetPidState()
  {
    linear_pid_.reset();
    angular_pid_.reset();
    last_pid_sec_ = -1.0;
  }

  void resetTrackingState()
  {
    resetPidState();
    pid_mode_initialized_ = false;
    last_metric_sec_ = -1.0;
    last_reference_linear_ = 0.0;
    last_acceleration_ = 0.0;
  }

  void publishZeroCommand()
  {
    resetTrackingState();
    geometry_msgs::msg::Twist command;
    cmd_pub_->publish(command);
  }

  VelocityCommandResult applyVelocityPid(
    double reference_linear,
    double reference_angular,
    double linear_limit,
    double angular_limit)
  {
    VelocityCommandResult result;
    const double bounded_reference_linear = clamp(
      reference_linear, 0.0, linear_limit);
    const double bounded_reference_angular = clamp(
      reference_angular, -angular_limit, angular_limit);
    result.twist.linear.x = bounded_reference_linear;
    result.twist.angular.z = bounded_reference_angular;
    result.saturated =
      bounded_reference_linear != reference_linear ||
      bounded_reference_angular != reference_angular;
    if (!enable_velocity_pid_) {
      return result;
    }

    const double now_sec = get_clock()->now().seconds();
    double dt = last_pid_sec_ < 0.0 ?
      1.0 / control_rate_hz_ : now_sec - last_pid_sec_;
    last_pid_sec_ = now_sec;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > pid_max_dt_sec_) {
      // Simulation time may pause or jump. Do not feed that discontinuity to
      // either the integral or derivative state.
      linear_pid_.reset();
      angular_pid_.reset();
      dt = 1.0 / control_rate_hz_;
    }
    const auto linear_result = linear_pid_.update(
      bounded_reference_linear, measured_linear_speed_, dt,
      -bounded_reference_linear,
      linear_limit - bounded_reference_linear);
    const auto angular_result = angular_pid_.update(
      bounded_reference_angular, measured_angular_speed_, dt,
      -angular_limit - bounded_reference_angular,
      angular_limit - bounded_reference_angular);
    if (!linear_result.valid || !angular_result.valid) {
      result.valid = false;
      result.twist = geometry_msgs::msg::Twist();
      return result;
    }
    result.twist.linear.x = clamp(
      bounded_reference_linear + linear_result.output, 0.0, linear_limit);
    result.twist.angular.z = clamp(
      bounded_reference_angular + angular_result.output,
      -angular_limit, angular_limit);
    result.saturated = result.saturated ||
      linear_result.saturated || angular_result.saturated;
    return result;
  }

  void pathCallback(const nav_msgs::msg::Path::SharedPtr message)
  {
    if (!message || message->poses.empty()) {
      path_.poses.clear();
      has_path_ = false;
      resetTrackingState();
      publishStatus("INVALID_PATH: empty path");
      return;
    }
    if (!message->header.frame_id.empty() &&
      message->header.frame_id != map_frame_)
    {
      path_.poses.clear();
      has_path_ = false;
      resetTrackingState();
      publishStatus("INVALID_PATH: frame mismatch");
      RCLCPP_WARN(
        get_logger(),
        "Ignoring path in frame '%s'; expected '%s'.",
        message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    path_ = *message;
    path_.header.frame_id = map_frame_;
    for (auto & pose : path_.poses) {
      pose.header.frame_id = map_frame_;
    }
    path_received_sec_ = get_clock()->now().seconds();
    nearest_index_ = 0U;
    ++path_version_;
    has_path_ = true;
    resetTrackingState();
    publishStatus("PATH_RECEIVED");
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!message || !enable_motion_) {
      has_odom_ = false;
      return;
    }

    if (message->header.frame_id.empty()) {
      has_odom_ = false;
      return;
    }

    geometry_msgs::msg::PoseStamped source_pose;
    source_pose.header = message->header;
    source_pose.pose = message->pose.pose;

    try {
      if (source_pose.header.frame_id == map_frame_) {
        map_pose_ = source_pose;
      } else {
        map_pose_ = tf_buffer_->transform(
          source_pose, map_frame_, tf2::durationFromSec(tf_timeout_sec_));
      }
    } catch (const tf2::TransformException & exception) {
      has_odom_ = false;
      if (!odom_tf_warning_logged_) {
        RCLCPP_WARN(
          get_logger(),
          "Cannot transform odometry from '%s' to '%s': %s",
          source_pose.header.frame_id.c_str(), map_frame_.c_str(),
          exception.what());
        odom_tf_warning_logged_ = true;
      }
      return;
    }

    odom_received_sec_ = get_clock()->now().seconds();
    // Preserve the sign so PID error and reported actual velocity expose
    // unintended rollback instead of treating it as forward progress.
    measured_linear_speed_ = message->twist.twist.linear.x;
    measured_angular_speed_ = message->twist.twist.angular.z;
    has_odom_ = true;
    if (odom_tf_warning_logged_) {
      RCLCPP_INFO(
        get_logger(), "Odometry transform to '%s' is available again.",
        map_frame_.c_str());
      odom_tf_warning_logged_ = false;
    }
  }

  void eStopCallback(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message) {
      return;
    }

    if (e_stop_active_ == message->data) {
      return;
    }

    e_stop_active_ = message->data;
    publishStatus(e_stop_active_ ?
      "E_STOP_ACTIVE: zero command" : "E_STOP_RELEASED");
  }

  void controlLoop()
  {
    if (e_stop_active_) {
      publishZeroCommand();
      publishStatus("E_STOP_ACTIVE: zero command");
      return;
    }

    if (!enable_motion_) {
      publishZeroCommand();
      publishStatus("DISABLED: zero command");
      return;
    }

    if (!has_path_) {
      publishZeroCommand();
      publishStatus("WAITING_PATH");
      return;
    }
    if (!has_odom_) {
      publishZeroCommand();
      publishStatus("WAITING_ODOM_TF");
      return;
    }
    if (path_timeout_sec_ > 0.0 &&
      !isFresh(path_received_sec_, path_timeout_sec_))
    {
      publishZeroCommand();
      publishStatus("STOPPED: stale path");
      return;
    }
    if (!isFresh(odom_received_sec_, odom_timeout_sec_)) {
      publishZeroCommand();
      publishStatus("STOPPED: stale odometry");
      return;
    }

    const auto & current_point = map_pose_.pose.position;
    const double current_yaw = yawFromQuaternion(map_pose_.pose.orientation);
    const auto & goal_point = path_.poses.back().pose.position;
    const double goal_distance = distance2d(current_point, goal_point);
    if (goal_distance <= goal_tolerance_) {
      publishZeroCommand();
      publishStatus("GOAL_REACHED: zero command");
      return;
    }

    std::size_t nearest_index = nearest_index_;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = nearest_index_; index < path_.poses.size(); ++index) {
      const double distance = distance2d(
        current_point, path_.poses[index].pose.position);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = index;
      }
    }
    nearest_index_ = nearest_index;

    if (nearest_distance > max_path_deviation_) {
      publishZeroCommand();
      publishStatus("STOPPED: path deviation exceeded");
      return;
    }

    PurePursuitConfig controller_config;
    controller_config.base_lookahead = lookahead_distance_;
    controller_config.min_lookahead = min_lookahead_distance_;
    controller_config.max_lookahead = max_lookahead_distance_;
    controller_config.lookahead_time = lookahead_time_;
    controller_config.goal_tolerance = goal_tolerance_;
    controller_config.goal_slowdown_distance = goal_slowdown_distance_;
    controller_config.max_linear_speed = max_linear_speed_;
    double terrain_impedance = 0.0;
    if (has_terrain_ && terrain_map_.info.resolution > 0.0) {
      const int column = static_cast<int>(std::floor(
        (current_point.x - terrain_map_.info.origin.position.x) / terrain_map_.info.resolution));
      const int row = static_cast<int>(std::floor(
        (current_point.y - terrain_map_.info.origin.position.y) / terrain_map_.info.resolution));
      if (column >= 0 && row >= 0 && column < static_cast<int>(terrain_map_.info.width) &&
        row < static_cast<int>(terrain_map_.info.height)) {
        const std::size_t cell = static_cast<std::size_t>(row) * terrain_map_.info.width + column;
        if (cell < terrain_map_.data.size() && terrain_map_.data[cell] >= 0)
          terrain_impedance = terrain_map_.data[cell];
      }
    }
    controller_config.max_linear_speed *= std::max(
      0.2, 1.0 - terrain_speed_gain_ * terrain_impedance / 100.0);
    controller_config.max_angular_speed = max_angular_speed_;
    controller_config.minimum_tracking_speed = minimum_tracking_speed_;
    controller_config.curvature_speed_gain = curvature_speed_gain_;
    controller_config.rotate_in_place_angular_gain =
      rotate_in_place_angular_gain_;

    const double dynamic_lookahead = computeDynamicLookahead(
      controller_config, std::abs(measured_linear_speed_));

    std::size_t target_index = path_.poses.size() - 1U;
    double arc_length = distance2d(
      current_point, path_.poses[nearest_index].pose.position);
    for (std::size_t index = nearest_index + 1U;
      index < path_.poses.size(); ++index)
    {
      arc_length += distance2d(
        path_.poses[index - 1U].pose.position,
        path_.poses[index].pose.position);
      if (arc_length >= dynamic_lookahead) {
        target_index = index;
        break;
      }
    }

    const auto & target_point = path_.poses[target_index].pose.position;
    const double dx = target_point.x - current_point.x;
    const double dy = target_point.y - current_point.y;
    const double target_x_robot =
      std::cos(current_yaw) * dx + std::sin(current_yaw) * dy;
    const double target_y_robot =
      -std::sin(current_yaw) * dx + std::cos(current_yaw) * dy;
    const PurePursuitCommand control = computePurePursuitCommand(
      controller_config, target_x_robot, target_y_robot, goal_distance);
    if (!control.valid) {
      publishZeroCommand();
      publishStatus("STOPPED: degenerate lookahead target");
      return;
    }

    if (control.aligning) {
      if (!pid_mode_initialized_ || !pid_was_aligning_) {
        resetPidState();
        last_metric_sec_ = -1.0;
        last_reference_linear_ = 0.0;
        last_acceleration_ = 0.0;
      }
      pid_mode_initialized_ = true;
      pid_was_aligning_ = true;
      const VelocityCommandResult rotate_command = applyVelocityPid(
        0.0, control.angular, 0.0, max_angular_speed_);
      cmd_pub_->publish(rotate_command.twist);

      std::ostringstream rotate_status;
      rotate_status << "ALIGNING_TO_PATH: target_index=" << target_index
                    << ", angular=" << control.angular
                    << ", path_deviation=" << nearest_distance;
      publishStatus(rotate_status.str());
      return;
    }

    if (!pid_mode_initialized_ || pid_was_aligning_) {
      resetPidState();
    }
    pid_mode_initialized_ = true;
    pid_was_aligning_ = false;
    const VelocityCommandResult command = applyVelocityPid(
      control.linear, control.angular,
      controller_config.max_linear_speed, max_angular_speed_);

    cmd_pub_->publish(command.twist);

    const std::size_t segment_end = std::min(nearest_index + 1U, path_.poses.size() - 1U);
    const auto & segment_a = path_.poses[nearest_index].pose.position;
    const auto & segment_b = path_.poses[segment_end].pose.position;
    const double sx = segment_b.x - segment_a.x;
    const double sy = segment_b.y - segment_a.y;
    const double segment_length = std::hypot(sx, sy);
    const double heading_error = segment_length > 1e-9 ?
      normalizeAngle(std::atan2(sy, sx) - current_yaw) : 0.0;
    const double lateral_error = segment_length > 1e-9 ?
      (sx * (current_point.y - segment_a.y) - sy * (current_point.x - segment_a.x)) /
      segment_length : nearest_distance;
    const double metric_now = get_clock()->now().seconds();
    const double metric_dt = last_metric_sec_ < 0.0 ? 0.0 : metric_now - last_metric_sec_;
    const double acceleration = metric_dt > 1e-6 ?
      (command.twist.linear.x - last_reference_linear_) / metric_dt : 0.0;
    const double jerk = metric_dt > 1e-6 ?
      (acceleration - last_acceleration_) / metric_dt : 0.0;
    radiation_interfaces::msg::ControlMetrics metrics;
    metrics.header.stamp = get_clock()->now();
    metrics.header.frame_id = map_frame_;
    metrics.path_version = path_version_;
    metrics.lateral_error_m = lateral_error;
    metrics.heading_error_rad = heading_error;
    metrics.curvature = control.curvature;
    metrics.reference_linear_mps = control.linear;
    metrics.actual_linear_mps = measured_linear_speed_;
    metrics.reference_angular_rps = control.angular;
    metrics.actual_angular_rps = measured_angular_speed_;
    metrics.saturated = command.saturated;
    metrics.linear_jerk_mps3 = jerk;
    metrics.terrain_impedance = terrain_impedance;
    metrics_pub_->publish(metrics);
    last_metric_sec_ = metric_now;
    last_reference_linear_ = command.twist.linear.x;
    last_acceleration_ = acceleration;

    std::ostringstream status;
    status << "TRACKING: target_index=" << target_index
           << ", goal_distance=" << goal_distance
           << ", lookahead=" << dynamic_lookahead
           << ", curvature=" << control.curvature
           << ", path_deviation=" << nearest_distance;
    if (enable_velocity_pid_) {
      status << ", pid_linear=" << command.twist.linear.x
             << ", pid_angular=" << command.twist.angular.z;
    }
    publishStatus(status.str());
  }

  std::string path_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string e_stop_topic_;
  std::string status_topic_;
  std::string map_frame_;

  bool enable_motion_{false};
  bool e_stop_active_{false};
  double control_rate_hz_{10.0};
  double lookahead_distance_{0.70};
  double min_lookahead_distance_{0.45};
  double max_lookahead_distance_{1.20};
  double lookahead_time_{1.50};
  double goal_tolerance_{0.35};
  double goal_slowdown_distance_{1.50};
  double max_path_deviation_{1.50};
  double path_timeout_sec_{0.0};
  double odom_timeout_sec_{1.0};
  double tf_timeout_sec_{0.2};
  double max_linear_speed_{0.20};
  double max_angular_speed_{0.80};
  double minimum_tracking_speed_{0.04};
  double curvature_speed_gain_{1.50};
  double rotate_in_place_angular_gain_{1.50};
  double measured_linear_speed_{0.0};
  double measured_angular_speed_{0.0};
  double terrain_speed_gain_{0.70};
  bool enable_velocity_pid_{true};
  double pid_max_dt_sec_{0.50};
  PidController linear_pid_;
  PidController angular_pid_;
  double last_pid_sec_{-1.0};
  double last_metric_sec_{-1.0};
  double last_reference_linear_{0.0};
  double last_acceleration_{0.0};
  bool pid_mode_initialized_{false};
  bool pid_was_aligning_{false};
  std::uint64_t path_version_{0U};

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped map_pose_;
  bool has_path_{false};
  bool has_odom_{false};
  bool odom_tf_warning_logged_{false};
  bool has_terrain_{false};
  nav_msgs::msg::OccupancyGrid terrain_map_;
  std::size_t nearest_index_{0U};
  double path_received_sec_{-1.0};
  double odom_received_sec_{-1.0};
  std::string last_status_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr terrain_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<radiation_interfaces::msg::ControlMetrics>::SharedPtr metrics_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace risk_aware_planner_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<risk_aware_planner_cpp::PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
