
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "risk_aware_planner_cpp/safety_gate_core.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace risk_aware_planner_cpp
{

class CmdVelSafetyNode : public rclcpp::Node
{
public:
  CmdVelSafetyNode()
  : Node("tp_asd_rrt_star_cmd_vel_safety_cpp")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/control/base_cmd");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/cmd_vel");
    estop_topic_ = declare_parameter<std::string>(
      "estop_topic", "/e_stop");
    status_topic_ = declare_parameter<std::string>(
      "status_topic",
      "/tp_asd_rrt_star_cpp_safety_status");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    map_topic_ = declare_parameter<std::string>("map_topic", "/risk_map");
    path_topic_ = declare_parameter<std::string>("path_topic", "/tp_asd_rrt_star_cpp_path");
    navigation_status_topic_ = declare_parameter<std::string>(
      "navigation_status_topic", "/tp_asd_rrt_star_cpp_follower_status");

    enable_motion_ = declare_parameter<bool>(
      "enable_motion", false);
    allow_reverse_ = declare_parameter<bool>(
      "allow_reverse", false);
    publish_rate_hz_ = declare_parameter<double>(
      "publish_rate_hz", 20.0);
    command_timeout_sec_ = declare_parameter<double>(
      "command_timeout_sec", 0.5);
    max_linear_speed_ = declare_parameter<double>(
      "max_linear_speed", 0.20);
    max_angular_speed_ = declare_parameter<double>(
      "max_angular_speed", 0.60);
    max_linear_accel_ = declare_parameter<double>(
      "max_linear_accel", 0.30);
    max_angular_accel_ = declare_parameter<double>(
      "max_angular_accel", 1.50);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 1.0);
    map_timeout_sec_ = declare_parameter<double>("map_timeout_sec", 5.0);
    state_timeout_sec_ = declare_parameter<double>("state_timeout_sec", 1.0);
    estop_timeout_sec_ = declare_parameter<double>("estop_timeout_sec", 1.0);

    cmd_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(
        output_topic_, rclcpp::QoS(10).reliable());

    status_pub_ =
      create_publisher<std_msgs::msg::String>(
        status_topic_, rclcpp::QoS(10).reliable());

    cmd_sub_ =
      create_subscription<geometry_msgs::msg::Twist>(
        input_topic_,
        rclcpp::QoS(10).reliable(),
        std::bind(
          &CmdVelSafetyNode::cmdCallback,
          this,
          std::placeholders::_1));

    estop_sub_ =
      create_subscription<std_msgs::msg::Bool>(
        estop_topic_,
        rclcpp::QoS(10).best_effort(),
        std::bind(
          &CmdVelSafetyNode::estopCallback,
          this,
          std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        if (msg) {std::lock_guard<std::mutex> lock(mutex_); last_odom_time_ = now(); has_odom_ = true;}
      });
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (msg) {std::lock_guard<std::mutex> lock(mutex_); last_map_time_ = now(); has_map_ = true;}
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10, [this](nav_msgs::msg::Path::SharedPtr msg) {
        if (!msg) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        const rclcpp::Time stamp(msg->header.stamp);
        path_version_valid_ = !has_path_ || stamp >= last_path_stamp_;
        if (path_version_valid_) {last_path_stamp_ = stamp;}
        has_path_ = !msg->poses.empty();
        last_path_time_ = now();
      });
    navigation_status_sub_ = create_subscription<std_msgs::msg::String>(
      navigation_status_topic_, 10, [this](std_msgs::msg::String::SharedPtr msg) {
        if (!msg) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        const auto & value = msg->data;
        navigation_tracking_ = value.find("TRACKING") == 0 ||
          value.find("ALIGNING") == 0 || value.find("PATH_RECEIVED") == 0;
        last_state_time_ = now();
        has_state_ = true;
      });

    last_command_time_ = now();
    last_update_time_ = now();

    const double period =
      1.0 / std::max(1.0, publish_rate_hz_);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period)),
      std::bind(
        &CmdVelSafetyNode::controlLoop,
        this));

    publishStatus(
      enable_motion_ ?
      "SAFETY_ENABLED" :
      "SAFETY_DISABLED");
  }

private:
  static double clamp(
    double value,
    double minimum,
    double maximum)
  {
    return std::max(minimum, std::min(maximum, value));
  }

  static double approach(
    double current,
    double target,
    double maximum_delta)
  {
    if (target > current) {
      return std::min(target, current + maximum_delta);
    }

    return std::max(target, current - maximum_delta);
  }

  void cmdCallback(
    const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_command_ = *message;
    has_command_ = true;
    command_valid_ =
      std::isfinite(message->linear.x) &&
      std::isfinite(message->angular.z);
    last_command_time_ = now();
  }

  void estopCallback(
    const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    estop_active_ = message->data;
    has_estop_ = true;
    last_estop_time_ = now();
  }

  void publishStatus(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }

    last_status_ = status;

    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist input;
    SafetyGateInput gate_input;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      input = latest_command_;
      const auto current = now();
      gate_input.enabled = enable_motion_;
      gate_input.estop = estop_active_;
      gate_input.command_finite = has_command_ && command_valid_;
      gate_input.navigation_tracking = has_state_ && navigation_tracking_;
      gate_input.path_valid = has_path_;
      gate_input.path_version_valid = path_version_valid_;
      gate_input.command_age_sec = has_command_ ? (current - last_command_time_).seconds() : -1.0;
      gate_input.odom_age_sec = has_odom_ ? (current - last_odom_time_).seconds() : -1.0;
      gate_input.map_age_sec = has_map_ ? (current - last_map_time_).seconds() : -1.0;
      gate_input.path_age_sec = has_path_ ? (current - last_path_time_).seconds() : -1.0;
      gate_input.state_age_sec = has_state_ ? (current - last_state_time_).seconds() : -1.0;
      gate_input.estop_age_sec = has_estop_ ? (current - last_estop_time_).seconds() : -1.0;
      gate_input.linear = input.linear.x;
      gate_input.angular = input.angular.z;
    }

    const rclcpp::Time current_time = now();
    double dt = (current_time - last_update_time_).seconds();

    if (dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / std::max(1.0, publish_rate_hz_);
    }

    last_update_time_ = current_time;

    SafetyGateConfig config;
    config.command_timeout_sec = command_timeout_sec_;
    config.odom_timeout_sec = odom_timeout_sec_;
    config.map_timeout_sec = map_timeout_sec_;
    config.path_timeout_sec = 0.0;
    config.state_timeout_sec = state_timeout_sec_;
    config.estop_timeout_sec = estop_timeout_sec_;
    config.max_linear_speed = max_linear_speed_;
    config.max_angular_speed = max_angular_speed_;
    config.max_linear_accel = max_linear_accel_;
    config.max_angular_accel = max_angular_accel_;
    config.allow_reverse = allow_reverse_;
    const auto output = evaluateSafetyGate(
      config, gate_input, current_command_.linear.x, current_command_.angular.z, dt);
    if (!output.allowed) {current_command_ = geometry_msgs::msg::Twist();}
    else {
      current_command_.linear.x = output.linear;
      current_command_.angular.z = output.angular;
    }
    publishStatus(output.reason);

    current_command_.linear.y = 0.0;
    current_command_.linear.z = 0.0;
    current_command_.angular.x = 0.0;
    current_command_.angular.y = 0.0;

    cmd_pub_->publish(current_command_);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string estop_topic_;
  std::string status_topic_;
  std::string odom_topic_, map_topic_, path_topic_, navigation_status_topic_;

  bool enable_motion_{false};
  bool allow_reverse_{false};
  bool estop_active_{false};
  bool has_command_{false};
  bool command_valid_{false};
  bool has_odom_{false}, has_map_{false}, has_path_{false}, has_state_{false}, has_estop_{false};
  bool navigation_tracking_{false}, path_version_valid_{true};

  double publish_rate_hz_{20.0};
  double command_timeout_sec_{0.5};
  double max_linear_speed_{0.20};
  double max_angular_speed_{0.60};
  double max_linear_accel_{0.30};
  double max_angular_accel_{1.50};
  double odom_timeout_sec_{1.0}, map_timeout_sec_{5.0};
  double state_timeout_sec_{1.0}, estop_timeout_sec_{1.0};

  geometry_msgs::msg::Twist latest_command_;
  geometry_msgs::msg::Twist current_command_;

  rclcpp::Time last_command_time_;
  rclcpp::Time last_update_time_;
  rclcpp::Time last_odom_time_, last_map_time_, last_path_time_, last_state_time_, last_estop_time_;
  rclcpp::Time last_path_stamp_{0, 0, RCL_ROS_TIME};

  std::string last_status_;
  std::mutex mutex_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<
    std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace risk_aware_planner_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      risk_aware_planner_cpp::CmdVelSafetyNode>());

  rclcpp::shutdown();
  return 0;
}
