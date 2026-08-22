#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace risk_aware_planner_cpp
{

class MapOdomAlignmentNode : public rclcpp::Node
{
public:
  MapOdomAlignmentNode()
  : Node("map_odom_alignment_cpp")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    ground_truth_topic_ = declare_parameter<std::string>(
      "ground_truth_topic", "/ground_truth/odom");
    filtered_odom_topic_ = declare_parameter<std::string>(
      "filtered_odom_topic", "/odometry/filtered");

    broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    ground_truth_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(
        &MapOdomAlignmentNode::groundTruthCallback,
        this,
        std::placeholders::_1));

    filtered_odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
      filtered_odom_topic_,
      rclcpp::QoS(10).reliable(),
      std::bind(
        &MapOdomAlignmentNode::filteredOdomCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Waiting to align %s -> %s from ground-truth and filtered odometry.",
      map_frame_.c_str(),
      odom_frame_.c_str());
  }

private:
  void groundTruthCallback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!message) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      ground_truth_ = *message;
      have_ground_truth_ = true;
    }

    tryPublishTransform();
  }

  void filteredOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!message) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      filtered_odom_ = *message;
      have_filtered_odom_ = true;
    }

    tryPublishTransform();
  }

  void tryPublishTransform()
  {
    nav_msgs::msg::Odometry ground_truth;
    nav_msgs::msg::Odometry filtered_odom;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (transform_published_ ||
        !have_ground_truth_ ||
        !have_filtered_odom_)
      {
        return;
      }

      ground_truth = ground_truth_;
      filtered_odom = filtered_odom_;
    }

    /*
     * 当前世界中 map 坐标与 Gazebo ground-truth world 坐标一致。
     * 求 T_map_odom = T_map_base * inverse(T_odom_base)。
     */
    const double yaw_map_base =
      tf2::getYaw(ground_truth.pose.pose.orientation);
    const double yaw_odom_base =
      tf2::getYaw(filtered_odom.pose.pose.orientation);
    const double yaw_map_odom =
      yaw_map_base - yaw_odom_base;

    const double c = std::cos(yaw_map_odom);
    const double s = std::sin(yaw_map_odom);

    const double odom_x = filtered_odom.pose.pose.position.x;
    const double odom_y = filtered_odom.pose.pose.position.y;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;

    transform.transform.translation.x =
      ground_truth.pose.pose.position.x - (c * odom_x - s * odom_y);
    transform.transform.translation.y =
      ground_truth.pose.pose.position.y - (s * odom_x + c * odom_y);
    transform.transform.translation.z = 0.0;

    tf2::Quaternion rotation;
    rotation.setRPY(0.0, 0.0, yaw_map_odom);
    transform.transform.rotation = tf2::toMsg(rotation);

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (transform_published_) {
        return;
      }

      broadcaster_->sendTransform(transform);
      transform_published_ = true;
    }

    RCLCPP_INFO(
      get_logger(),
      "Published %s -> %s: x=%.3f, y=%.3f, yaw=%.3f rad",
      map_frame_.c_str(),
      odom_frame_.c_str(),
      transform.transform.translation.x,
      transform.transform.translation.y,
      yaw_map_odom);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string ground_truth_topic_;
  std::string filtered_odom_topic_;

  std::mutex mutex_;
  bool have_ground_truth_{false};
  bool have_filtered_odom_{false};
  bool transform_published_{false};

  nav_msgs::msg::Odometry ground_truth_;
  nav_msgs::msg::Odometry filtered_odom_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    ground_truth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    filtered_odom_sub_;
};

}  // namespace risk_aware_planner_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      risk_aware_planner_cpp::MapOdomAlignmentNode>());
  rclcpp::shutdown();
  return 0;
}
