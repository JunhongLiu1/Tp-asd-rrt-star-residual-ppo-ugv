#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "radiation_interfaces/msg/radiation_measurement.hpp"
#include "radiation_interfaces/msg/risk_map.hpp"
#include "risk_aware_planner_cpp/risk_map_core.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace risk_aware_planner_cpp
{

class RadiationOnlineMapperNode : public rclcpp::Node
{
public:
  RadiationOnlineMapperNode()
  : Node("radiation_online_mapper_cpp"),
    tf_buffer_(this->get_clock())
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    geometry_topic_ = declare_parameter<std::string>(
      "geometry_topic", "/ground_truth/radiation_map");
    risk_map_topic_ = declare_parameter<std::string>(
      "risk_map_topic", "/risk_map");
    metadata_topic_ = declare_parameter<std::string>(
      "metadata_topic", "/risk_map/metadata");
    update_radius_m_ = declare_parameter<double>("update_radius_m", 2.5);
    sigma_m_ = declare_parameter<double>("sigma_m", 0.8);
    dose_to_risk_gain_ = declare_parameter<double>("dose_to_risk_gain", 80.0);
    dose_replan_threshold_ =
      declare_parameter<double>("dose_replan_threshold", 0.5);
    dose_stop_threshold_ =
      declare_parameter<double>("dose_stop_threshold", 8.0);
    path_risk_threshold_ =
      declare_parameter<int>("path_risk_threshold", 70);
    publish_rate_hz_ =
      declare_parameter<double>("publish_rate_hz", 2.0);
    RiskMapConfig core_config;
    core_config.update_radius_m = update_radius_m_;
    core_config.sigma_m = sigma_m_;
    core_config.filter_alpha = declare_parameter<double>("filter_alpha", 0.35);
    core_config.confidence_gain = declare_parameter<double>("confidence_gain", 0.25);
    core_config.confidence_decay_per_sec = declare_parameter<double>("confidence_decay_per_sec", 0.002);
    risk_core_.reset(new RiskMapCore(core_config));

    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    auto map_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      geometry_topic_,
      map_qos,
      std::bind(
        &RadiationOnlineMapperNode::mapCallback,
        this,
        std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered",
      rclcpp::QoS(10).reliable(),
      std::bind(
        &RadiationOnlineMapperNode::odomCallback,
        this,
        std::placeholders::_1));

    dose_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/radiation/dose_rate_usv_h",
      rclcpp::QoS(10).reliable(),
      std::bind(
        &RadiationOnlineMapperNode::doseCallback,
        this,
        std::placeholders::_1));
    measurement_sub_ = create_subscription<radiation_interfaces::msg::RadiationMeasurement>(
      "/radiation/sensor_measurement", rclcpp::QoS(10).reliable(),
      std::bind(&RadiationOnlineMapperNode::measurementCallback, this, std::placeholders::_1));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/tp_asd_rrt_star_cpp_path",
      rclcpp::QoS(10).reliable(),
      std::bind(
        &RadiationOnlineMapperNode::pathCallback,
        this,
        std::placeholders::_1));

    online_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      risk_map_topic_,
      map_qos);

    metadata_pub_ = create_publisher<std_msgs::msg::String>(
      metadata_topic_, map_qos);
    float_map_pub_ = create_publisher<radiation_interfaces::msg::RiskMap>(
      "/risk_map/continuous", map_qos);

    replan_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/tp_asd_rrt_star_radiation_replan_required",
      rclcpp::QoS(10).reliable());

    e_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/e_stop",
      rclcpp::QoS(10).reliable());

    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/radiation_online_status",
      rclcpp::QoS(1).reliable().transient_local());

    std_msgs::msg::String startup_status;
    startup_status.data = "MAPPER_STARTED";
    status_pub_->publish(startup_status);

    const auto period_ms = std::max(
      50,
      static_cast<int>(1000.0 / std::max(0.1, publish_rate_hz_)));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(
        &RadiationOnlineMapperNode::publishTimer,
        this));

    RCLCPP_INFO(
      get_logger(),
      "Online radiation mapper started. map_frame=%s, update_radius=%.2f m",
      map_frame_.c_str(),
      update_radius_m_);
  }

private:
  struct Point2D
  {
    double x{0.0};
    double y{0.0};
  };

  void mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    const bool geometry_changed =
      !map_ready_ ||
      online_map_.info.width != message->info.width ||
      online_map_.info.height != message->info.height ||
      std::abs(
        online_map_.info.resolution - message->info.resolution) > 1e-9;

    if (geometry_changed) {
      online_map_ = *message;
      // 真值图只提供栅格几何；在线风险只能来自局部传感器测量。
      std::fill(online_map_.data.begin(), online_map_.data.end(), -1);
      observed_cells_ = 0U;
      map_version_ = 0U;
      map_ready_ = true;
      RiskMapGeometry geometry;
      geometry.width = message->info.width;
      geometry.height = message->info.height;
      geometry.resolution = message->info.resolution;
      geometry.origin_x = message->info.origin.position.x;
      geometry.origin_y = message->info.origin.position.y;
      geometry.origin_yaw = tf2::getYaw(message->info.origin.orientation);
      risk_core_->resetGeometry(geometry);
      map_version_ = risk_core_->version();
    } else {
      online_map_.header = message->header;
    }

    updateHazardLocked();
  }

  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!message) {
      return;
    }

    geometry_msgs::msg::PoseStamped input;
    input.header = message->header;
    input.pose = message->pose.pose;

    geometry_msgs::msg::PoseStamped transformed;

    try {
      if (input.header.frame_id.empty() ||
        input.header.frame_id == map_frame_)
      {
        transformed = input;
      } else {
        transformed = tf_buffer_.transform(
          input,
          map_frame_,
          tf2::durationFromSec(0.05));
      }
    } catch (const tf2::TransformException &) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    robot_position_.x = transformed.pose.position.x;
    robot_position_.y = transformed.pose.position.y;
    robot_position_valid_ = true;
  }

  void publishSafetyState(
    bool hazard,
    const std::string & status)
  {
    std_msgs::msg::Bool replan_message;
    replan_message.data = hazard;

    std_msgs::msg::Bool e_stop_message;
    e_stop_message.data = hazard;

    std_msgs::msg::String status_message;
    status_message.data = status;

    replan_pub_->publish(replan_message);
    e_stop_pub_->publish(e_stop_message);
    status_pub_->publish(status_message);
  }

  void doseCallback(
    const std_msgs::msg::Float64::SharedPtr message)
  {
    if (!message || !std::isfinite(message->data)) {
      return;
    }

    bool immediate_stop = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      last_dose_rate_ = std::max(0.0, message->data);

      if (last_dose_rate_ >= dose_replan_threshold_) {
        dose_triggered_ = true;
      }
      immediate_stop = last_dose_rate_ >= dose_stop_threshold_;
      radiation_stop_ = immediate_stop;

      if (map_ready_ && robot_position_valid_ && risk_core_->applyMeasurement(
          robot_position_.x, robot_position_.y, last_dose_rate_, now().seconds())) {
        syncVisualizationFromCoreLocked();
      }
      updateHazardLocked();
    }

    /*
     * 安全信号不能等待地图定时发布：
     * 只要实时剂量超阈值，立即停车并请求重新规划。
     */
    if (immediate_stop) {
      publishSafetyState(
        true,
        "RADIATION_STOP: DOSE_RATE_THRESHOLD");
    }
  }

  void measurementCallback(
    const radiation_interfaces::msg::RadiationMeasurement::SharedPtr message)
  {
    if (!message || !std::isfinite(message->dose_rate_usv_h) ||
      message->dose_rate_usv_h < 0.0 || message->header.frame_id.empty()) return;
    geometry_msgs::msg::PoseStamped sensor_pose;
    sensor_pose.header = message->header;
    sensor_pose.pose.orientation.w = 1.0;
    try {
      sensor_pose = tf_buffer_.transform(sensor_pose, map_frame_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException &) {return;}
    bool immediate_stop = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      last_dose_rate_ = message->dose_rate_usv_h;
      const double stamp = rclcpp::Time(message->header.stamp).seconds();
      if (map_ready_ && risk_core_->applyMeasurement(
          sensor_pose.pose.position.x, sensor_pose.pose.position.y,
          last_dose_rate_, stamp)) syncVisualizationFromCoreLocked();
      dose_triggered_ = dose_triggered_ ||
        last_dose_rate_ >= dose_replan_threshold_;
      immediate_stop = last_dose_rate_ >= dose_stop_threshold_;
      radiation_stop_ = immediate_stop;
      updateHazardLocked();
    }
    if (immediate_stop) publishSafetyState(true, "RADIATION_STOP: DOSE_RATE_THRESHOLD");
  }

  void syncVisualizationFromCoreLocked()
  {
    const auto & dose = risk_core_->doseRate();
    const auto & confidence = risk_core_->confidence();
    observed_cells_ = 0U;
    for (std::size_t i = 0; i < online_map_.data.size() && i < dose.size(); ++i) {
      if (confidence[i] <= 0.0F) {online_map_.data[i] = -1; continue;}
      ++observed_cells_;
      online_map_.data[i] = static_cast<int8_t>(std::round(std::min(
        100.0, dose_to_risk_gain_ * static_cast<double>(dose[i]))));
    }
    map_version_ = risk_core_->version();
  }

  void pathCallback(
    const nav_msgs::msg::Path::SharedPtr message)
  {
    if (!message) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    latest_path_ = *message;
    path_received_ = true;

    /*
     * 收到一条新的、经过验证的低风险路径后，
     * 清除上一轮剂量触发标志，允许恢复运动。
     */
    if (!pathHasHighRiskLocked()) {
      dose_triggered_ = false;
    }

    updateHazardLocked();
  }

  bool worldToCell(
    double x,
    double y,
    unsigned int & column,
    unsigned int & row) const
  {
    if (!map_ready_ || online_map_.info.resolution <= 0.0) {
      return false;
    }

    const double origin_x = online_map_.info.origin.position.x;
    const double origin_y = online_map_.info.origin.position.y;
    const double yaw = tf2::getYaw(online_map_.info.origin.orientation);

    const double dx = x - origin_x;
    const double dy = y - origin_y;

    const double local_x = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    if (local_x < 0.0 || local_y < 0.0) {
      return false;
    }

    const int c = static_cast<int>(
      std::floor(local_x / online_map_.info.resolution));
    const int r = static_cast<int>(
      std::floor(local_y / online_map_.info.resolution));

    if (c < 0 || r < 0 ||
      c >= static_cast<int>(online_map_.info.width) ||
      r >= static_cast<int>(online_map_.info.height))
    {
      return false;
    }

    column = static_cast<unsigned int>(c);
    row = static_cast<unsigned int>(r);
    return true;
  }

  bool applyDoseUpdateLocked()
  {
    if (!map_ready_ || !robot_position_valid_ ||
      last_dose_rate_ <= 0.0)
    {
      return false;
    }

    unsigned int robot_column = 0;
    unsigned int robot_row = 0;

    if (!worldToCell(
        robot_position_.x,
        robot_position_.y,
        robot_column,
        robot_row))
    {
      return false;
    }

    bool changed = false;

    const double resolution = online_map_.info.resolution;
    const int radius_cells = static_cast<int>(
      std::ceil(update_radius_m_ / resolution));

    const double sigma2 =
      std::max(0.01, sigma_m_ * sigma_m_);

    for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
      for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
        const int c =
          static_cast<int>(robot_column) + dc;
        const int r =
          static_cast<int>(robot_row) + dr;

        if (c < 0 || r < 0 ||
          c >= static_cast<int>(online_map_.info.width) ||
          r >= static_cast<int>(online_map_.info.height))
        {
          continue;
        }

        const double distance_x = static_cast<double>(dc) * resolution;
        const double distance_y = static_cast<double>(dr) * resolution;
        const double distance2 =
          distance_x * distance_x + distance_y * distance_y;

        if (distance2 > update_radius_m_ * update_radius_m_) {
          continue;
        }

        const double gaussian =
          std::exp(-distance2 / (2.0 * sigma2));

        const double risk =
          std::min(
            100.0,
            std::max(
              0.0,
              dose_to_risk_gain_ * last_dose_rate_ * gaussian));

        const std::size_t index =
          static_cast<std::size_t>(r) *
          online_map_.info.width +
          static_cast<std::size_t>(c);

        const int old_value = online_map_.data[index];

        const int measured_value = static_cast<int>(std::round(risk));
        const int new_value = old_value < 0 ?
          measured_value : std::max(old_value, measured_value);
        if (new_value != old_value) {
          if (old_value < 0) {
            ++observed_cells_;
          }
          online_map_.data[index] = new_value;
          changed = true;
        }
      }
    }
    return changed;
  }

  bool pathHasHighRiskLocked() const
  {
    if (!map_ready_ || !path_received_) {
      return false;
    }

    for (const auto & pose : latest_path_.poses) {
      geometry_msgs::msg::PoseStamped input = pose;
      geometry_msgs::msg::PoseStamped transformed;

      try {
        if (input.header.frame_id.empty() ||
          input.header.frame_id == map_frame_)
        {
          transformed = input;
        } else {
          transformed = tf_buffer_.transform(
            input,
            map_frame_,
            tf2::durationFromSec(0.05));
        }
      } catch (const tf2::TransformException &) {
        continue;
      }

      unsigned int column = 0;
      unsigned int row = 0;

      if (!worldToCell(
          transformed.pose.position.x,
          transformed.pose.position.y,
          column,
          row))
      {
        continue;
      }

      const std::size_t index =
        static_cast<std::size_t>(row) *
        online_map_.info.width +
        static_cast<std::size_t>(column);

      if (index < online_map_.data.size() &&
        online_map_.data[index] >= path_risk_threshold_)
      {
        return true;
      }
    }

    return false;
  }

  void updateHazardLocked()
  {
    const bool path_risk = pathHasHighRiskLocked();
    radiation_hazard_ = dose_triggered_ || path_risk;

    if (radiation_hazard_) {
      hazard_reason_ = path_risk ?
        "PATH_HIGH_RADIATION" :
        "DOSE_RATE_THRESHOLD";
    } else {
      hazard_reason_ = "CLEAR";
    }
  }

  void publishTimer()
  {
    nav_msgs::msg::OccupancyGrid map_message;
    bool map_ready = false;
    bool hazard = false;
    bool emergency_stop = false;
    double dose = 0.0;
    std::uint64_t map_version = 0U;
    std::size_t observed_cells = 0U;
    std::string reason;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      map_ready = map_ready_;
      dose = last_dose_rate_;
      map_version = map_version_;
      observed_cells = observed_cells_;

      if (map_ready_) {
        updateHazardLocked();

        online_map_.header.stamp = now();
        map_message = online_map_;
        hazard = radiation_hazard_;
        emergency_stop = radiation_stop_;
        reason = hazard_reason_;
      } else {
        hazard = false;
        reason = "WAITING_RADIATION_MAP";
      }
    }

    std_msgs::msg::Bool replan_message;
    replan_message.data = hazard;

    std_msgs::msg::Bool e_stop_message;
    e_stop_message.data = emergency_stop;

    std_msgs::msg::String status_message;

    if (map_ready) {
      status_message.data =
        hazard ?
        "RADIATION_STOP: " + reason :
        "RADIATION_CLEAR: dose_rate=" + std::to_string(dose);
    } else {
      status_message.data = "WAITING_RADIATION_MAP";
    }

    if (map_ready) {
      online_map_pub_->publish(map_message);

      radiation_interfaces::msg::RiskMap float_map;
      float_map.header = map_message.header;
      float_map.info = map_message.info;
      float_map.version = risk_core_->version();
      float_map.updated_at_sec = risk_core_->updatedAt();
      float_map.coverage = static_cast<float>(risk_core_->coverage());
      float_map.dose_rate_usv_h = risk_core_->doseRate();
      float_map.confidence = risk_core_->confidence();
      float_map_pub_->publish(float_map);

      const std::size_t total_cells = map_message.data.size();
      const double coverage = total_cells == 0U ? 0.0 :
        static_cast<double>(observed_cells) /
        static_cast<double>(total_cells);
      std_msgs::msg::String metadata_message;
      std::ostringstream metadata;
      metadata << "map_version=" << map_version
               << ", coverage=" << coverage
               << ", observed_cells=" << observed_cells
               << ", total_cells=" << total_cells;
      metadata_message.data = metadata.str();
      metadata_pub_->publish(metadata_message);
    }

    replan_pub_->publish(replan_message);
    e_stop_pub_->publish(e_stop_message);
    status_pub_->publish(status_message);
  }

  std::string map_frame_;
  std::string geometry_topic_;
  std::string risk_map_topic_;
  std::string metadata_topic_;
  double update_radius_m_{2.5};
  double sigma_m_{0.8};
  double dose_to_risk_gain_{80.0};
  double dose_replan_threshold_{0.5};
  double dose_stop_threshold_{8.0};
  int path_risk_threshold_{70};
  double publish_rate_hz_{2.0};

  bool map_ready_{false};
  bool robot_position_valid_{false};
  bool path_received_{false};
  bool dose_triggered_{false};
  bool radiation_hazard_{false};
  bool radiation_stop_{false};

  double last_dose_rate_{0.0};
  std::uint64_t map_version_{0U};
  std::size_t observed_cells_{0U};
  std::string hazard_reason_{"CLEAR"};

  Point2D robot_position_;
  nav_msgs::msg::OccupancyGrid online_map_;
  nav_msgs::msg::Path latest_path_;
  std::unique_ptr<RiskMapCore> risk_core_;

  std::mutex mutex_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dose_sub_;
  rclcpp::Subscription<radiation_interfaces::msg::RadiationMeasurement>::SharedPtr measurement_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr online_map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub_;
  rclcpp::Publisher<radiation_interfaces::msg::RiskMap>::SharedPtr float_map_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace risk_aware_planner_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<
      risk_aware_planner_cpp::RadiationOnlineMapperNode>());
  rclcpp::shutdown();
  return 0;
}
