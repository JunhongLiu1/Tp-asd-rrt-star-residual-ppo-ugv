#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "radiation_interfaces/msg/planner_metrics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace risk_aware_planner_cpp
{

using Grid = nav_msgs::msg::OccupancyGrid;
using Path = nav_msgs::msg::Path;
using Metrics = radiation_interfaces::msg::PlannerMetrics;

struct PathQuality
{
  bool valid{false};
  double path_length_m{0.0};
  double terrain_cost{0.0};
  double radiation_cost{0.0};
  double time_cost{0.0};
  double estimated_time_sec{0.0};
  double minimum_traversability{1.0};
  double maximum_radiation{0.0};
};

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

std::string statusState(const std::string & status)
{
  const auto separator = status.find(':');
  if (separator == std::string::npos) {
    return status;
  }
  return status.substr(0, separator);
}

class PlannerMetricsNode : public rclcpp::Node
{
public:
  PlannerMetricsNode()
  : Node("tp_asd_rrt_star_metrics_cpp")
  {
    path_topic_ = declare_parameter<std::string>(
      "path_topic", "/tp_asd_rrt_star_cpp_path");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/tp_asd_rrt_star_cpp_status");
    metrics_topic_ = declare_parameter<std::string>(
      "metrics_topic", "/tp_asd_rrt_star_cpp_metrics");
    radiation_topic_ = declare_parameter<std::string>(
      "radiation_topic", "/radiation_map");
    terrain_topic_ = declare_parameter<std::string>(
      "terrain_topic", "/terrain_impedance_map");
    traversability_topic_ = declare_parameter<std::string>(
      "traversability_topic", "/terrain_traversability_mask");
    goal_topic_ = declare_parameter<std::string>(
      "goal_topic", "/goal_pose");

    sample_resolution_m_ = declare_parameter<double>(
      "sample_resolution_m", 0.10);
    nominal_speed_mps_ = declare_parameter<double>(
      "nominal_speed_mps", 0.20);
    terrain_weight_ = declare_parameter<double>(
      "terrain_weight", 1.0);
    radiation_weight_ = declare_parameter<double>(
      "radiation_weight", 1.0);
    time_weight_ = declare_parameter<double>(
      "time_weight", 0.2);

    metrics_pub_ = create_publisher<Metrics>(metrics_topic_, 10);

    path_sub_ = create_subscription<Path>(
      path_topic_,
      rclcpp::QoS(10).best_effort(),
      std::bind(&PlannerMetricsNode::pathCallback, this, std::placeholders::_1));

    status_sub_ = create_subscription<std_msgs::msg::String>(
      status_topic_,
      rclcpp::QoS(10).best_effort(),
      std::bind(&PlannerMetricsNode::statusCallback, this, std::placeholders::_1));

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_,
      rclcpp::QoS(10).best_effort(),
      std::bind(
        &PlannerMetricsNode::goalCallback,
        this,
        std::placeholders::_1));

    const auto map_qos =
      rclcpp::QoS(1).reliable().transient_local();

    radiation_sub_ = create_subscription<Grid>(
      radiation_topic_,
      map_qos,
      std::bind(
        &PlannerMetricsNode::radiationCallback, this, std::placeholders::_1));

    terrain_sub_ = create_subscription<Grid>(
      terrain_topic_,
      map_qos,
      std::bind(
        &PlannerMetricsNode::terrainCallback, this, std::placeholders::_1));

    traversability_sub_ = create_subscription<Grid>(
      traversability_topic_,
      map_qos,
      std::bind(
        &PlannerMetricsNode::traversabilityCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Planner metrics node started. Publishing: %s",
      metrics_topic_.c_str());
  }

private:
  static bool sampleGrid(
    const std::shared_ptr<const Grid> & grid,
    double x,
    double y,
    double & normalized_value)
  {
    if (!grid || grid->info.resolution <= 0.0) {
      return false;
    }

    const double origin_x = grid->info.origin.position.x;
    const double origin_y = grid->info.origin.position.y;
    const int ix = static_cast<int>(
      std::floor((x - origin_x) / grid->info.resolution));
    const int iy = static_cast<int>(
      std::floor((y - origin_y) / grid->info.resolution));

    if (ix < 0 || iy < 0 ||
      ix >= static_cast<int>(grid->info.width) ||
      iy >= static_cast<int>(grid->info.height))
    {
      return false;
    }

    const std::size_t index =
      static_cast<std::size_t>(iy) *
      static_cast<std::size_t>(grid->info.width) +
      static_cast<std::size_t>(ix);

    if (index >= grid->data.size() || grid->data[index] < 0) {
      return false;
    }

    normalized_value = clamp01(
      static_cast<double>(grid->data[index]) / 100.0);
    return true;
  }

  PathQuality evaluatePath(
    const Path & path,
    const std::shared_ptr<const Grid> & terrain,
    const std::shared_ptr<const Grid> & radiation,
    const std::shared_ptr<const Grid> & traversability) const
  {
    PathQuality quality;

    if (path.poses.empty()) {
      return quality;
    }

    quality.valid = true;
    quality.minimum_traversability = 1.0;

    for (std::size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;

      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double segment_length = std::hypot(dx, dy);

      if (segment_length <= 1e-9) {
        continue;
      }

      quality.path_length_m += segment_length;

      const int samples = std::max(
        1,
        static_cast<int>(
          std::ceil(segment_length / std::max(0.01, sample_resolution_m_))));

      for (int sample = 1; sample <= samples; ++sample) {
        const double ratio =
          static_cast<double>(sample) / static_cast<double>(samples);
        const double x = a.x + ratio * dx;
        const double y = a.y + ratio * dy;

        double value = 0.0;

        if (sampleGrid(terrain, x, y, value)) {
          quality.terrain_cost +=
            terrain_weight_ * value *
            segment_length / static_cast<double>(samples);
        }

        if (sampleGrid(radiation, x, y, value)) {
          quality.radiation_cost +=
            radiation_weight_ * value *
            segment_length / static_cast<double>(samples);
          quality.maximum_radiation =
            std::max(quality.maximum_radiation, value);
        }

        if (sampleGrid(traversability, x, y, value)) {
          quality.minimum_traversability =
            std::min(quality.minimum_traversability, value);
        }
      }
    }

    if (nominal_speed_mps_ > 1e-6) {
      quality.estimated_time_sec =
        quality.path_length_m / nominal_speed_mps_;
    }

    quality.time_cost = time_weight_ * quality.estimated_time_sec;
    return quality;
  }

  void pathCallback(const Path::SharedPtr message)
  {
    if (!message) {
      return;
    }

    Path path_copy;
    std::uint64_t plan_id;
    std::uint64_t goal_generation;
    std::uint64_t map_generation;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_path_ = *message;
      has_path_ = true;
      ++plan_id_;
      ++goal_generation_;

      path_copy = latest_path_;
      plan_id = plan_id_;
      goal_generation = goal_generation_;
      map_generation = map_generation_;
    }

    publishMetrics(
      "PATH_RECEIVED",
      "",
      "path",
      "planner path received",
      false,
      path_copy,
      plan_id,
      goal_generation,
      map_generation);
  }

  void goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (!message) {
      return;
    }

    Path goal_path;
    goal_path.header = message->header;
    goal_path.poses.push_back(*message);

    std::uint64_t plan_id;
    std::uint64_t goal_generation;
    std::uint64_t map_generation;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      ++goal_generation_;
      plan_id = plan_id_;
      goal_generation = goal_generation_;
      map_generation = map_generation_;
    }

    publishMetrics(
      "GOAL_RECEIVED",
      "",
      "goal_topic",
      "goal pose received by metrics node",
      false,
      goal_path,
      plan_id,
      goal_generation,
      map_generation);
  }

  void statusCallback(const std_msgs::msg::String::SharedPtr message)
  {
    if (!message) {
      return;
    }

    Path path_copy;
    bool has_path;
    std::uint64_t plan_id;
    std::uint64_t goal_generation;
    std::uint64_t map_generation;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      path_copy = latest_path_;
      has_path = has_path_;
      plan_id = plan_id_;
      goal_generation = goal_generation_;
      map_generation = map_generation_;
    }

    const std::string state = statusState(message->data);
    const bool success =
      state == "SUCCESS" || state == "GOAL_REACHED";

    const std::string failure_code =
      (!success &&
      (state == "FAILED" || state == "INVALID_GOAL" ||
      state == "TIMEOUT" || state == "CANCELLED")) ?
      state : "";

    publishMetrics(
      state,
      failure_code,
      "planner_status",
      message->data,
      success,
      has_path ? path_copy : Path(),
      plan_id,
      goal_generation,
      map_generation);
  }

  void publishMetrics(
    const std::string & state,
    const std::string & failure_code,
    const std::string & trigger,
    const std::string & message,
    bool success,
    const Path & path,
    std::uint64_t plan_id,
    std::uint64_t goal_generation,
    std::uint64_t map_generation)
  {
    std::shared_ptr<const Grid> terrain;
    std::shared_ptr<const Grid> radiation;
    std::shared_ptr<const Grid> traversability;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      terrain = terrain_map_;
      radiation = radiation_map_;
      traversability = traversability_map_;
    }

    const PathQuality quality =
      evaluatePath(path, terrain, radiation, traversability);

    Metrics output;
    output.header.stamp = now();
    output.header.frame_id =
      path.header.frame_id.empty() ? "map" : path.header.frame_id;

    output.state = state;
    output.failure_code = failure_code;
    output.trigger = trigger;
    output.message = message;

    output.plan_id = plan_id;
    output.goal_generation = goal_generation;
    output.map_generation = map_generation;
    output.success = success;
    output.quality_valid = quality.valid;

    if (!path.poses.empty()) {
      output.start_x = path.poses.front().pose.position.x;
      output.start_y = path.poses.front().pose.position.y;
      output.goal_x = path.poses.back().pose.position.x;
      output.goal_y = path.poses.back().pose.position.y;
    }

    output.path_points =
      static_cast<std::uint32_t>(path.poses.size());
    output.path_length_m = quality.path_length_m;
    output.terrain_cost = quality.terrain_cost;
    output.radiation_cost = quality.radiation_cost;
    output.time_cost = quality.time_cost;
    output.total_cost =
      quality.terrain_cost +
      quality.radiation_cost +
      quality.time_cost;
    output.estimated_time_sec = quality.estimated_time_sec;
    output.minimum_traversability =
      quality.minimum_traversability;
    output.maximum_radiation =
      quality.maximum_radiation;

    metrics_pub_->publish(output);
  }

  void radiationCallback(const Grid::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    radiation_map_ = message;
    ++map_generation_;
  }

  void terrainCallback(const Grid::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    terrain_map_ = message;
    ++map_generation_;
  }

  void traversabilityCallback(const Grid::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    traversability_map_ = message;
    ++map_generation_;
  }

  std::string path_topic_;
  std::string status_topic_;
  std::string metrics_topic_;
  std::string radiation_topic_;
  std::string terrain_topic_;
  std::string traversability_topic_;
  std::string goal_topic_;

  double sample_resolution_m_{0.10};
  double nominal_speed_mps_{0.20};
  double terrain_weight_{1.0};
  double radiation_weight_{1.0};
  double time_weight_{0.2};

  std::mutex data_mutex_;
  Path latest_path_;
  bool has_path_{false};
  std::uint64_t plan_id_{0};
  std::uint64_t goal_generation_{0};
  std::uint64_t map_generation_{0};

  std::shared_ptr<const Grid> radiation_map_;
  std::shared_ptr<const Grid> terrain_map_;
  std::shared_ptr<const Grid> traversability_map_;

  rclcpp::Publisher<Metrics>::SharedPtr metrics_pub_;
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<Grid>::SharedPtr radiation_sub_;
  rclcpp::Subscription<Grid>::SharedPtr terrain_sub_;
  rclcpp::Subscription<Grid>::SharedPtr traversability_sub_;
};

}  // namespace risk_aware_planner_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<risk_aware_planner_cpp::PlannerMetricsNode>());
  rclcpp::shutdown();
  return 0;
}
