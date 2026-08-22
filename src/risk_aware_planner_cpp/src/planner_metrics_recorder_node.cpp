
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>

#include "radiation_interfaces/msg/planner_metrics.hpp"
#include "rclcpp/rclcpp.hpp"

namespace risk_aware_planner_cpp
{

class PlannerMetricsRecorderNode : public rclcpp::Node
{
public:
  PlannerMetricsRecorderNode()
  : Node("tp_asd_rrt_star_metrics_recorder_cpp")
  {
    csv_path_ = declare_parameter<std::string>(
      "csv_path",
      "/tmp/tp_asd_rrt_star_metrics.csv");

    append_ = declare_parameter<bool>("append", false);
    flush_every_n_ = declare_parameter<int>("flush_every_n", 1);

    const auto mode = append_ ?
      std::ios::out | std::ios::app :
      std::ios::out | std::ios::trunc;

    file_.open(csv_path_, mode);

    if (!file_.is_open()) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot open CSV file: %s",
        csv_path_.c_str());
      return;
    }

    if (!append_ || file_.tellp() == std::streampos(0)) {
      writeHeader();
    }

    subscription_ =
      create_subscription<radiation_interfaces::msg::PlannerMetrics>(
        "/tp_asd_rrt_star_cpp_metrics",
        rclcpp::QoS(20).best_effort(),
        std::bind(
          &PlannerMetricsRecorderNode::metricsCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "CSV recorder started: %s",
      csv_path_.c_str());
  }

  ~PlannerMetricsRecorderNode()
  {
    if (file_.is_open()) {
      file_.flush();
      file_.close();
    }
  }

private:
  static std::string escapeCsv(const std::string & value)
  {
    std::string escaped = "\"";

    for (const char character : value) {
      if (character == '"') {
        escaped += "\"\"";
      } else {
        escaped += character;
      }
    }

    escaped += "\"";
    return escaped;
  }

  void writeHeader()
  {
    file_
      << "stamp_sec,stamp_nanosec,"
      << "state,failure_code,trigger,message,"
      << "plan_id,goal_generation,map_generation,"
      << "success,quality_valid,"
      << "start_x,start_y,goal_x,goal_y,"
      << "path_points,path_length_m,total_cost,"
      << "terrain_cost,radiation_cost,time_cost,"
      << "estimated_time_sec,minimum_traversability,"
      << "maximum_radiation,planning_time_sec,iterations"
      << "\n";

    file_.flush();
  }

  void metricsCallback(
    const radiation_interfaces::msg::PlannerMetrics::SharedPtr message)
  {
    if (!message || !file_.is_open()) {
      return;
    }

    file_
      << message->header.stamp.sec << ","
      << message->header.stamp.nanosec << ","
      << escapeCsv(message->state) << ","
      << escapeCsv(message->failure_code) << ","
      << escapeCsv(message->trigger) << ","
      << escapeCsv(message->message) << ","
      << message->plan_id << ","
      << message->goal_generation << ","
      << message->map_generation << ","
      << (message->success ? 1 : 0) << ","
      << (message->quality_valid ? 1 : 0) << ","
      << std::setprecision(10)
      << message->start_x << ","
      << message->start_y << ","
      << message->goal_x << ","
      << message->goal_y << ","
      << message->path_points << ","
      << message->path_length_m << ","
      << message->total_cost << ","
      << message->terrain_cost << ","
      << message->radiation_cost << ","
      << message->time_cost << ","
      << message->estimated_time_sec << ","
      << message->minimum_traversability << ","
      << message->maximum_radiation << ","
      << message->planning_time_sec << ","
      << message->iterations
      << "\n";

    ++rows_written_;

    if (rows_written_ % std::max(1, flush_every_n_) == 0) {
      file_.flush();
    }
  }

  std::string csv_path_;
  bool append_{false};
  int flush_every_n_{1};
  std::size_t rows_written_{0};

  std::ofstream file_;
  rclcpp::Subscription<
    radiation_interfaces::msg::PlannerMetrics>::SharedPtr subscription_;
};

}  // namespace risk_aware_planner_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<
      risk_aware_planner_cpp::PlannerMetricsRecorderNode>());

  rclcpp::shutdown();
  return 0;
}
