#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64.hpp>

namespace gazebo
{

class RadiationWorldPlugin : public WorldPlugin
{
public:
  RadiationWorldPlugin() = default;

  ~RadiationWorldPlugin() override
  {
    this->update_connection_.reset();
  }

  void Load(
    physics::WorldPtr world,
    sdf::ElementPtr sdf) override
  {
    if (!world)
    {
      gzerr
        << "[RadiationWorldPlugin] Invalid Gazebo world.\n";
      return;
    }

    this->world_ = world;
    this->ros_node_ = gazebo_ros::Node::Get(sdf);

    this->target_model_name_ = this->read_value<std::string>(
      sdf,
      "target_model",
      "turtlebot3_burger");

    this->background_dose_rate_ = this->read_value<double>(
      sdf,
      "background_dose_rate_usv_h",
      0.05);

    this->publish_rate_hz_ = this->read_value<double>(
      sdf,
      "publish_rate_hz",
      10.0);

    this->use_3d_distance_ = this->read_value<bool>(
      sdf,
      "use_3d_distance",
      true);

    this->dose_rate_topic_ = this->read_value<std::string>(
      sdf,
      "dose_rate_topic",
      "/radiation/dose_rate_usv_h");

    this->accumulated_dose_topic_ =
      this->read_value<std::string>(
        sdf,
        "accumulated_dose_topic",
        "/radiation/accumulated_dose_usv");

    this->nearest_distance_topic_ =
      this->read_value<std::string>(
        sdf,
        "nearest_distance_topic",
        "/radiation/nearest_source_distance_m");

    this->publish_radiation_map_ =
      this->read_value<bool>(
        sdf,
        "publish_radiation_map",
        false);

    this->radiation_map_topic_ =
      this->read_value<std::string>(
        sdf,
        "radiation_map_topic",
        "/radiation_map");

    this->terrain_map_topic_ =
      this->read_value<std::string>(
        sdf,
        "terrain_map_topic",
        "/terrain_impedance_map");

    this->radiation_map_saturation_usv_h_ =
      this->read_value<double>(
        sdf,
        "radiation_map_saturation_usv_h",
        8.0);

    this->radiation_map_use_3d_distance_ =
      this->read_value<bool>(
        sdf,
        "radiation_map_use_3d_distance",
        false);

    this->radiation_map_z_m_ =
      this->read_value<double>(
        sdf,
        "radiation_map_z_m",
        0.0);

    if (this->radiation_map_saturation_usv_h_ <= 0.0)
    {
      RCLCPP_WARN(
        this->ros_node_->get_logger(),
        "radiation_map_saturation_usv_h must be positive. Using 8.0.");

      this->radiation_map_saturation_usv_h_ = 8.0;
    }

    if (this->publish_rate_hz_ <= 0.0)
    {
      RCLCPP_WARN(
        this->ros_node_->get_logger(),
        "publish_rate_hz must be positive. Using 10 Hz.");

      this->publish_rate_hz_ = 10.0;
    }

    if (this->background_dose_rate_ < 0.0)
    {
      RCLCPP_WARN(
        this->ros_node_->get_logger(),
        "Negative background dose rate. Using zero.");

      this->background_dose_rate_ = 0.0;
    }

    this->parse_sources(sdf);

    if (this->sources_.empty())
    {
      RCLCPP_ERROR(
        this->ros_node_->get_logger(),
        "No <source> elements were configured.");

      return;
    }

    this->dose_rate_publisher_ =
      this->ros_node_->create_publisher<
        std_msgs::msg::Float64>(
        this->dose_rate_topic_,
        10);

    this->accumulated_dose_publisher_ =
      this->ros_node_->create_publisher<
        std_msgs::msg::Float64>(
        this->accumulated_dose_topic_,
        10);

    this->nearest_distance_publisher_ =
      this->ros_node_->create_publisher<
        std_msgs::msg::Float64>(
        this->nearest_distance_topic_,
        10);

    if (this->publish_radiation_map_)
    {
      rclcpp::QoS radiation_map_qos(
        rclcpp::KeepLast(1));

      radiation_map_qos.reliable();
      radiation_map_qos.transient_local();

      this->radiation_map_publisher_ =
        this->ros_node_->create_publisher<
          nav_msgs::msg::OccupancyGrid>(
          this->radiation_map_topic_,
          radiation_map_qos);

      rclcpp::QoS terrain_map_qos(
        rclcpp::KeepLast(10));

      terrain_map_qos.reliable();
      terrain_map_qos.durability_volatile();

      this->terrain_map_subscription_ =
        this->ros_node_->create_subscription<
          nav_msgs::msg::OccupancyGrid>(
          this->terrain_map_topic_,
          terrain_map_qos,
          std::bind(
            &RadiationWorldPlugin::on_terrain_map,
            this,
            std::placeholders::_1));
    }

    this->last_sim_time_ = this->world_->SimTime();
    this->last_publish_time_ = this->last_sim_time_;

    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
        std::bind(
          &RadiationWorldPlugin::on_update,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->ros_node_->get_logger(),
      "Gazebo radiation world plugin loaded.");

    RCLCPP_INFO(
      this->ros_node_->get_logger(),
      "Target model: %s",
      this->target_model_name_.c_str());

    RCLCPP_INFO(
      this->ros_node_->get_logger(),
      "Configured radiation sources: %zu",
      this->sources_.size());

    RCLCPP_INFO(
      this->ros_node_->get_logger(),
      "Distance mode: %s",
      this->use_3d_distance_ ? "3D" : "horizontal");
  }

private:
  struct Source
  {
    std::string model_name;
    double strength_usv_h_m2{0.0};
    double softening_radius_m{0.5};
    double attenuation_per_m{0.0};
    bool warned_missing{false};
  };

  template<typename T>
  T read_value(
    const sdf::ElementPtr & sdf,
    const std::string & name,
    const T & default_value)
  {
    if (sdf && sdf->HasElement(name))
    {
      return sdf->Get<T>(name);
    }

    return default_value;
  }

  void parse_sources(const sdf::ElementPtr & sdf)
  {
    if (!sdf || !sdf->HasElement("source"))
    {
      return;
    }

    sdf::ElementPtr source_element =
      sdf->GetElement("source");

    while (source_element)
    {
      Source source;

      source.model_name =
        this->read_value<std::string>(
          source_element,
          "model_name",
          "");

      source.strength_usv_h_m2 =
        this->read_value<double>(
          source_element,
          "strength_usv_h_m2",
          0.0);

      source.softening_radius_m =
        this->read_value<double>(
          source_element,
          "softening_radius_m",
          0.5);

      source.attenuation_per_m =
        this->read_value<double>(
          source_element,
          "attenuation_per_m",
          0.0);

      if (source.model_name.empty())
      {
        RCLCPP_WARN(
          this->ros_node_->get_logger(),
          "Ignoring source with empty model_name.");
      }
      else if (source.strength_usv_h_m2 < 0.0)
      {
        RCLCPP_WARN(
          this->ros_node_->get_logger(),
          "Ignoring source %s with negative strength.",
          source.model_name.c_str());
      }
      else if (source.softening_radius_m <= 0.0)
      {
        RCLCPP_WARN(
          this->ros_node_->get_logger(),
          "Ignoring source %s with invalid softening radius.",
          source.model_name.c_str());
      }
      else
      {
        source.attenuation_per_m =
          std::max(
            0.0,
            source.attenuation_per_m);

        this->sources_.push_back(source);
      }

      source_element =
        source_element->GetNextElement("source");
    }
  }

  void on_terrain_map(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (!msg)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(
      this->terrain_map_mutex_);

    this->terrain_map_ = *msg;
    this->terrain_map_received_ = true;
    this->radiation_map_dirty_ = true;
  }

  void publish_radiation_map()
  {
    if (!this->radiation_map_publisher_)
    {
      return;
    }

    // IMPORTANT:
    // Hold mutex only while copying the latest terrain map.
    // Do NOT perform radiation calculations while holding this mutex.
    nav_msgs::msg::OccupancyGrid terrain_map;

    {
      std::lock_guard<std::mutex> lock(
        this->terrain_map_mutex_);

      if (!this->terrain_map_received_ ||
          !this->radiation_map_dirty_)
      {
        return;
      }

      terrain_map = this->terrain_map_;

      // Clear now. If a newer terrain map arrives while calculating,
      // its callback will set this back to true.
      this->radiation_map_dirty_ = false;
    }

    const std::size_t width =
      terrain_map.info.width;

    const std::size_t height =
      terrain_map.info.height;

    const double resolution =
      terrain_map.info.resolution;

    if (width == 0 ||
        height == 0 ||
        resolution <= 0.0 ||
        terrain_map.data.size() != width * height)
    {
      RCLCPP_WARN(
        this->ros_node_->get_logger(),
        "Invalid terrain map received.");
      return;
    }

    // Get source positions ONCE.
    // Do not call ModelByName() for every grid cell.
    std::vector<const Source *> active_sources;
    std::vector<ignition::math::Vector3d> source_positions;

    for (const Source & source : this->sources_)
    {
      physics::ModelPtr source_model =
        this->world_->ModelByName(
          source.model_name);

      if (!source_model)
      {
        continue;
      }

      active_sources.push_back(&source);
      source_positions.push_back(
        source_model->WorldPose().Pos());
    }

    if (active_sources.empty())
    {
      return;
    }

    nav_msgs::msg::OccupancyGrid radiation_map;

    radiation_map.header =
      terrain_map.header;

    radiation_map.info =
      terrain_map.info;

    radiation_map.data.resize(
      terrain_map.data.size(),
      -1);

    const double origin_x =
      terrain_map.info.origin.position.x;

    const double origin_y =
      terrain_map.info.origin.position.y;

    for (std::size_t row = 0;
         row < height;
         ++row)
    {
      const double y =
        origin_y +
        (static_cast<double>(row) + 0.5)
        * resolution;

      for (std::size_t col = 0;
           col < width;
           ++col)
      {
        const std::size_t index =
          row * width + col;

        // Keep invalid terrain cells invalid.
        if (terrain_map.data[index] < 0)
        {
          radiation_map.data[index] = -1;
          continue;
        }

        const double x =
          origin_x +
          (static_cast<double>(col) + 0.5)
          * resolution;

        double total_dose_rate =
          this->background_dose_rate_;

        for (std::size_t i = 0;
             i < active_sources.size();
             ++i)
        {
          const Source & source =
            *active_sources[i];

          const ignition::math::Vector3d & source_position =
            source_positions[i];

          const double dx =
            x - source_position.X();

          const double dy =
            y - source_position.Y();

          double distance_squared =
            dx * dx + dy * dy;

          if (this->radiation_map_use_3d_distance_)
          {
            const double dz =
              this->radiation_map_z_m_
              - source_position.Z();

            distance_squared +=
              dz * dz;
          }

          const double distance =
            std::sqrt(distance_squared);

          const double denominator =
            distance_squared
            + source.softening_radius_m
            * source.softening_radius_m;

          const double attenuation =
            std::exp(
              -source.attenuation_per_m
              * distance);

          total_dose_rate +=
            (
              source.strength_usv_h_m2
              / denominator
            ) * attenuation;
        }

        // OccupancyGrid representation:
        // 0   = zero radiation
        // 100 = saturation value or above
        const double normalized =
          total_dose_rate
          / this->radiation_map_saturation_usv_h_;

        const int map_value =
          static_cast<int>(
            std::lround(
              100.0 *
              std::max(
                0.0,
                std::min(1.0, normalized))));

        radiation_map.data[index] =
          static_cast<int8_t>(map_value);
      }
    }

    this->radiation_map_publisher_->publish(
      radiation_map);

    RCLCPP_INFO(
      this->ros_node_->get_logger(),
      "Radiation map published: %u x %u",
      terrain_map.info.width,
      terrain_map.info.height);
  }

  void on_update(const common::UpdateInfo & info)
  {
    if (!this->world_ || !this->ros_node_)
    {
      return;
    }

    const common::Time current_time = info.simTime;

    this->publish_radiation_map();

    double delta_time =
      (current_time - this->last_sim_time_).Double();

    if (delta_time < 0.0)
    {
      this->accumulated_dose_usv_ = 0.0;
      delta_time = 0.0;
    }

    this->last_sim_time_ = current_time;

    physics::ModelPtr robot =
      this->world_->ModelByName(
        this->target_model_name_);

    if (!robot)
    {
      if (!this->warned_robot_missing_)
      {
        RCLCPP_WARN(
          this->ros_node_->get_logger(),
          "Waiting for target model: %s",
          this->target_model_name_.c_str());

        this->warned_robot_missing_ = true;
      }

      return;
    }

    this->warned_robot_missing_ = false;

    const ignition::math::Vector3d robot_position =
      robot->WorldPose().Pos();

    double total_dose_rate =
      this->background_dose_rate_;

    double nearest_distance =
      std::numeric_limits<double>::infinity();

    bool found_source_model = false;

    for (Source & source : this->sources_)
    {
      physics::ModelPtr source_model =
        this->world_->ModelByName(
          source.model_name);

      if (!source_model)
      {
        if (!source.warned_missing)
        {
          RCLCPP_WARN(
            this->ros_node_->get_logger(),
            "Waiting for source model: %s",
            source.model_name.c_str());

          source.warned_missing = true;
        }

        continue;
      }

      source.warned_missing = false;
      found_source_model = true;

      const ignition::math::Vector3d source_position =
        source_model->WorldPose().Pos();

      const double dx =
        robot_position.X() - source_position.X();

      const double dy =
        robot_position.Y() - source_position.Y();

      const double dz =
        robot_position.Z() - source_position.Z();

      double distance = 0.0;

      if (this->use_3d_distance_)
      {
        distance = std::sqrt(
          dx * dx
          + dy * dy
          + dz * dz);
      }
      else
      {
        distance = std::sqrt(
          dx * dx
          + dy * dy);
      }

      nearest_distance =
        std::min(
          nearest_distance,
          distance);

      const double denominator =
        distance * distance
        + source.softening_radius_m
        * source.softening_radius_m;

      const double attenuation =
        std::exp(
          -source.attenuation_per_m
          * distance);

      const double contribution =
        (
          source.strength_usv_h_m2
          / denominator
        ) * attenuation;

      total_dose_rate += contribution;
    }

    if (!found_source_model)
    {
      return;
    }

    // dose_rate is in micro-sieverts per hour.
    // Gazebo delta_time is in seconds.
    this->accumulated_dose_usv_ +=
      total_dose_rate
      * delta_time
      / 3600.0;

    const double publish_period =
      1.0 / this->publish_rate_hz_;

    if (
      (
        current_time
        - this->last_publish_time_
      ).Double() < publish_period)
    {
      return;
    }

    this->last_publish_time_ = current_time;

    std_msgs::msg::Float64 dose_message;
    dose_message.data = total_dose_rate;

    std_msgs::msg::Float64 accumulated_message;
    accumulated_message.data =
      this->accumulated_dose_usv_;

    std_msgs::msg::Float64 distance_message;
    distance_message.data =
      std::isfinite(nearest_distance)
      ? nearest_distance
      : -1.0;

    this->dose_rate_publisher_->publish(
      dose_message);

    this->accumulated_dose_publisher_->publish(
      accumulated_message);

    this->nearest_distance_publisher_->publish(
      distance_message);
  }

  std::mutex terrain_map_mutex_;
  nav_msgs::msg::OccupancyGrid terrain_map_;
  bool terrain_map_received_{false};
  bool radiation_map_dirty_{false};

  double radiation_map_saturation_usv_h_{8.0};
  double radiation_map_z_m_{0.0};
  bool radiation_map_use_3d_distance_{false};

  physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_node_;

  event::ConnectionPtr update_connection_;

  std::vector<Source> sources_;

  std::string target_model_name_;
  std::string dose_rate_topic_;
  std::string accumulated_dose_topic_;
  std::string nearest_distance_topic_;
  std::string radiation_map_topic_;
  std::string terrain_map_topic_;

  double background_dose_rate_{0.05};
  double publish_rate_hz_{10.0};
  double accumulated_dose_usv_{0.0};

  bool use_3d_distance_{true};
  bool publish_radiation_map_{false};
  bool warned_robot_missing_{false};

  common::Time last_sim_time_;
  common::Time last_publish_time_;

  rclcpp::Publisher<
    std_msgs::msg::Float64
  >::SharedPtr dose_rate_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::Float64
  >::SharedPtr accumulated_dose_publisher_;

  rclcpp::Publisher<
    std_msgs::msg::Float64
  >::SharedPtr nearest_distance_publisher_;

  rclcpp::Publisher<
    nav_msgs::msg::OccupancyGrid
  >::SharedPtr radiation_map_publisher_;

  rclcpp::Subscription<
    nav_msgs::msg::OccupancyGrid
  >::SharedPtr terrain_map_subscription_;
};

GZ_REGISTER_WORLD_PLUGIN(RadiationWorldPlugin)

}  // namespace gazebo
