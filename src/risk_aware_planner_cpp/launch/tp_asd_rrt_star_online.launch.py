#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("risk_aware_planner_cpp")
    default_config = os.path.join(
        package_share, "config", "tp_asd_rrt_star_online.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_motion = LaunchConfiguration("enable_motion")
    enable_adaptive_sampling = LaunchConfiguration(
        "enable_adaptive_sampling")
    config_file = LaunchConfiguration("config_file")

    use_sim_time_value = ParameterValue(
        use_sim_time, value_type=bool)
    enable_motion_value = ParameterValue(
        enable_motion, value_type=bool)

    planner = Node(
        package="risk_aware_planner_cpp",
        executable="planner_node",
        name="tp_asd_rrt_star_planner_cpp",
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": use_sim_time_value,
                "enable_adaptive_sampling": ParameterValue(
                    enable_adaptive_sampling, value_type=bool),
            },
        ],
    )

    follower = Node(
        package="risk_aware_planner_cpp",
        executable="path_follower_node",
        name="tp_asd_rrt_star_path_follower_cpp",
        output="screen",
        parameters=[
            config_file,
            {
                "use_sim_time": use_sim_time_value,
                "enable_motion": enable_motion_value,
            },
        ],
    )

    safety = Node(
        package="risk_aware_planner_cpp",
        executable="cmd_vel_safety_node",
        name="tp_asd_rrt_star_cmd_vel_safety_cpp",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_value,
            "enable_motion": enable_motion_value,
            "allow_reverse": False,
            "input_topic": "/control/base_cmd",
            "output_topic": "/cmd_vel",
            "estop_topic": "/e_stop",
            "max_linear_speed": 0.20,
            "max_angular_speed": 0.60,
            "max_linear_accel": 0.30,
            "max_angular_accel": 1.50,
            "command_timeout_sec": 0.50,
            "publish_rate_hz": 20.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo /clock.",
        ),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            description="Enable non-zero motion commands.",
        ),
        DeclareLaunchArgument(
            "enable_adaptive_sampling",
            default_value="true",
            description="Enable TP-ASD risk sampling and APF guidance.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Planner and follower parameter file.",
        ),
        planner,
        follower,
        safety,
    ])
