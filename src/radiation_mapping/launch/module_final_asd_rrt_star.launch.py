import os

from ament_index_python.packages import (
    get_package_share_directory
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'radiation_mapping'
    )

    default_cost_config = os.path.join(
        package_share,
        'config',
        'final_cost_model_v1.json'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'node_name',
            default_value='final_asd_rrt_star_planner'
        ),

        DeclareLaunchArgument(
            'output_path_topic',
            default_value='/asd_rrt_star_path'
        ),

        DeclareLaunchArgument(
            'cost_model_config',
            default_value=default_cost_config
        ),

        DeclareLaunchArgument(
            'cost_profile',
            default_value='balanced'
        ),

        DeclareLaunchArgument(
            'include_time_penalty',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'random_seed',
            default_value='31'
        ),

        DeclareLaunchArgument(
            'terrain_input_max',
            default_value='100.0'
        ),

        DeclareLaunchArgument(
            'radiation_input_mode',
            default_value='normalized_occupancy'
        ),

        DeclareLaunchArgument(
            'radiation_input_max',
            default_value='100.0'
        ),

        DeclareLaunchArgument(
            'terrain_topic',
            default_value='/terrain_cost_map'
        ),

        DeclareLaunchArgument(
            'radiation_topic',
            default_value='/radiation_map'
        ),

        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom'
        ),

        DeclareLaunchArgument(
            'odom_to_map_x',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'odom_to_map_y',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'odom_to_map_yaw',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        Node(
            package='radiation_mapping',
            executable='final_asd_rrt_star_planner',
            name=LaunchConfiguration('node_name'),
            output='screen',

            parameters=[{
                'cost_model_config':
                    LaunchConfiguration(
                        'cost_model_config'
                    ),

                'cost_profile':
                    LaunchConfiguration(
                        'cost_profile'
                    ),

                'include_time_penalty':
                    LaunchConfiguration(
                        'include_time_penalty'
                    ),

                'random_seed':
                    LaunchConfiguration(
                        'random_seed'
                    ),

                'terrain_input_max':
                    LaunchConfiguration(
                        'terrain_input_max'
                    ),

                'radiation_input_mode':
                    LaunchConfiguration(
                        'radiation_input_mode'
                    ),

                'radiation_input_max':
                    LaunchConfiguration(
                        'radiation_input_max'
                    ),

                'odom_to_map_x':
                    LaunchConfiguration(
                        'odom_to_map_x'
                    ),

                'odom_to_map_y':
                    LaunchConfiguration(
                        'odom_to_map_y'
                    ),

                'odom_to_map_yaw':
                    LaunchConfiguration(
                        'odom_to_map_yaw'
                    ),

                'use_sim_time':
                    LaunchConfiguration(
                        'use_sim_time'
                    ),
            }],

            remappings=[
                (
                    '/rviz_asd_time_aware_rrt_star_path',
                    LaunchConfiguration(
                        'output_path_topic'
                    )
                ),

                (
                    '/terrain_cost_map',
                    LaunchConfiguration(
                        'terrain_topic'
                    )
                ),

                (
                    '/radiation_map',
                    LaunchConfiguration(
                        'radiation_topic'
                    )
                ),

                (
                    '/odom',
                    LaunchConfiguration(
                        'odom_topic'
                    )
                ),
            ],
        ),
    ])
