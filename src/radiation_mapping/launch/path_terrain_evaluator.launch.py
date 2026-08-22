from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'terrain',
            default_value='easy'
        ),

        DeclareLaunchArgument(
            'data_directory',
            default_value=''
        ),

        DeclareLaunchArgument(
            'path_topic',
            default_value='/planned_path'
        ),

        DeclareLaunchArgument(
            'planner_name',
            default_value='planner'
        ),

        DeclareLaunchArgument(
            'radiation_topic',
            default_value='/radiation_map'
        ),

        DeclareLaunchArgument(
            'cost_model_config',
            default_value=''
        ),

        DeclareLaunchArgument(
            'cost_profile',
            default_value='balanced'
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
            'metrics_topic',
            default_value='/terrain_path_metrics'
        ),

        DeclareLaunchArgument(
            'sample_step_m',
            default_value='0.10'
        ),

        DeclareLaunchArgument(
            'csv_path',
            default_value=(
                '~/terrain_radiation_ws/results/'
                'path_metrics/terrain_radiation_path_metrics.csv'
            )
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        Node(
            package='radiation_mapping',
            executable='terrain_path_evaluator',
            name='terrain_path_evaluator',
            output='screen',
            parameters=[{
                'terrain_level': LaunchConfiguration(
                    'terrain'
                ),
                'data_directory': LaunchConfiguration(
                    'data_directory'
                ),
                'path_topic': LaunchConfiguration(
                    'path_topic'
                ),
                'planner_name': LaunchConfiguration(
                    'planner_name'
                ),
                'radiation_topic': LaunchConfiguration(
                    'radiation_topic'
                ),
                'cost_model_config': LaunchConfiguration(
                    'cost_model_config'
                ),
                'cost_profile': LaunchConfiguration(
                    'cost_profile'
                ),
                'radiation_input_mode': LaunchConfiguration(
                    'radiation_input_mode'
                ),
                'radiation_input_max': LaunchConfiguration(
                    'radiation_input_max'
                ),
                'metrics_topic': LaunchConfiguration(
                    'metrics_topic'
                ),
                'sample_step_m': LaunchConfiguration(
                    'sample_step_m'
                ),
                'csv_path': LaunchConfiguration(
                    'csv_path'
                ),
                'use_sim_time': LaunchConfiguration(
                    'use_sim_time'
                ),
            }],
        ),
    ])
