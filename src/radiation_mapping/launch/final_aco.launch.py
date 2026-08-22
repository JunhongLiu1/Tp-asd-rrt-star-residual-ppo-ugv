import os

from ament_index_python.packages import (
    get_package_share_directory,
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
        'final_cost_model_v1.json',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'cost_model_config',
            default_value=default_cost_config,
        ),

        DeclareLaunchArgument(
            'cost_profile',
            default_value='balanced',
        ),

        DeclareLaunchArgument(
            'include_time_penalty',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'aco_ant_count',
            default_value='50',
        ),

        DeclareLaunchArgument(
            'aco_iterations',
            default_value='70',
        ),

        DeclareLaunchArgument(
            'aco_alpha',
            default_value='1.0',
        ),

        DeclareLaunchArgument(
            'aco_beta',
            default_value='4.0',
        ),

        DeclareLaunchArgument(
            'aco_evaporation_rate',
            default_value='0.25',
        ),

        DeclareLaunchArgument(
            'aco_deposit_scale',
            default_value='1.0',
        ),

        DeclareLaunchArgument(
            'aco_elite_weight',
            default_value='3.0',
        ),

        DeclareLaunchArgument(
            'aco_grid_step_m',
            default_value='0.45',
        ),

        DeclareLaunchArgument(
            'aco_goal_connection_radius_m',
            default_value='0.75',
        ),

        DeclareLaunchArgument(
            'aco_goal_heuristic_weight',
            default_value='0.35',
        ),

        DeclareLaunchArgument(
            'aco_max_steps',
            default_value='240',
        ),

        DeclareLaunchArgument(
            'aco_seed',
            default_value='31',
        ),

        Node(
            package='radiation_mapping',
            executable='aco_planner',
            name='aco_planner',
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

                'terrain_input_max': 100.0,

                'radiation_input_mode':
                    'normalized_occupancy',

                'radiation_input_max': 100.0,

                'use_sim_time':
                    LaunchConfiguration(
                        'use_sim_time'
                    ),

                'aco_ant_count':
                    LaunchConfiguration(
                        'aco_ant_count'
                    ),

                'aco_iterations':
                    LaunchConfiguration(
                        'aco_iterations'
                    ),

                'aco_alpha':
                    LaunchConfiguration(
                        'aco_alpha'
                    ),

                'aco_beta':
                    LaunchConfiguration(
                        'aco_beta'
                    ),

                'aco_evaporation_rate':
                    LaunchConfiguration(
                        'aco_evaporation_rate'
                    ),

                'aco_deposit_scale':
                    LaunchConfiguration(
                        'aco_deposit_scale'
                    ),

                'aco_elite_weight':
                    LaunchConfiguration(
                        'aco_elite_weight'
                    ),

                'aco_grid_step_m':
                    LaunchConfiguration(
                        'aco_grid_step_m'
                    ),

                'aco_goal_connection_radius_m':
                    LaunchConfiguration(
                        'aco_goal_connection_radius_m'
                    ),

                'aco_goal_heuristic_weight':
                    LaunchConfiguration(
                        'aco_goal_heuristic_weight'
                    ),

                'aco_max_steps':
                    LaunchConfiguration(
                        'aco_max_steps'
                    ),

                'aco_seed':
                    LaunchConfiguration(
                        'aco_seed'
                    ),
            }],
            remappings=[
                (
                    '/rviz_asd_time_aware_rrt_star_path',
                    '/aco_path',
                ),
                (
                    '/rviz_asd_time_aware_rrt_star_markers',
                    '/aco_markers',
                ),
                (
                    '/terrain_cost_map',
                    '/terrain_impedance_map',
                ),
                (
                    '/radiation_map',
                    '/radiation_map',
                ),
            ],
        ),
    ])
