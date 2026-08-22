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
            default_value='60',
        ),

        DeclareLaunchArgument(
            'aco_iterations',
            default_value='80',
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

        DeclareLaunchArgument(
            'odom_topic',
            default_value='/ground_truth/odom',
        ),

        # ACO_VEHICLE_SAFE_V7: conservative Husky footprint map.
        # Predicted chassis clearance <= 1 cm is blocked before planning.
        Node(
            package='radiation_mapping',
            executable='husky_vehicle_aware_terrain_node',
            name='aco_husky_vehicle_aware_terrain',
            output='screen',
            parameters=[{
                'data_directory': os.path.join(
                    package_share,
                    'dem',
                    'processed',
                ),
                'terrain_level': 'hard',
                'terrain_map_topic': '/terrain_impedance_map',
                'clearance_warning_m': 0.060,
                'clearance_block_m': 0.010,
                'vehicle_risk_weight': 0.75,
                'evaluation_yaws_deg': [
                    0.0,
                    22.5,
                    45.0,
                    67.5,
                    90.0,
                    112.5,
                    135.0,
                    157.5,
                ],
                'path_topics': ['/aco_path'],
                'publish_rate_hz': 0.5,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        Node(
            package='radiation_mapping',
            executable='aco_trackable_safe_planner',
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


                'aco_vehicle_impedance_topic':
                    '/husky_vehicle_impedance_map',

                'aco_vehicle_risk_topic':
                    '/husky_vehicle_collision_risk_map',

                'aco_require_vehicle_map': True,
                'aco_vehicle_block_threshold': 99.5,
                'aco_vehicle_risk_hard_threshold': 90.0,
                'aco_vehicle_warning_threshold': 70.0,
                'aco_vehicle_risk_penalty_weight': 0.35,
                'aco_vehicle_path_sample_spacing_m': 0.04,


                # ACO_TRACKABLE_SAFE_V8_PARAMETERS
                'aco_execution_corridor_half_width_m': 0.20,
                'aco_execution_corridor_lateral_samples': 5,
                'aco_corridor_penalty_weight': 0.20,
                'aco_trackability_resample_spacing_m': 0.35,
                'aco_trackability_max_turn_deg': 55.0,
                'aco_max_path_to_direct_ratio': 1.30,
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
                (
                    '/odom',
                    LaunchConfiguration('odom_topic'),
                ),
            ],
        ),
    ])
