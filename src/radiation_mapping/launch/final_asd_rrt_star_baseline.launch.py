from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    common_launch = PathJoinSubstitution([
        FindPackageShare('radiation_mapping'),
        'launch',
        'module_final_asd_rrt_star.launch.py',
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                common_launch
            ),

            launch_arguments={
                'node_name':
                    'asd_rrt_star_planner',

                'output_path_topic':
                    '/asd_rrt_star_path',

                'terrain_topic':
                    '/terrain_impedance_map',

                'radiation_topic':
                    '/radiation_map',

                'cost_profile':
                    'balanced',

                'include_time_penalty':
                    'false',

                'terrain_input_max':
                    '100.0',

                'radiation_input_mode':
                    'normalized_occupancy',

                'radiation_input_max':
                    '100.0',

                'odom_topic':
                    '/ground_truth/odom',

                'odom_to_map_x':
                    '0.0',

                'odom_to_map_y':
                    '0.0',

                'odom_to_map_yaw':
                    '0.0',

                'use_sim_time':
                    'true',
            }.items(),
        )
    ])
