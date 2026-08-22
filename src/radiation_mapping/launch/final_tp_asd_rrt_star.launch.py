from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    common_launch = PathJoinSubstitution([
        FindPackageShare('radiation_mapping'),
        'launch',
        'module_final_asd_rrt_star.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'cost_model_config',
            default_value=''
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                common_launch
            ),
            launch_arguments={
                'node_name':
                    'tp_asd_rrt_star_planner',

                'output_path_topic':
                    '/tp_asd_rrt_star_path',

                'terrain_topic':
                    '/terrain_impedance_map',

                'radiation_topic':
                    '/radiation_map',

                'cost_model_config':
                    LaunchConfiguration(
                        'cost_model_config'
                    ),

                'cost_profile':
                    'balanced',

                'include_time_penalty':
                    'true',

                'terrain_input_max':
                    '100.0',

                'radiation_input_mode':
                    'normalized_occupancy',

                'radiation_input_max':
                    '100.0',

                'use_sim_time':
                    'true',
            }.items(),
        )
    ])
