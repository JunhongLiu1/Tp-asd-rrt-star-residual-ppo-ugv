from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    radiation_map_node = Node(
        package='radiation_mapping',
        executable='radiation_map_node',
        name='radiation_map_node',
        output='screen'
    )

    terrain_cost_map_node = Node(
        package='radiation_mapping',
        executable='terrain_cost_map_node',
        name='terrain_cost_map_node',
        output='screen'
    )

    fusion_cost_map_node = Node(
        package='radiation_mapping',
        executable='fusion_cost_map_node',
        name='fusion_cost_map_node',
        output='screen'
    )

    rrt_star_node = Node(
        package='radiation_mapping',
        executable='rrt_star_baseline_planner',
        name='rrt_star_baseline_planner',
        output='screen'
    )

    asd_rrt_star_node = Node(
        package='radiation_mapping',
        executable='asd_time_aware_rrt_star_planner',
        name='asd_time_aware_rrt_star_planner',
        output='screen'
    )

    asd_rrt_star_teb_node = Node(
        package='radiation_mapping',
        executable='rviz_asd_time_aware_rrt_star_planner',
        name='asd_rrt_star_teb_planner',
        output='screen'
    )

    recorder_node = Node(
        package='radiation_mapping',
        executable='planner_comparison_recorder',
        name='planner_comparison_recorder',
        output='screen',
        parameters=[{
            'planner_names': LaunchConfiguration('planner_names'),
            'path_topics': LaunchConfiguration('path_topics'),
            'output_dir': LaunchConfiguration('output_dir'),
            'record_duration_sec': LaunchConfiguration('record_duration_sec'),
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_names',
            default_value='rrt_star,asd_rrt_star,asd_rrt_star_teb'
        ),
        DeclareLaunchArgument(
            'path_topics',
            default_value='/rrt_star_baseline_path,/asd_time_aware_rrt_star_path,/rviz_asd_time_aware_rrt_star_path'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='~/terrain_radiation_ws/module31_experiment_results'
        ),
        DeclareLaunchArgument(
            'record_duration_sec',
            default_value='60.0'
        ),

        radiation_map_node,
        terrain_cost_map_node,
        fusion_cost_map_node,

        TimerAction(period=1.0, actions=[rrt_star_node]),
        TimerAction(period=3.0, actions=[asd_rrt_star_node]),
        TimerAction(period=4.0, actions=[asd_rrt_star_teb_node]),
        TimerAction(period=5.0, actions=[recorder_node]),
    ])
