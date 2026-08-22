from launch import LaunchDescription
from launch.actions import TimerAction
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

    rviz_asd_planner_node = Node(
        package='radiation_mapping',
        executable='rviz_asd_time_aware_rrt_star_planner',
        name='rviz_asd_time_aware_rrt_star_planner',
        output='screen'
    )

    dose_monitor_node = Node(
        package='radiation_mapping',
        executable='robot_radiation_dose_monitor',
        name='robot_radiation_dose_monitor',
        output='log',
        parameters=[
            {
                'verbose': False,
                'print_interval': 10.0
            }
        ],
        arguments=[
            '--ros-args',
            '--log-level',
            'warn'
        ]
    )

    dynamic_follower_node = Node(
        package='radiation_mapping',
        executable='rviz_dynamic_path_follower',
        name='rviz_dynamic_path_follower',
        output='screen',
        parameters=[
            {
                'planner_name': 'RViz ASD-Time-Aware RRT*',
                'path_topic': '/rviz_asd_time_aware_rrt_star_path',
                'dose_topic': '/robot_accumulated_dose',
                'terrain_topic': '/terrain_cost_map',
                'result_csv': '~/terrain_radiation_ws/experiment_results/rviz_navigation_results.csv'
            }
        ]
    )

    return LaunchDescription([
        radiation_map_node,
        terrain_cost_map_node,
        fusion_cost_map_node,

        TimerAction(
            period=2.0,
            actions=[rviz_asd_planner_node]
        ),

        TimerAction(
            period=3.0,
            actions=[dose_monitor_node]
        ),

        TimerAction(
            period=4.0,
            actions=[dynamic_follower_node]
        ),
    ])
