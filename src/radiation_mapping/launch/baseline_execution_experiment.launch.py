from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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

    baseline_planner_node = Node(
        package='radiation_mapping',
        executable='rrt_star_baseline_planner',
        name='rrt_star_baseline_planner',
        output='screen'
    )

    dose_monitor_node = Node(
        package='radiation_mapping',
        executable='robot_radiation_dose_monitor',
        name='robot_radiation_dose_monitor',
        output='screen'
    )

    follower_node = Node(
        package='radiation_mapping',
        executable='asd_path_waypoint_follower',
        name='baseline_path_waypoint_follower',
        output='screen',
        parameters=[
            {
                'planner_name': 'Baseline RRT*',
                'path_topic': '/rrt_star_baseline_path',
                'dose_topic': '/robot_accumulated_dose',
                'terrain_topic': '/terrain_cost_map',
                'result_csv': '~/terrain_radiation_ws/experiment_results/execution_results.csv',
                'shutdown_on_finish': True
            }
        ]
    )

    return LaunchDescription([
        radiation_map_node,
        terrain_cost_map_node,
        fusion_cost_map_node,

        TimerAction(
            period=2.0,
            actions=[baseline_planner_node]
        ),

        TimerAction(
            period=3.0,
            actions=[dose_monitor_node]
        ),

        TimerAction(
            period=6.0,
            actions=[follower_node]
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=follower_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason='Experiment follower finished.'
                        )
                    )
                ]
            )
        ),
    ])
