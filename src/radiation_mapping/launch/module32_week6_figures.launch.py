from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot3_gazebo'),
            '/launch/empty_world.launch.py'
        ])
    )

    radiation_map_node = Node(
        package='radiation_mapping',
        executable='radiation_map_node',
        name='radiation_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    terrain_cost_map_node = Node(
        package='radiation_mapping',
        executable='terrain_cost_map_node',
        name='terrain_cost_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    fusion_cost_map_node = Node(
        package='radiation_mapping',
        executable='fusion_cost_map_node',
        name='fusion_cost_map_node',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    planner_node = Node(
        package='radiation_mapping',
        executable=LaunchConfiguration('planner_executable'),
        name='module32_figure_planner',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    dose_monitor_node = Node(
        package='radiation_mapping',
        executable='robot_radiation_dose_monitor',
        name='robot_radiation_dose_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'verbose': False,
            'print_interval': 10.0
        }]
    )

    follower_node = Node(
        package='radiation_mapping',
        executable='rviz_dynamic_path_follower',
        name='module32_figure_path_follower',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'planner_name': LaunchConfiguration('planner_name'),
            'path_topic': LaunchConfiguration('path_topic'),
            'dose_topic': '/robot_accumulated_dose',
            'terrain_topic': '/terrain_cost_map',
            'result_csv': LaunchConfiguration('result_csv')
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value='burger'
        ),

        DeclareLaunchArgument(
            'planner_name',
            default_value='asd_rrt_star_teb'
        ),

        DeclareLaunchArgument(
            'planner_executable',
            default_value='rviz_asd_time_aware_rrt_star_planner'
        ),

        DeclareLaunchArgument(
            'path_topic',
            default_value='/rviz_asd_time_aware_rrt_star_path'
        ),

        DeclareLaunchArgument(
            'result_csv',
            default_value='~/terrain_radiation_ws/module32_execution_results/module32_week6_figure_results.csv'
        ),

        turtlebot3_gazebo_launch,

        TimerAction(
            period=5.0,
            actions=[
                radiation_map_node,
                terrain_cost_map_node,
                fusion_cost_map_node
            ]
        ),

        TimerAction(
            period=8.0,
            actions=[planner_node]
        ),

        TimerAction(
            period=10.0,
            actions=[dose_monitor_node]
        ),

        TimerAction(
            period=12.0,
            actions=[follower_node]
        ),

        TimerAction(
            period=14.0,
            actions=[rviz_node]
        ),
    ])
