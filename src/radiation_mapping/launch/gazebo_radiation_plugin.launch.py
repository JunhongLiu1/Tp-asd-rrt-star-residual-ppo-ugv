import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_robot_state_publisher_launch():
    candidates = []

    try:
        turtlebot3_gazebo_share = get_package_share_directory(
            'turtlebot3_gazebo'
        )

        candidates.append(
            os.path.join(
                turtlebot3_gazebo_share,
                'launch',
                'robot_state_publisher.launch.py',
            )
        )
    except Exception:
        pass

    try:
        turtlebot3_bringup_share = get_package_share_directory(
            'turtlebot3_bringup'
        )

        candidates.append(
            os.path.join(
                turtlebot3_bringup_share,
                'launch',
                'robot_state_publisher.launch.py',
            )
        )
    except Exception:
        pass

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    raise RuntimeError(
        'Could not find TurtleBot3 '
        'robot_state_publisher.launch.py'
    )


def generate_launch_description():
    radiation_mapping_share = get_package_share_directory(
        'radiation_mapping'
    )

    gazebo_ros_share = get_package_share_directory(
        'gazebo_ros'
    )

    world_path = os.path.join(
        radiation_mapping_share,
        'worlds',
        'module36_hard_radiation_plugin.world',
    )

    if not os.path.isfile(world_path):
        raise RuntimeError(
            f'Radiation world does not exist: {world_path}'
        )

    robot_state_publisher_launch = (
        find_robot_state_publisher_launch()
    )

    plugin_directory = os.path.join(
        os.path.expanduser(
            '~/terrain_radiation_ws'
        ),
        'install',
        'gazebo_radiation_plugins',
        'lib',
    )

    existing_plugin_path = os.environ.get(
        'GAZEBO_PLUGIN_PATH',
        '',
    )

    combined_plugin_path = plugin_directory

    if existing_plugin_path:
        combined_plugin_path += (
            os.pathsep
            + existing_plugin_path
        )

    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    start_x = LaunchConfiguration(
        'start_x'
    )

    start_y = LaunchConfiguration(
        'start_y'
    )

    start_z = LaunchConfiguration(
        'start_z'
    )

    start_yaw = LaunchConfiguration(
        'start_yaw'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_ros_share,
                'launch',
                'gazebo.launch.py',
            )
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
        }.items(),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            robot_state_publisher_launch
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3_burger',
        output='screen',
        arguments=[
            '-entity',
            'turtlebot3_burger',
            '-file',
            '/home/i/terrain_radiation_ws/src/radiation_mapping/models/turtlebot3_burger_world_odom.sdf',
            '-x',
            start_x,
            '-y',
            start_y,
            '-z',
            start_z,
            '-Y',
            start_yaw,
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            'TURTLEBOT3_MODEL',
            'burger',
        ),

        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            combined_plugin_path,
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_x',
            default_value='5.134',
        ),

        DeclareLaunchArgument(
            'start_y',
            default_value='5.977',
        ),

        DeclareLaunchArgument(
            'start_z',
            default_value='0.448',
        ),

        DeclareLaunchArgument(
            'start_yaw',
            default_value='0.0',
        ),

        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
