import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)
from launch_ros.actions import Node


def generate_launch_description():
    radiation_share = get_package_share_directory(
        'radiation_mapping'
    )

    gazebo_ros_share = get_package_share_directory(
        'gazebo_ros'
    )

    turtlebot3_share = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    plugin_prefix = get_package_prefix(
        'gazebo_radiation_plugins'
    )

    plugin_directory = os.path.join(
        plugin_prefix,
        'lib'
    )

    existing_plugin_path = os.environ.get(
        'GAZEBO_PLUGIN_PATH',
        ''
    )

    combined_plugin_path = plugin_directory

    if existing_plugin_path:
        combined_plugin_path += (
            ':' + existing_plugin_path
        )

    world_path = os.path.join(
        radiation_share,
        'worlds',
        'module36_hard_radiation_plugin.world'
    )

    robot_model_path = os.path.join(
        turtlebot3_share,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    gazebo_launch_path = os.path.join(
        gazebo_ros_share,
        'launch',
        'gazebo.launch.py'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            gazebo_launch_path
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
            'pause': 'false',
        }.items()
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_module36_turtlebot3',
                output='screen',
                arguments=[
                    '-entity',
                    'turtlebot3_burger',
                    '-file',
                    robot_model_path,
                    '-x',
                    '5.134',
                    '-y',
                    '5.977',
                    '-z',
                    '0.448',
                    '-Y',
                    '0.0',
                ],
            )
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value='burger'
        ),

        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=combined_plugin_path
        ),

        gazebo,
        spawn_robot,
    ])
