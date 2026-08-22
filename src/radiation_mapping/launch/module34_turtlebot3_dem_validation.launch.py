import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    terrain_name = LaunchConfiguration(
        'terrain'
    ).perform(context).strip().lower()

    start_x = LaunchConfiguration(
        'start_x'
    ).perform(context)

    start_y = LaunchConfiguration(
        'start_y'
    ).perform(context)

    start_z = LaunchConfiguration(
        'start_z'
    ).perform(context)

    start_yaw = LaunchConfiguration(
        'start_yaw'
    ).perform(context)

    valid_terrains = {
        'easy': 'module34_easy.world',
        'medium': 'module34_medium.world',
        'hard': 'module34_hard.world',
    }

    if terrain_name not in valid_terrains:
        raise RuntimeError(
            'Invalid terrain argument: '
            f'{terrain_name}. '
            'Use easy, medium, or hard.'
        )

    radiation_mapping_share = get_package_share_directory(
        'radiation_mapping'
    )

    gazebo_ros_share = get_package_share_directory(
        'gazebo_ros'
    )

    turtlebot3_gazebo_share = get_package_share_directory(
        'turtlebot3_gazebo'
    )

    world_path = os.path.join(
        radiation_mapping_share,
        'worlds',
        valid_terrains[terrain_name]
    )

    model_path = os.path.join(
        turtlebot3_gazebo_share,
        'models',
        'turtlebot3_burger',
        'model.sdf'
    )

    gazebo_launch_path = os.path.join(
        gazebo_ros_share,
        'launch',
        'gazebo.launch.py'
    )

    if not os.path.isfile(world_path):
        raise RuntimeError(
            f'Gazebo world does not exist: {world_path}'
        )

    if not os.path.isfile(model_path):
        raise RuntimeError(
            f'TurtleBot3 model does not exist: {model_path}'
        )

    print()
    print('Module 34 configuration')
    print('-----------------------')
    print(f'Terrain: {terrain_name}')
    print(f'World: {world_path}')
    print(f'Robot model: {model_path}')
    print(
        'Spawn pose: '
        f'x={start_x}, '
        f'y={start_y}, '
        f'z={start_z}, '
        f'yaw={start_yaw}'
    )
    print()

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
                name=(
                    'spawn_turtlebot3_'
                    + terrain_name
                ),
                output='screen',
                arguments=[
                    '-entity',
                    'turtlebot3_burger',
                    '-file',
                    model_path,
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
        ],
    )

    return [
        gazebo,
        spawn_robot,
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value='burger'
        ),

        DeclareLaunchArgument(
            'terrain',
            default_value='easy',
            description=(
                'Terrain level: easy, medium, or hard'
            )
        ),

        DeclareLaunchArgument(
            'start_x',
            default_value='0.0',
            description='Robot initial Gazebo x coordinate'
        ),

        DeclareLaunchArgument(
            'start_y',
            default_value='0.0',
            description='Robot initial Gazebo y coordinate'
        ),

        DeclareLaunchArgument(
            'start_z',
            default_value='3.0',
            description=(
                'Robot initial height. It will fall onto '
                'the terrain surface.'
            )
        ),

        DeclareLaunchArgument(
            'start_yaw',
            default_value='0.0',
            description='Robot initial yaw in radians'
        ),

        OpaqueFunction(
            function=launch_setup
        ),
    ])
