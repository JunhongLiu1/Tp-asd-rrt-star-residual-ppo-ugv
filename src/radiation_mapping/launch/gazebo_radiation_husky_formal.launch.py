import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    radiation_mapping_share = get_package_share_directory(
        'radiation_mapping'
    )

    gazebo_ros_share = get_package_share_directory(
        'gazebo_ros'
    )

    husky_description_share = get_package_share_directory(
        'husky_description'
    )

    default_world_path = os.path.join(
        radiation_mapping_share,
        'worlds',
        'module36_hard_radiation_mesh_visual_colored_r3.world',
    )

    if not os.path.isfile(default_world_path):
        raise RuntimeError(
            f'Default radiation world does not exist: {default_world_path}'
        )

    plugin_directory = os.path.join(
        os.path.expanduser('~/terrain_radiation_ws'),
        'install',
        'gazebo_radiation_plugins',
        'lib',
    )

    existing_plugin_path = os.environ.get(
        'GAZEBO_PLUGIN_PATH',
        '',
    )

    plugin_paths = [
        plugin_directory,
    ]

    if existing_plugin_path:
        plugin_paths.append(existing_plugin_path)

    combined_plugin_path = os.pathsep.join(plugin_paths)

    existing_model_path = os.environ.get(
        'GAZEBO_MODEL_PATH',
        '',
    )

    model_paths = [
        '/usr/share/gazebo-11/models',
        str(Path(husky_description_share).parent.resolve()),
    ]

    if existing_model_path:
        model_paths.insert(0, existing_model_path)

    combined_model_path = os.pathsep.join(model_paths)

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world_path = LaunchConfiguration('world_path')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_z = LaunchConfiguration('start_z')
    start_yaw = LaunchConfiguration('start_yaw')

    controller_config = PathJoinSubstitution([
        FindPackageShare('husky_control'),
        'config',
        'control.yaml',
    ])

    robot_description_content = Command([
        PathJoinSubstitution([
            FindExecutable(name='xacro'),
        ]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('husky_description'),
            'urdf',
            'husky.urdf.xacro',
        ]),
        ' ',
        'name:=husky',
        ' ',
        "prefix:=''",
        ' ',
        'is_sim:=true',
        ' ',
        'gazebo_controllers:=',
        controller_config,
    ])

    robot_description = {
        'robot_description': robot_description_content,
    }

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
            'gui': gui,
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
            },
            robot_description,
        ],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        output='screen',
        arguments=[
            '-entity',
            'husky',
            '-topic',
            'robot_description',
            '-timeout',
            '120.0',
            '-spawn_service_timeout',
            '120.0',
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

    # Allow the large terrain visual and Gazebo factory service
    # to finish loading before spawning Husky.
    delayed_spawn_robot = TimerAction(
        period=15.0,
        actions=[
            spawn_robot,
        ],
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=[
            start_x,
            start_y,
            '0.0',
            start_yaw,
            '0.0',
            '0.0',
            'map',
            'odom',
        ],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        name='spawn_joint_state_broadcaster',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '-c',
            '/controller_manager',
        ],
    )

    spawn_velocity_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        name='spawn_husky_velocity_controller',
        output='screen',
        arguments=[
            'husky_velocity_controller',
            '-c',
            '/controller_manager',
        ],
    )

    start_joint_controller_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                spawn_joint_state_broadcaster,
            ],
        )
    )

    start_velocity_controller_after_joint_controller = (
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[
                    spawn_velocity_controller,
                ],
            )
        )
    )

    husky_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('husky_control'),
                'launch',
                'control.launch.py',
            ])
        )
    )

    husky_twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('husky_control'),
                'launch',
                'teleop_base.launch.py',
            ])
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'gui',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'world_path',
            default_value=default_world_path,
            description='Absolute path to the Gazebo world file',
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

        SetEnvironmentVariable(
            'GAZEBO_PLUGIN_PATH',
            combined_plugin_path,
        ),

        SetEnvironmentVariable(
            'GAZEBO_MODEL_PATH',
            combined_model_path,
        ),

        gazebo,
        robot_state_publisher,
        map_to_odom,
        delayed_spawn_robot,
        start_joint_controller_after_spawn,
        start_velocity_controller_after_joint_controller,
        husky_localization,
        husky_twist_mux,
    ])
