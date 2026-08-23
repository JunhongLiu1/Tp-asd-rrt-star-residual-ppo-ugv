"""Start Husky on the reconstructed DEM without spawning below terrain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


WORLD_FILE = (
    "/home/i/terrain_radiation_ws/src/radiation_mapping/worlds/"
    "module36_hard_radiation_mesh_visual_colored_r3.world"
)


def generate_launch_description():
    gazebo_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
               "/usr/share/gazebo-11/models/:",
               str(Path(get_package_share_directory("husky_description")).parent.resolve())],
    )
    controller_config = PathJoinSubstitution([
        FindPackageShare("husky_control"), "config", "control.yaml"
    ])
    description = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
            PathJoinSubstitution([FindPackageShare("husky_description"),
                                  "urdf", "husky.urdf.xacro"]),
            " name:=husky prefix:='' is_sim:=true gazebo_controllers:=",
            controller_config,
        ])
    }
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="screen", parameters=[{"use_sim_time": True}, description])
    joint_spawner = Node(
        package="controller_manager", executable="spawner.py",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen")
    velocity_spawner = Node(
        package="controller_manager", executable="spawner.py",
        arguments=["husky_velocity_controller", "-c", "/controller_manager"],
        output="screen")
    controller_order = RegisterEventHandler(OnProcessExit(
        target_action=joint_spawner, on_exit=[velocity_spawner]))
    gzserver = ExecuteProcess(
        cmd=["gzserver", "-s", "libgazebo_ros_init.so", "-s",
             "libgazebo_ros_factory.so", WORLD_FILE], output="screen")
    gzclient = ExecuteProcess(cmd=["gzclient"], output="screen")
    # This point is inside the region previously validated by the Gazebo
    # collision probe. Spawn above it so contact resolves downward.
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", name="spawn_husky",
        arguments=["-entity", "husky", "-topic", "robot_description",
                   "-x", "-1.13", "-y", "-7.80", "-z", "1.50"],
        output="screen")
    localization = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("husky_control"),
                              "launch", "control.launch.py"])))
    teleop = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("husky_control"),
                              "launch", "teleop_base.launch.py"])))
    return LaunchDescription([
        gazebo_path, robot_state_publisher, joint_spawner, controller_order,
        gzserver, gzclient, spawn, localization, teleop,
    ])
