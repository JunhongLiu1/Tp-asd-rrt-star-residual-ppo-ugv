from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


WORLD_FILE = (
    "/home/i/terrain_radiation_ws/src/radiation_mapping/worlds/"
    "module36_hard_radiation_mesh_visual_colored_r3.world"
)


def generate_launch_description():
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare("husky_gazebo"),
        "launch",
        "gazebo.launch.py",
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "world_path": WORLD_FILE,
            }.items(),
        )
    ])
