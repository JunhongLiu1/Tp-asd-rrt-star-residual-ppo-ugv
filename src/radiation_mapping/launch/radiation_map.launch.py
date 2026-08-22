from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='radiation_mapping',
            executable='radiation_map_node',
            name='radiation_map_node',
            output='screen'
        )
    ])
