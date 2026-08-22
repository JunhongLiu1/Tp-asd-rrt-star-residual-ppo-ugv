from launch import LaunchDescription
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

    return LaunchDescription([
        radiation_map_node,
        terrain_cost_map_node,
        fusion_cost_map_node,
    ])
