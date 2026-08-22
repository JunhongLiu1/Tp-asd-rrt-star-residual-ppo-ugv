from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    terrain = LaunchConfiguration('terrain')
    data_directory = LaunchConfiguration(
        'data_directory'
    )
    frame_id = LaunchConfiguration('frame_id')
    publish_rate_hz = LaunchConfiguration(
        'publish_rate_hz'
    )
    use_sim_time = LaunchConfiguration(
        'use_sim_time'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'terrain',
            default_value='easy',
            description=(
                'Terrain level: easy, medium, or hard'
            )
        ),

        DeclareLaunchArgument(
            'data_directory',
            default_value='',
            description=(
                'Directory containing terrain layer '
                'NPZ and metadata files'
            )
        ),

        DeclareLaunchArgument(
            'frame_id',
            default_value='map',
            description='OccupancyGrid frame'
        ),

        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='1.0',
            description='Terrain map publication rate'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use Gazebo simulation clock'
        ),

        Node(
            package='radiation_mapping',
            executable='terrain_layer_publisher',
            name='terrain_layer_publisher',
            output='screen',
            parameters=[{
                'terrain_level': terrain,
                'data_directory': data_directory,
                'frame_id': frame_id,
                'publish_rate_hz': publish_rate_hz,
                'use_sim_time': use_sim_time,
            }],
        ),

        Node(
            package='radiation_mapping',
            executable='terrain_query_server',
            name='terrain_query_server',
            output='screen',
            parameters=[{
                'terrain_level': terrain,
                'data_directory': data_directory,
                'service_name': '/query_terrain',
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
