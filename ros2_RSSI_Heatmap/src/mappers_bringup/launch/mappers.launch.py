import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_yaml = os.path.expanduser('~/ros2_RSSI_Heatmap/src/maps/map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'path_to_yaml',
            default_value=default_yaml,
            description='Full path to the map .yaml file'),
        DeclareLaunchArgument(
            'heatmaps_dir',
            default_value=os.path.expanduser('~/ros2_RSSI_heatmap/src/heatmaps'),
            description='Directory to save generated heatmap images'),
        DeclareLaunchArgument(
            'density',
            default_value='8',
            description='Waypoint density (lower = more waypoints)'),
        DeclareLaunchArgument(
            'collision_range',
            default_value='4',
            description='Safety margin from obstacles in pixels'),

        Node(
            package='waypoint_publisher_package',
            executable='node',
            name='waypoint_publisher',
            parameters=[{
                'density': LaunchConfiguration('density'),
                'collision_range': LaunchConfiguration('collision_range'),
                'path_to_yaml': LaunchConfiguration('path_to_yaml'),
            }],
            output='screen',
        ),
        Node(
            package='rssi_heatmap_generator_package',
            executable='node',
            name='heatmap_generator',
            parameters=[{
                'path_to_yaml': LaunchConfiguration('path_to_yaml'),
                'heatmaps_dir': LaunchConfiguration('heatmaps_dir'),
            }],
            output='screen',
        ),
    ])
