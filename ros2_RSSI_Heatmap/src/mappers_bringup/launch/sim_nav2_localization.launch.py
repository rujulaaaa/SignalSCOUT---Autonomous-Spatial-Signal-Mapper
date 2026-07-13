import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('mappers_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock from Gazebo'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/ros2_RSSI_Heatmap/src/maps/map.yaml'),
            description='Full path to the saved map YAML file'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=nav2_params_file,
            description='Full path to the Nav2 parameters YAML'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically start Nav2 lifecycle nodes'),

        # ── Nav2 Localization (map_server + AMCL) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml_file,
                'params_file': LaunchConfiguration('nav2_params_file'),
                'autostart': autostart,
            }.items(),
        ),

        # ── Nav2 Navigation Stack ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': LaunchConfiguration('nav2_params_file'),
                'autostart': autostart,
            }.items(),
        ),

        # ── Twist Stamper (Converts Twist to TwistStamped) ──
        Node(
            package='twist_stamper',
            executable='twist_stamper',
            name='twist_stamper_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cmd_vel_in', 'cmd_vel_smoothed'),
                ('cmd_vel_out', 'cmd_vel'),
            ],
        ),

        # ── RViz2 ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # ── Publish Initial Pose ──
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '-1', '/initialpose',
                        'geometry_msgs/msg/PoseWithCovarianceStamped',
                        '{header: {frame_id: "map"}, '
                        'pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, '
                        'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, '
                        'covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.25, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
                        '0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'
                    ],
                    output='screen',
                ),
            ]
        ),

        # ── Activate All Nav2 Lifecycle Nodes ──
        TimerAction(
            period=8.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        'ros2 lifecycle set /controller_server configure ; '                       
                        'ros2 lifecycle set /controller_server activate ; '
                        'ros2 lifecycle set /planner_server configure ; '
                        'ros2 lifecycle set /planner_server activate ; '
                        'ros2 lifecycle set /waypoint_follower configure ; '
                        'ros2 lifecycle set /waypoint_follower activate ; '
                        'ros2 lifecycle set /bt_navigator configure ; '
                        'ros2 lifecycle set /bt_navigator activate ; '
                        'ros2 lifecycle set /velocity_smoother configure ; '
                        'ros2 lifecycle set /velocity_smoother activate ; '
                        'ros2 lifecycle set /behavior_server configure ; '
                        'ros2 lifecycle set /behavior_server activate ; '
                        'echo "All Nav2 nodes activated!"'
                    ],
                    output='screen',
                ),
            ]
        ),
    ])
