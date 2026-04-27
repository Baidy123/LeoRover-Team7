#!/usr/bin/env python3

import os
import launch
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, DeclareLaunchArgument,
    EmitEvent, RegisterEventHandler
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir     = get_package_share_directory('leo_test')
    pkg_nav2    = get_package_share_directory('nav2_bringup')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_params_real.yaml')
    nav2_config = os.path.join(pkg_dir, 'config', 'nav2_params_real.yaml')

    # ===== LIDAR =====
    home_dir = os.path.expanduser('~')
    rplidar_launch_path = os.path.join(
        home_dir, 'leo_test', 'src', 'rplidar_ros', 'launch',
        'view_rplidar_a2m12_launch.py'
    )
    if os.path.exists(rplidar_launch_path):
        lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_path)
        )
    else:
        lidar = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser',
                'angle_compensate': True,
                'scan_frequency': 10.0
            }],
            output='screen'
        )

    # ===== STATIC TFs =====
    # Use named arguments (--x, --frame-id etc.) to avoid zero-timestamp bug in Jazzy
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_to_base_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.25',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser'
        ]
    )

    # Override rover's zero-timestamp base_footprint->base_link with a live one
    # z=0.198 measured from tf2_echo base_footprint base_link
    base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_tf',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.198',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'
        ]
    )

    # ===== SLAM TOOLBOX as LifecycleNode =====
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        parameters=[slam_config],
        output='screen',
    )

    # Auto-activate once configure succeeds (node reaches 'inactive')
    slam_activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ]
        )
    )

    # Configure at t=5s (gives lidar + TF time to settle)
    slam_configure_timer = TimerAction(
        period=5.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(slam_node),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            )
        ]
    )

    # ===== NAV2 at t=15s (map frame must exist first) =====
    nav2 = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file':  nav2_config,
                }.items()
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        lidar,
        laser_tf,
        base_tf,
        slam_node,
        slam_activate_on_inactive,
        slam_configure_timer,
        nav2,
    ])