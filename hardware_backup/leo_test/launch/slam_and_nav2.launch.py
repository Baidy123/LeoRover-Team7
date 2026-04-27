#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_leo_test    = get_package_share_directory('leo_test')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    slam_params = os.path.join(pkg_leo_test, 'config', 'slam_params_real.yaml')
    nav2_params = os.path.join(pkg_leo_test, 'config', 'nav2_params_real.yaml')

    # ===== SLAM =====
    # Pass slam_params_file so online_async_launch uses YOUR yaml, not defaults
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time':      'false',
            'slam_params_file':  slam_params,   # <-- THIS was missing before
        }.items()
    )

    # ===== NAV2 =====
    # Delayed to give SLAM time to publish /map and map->odom TF before Nav2 starts
    nav2 = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file':  nav2_params,
                    # Remove odom_topic override unless you actually publish /merged_odom
                    # 'odom_topic': '/merged_odom',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        slam,
        nav2,
    ])
