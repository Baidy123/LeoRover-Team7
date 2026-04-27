#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file that properly configures detection_node with use_sim_time
    """
    
    return LaunchDescription([
        Node(
            package='leo_lidar_sim',
            executable='detection_node.py',
            name='detection_node',
            output='screen',
            parameters=[{
                'use_sim_time': True  # CRITICAL!
            }]
        ),
        
        Node(
            package='leo_lidar_sim',
            executable='vision_to_navigation.py',
            name='vision_to_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True  # CRITICAL!
            }]
        )
    ])
