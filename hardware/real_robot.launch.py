#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for REAL Leo Rover - runs on NUC
    
    Assumptions:
    - Raspberry Pi is running Leo base (via Ethernet at 10.0.0.1)
    - LiDAR connected to NUC via USB
    - RealSense camera connected to NUC via USB
    - NUC and Pi on same ROS_DOMAIN_ID
    """
    
    # Package directories
    pkg_leo_lidar_sim = get_package_share_directory('leo_lidar_sim')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    
    # Paths
    slam_config = os.path.join(pkg_leo_lidar_sim, 'config', 'slam_params_real.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # ============ SENSORS (Connected to NUC) ============
    
    # RPLidar A2/A3
    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',  # Adjust if needed
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_frequency': 10.0,
            'scan_mode': 'Standard'  # or 'Sensitivity' for A3
        }],
        output='screen'
    )
    
    # Intel RealSense D435
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'camera_name': 'camera',
            'camera_namespace': 'camera'
        }.items()
    )
    
    # ============ STATIC TRANSFORMS ============
    
    # LiDAR TF (adjust based on YOUR measurements!)
    # Format: [x, y, z, roll, pitch, yaw, parent, child]
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf',
        arguments=['0.0', '0.0', '0.25', '0', '0', '0',
                  'base_link', 'laser_frame'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Camera TF (adjust based on YOUR measurements!)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=['0.15', '0', '0.15', '0', '0', '0',
                  'base_link', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Depth camera link (RealSense specific)
    depth_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_camera_tf',
        arguments=['0', '0', '0', '0', '0', '0',
                  'camera_link', 'camera_depth_frame'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # ============ SLAM TOOLBOX ============
    
    slam = TimerAction(
        period=5.0,  # Wait 5 seconds for sensors to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    pkg_slam_toolbox,
                    '/launch/online_async_launch.py'
                ]),
                launch_arguments={
                    'slam_params_file': slam_config,
                    'use_sim_time': 'false'
                }.items()
            )
        ]
    )
    
    # ============ OPTIONAL: RVIZ ============
    
    rviz_config = os.path.join(pkg_leo_lidar_sim, 'config', 'rviz_config.rviz')
    
    rviz = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Sensors
        lidar,
        camera,
        
        # TFs
        laser_tf,
        camera_tf,
        depth_camera_tf,
        
        # SLAM
        slam,
        
        # Visualization (comment out if not needed)
        rviz
    ])
