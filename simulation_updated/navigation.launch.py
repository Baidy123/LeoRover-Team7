#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch Leo Rover simulation WITH PILLARS for LiDAR comparison
    Package: leo_lidar_sim
    """
    
    pkg_leo_lidar_sim = get_package_share_directory('leo_lidar_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Paths
    world_file = os.path.join(pkg_leo_lidar_sim, 'worlds', 'warehouse.world')
    urdf_file = os.path.join(pkg_leo_lidar_sim, 'urdf', 'leo_rover_with_pillars.urdf.xacro')
    slam_config = os.path.join(pkg_leo_lidar_sim, 'config', 'slam_params_sim.yaml')
    nav2_config = os.path.join(pkg_leo_lidar_sim, 'config', 'nav2_params_sim.yaml')
    rviz_config = os.path.join(pkg_leo_lidar_sim, 'config', 'rviz_config.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # ===== GAZEBO =====
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    
    # ===== ROBOT STATE PUBLISHER (with pillars URDF) =====
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # ===== SPAWN ROBOT =====
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'leo_rover_pillars'],
        output='screen'
    )
    
    # ===== SLAM TOOLBOX =====
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen'
    )
    
    # ===== RVIZ =====
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        slam,
        rviz
    ])
