import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_leo_lidar_sim = get_package_share_directory('leo_lidar_sim')
    urdf_file = os.path.join(pkg_leo_lidar_sim, 'urdf', 'leo_with_visible_arm.urdf.xacro')
    
    # Process xacro file
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )
    
    return LaunchDescription([
        robot_state_publisher
    ])
