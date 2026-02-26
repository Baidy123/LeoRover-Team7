import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    detector_type = LaunchConfiguration('detector', default='python')  # 'python' or 'cpp'
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    
    declare_detector_type = DeclareLaunchArgument(
        'detector',
        default_value='python',
        description='Type of detector to use: python or cpp'
    )
    
    # Python detector (default, simpler)
    python_detector = Node(
        package='leo_lidar_sim',
        executable='simple_object_detector.py',
        name='object_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'min_cluster_size': 3,
            'max_cluster_size': 100,
            'cluster_tolerance': 0.3,
            'min_object_distance': 0.2,
            'max_object_distance': 5.0
        }]
    )
    
    # C++ detector (alternative, more performance)
    # Uncomment if you want to use C++ version
    # cpp_detector = Node(
    #     package='leo_lidar_sim',
    #     executable='lidar_object_detector',
    #     name='object_detector',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'min_cluster_size': 3,
    #         'max_cluster_size': 100,
    #         'cluster_tolerance': 0.3,
    #         'min_object_distance': 0.2,
    #         'max_object_distance': 5.0
    #     }]
    # )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_detector_type,
        python_detector
    ])
