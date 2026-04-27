from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    pkg_share = get_package_share_directory('leo_lidar_sim')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    
    # Use nav2_bringup's navigation launch
    nav2_bringup_dir = FindPackageShare('nav2_bringup')
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch/navigation_launch.py']
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file
        }.items()
    )
    
    return LaunchDescription([
        navigation_launch
    ])
