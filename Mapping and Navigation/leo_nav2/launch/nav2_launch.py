from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg_share = get_package_share_directory('leo_nav2')
    nav_config = Path(pkg_share) / 'config' / 'nav2_params.yaml'

    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2_bringup',
            output='screen',
            parameters=[str(nav_config)],
            arguments=['--ros-args', '--log-level', 'INFO']
        )
    ])
