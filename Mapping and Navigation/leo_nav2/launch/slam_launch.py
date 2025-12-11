from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg_share = get_package_share_directory('leo_nav2')
    slam_config = Path(pkg_share) / 'config' / 'slam_toolbox.yaml'

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[str(slam_config)]
        )
    ])
