"""
mission_launch.py
=================
Launch file for the Leo Rover autonomous pick-and-place mission.

Run this AFTER your existing simulation is already running
(Gazebo, Nav2, SLAM, existing controller_manager).

Usage:
  ros2 launch leo_lidar_sim mission_launch.py \\
      bin_x:=3.0 bin_y:=2.0 \\
      home_x:=0.0 home_y:=0.0 \\
      target_color:=red

Arguments
---------
  bin_x / bin_y / bin_yaw     — map coordinates of the bin
  home_x / home_y / home_yaw  — initial pose to return to
  target_color                — 'red' | 'green' | 'blue'
  approach_offset             — how far in front of box the rover stops (m)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Declare arguments ────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('bin_x',          default_value='3.0'),
        DeclareLaunchArgument('bin_y',          default_value='2.0'),
        DeclareLaunchArgument('bin_yaw',        default_value='0.0'),
        DeclareLaunchArgument('home_x',         default_value='0.0'),
        DeclareLaunchArgument('home_y',         default_value='0.0'),
        DeclareLaunchArgument('home_yaw',       default_value='0.0'),
        DeclareLaunchArgument('target_color',   default_value='red'),
        DeclareLaunchArgument('approach_offset',default_value='0.35'),
        DeclareLaunchArgument('camera_hfov',    default_value='1.047'),
        DeclareLaunchArgument('camera_frame',   default_value='camera_link'),
    ]

    # ── Box Detector node ────────────────────────────────────────────────────
    box_detector = Node(
        package='leo_lidar_sim',
        executable='box_detector.py',
        name='box_detector',
        output='screen',
        parameters=[{
            'target_color':     LaunchConfiguration('target_color'),
            'camera_hfov':      LaunchConfiguration('camera_hfov'),
            'camera_frame':     LaunchConfiguration('camera_frame'),
            'min_contour_area': 800,
        }],
    )

    # ── Arm Controller node ──────────────────────────────────────────────────
    arm_controller = Node(
        package='leo_lidar_sim',
        executable='arm_controller.py',
        name='arm_controller',
        output='screen',
    )

    # ── Autonomous Mission node ──────────────────────────────────────────────
    mission = Node(
        package='leo_lidar_sim',
        executable='autonomous_mission.py',
        name='autonomous_mission',
        output='screen',
        parameters=[{
            'bin_x':          LaunchConfiguration('bin_x'),
            'bin_y':          LaunchConfiguration('bin_y'),
            'bin_yaw':        LaunchConfiguration('bin_yaw'),
            'home_x':         LaunchConfiguration('home_x'),
            'home_y':         LaunchConfiguration('home_y'),
            'home_yaw':       LaunchConfiguration('home_yaw'),
            'approach_offset':LaunchConfiguration('approach_offset'),
            'nav_timeout':    60.0,
            'arm_timeout':    20.0,
        }],
    )

    info = LogInfo(msg='🚀 Starting Leo Rover autonomous pick-and-place mission...')

    return LaunchDescription(args + [info, box_detector, arm_controller, mission])
