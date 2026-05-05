"""
Team 7 - Detect + Approach launch (real robot)

Starts:
  - detection_node    (vision, publishes /detected_objects and /detected_boxes)
  - detect_approach   (mission FSM, subscribes to detections, drives Nav2
                       via NavigateToPose action)

Does NOT start Nav2 / slam_toolbox / PS4 teleop — launch those separately.

Usage:
  ros2 launch leo_test detect_approach.launch.py
  ros2 launch leo_test detect_approach.launch.py world_frame:=odom sim_mode:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---------- Configurable arguments ----------
    color_params_file = DeclareLaunchArgument(
        'color_params_file',
        default_value='/home/student15/rspd_venv/src/colour_params.csv',
        description='HSV color params CSV')

    world_frame = DeclareLaunchArgument(
        'world_frame',
        default_value='map',
        description='Nav2 global frame (real: map, sim: odom)')

    camera_frame = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_color_optical_frame',
        description='RealSense optical frame (real); '
                    'sim: depth_camera_link')

    rgb_topic = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/depth_camera/image',
        description='Sim only: Gazebo RGB image topic '
                    '(ignored in real mode — DetectionSystem reads camera directly)')

    depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/depth_camera/depth_image',
        description='Sim only: Gazebo depth image topic '
                    '(ignored in real mode)')

    sim_mode = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        description='True for Gazebo sim, False for real robot')

    only_blocks = DeclareLaunchArgument(
        'only_blocks',
        default_value='false',
        description='If true detection_node ignores boxes')

    process_rate = DeclareLaunchArgument(
        'process_rate',
        default_value='10.0',
        description='Detection processing rate (Hz)')

    publish_images = DeclareLaunchArgument(
        'publish_images',
        default_value='true',
        description='Republish raw camera images on /camera/* topics so '
                    'other nodes can subscribe instead of opening the camera')

    stop_dist = DeclareLaunchArgument(
        'stop_dist',
        default_value='0.5',
        description='Distance (m) to stop in front of the block, so the '
                    'rover does not hit the obstacle the block sits on')

    base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Rover body frame (used to look up current rover '
                    'position when computing the stand-off pose)')

    # ---------- Nodes ----------
    detection = Node(
        package='leo_test',
        executable='detection_node',
        name='detection_node',
        output='screen',
        parameters=[{
            'color_params_file': LaunchConfiguration('color_params_file'),
            'world_frame': LaunchConfiguration('world_frame'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'sim_mode': LaunchConfiguration('sim_mode'),
            'only_blocks': LaunchConfiguration('only_blocks'),
            'process_rate': LaunchConfiguration('process_rate'),
            'publish_images': LaunchConfiguration('publish_images'),
        }],
    )

    detect_approach = Node(
        package='leo_test',
        executable='detect_approach',
        name='detect_approach',
        output='screen',
        parameters=[{
            'world_frame': LaunchConfiguration('world_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'stop_dist': LaunchConfiguration('stop_dist'),
        }],
    )

    return LaunchDescription([
        color_params_file,
        world_frame,
        camera_frame,
        rgb_topic,
        depth_topic,
        sim_mode,
        only_blocks,
        process_rate,
        publish_images,
        stop_dist,
        base_frame,
        detection,
        detect_approach,
    ])
