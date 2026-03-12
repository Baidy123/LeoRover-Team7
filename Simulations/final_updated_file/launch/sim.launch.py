import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_leo_lidar_sim = get_package_share_directory('leo_lidar_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Use our wrapper that includes official leo_description + LiDAR
    pkg_leo_description = get_package_share_directory('leo_description')
    urdf_file = os.path.join(pkg_leo_lidar_sim, 'urdf', 'leo_sim_lidar.urdf.xacro')
    
    world_file = os.path.join(pkg_leo_lidar_sim, 'worlds', 'leo_world.sdf')
    rviz_config = os.path.join(pkg_leo_lidar_sim, 'rviz', 'leo_sim.rviz')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_arg = LaunchConfiguration('rviz', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )
    
    # Robot state publisher - use xacro to process the file
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'leo_rover',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for LiDAR
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for odometry
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        output='screen'
    )
    
    # ROS-Gazebo bridge for clock
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )
    
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz_arg),
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge_lidar,
        bridge_odom,
        bridge_cmd_vel,
        bridge_clock,
        rviz
    ])
