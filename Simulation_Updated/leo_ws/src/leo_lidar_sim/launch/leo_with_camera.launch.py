import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_leo_lidar_sim = get_package_share_directory('leo_lidar_sim')
    pkg_leo_description = get_package_share_directory('leo_description')
    
    # Files
    world_file = os.path.join(pkg_leo_lidar_sim, 'worlds', 'sorting_room.sdf')
    rviz_config = os.path.join(pkg_leo_lidar_sim, 'rviz', 'leo_sim.rviz')
    urdf_path = os.path.join(pkg_leo_description, 'urdf', 'leo_sim.urdf.xacro')
    lidar_model = os.path.join(pkg_leo_lidar_sim, 'models', 'lidar_sensor', 'model.sdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_arg = LaunchConfiguration('rviz')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2'
    )
    
    # Robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )
    
    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen'
    )
    
    # Spawn Leo
    spawn_leo = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                     '-name', 'leo',
                     '-topic', 'robot_description',
                     '-x', '1.8',
                     '-y', '-1.8',
                     '-z', '0.3'],
                output='screen'
            )
        ]
    )
    
    # Spawn LiDAR
    spawn_lidar = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                     '-name', 'lidar_sensor',
                     '-file', lidar_model,
                     '-x', '1.8',
                     '-y', '-1.8',
                     '-z', '0.5'],
                output='screen'
            )
        ]
    )
    
    # Bridges
    bridge_clock = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                output='screen'
            )
        ]
    )
    
    bridge_cmd_vel = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
                output='screen'
            )
        ]
    )
    
    bridge_odom = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                output='screen'
            )
        ]
    )
    
    bridge_scan = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
                output='screen'
            )
        ]
    )
    
    bridge_joint_states = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
                output='screen'
            )
        ]
    )
    
    # NEW: Camera bridge
    bridge_camera = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
                output='screen'
            )
        ]
    )
    
    # TF transform for LiDAR
    lidar_tf = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='lidar_tf_publisher',
                arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'lidar_sensor/lidar_link/lidar'],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # Odom to TF publisher
    odom_tf = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='leo_lidar_sim',
                executable='odom_to_tf.py',
                name='odom_to_tf',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # SLAM
    slam = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                     'use_sim_time:=true'],
                output='screen'
            )
        ]
    )
    
    # RViz
    rviz = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(rviz_arg),
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        robot_state_publisher,
        gazebo,
        spawn_leo,
        spawn_lidar,
        bridge_clock,
        bridge_cmd_vel,
        bridge_odom,
        bridge_scan,
        bridge_joint_states,
        bridge_camera,  # Camera added!
        lidar_tf,
        odom_tf,
        slam,
        rviz
    ])
