import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Which box to pick. Color is derived from the name.
    target_box_arg = DeclareLaunchArgument(
        'target_box', default_value='red_box_2',
        description='Box to pick up (red_box_1, red_box_2, blue_box_1, '
                    'blue_box_2, green_box_1, green_box_2). The matching '
                    'colored corner basket is selected automatically.')
    # Set to 'false' to launch the simulation without auto-running the mission
    autostart_mission_arg = DeclareLaunchArgument(
        'autostart_mission', default_value='true',
        description='If true, autonomous_mission node is launched after '
                    'everything is up and starts driving immediately.')
    
    # Package paths
    pkg_leo_lidar_sim = get_package_share_directory('leo_lidar_sim')
    
    # File paths
    urdf_file = os.path.join(pkg_leo_lidar_sim, 'urdf', 'leo_with_visible_arm.urdf.xacro')
    world_file = os.path.join(pkg_leo_lidar_sim, 'worlds', 'sorting_room.sdf')
    rviz_config = os.path.join(pkg_leo_lidar_sim, 'rviz', 'leo_sim.rviz')
    lidar_model = os.path.join(pkg_leo_lidar_sim, 'models', 'lidar_sensor', 'model.sdf')
    
    # Process URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # 1. Robot State Publisher
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
    
    # 2. Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )
    
    # 3. Spawn Leo (after 5 seconds)
    spawn_leo = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                     '-name', 'leo',
                     '-topic', 'robot_description',
                     '-x', '1.8', '-y', '-1.8', '-z', '0.3'],
                output='screen'
            )
        ]
    )
    
    # 4. Spawn LiDAR (after 7 seconds)
    spawn_lidar = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                     '-name', 'lidar_sensor',
                     '-file', lidar_model,
                     '-x', '1.8', '-y', '-1.8', '-z', '0.5'],
                output='screen'
            )
        ]
    )


    # Controller spawners — JSB first at 17s, then arm/gripper at 19s
    spawn_jsb = TimerAction(
        period=17.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    spawn_arm_ctrl = TimerAction(
        period=19.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['arm_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    spawn_gripper_ctrl = TimerAction(
        period=19.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )





    
    # 5. Bridges (after 9 seconds)
    bridge_clock = TimerAction(
        period=9.0,
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
        period=9.0,
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
        period=9.0,
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
        period=9.0,
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
        period=9.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
                output='screen'
            )
        ]
    )
    ######
    bridge_depth_image = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_depth_image',
                arguments=['/depth_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
                output='screen'
            )
        ]
    )

    bridge_depth = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_depth', 
                arguments=['/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'],
                output='screen'
            )
        ]
    )

    bridge_depth_points = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_depth_points',
                arguments=['/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
                output='screen'
            )
        ]
    )
    ######
    # 6. TF Publishers (after 11 seconds)
    odom_to_tf = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='leo_lidar_sim',
                executable='odom_to_tf.py',
                name='odom_to_tf',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    lidar_tf = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='lidar_tf',
                arguments=['0', '0', '0.2', '0', '0', '0', 
                          'base_link', 'lidar_sensor/lidar_link/lidar'],
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # 7. SLAM (after 13 seconds)
    slam = TimerAction(
        period=13.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                     'use_sim_time:=true'],
                output='screen'
            )
        ]
    )
    
    # 8. RViz (after 15 seconds)
    rviz = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    # 9. Arm controller — starts at 12s, after Gazebo and bridges are alive.
    # On startup it broadcasts `detach` to all 6 boxes to undo the
    # gz-sim-detachable-joint-system "starts attached" default.
    arm_controller_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='leo_lidar_sim',
                executable='arm_controller.py',
                name='arm_controller',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ]
    )

    # 10. Autonomous mission — starts at 25s so SLAM + Nav2 have time to come up.
    # User can disable with `autostart_mission:=false` to drive manually.
    from launch.conditions import IfCondition
    mission_node = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='leo_lidar_sim',
                executable='autonomous_mission.py',
                name='autonomous_mission',
                output='screen',
                condition=IfCondition(LaunchConfiguration('autostart_mission')),
                parameters=[{
                    'use_sim_time': True,
                    'target_box':   LaunchConfiguration('target_box'),
                }],
            )
        ]
    )

    return LaunchDescription([
        target_box_arg,
        autostart_mission_arg,
        robot_state_publisher,
        gazebo,
        spawn_leo,
        spawn_lidar,
        spawn_jsb,
        spawn_arm_ctrl,
        spawn_gripper_ctrl,
        bridge_clock,
        bridge_cmd_vel,
        bridge_odom,
        bridge_scan,
        bridge_joint_states,
        bridge_depth_image,
        bridge_depth,
        bridge_depth_points,
        odom_to_tf,
        lidar_tf,
        slam,
        rviz,
        arm_controller_node,
        mission_node,
    ])
