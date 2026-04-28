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

    # 9. EARLY box release — fires at t=6s, right after spawn_leo (t=5s).
    # The DetachableJoint plugins start ATTACHED by default. With 6 boxes
    # rigidly fixed to arm_gripper at scattered floor positions, Gazebo's
    # constraint solver tries to satisfy all 6 simultaneously — yanking the
    # rover into the air or teleporting boxes around. We MUST detach within
    # ~1s of the rover spawning, before physics builds up large constraint
    # forces. Three repeats spaced 0.5s apart for reliability (the gz-sim
    # subscriber may not be ready on the very first publish).
    BOX_NAMES = ['red_box_1', 'red_box_2',
                 'blue_box_1', 'blue_box_2',
                 'green_box_1', 'green_box_2']
    early_release_actions = []
    for repeat in range(3):
        for name in BOX_NAMES:
            early_release_actions.append(
                ExecuteProcess(
                    cmd=['gz', 'topic', '-t', f'/box/{name}/detach',
                         '-m', 'gz.msgs.Empty', '-p', ''],
                    output='log',  # quiet — 18 lines is too much screen spam
                )
            )
    early_release = TimerAction(
        period=6.0 + 0.5,  # rover spawns at 5s; let it settle ~1.5s then release
        actions=early_release_actions,
    )

    # 10. Arm controller — starts at 8s, after early release. Re-broadcasts
    # detach itself in __init__ as belt-and-suspenders.
    arm_controller_node = TimerAction(
        period=8.0,
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

    # 11. Autonomous mission — starts LATE (45s) so the user has time to
    # `bash ~/start_nav2_minimal.sh` in another terminal and Nav2's
    # action server is alive before we send the first goal.
    # Set `autostart_mission:=false` to skip and drive manually with teleop.
    from launch.conditions import IfCondition
    mission_node = TimerAction(
        period=45.0,
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
        early_release,
        arm_controller_node,
        mission_node,
    ])
