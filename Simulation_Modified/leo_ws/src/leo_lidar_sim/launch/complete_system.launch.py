import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_leo_lidar_sim = get_package_share_directory('leo_lidar_sim')

    # ── Launch arguments ──────────────────────────────────────────────────────
    target_boxes_arg = DeclareLaunchArgument(
        'target_boxes',
        default_value='red_box_2,red_box_1,blue_box_1,blue_box_2,green_box_1,green_box_2',
        description='Comma-separated boxes to pick up in order.')
    target_box_arg = DeclareLaunchArgument(
        'target_box', default_value='',
        description='Single-box override. If non-empty, only this box is picked.')
    autostart_mission_arg = DeclareLaunchArgument(
        'autostart_mission', default_value='true',
        description='Set false to skip auto-running the mission.')

    # ── File paths ────────────────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_leo_lidar_sim, 'urdf', 'leo_with_visible_arm.urdf.xacro')
    world_file = os.path.join(pkg_leo_lidar_sim, 'worlds', 'sorting_room.sdf')
    rviz_config = os.path.join(pkg_leo_lidar_sim, 'rviz', 'leo_sim.rviz')
    lidar_model = os.path.join(pkg_leo_lidar_sim, 'models', 'lidar_sensor', 'model.sdf')

    # ── Robot description (URDF) ──────────────────────────────────────────────
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

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

    # ── Gazebo + spawn ────────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

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

    # No separate lidar — using leo_description's embedded URDF lidar,
    # which is at base_link + (0.15, 0, 0.10) → world z=0.26, forward
    # mounted, 180° forward FOV (no rover-self-rays). After patching
    # macros.xacro from sensor type="ray" → "gpu_lidar" (gz Harmonic's
    # sensors system uses GPU rendering), the embedded sensor produces
    # scans on /scan with frame_id 'lidar_link' and physically tracks
    # the rover via robot_state_publisher's URDF chain.

    # ── Bridges ───────────────────────────────────────────────────────────────
    def _bridge(args, name=None):
        node_kwargs = {
            'package': 'ros_gz_bridge',
            'executable': 'parameter_bridge',
            'arguments': args,
            'output': 'screen',
        }
        if name:
            node_kwargs['name'] = name
        return Node(**node_kwargs)

    # bridge_scan: gz topic /scan_gpu (our URDF gpu_lidar sensor) is
    # remapped to ROS /scan so SLAM and Nav2 see the published name they
    # expect. We don't bridge gz /scan because leo_description's silent
    # type="ray" sensor "publishes" there.
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_scan',
        arguments=['/scan_gpu@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/scan_gpu', '/scan')],
        output='screen',
    )

    bridges = TimerAction(
        period=9.0,
        actions=[
            _bridge(['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], 'bridge_clock'),
            _bridge(['/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'], 'bridge_cmd_vel'),
            _bridge(['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'], 'bridge_odom'),
            scan_bridge,
            _bridge(['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'], 'bridge_joint_states'),
        ]
    )

    # ── TF: lidar attachment + odom → tf ──────────────────────────────────────
    # Run odom_to_tf via python3 directly so a missing libexec executable bit
    # (which aborted the whole launch on the first attempt) can never crash
    # the launch again. odom_to_tf.py listens on /odom and broadcasts the
    # dynamic odom → base_footprint TF that SLAM and Nav2 both require.
    odom_to_tf_path = os.path.join(pkg_leo_lidar_sim, 'scripts', 'odom_to_tf.py')
    if not os.path.isfile(odom_to_tf_path):
        # Fall back to the source path if the share copy isn't installed.
        odom_to_tf_path = '/home/student31/leo_ws/src/leo_lidar_sim/scripts/odom_to_tf.py'
    odom_to_tf = TimerAction(
        period=11.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', odom_to_tf_path,
                     '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen',
                name='odom_to_tf',
            )
        ]
    )

    # No static lidar TF needed — robot_state_publisher already chains
    # base_link → lidar_link from the leo_description URDF.

    # ── SLAM ──────────────────────────────────────────────────────────────────
    # Use the bringup launch; it does the lifecycle configure+activate
    # dance for the lifecycle slam_toolbox node. Direct Node spawning
    # skipped that step and the node stayed unconfigured (silent after
    # "Node using stack size", never published /map). Pass our config
    # file so transform_timeout / tf_buffer_duration are big enough.
    slam_config = os.path.join(pkg_leo_lidar_sim, 'config', 'slam_toolbox.yaml')
    slam = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                     'use_sim_time:=true',
                     f'slam_params_file:={slam_config}'],
                output='screen',
            )
        ]
    )

    # ── RViz ──────────────────────────────────────────────────────────────────
    rviz = TimerAction(
        period=15.0,
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

    # ── Early box release ────────────────────────────────────────────────────
    # DetachableJoint plugins on the rover URDF start ATTACHED, so all 6
    # boxes get pulled to the gripper at sim start. Detach them in the
    # first second so they fall to their pedestal initial poses. Three
    # bursts spaced 0.3 s apart so the gz subscriber definitely catches
    # at least one — single message can be missed during plugin init.
    BOX_NAMES = ['red_box_1', 'red_box_2',
                 'blue_box_1', 'blue_box_2',
                 'green_box_1', 'green_box_2']
    early_release_actions = []
    for _ in range(3):
        for name in BOX_NAMES:
            early_release_actions.append(
                ExecuteProcess(
                    cmd=['gz', 'topic', '-t', f'/box/{name}/detach',
                         '-m', 'gz.msgs.Empty', '-p', ''],
                    output='log',
                )
            )
    early_release = TimerAction(
        period=6.0,
        actions=early_release_actions,
    )

    # ── Arm controller ────────────────────────────────────────────────────────
    arm_controller_node = TimerAction(
        period=15.0,
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

    # ── Autonomous mission (last; needs Nav2 action server up) ────────────────
    # Nav2 is launched SEPARATELY by the user (e.g. `bash ~/start_nav2_minimal.sh`
    # or `ros2 launch leo_lidar_sim navigation.launch.py`). The mission node
    # below patiently waits up to 2 minutes for the /navigate_to_pose action
    # server, so start Nav2 in another terminal any time before t≈45s.
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
                    'use_sim_time':  True,
                    'target_boxes': LaunchConfiguration('target_boxes'),
                    'target_box':   LaunchConfiguration('target_box'),
                }],
            )
        ]
    )

    return LaunchDescription([
        target_boxes_arg,
        target_box_arg,
        autostart_mission_arg,
        robot_state_publisher,
        gazebo,
        spawn_leo,
        bridges,
        odom_to_tf,
        slam,
        rviz,
        early_release,
        arm_controller_node,
        mission_node,
    ])
