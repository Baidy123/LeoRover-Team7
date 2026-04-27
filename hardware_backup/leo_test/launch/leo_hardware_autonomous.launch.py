import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ld = LaunchDescription()
    
    # Config paths
    slam_params = os.path.join(os.path.expanduser('~'), 'leo_test', 'config', 'slam_params_real.yaml')
    nav2_params = os.path.join(os.path.expanduser('~'), 'leo_test', 'config', 'nav2_params_real.yaml')
    
    # Get Nav2 config paths from leo_v2 package
    pkg_leo_v2 = get_package_share_directory('leo_v2')
    bt_nav_config = PathJoinSubstitution([pkg_leo_v2, 'config', 'bt_nav.yaml'])
    controller_config = PathJoinSubstitution([pkg_leo_v2, 'config', 'controller.yaml'])
    planner_config = PathJoinSubstitution([pkg_leo_v2, 'config', 'planner.yaml'])
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # Lifecycle nodes for Nav2
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]
    
    # 1. SLAM Toolbox (auto-activates with use_lifecycle_manager: false)
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'), 
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'false'
        }.items(),
    )
    
    # 2. Nav2 Behavior Tree Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[bt_nav_config, {'use_sim_time': False}],
        remappings=remappings,
    )
    
    # 3. Nav2 Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[bt_nav_config, {'use_sim_time': False}],
        remappings=[('/cmd_vel', '/cmd_vel')],
    )
    
    # 4. Nav2 Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[planner_config, {'use_sim_time': False}],
        remappings=remappings,
    )
    
    # 5. Nav2 Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[controller_config, {'use_sim_time': False}],
        remappings=[('/cmd_vel', '/cmd_vel')],
    )
    
    # 6. Nav2 Lifecycle Manager (auto-starts Nav2 nodes)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': lifecycle_nodes,
            'use_sim_time': False
        }],
    )
    
    # Add actions with proper timing
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(slam_node)
    
    # Wait 5 seconds for SLAM to stabilize, then start Nav2
    ld.add_action(TimerAction(
        period=5.0,
        actions=[
            bt_navigator_node,
            behavior_server_node,
            planner_server_node,
            controller_server_node,
            lifecycle_manager_node
        ]
    ))
    
    return ld