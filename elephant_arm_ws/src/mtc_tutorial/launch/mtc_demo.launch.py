import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    description_pkg_share = get_package_share_directory("mycobot_description")
    urdf_file = os.path.join(description_pkg_share, "urdf", "robots", "mycobot_280.urdf.xacro")

    moveit_config_package = "mycobot_moveit_config"

    moveit_controllers_file = os.path.join(
        get_package_share_directory(moveit_config_package),
        "config", "mycobot_280", "moveit_controllers.yaml"
    )

    # planning_context
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name=moveit_config_package)
        .robot_description(file_path=urdf_file)
        .robot_description_kinematics(file_path="config/mycobot_280/kinematics.yaml")
        .robot_description_semantic(file_path="config/mycobot_280/mycobot_280.srdf")
        .joint_limits(file_path='config/mycobot_280/joint_limits.yaml')
        .pilz_cartesian_limits(file_path='config/mycobot_280/pilz_cartesian_limits.yaml')
        .trajectory_execution(file_path=moveit_controllers_file)
        .to_moveit_configs()
    )

    moveit_config_dict = moveit_config.to_dict()

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config_dict,
            move_group_capabilities,
            {"use_sim_time": True}
        ],
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit2_tutorials") + "/launch/mtc.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time":True}
        ],
    )

    # # ros2_control using FakeSystem as hardware
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory(moveit_config_package),
    #     "config",
    #     "mycobot_280",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         moveit_config.robot_description,
    #         ros2_controllers_path
    #         ],
    #     remappings=[
    #         ("/controller_manager/robot_description", "/robot_description"),
    #     ],
    #     output="both",
    # )

    # Load controllers
    load_controllers = []
    for controller in [
        "arm_controller",
        "gripper_action_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {} --ros-args -p use_sim_time:=true".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            # ros2_control_node,
        ]
        + load_controllers
    )