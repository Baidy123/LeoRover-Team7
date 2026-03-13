import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_pkg_share = get_package_share_directory("mycobot_description")
    urdf_file = os.path.join(description_pkg_share, "urdf", "robots", "mycobot_280.urdf.xacro")

    moveit_config_package = "mycobot_moveit_config"
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name=moveit_config_package)
        .robot_description(file_path=urdf_file)
        .robot_description_kinematics(file_path="config/mycobot_280/kinematics.yaml")
        .robot_description_semantic(file_path="config/mycobot_280/mycobot_280.srdf")
        .joint_limits(file_path='config/mycobot_280/joint_limits.yaml')
        .pilz_cartesian_limits(file_path='config/mycobot_280/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    ).to_dict()
    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_tutorial",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])