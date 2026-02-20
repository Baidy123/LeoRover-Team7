#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

int main(int argc, char * argv[]){
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm_with_gripper");

    move_group_interface.startStateMonitor();
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Set a target Pose
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.28;
        msg.position.y = -0.2;
        msg.position.z = 0.5;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Check if the target is reachable
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

    if (!current_state){
        RCLCPP_ERROR(logger, "FATAl failes to get robot state");
        return 1;
    }

    const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup("arm_with_gripper");

    bool found_ik = current_state->setFromIK(joint_model_group, target_pose);

    if(found_ik){
        RCLCPP_INFO(logger, "Point is reachable");

        //Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok,msg);
        }();

        // Execute plan
        if (success){
            move_group_interface.execute(plan);
        } else{
            RCLCPP_ERROR(logger, "Planning Failed");
    }
    }
    else{
        RCLCPP_ERROR(logger, "Point is outside the workspace");
    }

    rclcpp::shutdown();
    return 0;
}