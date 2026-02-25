#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

int main(int argc, char * argv[]){
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Wait for the simulation clode to sync
    while (node->now().seconds() == 0 && rclcpp::ok()) {
        RCLCPP_INFO(logger, "Waiting for simulation clock...");
        rclcpp::spin_some(node);
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // SingleThreadedExecutor for Visual Tools to interact with
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {executor.spin();});

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm_with_gripper");

    // Wait until we can get the current state of the cobot from joint_states
    move_group_interface.startStateMonitor();

    RCLCPP_INFO(logger, "Spinning to catch joint states...");
    auto start_wait = node->now();
    bool synced = false;

    while (rclcpp::ok() && (node->now() - start_wait).seconds() < 5.0) {
        // rclcpp::spin_some(node);
        // If we can get a state with a tiny timeout, we are synced
        if (move_group_interface.getCurrentState(0.01)) {
            RCLCPP_INFO(logger, "Successfully synced with robot state.");
            synced = true;
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    if (!synced) {
        RCLCPP_ERROR(logger, "Failed to sync with joint states within timeout.");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    // Create and initialize MoveItVisualTools
    auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
        node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
        move_group_interface.getRobotModel()
    };
    moveit_visual_tools.deleteAllMarkers();
    moveit_visual_tools.loadRemoteControl();

    // Create closures for visualization
    auto const draw_title = [&moveit_visual_tools](auto text){
        auto const text_pose = []{
            auto msg = Eigen::Isometry3d::Identity();
            msg.translation().z() = 1.0;
            return msg;
        }();
        moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                        rviz_visual_tools::XLARGE);
    };
    auto const prompt = [&moveit_visual_tools](auto text){
        moveit_visual_tools.prompt(text);
    };
    auto const draw_trajectory_tool_path =
        [&moveit_visual_tools,
        jmg = move_group_interface.getRobotModel()->getJointModelGroup(
            "arm_with_gripper"
        )](auto const trajectory){
            moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        };

    // Set a target Pose
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.18;
        msg.position.y = 0.0;
        msg.position.z = 0.2;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Check if the target is reachable
    moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

    if (!current_state){
        RCLCPP_ERROR(logger, "FATAL failed to get robot state");
        rclcpp::shutdown();
        spinner.join();
        return 1;
    }

    // Check that the point is reachable from the current pose
    const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup("arm_with_gripper");
    bool found_ik = current_state->setFromIK(joint_model_group, target_pose);

    if(found_ik){
        RCLCPP_INFO(logger, "Point is reachable");

        prompt("Press Next in the RvizVisualToolsGui window to plan");
        draw_title("Planning");
        moveit_visual_tools.trigger();
        //Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok,msg);
        }();

        // Execute plan
        if (success){
            draw_trajectory_tool_path(plan.trajectory);
            moveit_visual_tools.trigger();
            prompt("Press Next in RvizVIsualToolsGui window to execute");
            draw_title("Executing");
            moveit_visual_tools.trigger();
            move_group_interface.execute(plan);
        } else{
            draw_title("Planning Failed!");
            moveit_visual_tools.trigger();
            RCLCPP_ERROR(logger, "Planning Failed");
    }
    }
    else{
        RCLCPP_ERROR(logger, "Point is outside the workspace");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}