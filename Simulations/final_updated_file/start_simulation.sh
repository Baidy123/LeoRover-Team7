#!/bin/bash

echo "Leo Rover Simulation - Simple Launcher"
echo "======================================="

# Source workspace
source /opt/ros/jazzy/setup.bash
source ~/leo_ws/install/setup.bash

echo ""
echo "This script will open multiple terminal windows."
echo "Make sure you have a terminal that supports tabs or use tmux/screen"
echo ""
read -p "Press Enter to continue..."

# Get the package path
PKG_PATH=$(ros2 pkg prefix leo_lidar_sim)/share/leo_lidar_sim

echo ""
echo "MANUAL LAUNCH STEPS:"
echo "===================="
echo ""
echo "Terminal 1 - Robot State Publisher:"
echo "-----------------------------------"
echo "ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:=\"\$(xacro /opt/ros/jazzy/share/leo_description/urdf/leo_sim.urdf.xacro)\""
echo ""
echo "Terminal 2 - Gazebo:"
echo "-------------------"
echo "ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=\"-r $PKG_PATH/worlds/leo_world_simple.sdf\""
echo ""
echo "Terminal 3 - Spawn Robot:"
echo "------------------------"
echo "sleep 5 && ros2 run ros_gz_sim create -name leo -topic robot_description -x 0.0 -y 0.0 -z 0.3"
echo ""
echo "Terminal 4 - Bridges:"
echo "--------------------"
echo "ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
echo ""
echo "Terminal 5 - SLAM:"
echo "-----------------"
echo "ros2 launch leo_lidar_sim slam.launch.py"
echo ""
echo "Terminal 6 - RViz:"
echo "-----------------"
echo "ros2 run rviz2 rviz2 -d $PKG_PATH/rviz/leo_sim.rviz"
echo ""
echo "Terminal 7 - Teleop:"
echo "-------------------"
echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""

