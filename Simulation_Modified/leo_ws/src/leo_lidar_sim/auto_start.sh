#!/bin/bash

echo "=========================================="
echo "Leo Rover Complete System Launcher"
echo "=========================================="
echo ""
echo "This will launch in the correct order:"
echo "1. Robot State Publisher"
echo "2. Gazebo Simulation" 
echo "3. Spawn Leo + LiDAR"
echo "4. ROS-Gazebo Bridges"
echo "5. LiDAR TF Transform"
echo "6. SLAM Toolbox"
echo "7. RViz"
echo ""
echo "Press Ctrl+C to stop everything"
echo ""

# Source workspace
source /opt/ros/jazzy/setup.bash
source ~/leo_ws/install/setup.bash

# Cleanup
echo "Cleaning up old processes..."
killall -9 gz ruby 2>/dev/null
sleep 2

echo ""
echo "[1/7] Starting Robot State Publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(xacro /opt/ros/jazzy/share/leo_description/urdf/leo_sim.urdf.xacro)" &
RSP_PID=$!
sleep 3

echo "[2/7] Starting Gazebo..."
ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:="-r ~/leo_ws/src/leo_lidar_sim/worlds/leo_world.sdf" &
GZ_PID=$!
sleep 5

echo "[3/7] Spawning Leo Rover and LiDAR..."
ros2 run ros_gz_sim create \
  -name leo \
  -topic robot_description \
  -x 0.0 -y 0.0 -z 0.3 &
sleep 2

ros2 run ros_gz_sim create \
  -name lidar_sensor \
  -file ~/leo_ws/src/leo_lidar_sim/models/lidar_sensor/model.sdf \
  -x 0.0 -y 0.0 -z 0.5 &
sleep 3

echo "[4/7] Starting ROS-Gazebo Bridges..."
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
ros2 run ros_gz_bridge parameter_bridge /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model &
sleep 2

echo "[5/7] Starting LiDAR TF Transform..."
ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link lidar_sensor/lidar_link/lidar &
sleep 2

echo "[6/7] Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true &
sleep 3

echo "[7/7] Starting RViz..."
ros2 run rviz2 rviz2 -d ~/leo_ws/src/leo_lidar_sim/rviz/leo_sim.rviz &
sleep 2

echo ""
echo "=========================================="
echo "✓ All systems running!"
echo "=========================================="
echo ""
echo "To control the robot, open a new terminal and run:"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""
echo "To record rosbag data:"
echo "  ros2 bag record /scan /odom /tf /tf_static /cmd_vel"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Keep script running
wait
