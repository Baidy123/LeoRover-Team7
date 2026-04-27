#!/bin/bash

echo "=========================================="
echo "Leo Rover - Rosbag Data Recorder"
echo "=========================================="
echo ""

# Source workspace
source ~/leo_ws/install/setup.bash

# Create rosbag directory
mkdir -p ~/leo_ws/rosbag_data
cd ~/leo_ws/rosbag_data

# Get bag name from user or use default
read -p "Enter bag name (default: leo_data): " BAG_NAME
BAG_NAME=${BAG_NAME:-leo_data}

echo ""
echo "Recording data to: ~/leo_ws/rosbag_data/${BAG_NAME}"
echo ""
echo "Topics being recorded:"
echo "  /scan         - LiDAR data"
echo "  /odom         - Odometry"
echo "  /tf           - Transform tree"
echo "  /tf_static    - Static transforms"
echo "  /cmd_vel      - Velocity commands"
echo "  /map          - SLAM map"
echo ""
echo "Drive the robot around with teleop for 2-3 minutes"
echo "Press Ctrl+C when done"
echo ""

# Record
ros2 bag record \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /cmd_vel \
  /map \
  -o ${BAG_NAME}

echo ""
echo "Recording saved to: ~/leo_ws/rosbag_data/${BAG_NAME}"
echo ""
