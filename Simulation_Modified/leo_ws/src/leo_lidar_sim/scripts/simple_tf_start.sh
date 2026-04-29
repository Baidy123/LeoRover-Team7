#!/bin/bash

echo "================================================"
echo "Starting ALL Required TF Transforms"
echo "================================================"

source ~/leo_ws/install/setup.bash

# Clean up any old transforms
echo "Cleaning old TF publishers..."
pkill -9 -f "odom_to_tf"
pkill -9 -f "static_transform.*lidar"
pkill -9 -f "static_transform.*camera"
sleep 2

echo ""
echo "Starting transforms..."

# 1. Odom to base_footprint (DYNAMIC - from odometry)
echo "[1/2] Odom → base_footprint (dynamic)"
ros2 run leo_lidar_sim odom_to_tf.py --ros-args -p use_sim_time:=true &
sleep 3

# 2. Base_link to LiDAR (STATIC)
echo "[2/2] base_link → LiDAR (static)"
ros2 run tf2_ros static_transform_publisher \
  0 0 0.2 0 0 0 base_link lidar_sensor/lidar_link/lidar \
  --ros-args -p use_sim_time:=true &

sleep 2

echo ""
echo "================================================"
echo "✓ TF transforms running!"
echo "================================================"
echo ""
echo "Verify: ros2 run tf2_tools view_frames"
echo "Press Ctrl+C to stop"
echo ""

wait
