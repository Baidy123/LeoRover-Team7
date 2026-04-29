#!/bin/bash

# Leo Rover Complete Startup - Fixes ALL TF Issues
# Run this AFTER all other terminals are started

echo "=========================================="
echo "Leo Rover TF Fix - Starting All Transforms"
echo "=========================================="

source ~/leo_ws/install/setup.bash

# Kill any existing TF publishers to avoid conflicts
echo "Cleaning up old TF publishers..."
pkill -f "static_transform_publisher.*base_link.*lidar"
pkill -f "static_transform_publisher.*base_link.*camera"
pkill -f "static_transform_publisher.*odom.*base_footprint"
pkill -f odom_to_tf
sleep 2

echo ""
echo "Starting TF publishers..."
echo ""

# IMPORTANT: Only publish transforms that are NOT already being published!

# 1. Odom to Base Footprint (dynamic from /odom topic)
echo "[1/3] Starting odom → base_footprint publisher (dynamic)..."
python3 ~/leo_ws/src/leo_lidar_sim/scripts/odom_to_tf.py &
ODOM_TF_PID=$!
sleep 3

# 2. Base Link to LiDAR (static)
echo "[2/3] Starting base_link → lidar transform (static)..."
ros2 run tf2_ros static_transform_publisher \
  --ros-args -p use_sim_time:=true \
  --frame-id base_link --child-frame-id lidar_sensor/lidar_link/lidar \
  --x 0 --y 0 --z 0.2 --roll 0 --pitch 0 --yaw 0 &
LIDAR_TF_PID=$!
sleep 1

# 3. Base Link to Camera (static)
echo "[3/3] Starting base_link → camera_link transform (static)..."
ros2 run tf2_ros static_transform_publisher \
  --ros-args -p use_sim_time:=true \
  --frame-id base_link --child-frame-id camera_link \
  --x 0.15 --y 0 --z 0.08 --roll 0 --pitch 0.2 --yaw 0 &
CAMERA_TF_PID=$!
sleep 1

echo ""
echo "=========================================="
echo "✓ All TF transforms started!"
echo "=========================================="
echo ""
echo "Process IDs:"
echo "  Odom TF: $ODOM_TF_PID"
echo "  LiDAR TF: $LIDAR_TF_PID"
echo "  Camera TF: $CAMERA_TF_PID"
echo ""
echo "TF Tree should now be:"
echo "  map → odom → base_footprint → base_link → [camera_link, lidar, arm, wheels, etc.]"
echo ""
echo "Verify with: ros2 run tf2_tools view_frames"
echo ""
echo "Press Ctrl+C to stop all TF publishers"
echo ""

# Function to clean up on exit
cleanup() {
    echo ""
    echo "Stopping TF publishers..."
    kill $ODOM_TF_PID $LIDAR_TF_PID $CAMERA_TF_PID 2>/dev/null
    exit 0
}

trap cleanup SIGINT SIGTERM

wait
