#!/bin/bash

# Nav2 Startup Script for Real Leo Rover (Running on NUC)
# Installed via colcon build

echo "============================================"
echo "  Starting Nav2 for Real Leo Rover (NUC)"
echo "============================================"

# Source ROS2
source ~/leo_ws/install/setup.bash

# Get package share directory
PACKAGE_DIR=$(ros2 pkg prefix leo_lidar_sim)/share/leo_lidar_sim

# Nav2 params file
NAV2_PARAMS="${PACKAGE_DIR}/config/nav2_params_real.yaml"

echo "Using Nav2 params: $NAV2_PARAMS"
echo ""

# Start Nav2 nodes in background
echo "Starting controller_server..."
ros2 run nav2_controller controller_server --ros-args \
  -p use_sim_time:=false \
  --params-file $NAV2_PARAMS &
CONTROLLER_PID=$!

sleep 3

echo "Starting planner_server..."
ros2 run nav2_planner planner_server --ros-args \
  -p use_sim_time:=false \
  --params-file $NAV2_PARAMS &
PLANNER_PID=$!

sleep 3

echo "Starting behavior_server..."
ros2 run nav2_behaviors behavior_server --ros-args \
  -p use_sim_time:=false \
  --params-file $NAV2_PARAMS &
BEHAVIOR_PID=$!

sleep 3

echo "Starting bt_navigator..."
ros2 run nav2_bt_navigator bt_navigator --ros-args \
  -p use_sim_time:=false \
  --params-file $NAV2_PARAMS &
BT_PID=$!

echo ""
echo "Waiting for all nodes to spawn..."
sleep 10

# Wait for controller_server to be ready
echo "Waiting for controller_server..."
until ros2 node list 2>/dev/null | grep -q controller_server; do
  sleep 1
done

echo ""
echo "============================================"
echo "  Configuring Lifecycle Nodes"
echo "============================================"

# Configure and activate controller
echo "Configuring controller_server..."
ros2 lifecycle set /controller_server configure
sleep 3
echo "Activating controller_server..."
ros2 lifecycle set /controller_server activate
sleep 2

# Configure and activate planner
echo "Configuring planner_server..."
ros2 lifecycle set /planner_server configure
sleep 3
echo "Activating planner_server..."
ros2 lifecycle set /planner_server activate
sleep 2

# Configure and activate behavior
echo "Configuring behavior_server..."
ros2 lifecycle set /behavior_server configure
sleep 3
echo "Activating behavior_server..."
ros2 lifecycle set /behavior_server activate
sleep 2

# Configure and activate bt_navigator
echo "Configuring bt_navigator..."
ros2 lifecycle set /bt_navigator configure
sleep 3
echo "Activating bt_navigator..."
ros2 lifecycle set /bt_navigator activate

echo ""
echo "============================================"
echo "  Nav2 is READY!"
echo "============================================"
echo ""
echo "You can now:"
echo "  1. Use '2D Goal Pose' in RViz to send navigation goals"
echo "  2. Run vision system to navigate to detected objects"
echo "  3. Send programmatic goals via /navigate_to_pose action"
echo ""
echo "Press Ctrl+C to stop Nav2"
echo ""

# Wait for all background processes
wait $CONTROLLER_PID $PLANNER_PID $BEHAVIOR_PID $BT_PID
