#!/bin/bash

echo "Leo Rover Simulation - Quick Fix Script"
echo "========================================"
echo ""

# Check if leo_description is installed
if ros2 pkg list | grep -q "leo_description"; then
    echo "✓ Found official leo_description package"
    USE_OFFICIAL=true
else
    echo "✗ Official leo_description not found"
    echo "Installing leo_common_ros2..."
    
    cd ~/leo_ws/src
    if [ ! -d "leo_common_ros2" ]; then
        git clone https://github.com/LeoRover/leo_common_ros2.git -b ros2
    fi
    USE_OFFICIAL=true
fi

echo ""
echo "Rebuilding workspace..."
cd ~/leo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

echo ""
echo "✓ Build complete!"
echo ""
echo "To launch the simulation:"
echo "  source ~/leo_ws/install/setup.bash"
echo "  ros2 launch leo_lidar_sim complete_system.launch.py"
echo ""
echo "Tips to fix 'No map received':"
echo "1. Drive the robot around slowly with teleop"
echo "2. Wait a few seconds for SLAM to initialize"
echo "3. Check SLAM status: ros2 topic echo /map --once"
