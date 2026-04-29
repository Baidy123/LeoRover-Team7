# Leo Rover Simulation - Quick Reference Card

## 🚀 Quick Start Commands

### Complete System Launch
```bash
ros2 launch leo_lidar_sim complete_system.launch.py
```

### Individual Components
```bash
# Simulation only
ros2 launch leo_lidar_sim sim.launch.py

# SLAM only
ros2 launch leo_lidar_sim slam.launch.py

# Navigation only (requires pre-built map)
ros2 launch leo_lidar_sim navigation.launch.py

# Object detection
ros2 launch leo_lidar_sim object_detection.launch.py
```

## 🎮 Robot Control

### Keyboard Teleoperation
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Manual Command
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

## 🗺️ Mapping

### Save Map
```bash
# Save current SLAM map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'my_map'"

# Map will be saved as my_map.yaml and my_map.pgm in current directory
```

### Use Saved Map
```bash
# Copy to package maps directory
cp my_map.* ~/leo_ws/src/leo_lidar_sim/maps/

# Rebuild package
cd ~/leo_ws && colcon build

# Launch with map
ros2 launch leo_lidar_sim navigation.launch.py map:=my_map.yaml
```

## 🔍 Debugging Commands

### List All Topics
```bash
ros2 topic list
```

### Monitor Specific Topics
```bash
# LiDAR data
ros2 topic echo /scan

# Odometry
ros2 topic echo /odom

# Detected objects
ros2 topic echo /detected_objects

# Map
ros2 topic echo /map
```

### Check Topic Info
```bash
ros2 topic info /scan
ros2 topic hz /scan     # Check publishing rate
```

### List All Nodes
```bash
ros2 node list
```

### Check Node Info
```bash
ros2 node info /slam_toolbox
```

### View TF Tree
```bash
ros2 run tf2_tools view_frames
# Creates frames.pdf in current directory
```

### Check Parameters
```bash
# List parameters of a node
ros2 param list /slam_toolbox

# Get specific parameter
ros2 param get /slam_toolbox resolution

# Set parameter
ros2 param set /slam_toolbox resolution 0.05
```

## 📊 Monitoring

### RQT Tools
```bash
# Graph of nodes and topics
rqt_graph

# All RQT plugins
rqt
```

### System Monitor
```bash
# Monitor system resources
ros2 run rqt_top rqt_top
```

## 🛠️ Common Fixes

### Restart Gazebo
```bash
# Kill all Gazebo processes
killall -9 gz ruby

# Relaunch
ros2 launch leo_lidar_sim sim.launch.py
```

### Reset Simulation
```bash
# Stop all ROS2 nodes
# Ctrl+C in all terminals

# Clean workspace (if needed)
cd ~/leo_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

### Check Gazebo Status
```bash
gz sim --version
gz topic -l
```

## 📝 File Locations

```
~/leo_ws/src/leo_lidar_sim/
├── config/           # Parameter files
├── launch/           # Launch files
├── urdf/             # Robot description
├── worlds/           # Gazebo worlds
├── rviz/             # RViz configurations
├── maps/             # Saved maps
├── src/              # C++ source code
└── scripts/          # Python scripts
```

## 🎯 Navigation in RViz

1. **Set Initial Pose:**
   - Click "2D Pose Estimate" button in RViz
   - Click and drag on map to set robot's initial position and orientation

2. **Set Navigation Goal:**
   - Click "Nav2 Goal" (or "2D Nav Goal") button
   - Click and drag on map to set goal position and orientation

3. **Monitor:**
   - Green path = global plan
   - Red path = local plan
   - Robot should navigate autonomously

## 💡 Tips

- **SLAM works best when:**
  - Robot moves slowly
  - Environment has distinct features
  - You complete loop closures

- **Navigation works best when:**
  - Good quality map
  - Clear initial pose estimate
  - Reasonable velocity limits

- **Object detection works best when:**
  - Objects are within LiDAR range (0.2-5m)
  - Objects have distinct boundaries
  - Proper cluster parameters

## 📞 Get Help

- Check README.md for detailed documentation
- ROS2 Jazzy Docs: https://docs.ros.org/en/jazzy/
- Nav2 Docs: https://navigation.ros.org/
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox

## 🔧 Environment Setup

Add to ~/.bashrc:
```bash
source /opt/ros/jazzy/setup.bash
source ~/leo_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/leo_ws/src/leo_lidar_sim/models
```

Then:
```bash
source ~/.bashrc
```
