# Leo Rover LiDAR Simulation - ROS2 Jazzy

Complete simulation package for Leo Rover with LiDAR for SLAM, navigation, and object detection in Gazebo Harmonic.

## 📋 Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Package Structure](#package-structure)
- [Quick Start](#quick-start)
- [Detailed Usage](#detailed-usage)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)

## ✨ Features

- **Complete Leo Rover Model** with differential drive
- **360° LiDAR Sensor** (10m range, 360 samples)
- **Gazebo Harmonic** simulation with obstacles
- **SLAM Toolbox** for mapping
- **Nav2** for autonomous navigation
- **Object Detection** from LiDAR data
- **RViz2** visualization

## 🔧 Prerequisites

### System Requirements
- Ubuntu 24.04 (Noble)
- ROS2 Jazzy Jalisco
- Gazebo Harmonic

### Install ROS2 Jazzy
```bash
# Add ROS2 apt repository
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### Install Gazebo Harmonic
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image
```

### Install Navigation and SLAM
```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2
```

### Install Additional Dependencies
```bash
sudo apt install -y \
  python3-pip \
  python3-numpy \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions
```

## 📦 Installation

### 1. Create Workspace
```bash
mkdir -p ~/leo_ws/src
cd ~/leo_ws/src
```

### 2. Copy Package
```bash
# Copy the leo_lidar_sim package to your workspace
cp -r /path/to/leo_lidar_sim ~/leo_ws/src/
```

### 3. Install Dependencies
```bash
cd ~/leo_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build Package
```bash
cd ~/leo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

### 5. Source Workspace
```bash
source ~/leo_ws/install/setup.bash
```

**Add to ~/.bashrc for convenience:**
```bash
echo "source ~/leo_ws/install/setup.bash" >> ~/.bashrc
```

## 📁 Package Structure

```
leo_lidar_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── nav2_params.yaml      # Nav2 configuration
│   └── slam_toolbox.yaml     # SLAM parameters
├── launch/
│   ├── sim.launch.py         # Main simulation
│   ├── slam.launch.py        # SLAM only
│   ├── navigation.launch.py  # Navigation only
│   ├── object_detection.launch.py  # Object detection
│   └── complete_system.launch.py   # Full system
├── urdf/
│   └── leo_rover.urdf.xacro  # Robot description
├── worlds/
│   └── leo_world.sdf         # Gazebo world
├── rviz/
│   └── leo_sim.rviz          # RViz config
├── maps/
│   └── map.yaml              # Map file
├── src/
│   └── lidar_object_detector.cpp  # C++ detector
└── scripts/
    └── simple_object_detector.py  # Python detector
```

## 🚀 Quick Start

### Option 1: Complete System (Simulation + SLAM)
```bash
ros2 launch leo_lidar_sim complete_system.launch.py
```

### Option 2: Step-by-Step

**Terminal 1 - Launch Simulation:**
```bash
ros2 launch leo_lidar_sim sim.launch.py
```

**Terminal 2 - Launch SLAM:**
```bash
ros2 launch leo_lidar_sim slam.launch.py
```

**Terminal 3 - Launch Object Detection:**
```bash
ros2 launch leo_lidar_sim object_detection.launch.py
```

## 📖 Detailed Usage

### 1. Build a Map with SLAM

**Start simulation + SLAM:**
```bash
ros2 launch leo_lidar_sim complete_system.launch.py
```

**Control the robot manually to explore:**
```bash
# Keyboard teleop (install if needed)
sudo apt install ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Save the map:**
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'my_map'"
```

The map will be saved in your current directory.

### 2. Autonomous Navigation

**After building a map, launch navigation:**
```bash
# Copy your saved map to the maps directory
cp my_map.yaml ~/leo_ws/src/leo_lidar_sim/maps/map.yaml
cp my_map.pgm ~/leo_ws/src/leo_lidar_sim/maps/map.pgm

# Rebuild to include new map
cd ~/leo_ws
colcon build

# Launch navigation with your map
ros2 launch leo_lidar_sim navigation.launch.py
```

**Set navigation goals in RViz:**
1. Click "2D Pose Estimate" button
2. Click on map to set initial pose
3. Click "Nav2 Goal" button
4. Click on map to set goal pose

### 3. Object Detection

The object detection node is already included in `complete_system.launch.py`.

**To run standalone:**
```bash
ros2 launch leo_lidar_sim object_detection.launch.py
```

**View detected objects:**
- Check `/detected_objects` topic (PoseArray)
- Check `/object_markers` topic (MarkerArray) - visualized in RViz

**Test detection:**
```bash
# List detected objects
ros2 topic echo /detected_objects

# Check marker visualization
ros2 topic echo /object_markers
```

## ⚙️ Configuration

### Adjust Robot Parameters

Edit `urdf/leo_rover.urdf.xacro`:
```xml
<!-- Wheel properties -->
<xacro:property name="wheel_radius" value="0.0625"/>
<xacro:property name="wheel_separation" value="0.358"/>

<!-- LiDAR range -->
<range>
  <min>0.12</min>
  <max>10.0</max>  <!-- Change max range -->
</range>
```

### Adjust SLAM Parameters

Edit `config/slam_toolbox.yaml`:
```yaml
slam_toolbox:
  ros__parameters:
    resolution: 0.05  # Map resolution
    max_laser_range: 10.0  # Max LiDAR range for SLAM
    minimum_travel_distance: 0.2  # Min distance to trigger update
```

### Adjust Navigation Parameters

Edit `config/nav2_params.yaml`:
```yaml
controller_server:
  ros__parameters:
    max_vel_x: 0.5  # Maximum linear velocity
    max_vel_theta: 1.0  # Maximum angular velocity
```

### Adjust Object Detection

Edit `launch/object_detection.launch.py`:
```python
parameters=[{
    'min_cluster_size': 3,        # Minimum points per object
    'max_cluster_size': 100,      # Maximum points per object
    'cluster_tolerance': 0.3,     # Clustering distance threshold
    'min_object_distance': 0.2,   # Ignore objects closer than this
    'max_object_distance': 5.0    # Ignore objects farther than this
}]
```

## 🛠️ Troubleshooting

### Gazebo doesn't start
```bash
# Check Gazebo installation
gz sim --version

# Try launching Gazebo separately
gz sim -r leo_world.sdf
```

### No LiDAR data
```bash
# Check if scan topic is publishing
ros2 topic list | grep scan
ros2 topic echo /scan

# Check ROS-Gazebo bridge
ros2 node list | grep bridge
```

### Robot doesn't move
```bash
# Check cmd_vel topic
ros2 topic list | grep cmd_vel

# Test manual control
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

### SLAM not working
```bash
# Check SLAM node
ros2 node list | grep slam

# Check TF tree
ros2 run tf2_tools view_frames

# Verify odometry
ros2 topic echo /odom
```

### Navigation fails
```bash
# Check Nav2 nodes
ros2 node list | grep nav2

# Check costmaps
ros2 topic echo /local_costmap/costmap
ros2 topic echo /global_costmap/costmap

# Verify map
ros2 topic echo /map
```

### Build errors
```bash
# Clean and rebuild
cd ~/leo_ws
rm -rf build/ install/ log/
colcon build --symlink-install

# Check dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## 📊 Topics Reference

### Published Topics
- `/scan` - LaserScan data
- `/odom` - Odometry
- `/map` - SLAM generated map
- `/detected_objects` - Detected object poses
- `/object_markers` - Visualization markers
- `/tf` - Transform tree

### Subscribed Topics
- `/cmd_vel` - Velocity commands
- `/initialpose` - Initial pose for localization
- `/goal_pose` - Navigation goals

## 🎮 Keyboard Controls

When using teleop_twist_keyboard:
```
Moving around:
   u    i    o
   j    k    l
   m    ,    .

u/o: turn left/right while moving forward
j/l: turn left/right
i/,: move forward/backward
k: stop

q/z: increase/decrease max speeds by 10%
w/x: increase/decrease only linear speed by 10%
e/c: increase/decrease only angular speed by 10%
```

## 📝 Next Steps

1. **Tune SLAM parameters** for your environment
2. **Adjust Nav2 parameters** for better navigation
3. **Improve object detection** with filtering and tracking
4. **Add more sensors** (camera, IMU)
5. **Implement object avoidance** using detected objects
6. **Create waypoint missions** for autonomous operation

## 📚 Additional Resources

- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/getstarted)
- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Leo Rover Official](https://www.leorover.tech/)

## 🤝 Contributing

Feel free to improve this package:
1. Add more sophisticated object detection
2. Implement object tracking
3. Add more sensors
4. Improve world complexity
5. Add AI-based navigation

## 📄 License

MIT License

## 👤 Author

Created for ROS2 Jazzy with Gazebo Harmonic

---

**Happy Simulating! 🚀**
