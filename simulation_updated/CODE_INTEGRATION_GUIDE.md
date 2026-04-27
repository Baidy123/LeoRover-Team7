# Integrating Your Custom Navigation & Object Detection Code

This guide shows you how to add your autonomous navigation and color-based object detection code to the Leo Rover simulation.

## Environment Overview

The **sorting_room.sdf** world contains:
- **Room**: 5m x 5m with 4 walls
- **Colored Boxes** (small, 0.2m):
  - 2x Red boxes
  - 2x Green boxes  
  - 3x Blue boxes
- **Colored Baskets** (large, 0.5m):
  - Red basket (top-right corner: 2.0, 2.0)
  - Green basket (top-left corner: -2.0, 2.0)
  - Blue basket (bottom-left corner: -2.0, -2.0)
- **Obstacles**: 1 gray obstacle in center

**Task**: Navigate, detect colored boxes, pick them up, and place in matching colored basket.

---

## Step 1: Where to Put Your Code

### Option A: Python Nodes (Recommended if your code is Python)

```bash
cd ~/leo_ws/src/leo_lidar_sim/scripts/

# Copy your code files here
# For example:
cp ~/my_code/object_detector.py .
cp ~/my_code/path_planner.py .
cp ~/my_code/color_sorter.py .
```

### Option B: C++ Nodes (If your code is C++)

```bash
cd ~/leo_ws/src/leo_lidar_sim/src/

# Copy your .cpp files here
cp ~/my_code/object_detector.cpp .
cp ~/my_code/path_planner.cpp .
```

Then edit `CMakeLists.txt`:

```cmake
# Add your executables
add_executable(object_detector src/object_detector.cpp)
ament_target_dependencies(object_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  # Add your other dependencies
)

install(TARGETS
  object_detector
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## Step 2: Update Package Dependencies

Edit `package.xml` to add any extra dependencies your code needs:

```xml
<depend>cv_bridge</depend>  <!-- If using camera/vision -->
<depend>image_transport</depend>
<depend>opencv</depend>
<!-- Add whatever your code needs -->
```

---

## Step 3: Launch with Sorting Room

### Method 1: Use New World Directly

```bash
# Edit auto_start.sh, change world file line:
# OLD:
gz_args:="-r ~/leo_ws/src/leo_lidar_sim/worlds/leo_world.sdf"

# NEW:
gz_args:="-r ~/leo_ws/src/leo_lidar_sim/worlds/sorting_room.sdf"
```

### Method 2: Create Dedicated Launch File

Create `launch/sorting_mission.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_leo_lidar_sim = get_package_share_directory('leo_lidar_sim')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot State Publisher
    robot_state_pub = ExecuteProcess(
        cmd=['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
             '--ros-args', '-p', 'use_sim_time:=true',
             '-p', 'robot_description:=$(xacro /opt/ros/jazzy/share/leo_description/urdf/leo_sim.urdf.xacro)'],
        output='screen'
    )
    
    # Gazebo with sorting room
    gazebo = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
             f'gz_args:=-r {pkg_leo_lidar_sim}/worlds/sorting_room.sdf'],
        output='screen'
    )
    
    # Your custom nodes
    object_detector = Node(
        package='leo_lidar_sim',
        executable='object_detector.py',  # or your node name
        name='object_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    path_planner = Node(
        package='leo_lidar_sim',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    color_sorter = Node(
        package='leo_lidar_sim',
        executable='color_sorter.py',
        name='color_sorter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        robot_state_pub,
        gazebo,
        # Add delay, then spawn, bridges, etc...
        object_detector,
        path_planner,
        color_sorter,
    ])
```

---

## Step 4: Common Integration Points

### Your Code Probably Needs These Topics:

**INPUTS (subscribe to):**
- `/scan` - LiDAR data for obstacle avoidance
- `/odom` - Robot position/velocity
- `/camera/image_raw` - Camera for color detection (if using vision)
- `/map` - SLAM map for navigation
- `/tf` - Transform tree

**OUTPUTS (publish to):**
- `/cmd_vel` - Velocity commands to move robot
- `/detected_objects` - Your detected objects
- `/goal_pose` - Navigation goals
- `/pickup_target` - Which box to pick up

### Example Integration Pattern:

If your code has a main class like:

```python
class ObjectDetector:
    def __init__(self):
        # Your existing init code
        pass
    
    def detect_boxes(self, lidar_data):
        # Your existing detection logic
        pass
```

Wrap it as a ROS2 node:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from your_module import ObjectDetector  # Your existing code

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        
        # Initialize your existing detector
        self.detector = ObjectDetector()
        
        # ROS2 pub/sub
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.detections_pub = self.create_publisher(
            # Your message type
            DetectedObjects,
            '/detected_objects',
            10
        )
    
    def scan_callback(self, msg):
        # Call your existing detection code
        boxes = self.detector.detect_boxes(msg)
        
        # Publish results
        self.detections_pub.publish(boxes)

def main():
    rclpy.init()
    node = ObjectDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 5: Add Camera for Color Detection (Optional)

If your code needs camera to detect colors:

### Add Camera to Leo URDF

You need to add a camera sensor. Create `urdf/leo_with_camera.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:include filename="$(find leo_description)/urdf/leo.urdf.xacro"/>
  
  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.03 0.03 0.03"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.03 0.03 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.00001" iyy="0.00001" izz="0.00001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.12 0 0.08" rpy="0 0 0"/>
  </joint>
  
  <!-- Camera Sensor -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <topic>camera/image_raw</topic>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <visualize>true</visualize>
    </sensor>
  </gazebo>
  
</robot>
```

Then add camera bridge in your launch file:

```python
bridge_camera = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'],
    output='screen'
)
```

---

## Step 6: Test Your Integration

### Build:
```bash
cd ~/leo_ws
colcon build
source install/setup.bash
```

### Launch with sorting room:
```bash
# If you edited auto_start.sh:
./auto_start.sh

# Or use your custom launch:
ros2 launch leo_lidar_sim sorting_mission.launch.py
```

### Check your nodes are running:
```bash
ros2 node list
# Should show your nodes: object_detector, path_planner, etc.

# Check topics
ros2 topic list

# Test your node
ros2 topic echo /detected_objects
```

---

## Step 7: Typical Code Structure

```
leo_lidar_sim/
├── scripts/
│   ├── object_detector.py      # Your object detection
│   ├── path_planner.py         # Your navigation/path planning
│   ├── color_sorter.py         # Your color matching logic
│   └── mission_controller.py   # Main coordinator
├── launch/
│   └── sorting_mission.launch.py
└── config/
    └── sorting_params.yaml     # Your parameters
```

---

## Basket Positions for Your Code

```python
BASKET_POSITIONS = {
    'red': {'x': 2.0, 'y': 2.0},
    'green': {'x': -2.0, 'y': 2.0},
    'blue': {'x': -2.0, 'y': -2.0}
}
```

---

## Common Issues & Solutions

**Issue**: "Module not found"
```bash
# Make sure your scripts are executable
chmod +x ~/leo_ws/src/leo_lidar_sim/scripts/*.py

# Make sure they have shebang
# Add to top of .py file:
#!/usr/bin/env python3
```

**Issue**: "Package not found in CMakeLists"
```bash
# Rebuild
cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash
```

**Issue**: "Camera not publishing"
```bash
# Check if camera bridge is running
ros2 node list | grep bridge

# Check camera topic
ros2 topic list | grep camera
```

---

## Next Steps

1. ✅ Copy your code to scripts/ or src/
2. ✅ Update package.xml with dependencies
3. ✅ Update CMakeLists.txt (if C++)
4. ✅ Create launch file or modify auto_start.sh
5. ✅ Build and test
6. ✅ Run in sorting_room world!

Let me know which files you have and I'll help you integrate them! 🚀
