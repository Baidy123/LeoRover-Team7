# INTEGRATING YOUR CUSTOM CODE

## Step-by-Step Guide to Add Your Navigation & Object Detection Code

---

## STEP 1: Copy Your Code Files

### If Your Code is Python:

```bash
# Copy your files to scripts folder
cp /path/to/your/object_detector.py ~/leo_ws/src/leo_lidar_sim/scripts/
cp /path/to/your/path_planner.py ~/leo_ws/src/leo_lidar_sim/scripts/
cp /path/to/your/navigation.py ~/leo_ws/src/leo_lidar_sim/scripts/
cp /path/to/your/color_sorter.py ~/leo_ws/src/leo_lidar_sim/scripts/

# Make them executable
chmod +x ~/leo_ws/src/leo_lidar_sim/scripts/*.py
```

### If Your Code is C++:

```bash
# Copy to src folder
cp /path/to/your/detector.cpp ~/leo_ws/src/leo_lidar_sim/src/
cp /path/to/your/planner.cpp ~/leo_ws/src/leo_lidar_sim/src/
```

---

## STEP 2: Update CMakeLists.txt

### For Python Files:

```bash
nano ~/leo_ws/src/leo_lidar_sim/CMakeLists.txt
```

**Add your scripts to the install section:**

```cmake
# Install Python scripts
install(PROGRAMS
  scripts/simple_object_detector.py
  scripts/odom_to_tf.py
  scripts/camera_info_publisher.py
  scripts/scan_health_checker.py
  scripts/scan_corruptor.py
  scripts/simple_arm_controller.py
  scripts/YOUR_OBJECT_DETECTOR.py        # <-- Add this
  scripts/YOUR_PATH_PLANNER.py           # <-- Add this
  scripts/YOUR_NAVIGATION.py             # <-- Add this
  scripts/YOUR_COLOR_SORTER.py           # <-- Add this
  DESTINATION lib/${PROJECT_NAME}
)
```

### For C++ Files:

**Add dependencies (if needed):**

```cmake
find_package(cv_bridge REQUIRED)  # If using camera
find_package(image_transport REQUIRED)
# Add whatever your code needs
```

**Add executables:**

```cmake
# Your C++ nodes
add_executable(my_object_detector src/your_detector.cpp)
ament_target_dependencies(my_object_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  # Add your dependencies
)

# Install
install(TARGETS
  my_object_detector
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## STEP 3: Update package.xml (If You Have New Dependencies)

```bash
nano ~/leo_ws/src/leo_lidar_sim/package.xml
```

**Add any dependencies your code needs:**

```xml
<!-- If using camera -->
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>opencv</depend>

<!-- If using point clouds -->
<depend>pcl_ros</depend>

<!-- If using navigation -->
<depend>nav2_msgs</depend>

<!-- Add whatever YOUR code needs -->
```

---

## STEP 4: Rebuild Package

```bash
cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash
```

---

## STEP 5: Test Your Code

### Run Individual Nodes:

```bash
# Test your object detector
ros2 run leo_lidar_sim YOUR_OBJECT_DETECTOR.py

# Test your path planner
ros2 run leo_lidar_sim YOUR_PATH_PLANNER.py

# Test your navigation
ros2 run leo_lidar_sim YOUR_NAVIGATION.py
```

### Check Topics:

```bash
# See what topics your nodes subscribe to
ros2 node info /your_node_name

# Check if they're publishing
ros2 topic list
ros2 topic echo /your_output_topic
```

---

## STEP 6: Create Custom Launch File (Optional but Recommended)

Create `launch/my_sorting_mission.launch.py`:

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_leo_lidar_sim = get_package_share_directory('leo_lidar_sim')
    
    # Start complete system first
    complete_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leo_lidar_sim, 'launch', 'complete_system.launch.py')
        )
    )
    
    # Your custom nodes (start after 20 seconds to let system initialize)
    from launch.actions import TimerAction
    
    your_detector = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='leo_lidar_sim',
                executable='YOUR_OBJECT_DETECTOR.py',
                name='object_detector',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    your_planner = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='leo_lidar_sim',
                executable='YOUR_PATH_PLANNER.py',
                name='path_planner',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    your_sorter = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='leo_lidar_sim',
                executable='YOUR_COLOR_SORTER.py',
                name='color_sorter',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    return LaunchDescription([
        complete_system,
        your_detector,
        your_planner,
        your_sorter
    ])
```

**Then launch everything with:**

```bash
ros2 launch leo_lidar_sim my_sorting_mission.launch.py
```

---

## WHAT YOUR CODE CAN USE

### Available Topics:

```bash
# LiDAR for obstacle detection
/scan                    # sensor_msgs/LaserScan

# Odometry for position
/odom                    # nav_msgs/Odometry

# Camera (if you enabled it)
/camera/image_raw        # sensor_msgs/Image
/camera/camera_info      # sensor_msgs/CameraInfo

# Map from SLAM
/map                     # nav_msgs/OccupancyGrid

# Robot control
/cmd_vel                 # geometry_msgs/Twist (publish here to move robot)

# Arm control
/joint_states            # sensor_msgs/JointState (publish to move arm)
```

### Basket Positions (for your color sorting):

```python
BASKET_POSITIONS = {
    'red':   {'x': 2.0,  'y': 2.0},   # Top-right
    'green': {'x': -2.0, 'y': 2.0},   # Top-left  
    'blue':  {'x': -2.0, 'y': -2.0}   # Bottom-left
}
```

### Example Integration Patterns:

**If your code has a class like:**

```python
class ObjectDetector:
    def __init__(self):
        # Your existing code
        pass
    
    def detect_objects(self, scan_data):
        # Your existing logic
        return detected_objects
```

**Wrap it as ROS2 node:**

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from your_module import ObjectDetector  # Your existing code

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        
        # Use your existing detector
        self.detector = ObjectDetector()
        
        # ROS2 interface
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        
        self.detections_pub = self.create_publisher(
            # Your message type
            YourDetectionMsg, '/detected_objects', 10
        )
    
    def scan_callback(self, msg):
        # Call YOUR code
        objects = self.detector.detect_objects(msg)
        
        # Publish results
        self.detections_pub.publish(objects)

def main():
    rclpy.init()
    node = ObjectDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## COMMON SCENARIOS

### Scenario 1: Your Code Needs Camera

**Enable camera bridge:**

```bash
# Terminal 3 (after complete_system is running):
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image

# Terminal 4:
ros2 run leo_lidar_sim camera_info_publisher.py
```

**In your code:**

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class YourDetector(Node):
    def __init__(self):
        super().__init__('detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Your OpenCV/detection code here
```

### Scenario 2: Your Code Controls Robot Movement

**Publish to /cmd_vel:**

```python
from geometry_msgs.msg import Twist

class YourNavigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def move_forward(self, speed=0.2):
        msg = Twist()
        msg.linear.x = speed
        self.cmd_pub.publish(msg)
    
    def turn_left(self, angular_speed=0.5):
        msg = Twist()
        msg.angular.z = angular_speed
        self.cmd_pub.publish(msg)
```

### Scenario 3: Your Code Picks Up Objects

**Control arm and move to basket:**

```python
from sensor_msgs.msg import JointState

class YourSorter(Node):
    def __init__(self):
        super().__init__('sorter')
        self.arm_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    def pick_object(self):
        # Lower arm
        self.move_arm(0.0, -0.5)
        # Close gripper (if you add gripper control)
        # Raise arm
        self.move_arm(0.0, 0.5)
    
    def move_arm(self, base_angle, shoulder_angle):
        msg = JointState()
        msg.name = ['arm_segment1_joint', 'arm_segment2_joint']
        msg.position = [base_angle, shoulder_angle]
        self.arm_pub.publish(msg)
    
    def navigate_to_basket(self, color):
        positions = {
            'red': (2.0, 2.0),
            'green': (-2.0, 2.0),
            'blue': (-2.0, -2.0)
        }
        x, y = positions[color]
        # Your navigation logic to reach (x, y)
```

---

## DEBUGGING YOUR CODE

```bash
# Check if your node is running
ros2 node list

# Check what topics it's using
ros2 node info /your_node_name

# Monitor published data
ros2 topic echo /your_topic_name

# Check for errors
ros2 topic echo /rosout | grep ERROR

# Visualize in RViz
# Add → By topic → Select your topic
```

---

## TESTING CHECKLIST

Before running your autonomous system:

✅ **Manual control works** (teleop)
✅ **SLAM builds map** (drive around)
✅ **LiDAR data flowing** (`ros2 topic hz /scan`)
✅ **Camera working** (if using - `ros2 topic hz /camera/image_raw`)
✅ **Your nodes start** without errors
✅ **Your nodes subscribe** to correct topics
✅ **Your nodes publish** on correct topics
✅ **Robot responds** to your /cmd_vel commands
✅ **Arm responds** to your /joint_states commands

---

## QUICK START TEMPLATE

**Minimal structure for your code:**

```bash
leo_lidar_sim/
├── scripts/
│   ├── my_detector.py         # Your object detection
│   ├── my_planner.py          # Your path planning
│   ├── my_controller.py       # Your main controller
│   └── my_config.py           # Your parameters
├── launch/
│   └── my_mission.launch.py   # Launch everything
└── config/
    └── my_params.yaml         # Your configuration
```

**Then:**
1. Update CMakeLists.txt
2. Rebuild: `colcon build`
3. Launch: `ros2 launch leo_lidar_sim my_mission.launch.py`

---

## NEED HELP?

**Tell me:**
1. What language is your code? (Python/C++)
2. What does it do? (object detection, navigation, both?)
3. What topics/sensors does it need?
4. Any special dependencies?

And I'll create the exact integration files you need! 🚀
