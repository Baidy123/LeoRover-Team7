# AUTONOMOUS NAVIGATION - COMPLETE GUIDE

## IMPORTANT: What You Already Have! ✅

Your `complete_system.launch.py` ALREADY includes:
- ✅ **SLAM** (slam_toolbox) - Builds map
- ✅ **Odometry** (/odom) - Robot position
- ✅ **LiDAR** (/scan) - Obstacle detection
- ✅ **Manual control** (teleop)

## What's MISSING for Autonomous Navigation: ❌

- ❌ **Nav2** - This makes the robot navigate autonomously!

**Nav2 is the KEY!** Without it, you can only drive manually with teleop.

---

## UNDERSTANDING: Manual vs Autonomous

### What You Have Now (Manual):
```
You press keys → teleop → /cmd_vel → Robot moves
```
**You control the robot**

### What Nav2 Adds (Autonomous):
```
You set goal → Nav2 plans path → Nav2 sends /cmd_vel → Robot moves autonomously
```
**Robot controls itself!**

---

## YOUR FILES - WHERE TO PUT THEM

Your files reference `leo_nav2` package, but you don't need a separate package!
**Put everything in `leo_lidar_sim`** (the package you already have).

### Step 1: Copy Config Files

```bash
# The config files are ALREADY copied by me, but to be sure:
cp ~/Downloads/nav2_params.yaml ~/leo_ws/src/leo_lidar_sim/config/
cp ~/Downloads/ekf.yaml ~/leo_ws/src/leo_lidar_sim/config/
cp ~/Downloads/slam_toolbox.yaml ~/leo_ws/src/leo_lidar_sim/config/

# Don't need slam_launch.py - you already have SLAM running!
```

---

## SETUP: Install Nav2

```bash
# Install Nav2 packages
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs \
  ros-jazzy-navigation2 ros-jazzy-nav2-util -y
```

---

## OPTION 1: SIMPLEST - Add Nav2 to Your Launch (RECOMMENDED)

I'll create an updated `complete_system.launch.py` that includes Nav2!

### Update Your Package:

Download the new package I'm creating, or manually add Nav2 to your launch file.

---

## OPTION 2: Run Nav2 Separately (3 Terminals)

### Terminal 1: Complete System (As Before)
```bash
source ~/leo_ws/install/setup.bash
ros2 launch leo_lidar_sim complete_system.launch.py
```

**Wait 20 seconds** for system to start.

### Terminal 2: Build Map First (Manual Driving)
```bash
source ~/leo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Drive around for 2-3 minutes** to build a good map.

### Terminal 3: Launch Nav2 (AFTER map is built)
```bash
source ~/leo_ws/install/setup.bash

# Use the standard Nav2 launch
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=~/leo_ws/src/leo_lidar_sim/config/nav2_params.yaml
```

---

## HOW TO MAKE ROBOT MOVE AUTONOMOUSLY

Once Nav2 is running (Terminal 3), you can send goals:

### Method 1: RViz (Easiest!)

In RViz:
1. Click **"2D Goal Pose"** button (top toolbar)
2. Click on map where you want robot to go
3. Drag arrow to set direction
4. **Robot navigates autonomously!** 🚀

### Method 2: Command Line

```bash
# Navigate to red basket (2.0, 2.0)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: 2.0, z: 0.0},
      orientation: {w: 1.0}
    }
  }
}"
```

### Method 3: Python Script (For Your Code!)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y, theta=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert angle to quaternion
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal: ({x}, {y})')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        return send_goal_future

def main():
    rclpy.init()
    navigator = NavigationClient()
    
    # Go to red basket
    navigator.send_goal(2.0, 2.0)
    
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## COMPLETE AUTONOMOUS SORTING WORKFLOW

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np

class AutonomousSorter(Node):
    def __init__(self):
        super().__init__('autonomous_sorter')
        
        # Navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Camera
        self.bridge = CvBridge()
        self.rgb_sub = self.create_subscription(
            Image, '/realsense/color/image', self.camera_callback, 10
        )
        
        # Arm control
        self.arm_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Baskets
        self.baskets = {
            'red': (2.0, 2.0),
            'green': (-2.0, 2.0),
            'blue': (-2.0, -2.0)
        }
        
        self.current_image = None
    
    def camera_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def detect_color_box(self):
        if self.current_image is None:
            return None
        
        hsv = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2HSV)
        
        # Detect red (example)
        red_mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            return 'red'
        return None
    
    def navigate_to(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'Navigating to ({x}, {y})')
        future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        # Wait for result
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
    
    def pick_up_box(self):
        # Lower arm
        msg = JointState()
        msg.name = ['arm_segment1_joint', 'arm_segment2_joint']
        msg.position = [0.0, -0.5]  # Reach down
        self.arm_pub.publish(msg)
        
        self.get_logger().info('Picking up box')
        import time
        time.sleep(2)  # Wait for arm to move
        
        # Raise arm
        msg.position = [0.0, 0.5]
        self.arm_pub.publish(msg)
    
    def run_sorting_mission(self):
        # 1. Detect box color
        color = self.detect_color_box()
        if not color:
            self.get_logger().info('No box detected')
            return
        
        self.get_logger().info(f'Detected {color} box')
        
        # 2. Navigate to box (you'd need to calculate this from camera)
        # For demo, assume box is at (1.0, 0.0)
        self.navigate_to(1.0, 0.0)
        
        # 3. Pick up box
        self.pick_up_box()
        
        # 4. Navigate to matching basket
        basket_x, basket_y = self.baskets[color]
        self.navigate_to(basket_x, basket_y)
        
        # 5. Drop box
        msg = JointState()
        msg.name = ['arm_segment1_joint', 'arm_segment2_joint']
        msg.position = [0.0, -0.5]
        self.arm_pub.publish(msg)
        
        self.get_logger().info(f'{color} box delivered!')

def main():
    rclpy.init()
    sorter = AutonomousSorter()
    
    # Wait for camera
    import time
    time.sleep(5)
    
    # Run mission
    sorter.run_sorting_mission()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## VERIFY IT'S WORKING

### Check Nav2 is running:
```bash
ros2 node list | grep nav2
# Should see: controller_server, planner_server, bt_navigator, etc.
```

### Check Nav2 topics:
```bash
ros2 topic list | grep nav
# Should see: /navigate_to_pose, /plan, /local_costmap, etc.
```

### Send test goal:
```bash
# Simple test - go 1 meter forward
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}}}}"
```

---

## TROUBLESHOOTING

### "Nav2 not found"
```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-navigation2
```

### "No path found"
- Map might not be complete - drive around more with teleop
- Goal might be in obstacle - try different position
- Check costmaps in RViz

### "Robot doesn't move"
```bash
# Check if Nav2 is sending commands
ros2 topic echo /cmd_vel

# If nothing, check Nav2 logs
ros2 node list | grep nav2
```

---

## SUMMARY - WHAT YOU NEED TO DO

### ✅ You Already Have:
- SLAM (builds map)
- Odometry (tracks position)
- LiDAR (sees obstacles)
- Camera (detects colors)
- Arm (picks boxes)

### ❌ You Need to Add:
- **Nav2** (autonomous navigation)

### How to Add Nav2:

**Quick way:**
```bash
# Terminal 3 (after complete_system is running)
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=~/leo_ws/src/leo_lidar_sim/config/nav2_params.yaml
```

### Then Send Goals:
- Use RViz "2D Goal Pose" button, OR
- Use Python script with NavigateToPose action

**That's it! Nav2 = Autonomous navigation!** 🚀

---

## DO YOU NEED EKF?

**No, not for simulation!** 

EKF is only for real hardware with IMU sensor. In simulation, odometry is already perfect.

**Skip EKF for now!**

---

## FINAL WORKFLOW

```bash
# Terminal 1: System + SLAM
ros2 launch leo_lidar_sim complete_system.launch.py

# Terminal 2: Build map (manual)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Drive 2-3 minutes

# Terminal 3: Nav2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true \
  params_file:=~/leo_ws/src/leo_lidar_sim/config/nav2_params.yaml

# Then: Click "2D Goal Pose" in RViz = Robot moves autonomously!
```

That's everything you need! 🎯
