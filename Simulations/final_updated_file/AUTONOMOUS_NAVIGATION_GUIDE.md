# AUTONOMOUS NAVIGATION SETUP

## What You Have Now:

✅ **SLAM** - Already running in complete_system.launch.py (builds map)
✅ **Manual control** - Teleop keyboard
✅ **Arm control** - Manual joint control

## What's Missing for AUTONOMOUS Navigation:

❌ **Nav2** - Path planning and autonomous navigation
❌ **EKF** (Optional) - Sensor fusion for better odometry

---

## Understanding the Components:

### 1. SLAM (Already Have This!)
- **What**: Builds map while robot moves
- **Status**: ✅ Running in `complete_system.launch.py`
- **You see**: Map building in RViz as you drive

### 2. Nav2 (Need to Add)
- **What**: Makes robot navigate autonomously
  - Path planning (finds path to goal)
  - Obstacle avoidance (avoids obstacles)
  - Goal execution (drives to waypoint)
- **Status**: ❌ Not running yet
- **You need**: This to make robot move on its own!

### 3. EKF (Optional - for real hardware)
- **What**: Fuses IMU + odometry for better position estimate
- **Status**: ❌ Not needed in simulation (no IMU in sim)
- **When to use**: Only with real Leo Rover hardware

---

## SETUP: Add Autonomous Navigation

### Step 1: Install Nav2 (if not already installed)

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs \
  ros-jazzy-robot-localization -y
```

### Step 2: Update Package Dependencies

Add to `~/leo_ws/src/leo_lidar_sim/package.xml`:

```xml
<depend>nav2_bringup</depend>
<depend>nav2_msgs</depend>
<depend>robot_localization</depend>
```

### Step 3: Rebuild

```bash
cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash
```

---

## HOW TO USE: Autonomous Navigation

### Terminal 1: Complete System (As Before)

```bash
source ~/leo_ws/install/setup.bash
ros2 launch leo_lidar_sim complete_system.launch.py
```

**Wait 20 seconds** for everything to start.

### Terminal 2: Build Map First (Drive Around Manually)

```bash
source ~/leo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Drive around the room for 2-3 minutes** to build a complete map.

### Terminal 3: Launch Nav2 (After Map is Built)

```bash
source ~/leo_ws/install/setup.bash
ros2 launch leo_lidar_sim nav2.launch.py
```

**Wait ~10 seconds** for Nav2 to initialize.

---

## SEND NAVIGATION GOALS

### Method 1: RViz (Easiest - Visual)

In RViz:
1. Click **"2D Goal Pose"** button (top toolbar)
2. Click on map where you want robot to go
3. Drag to set direction
4. **Robot moves autonomously!** 🚀

### Method 2: Command Line

```bash
# Send goal to position (x, y) with orientation
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 2.0, z: 0.0},
    orientation: {w: 1.0}
  }
}"
```

### Method 3: Python Script (For Your Code)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NavigationGoal(Node):
    def __init__(self):
        super().__init__('navigation_goal')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
    
    def send_goal(self, x, y, theta=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # Convert theta to quaternion (simplified for 2D)
        import math
        goal.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.orientation.w = math.cos(theta / 2.0)
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Sent goal: ({x}, {y})')

# Usage:
# navigator = NavigationGoal()
# navigator.send_goal(2.0, 2.0)  # Go to red basket
```

---

## EXAMPLE: Navigate to Colored Baskets

### Basket Positions:

```python
BASKETS = {
    'red':   (2.0,  2.0),   # Top-right
    'green': (-2.0, 2.0),   # Top-left
    'blue':  (-2.0, -2.0)   # Bottom-left
}
```

### Complete Navigation Script:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class BasketNavigator(Node):
    def __init__(self):
        super().__init__('basket_navigator')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.baskets = {
            'red':   (2.0,  2.0),
            'green': (-2.0, 2.0),
            'blue':  (-2.0, -2.0)
        }
    
    def go_to_basket(self, color):
        if color not in self.baskets:
            self.get_logger().error(f'Unknown color: {color}')
            return
        
        x, y = self.baskets[color]
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Navigating to {color} basket at ({x}, {y})')

def main():
    rclpy.init()
    navigator = BasketNavigator()
    
    # Example: Visit all baskets
    colors = ['red', 'green', 'blue']
    
    for color in colors:
        navigator.go_to_basket(color)
        time.sleep(15)  # Wait for robot to reach goal
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## MONITORING Navigation

```bash
# Check Nav2 status
ros2 topic list | grep nav

# Check if goal was received
ros2 topic echo /goal_pose

# Check current plan
ros2 topic echo /plan

# Check robot velocity commands from Nav2
ros2 topic echo /cmd_vel
```

---

## VISUALIZE in RViz

Add these displays:
1. **Map** - Shows SLAM map
2. **Path** - Topic: `/plan` (shows planned path)
3. **Global Costmap** - Topic: `/global_costmap/costmap`
4. **Local Costmap** - Topic: `/local_costmap/costmap`
5. **RobotModel** - Shows robot + arm

---

## COMPLETE WORKFLOW

### 1. Start System
```bash
ros2 launch leo_lidar_sim complete_system.launch.py
```

### 2. Build Map (Manual)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Drive around for 2-3 minutes
```

### 3. Start Nav2
```bash
ros2 launch leo_lidar_sim nav2.launch.py
```

### 4. Send Goals
- Use RViz "2D Goal Pose" button
- Or run your Python navigation script
- Robot navigates autonomously!

---

## TROUBLESHOOTING

**Nav2 not starting:**
```bash
# Install missing packages
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-nav2-msgs
```

**Robot doesn't move to goal:**
```bash
# Check Nav2 is running
ros2 node list | grep nav

# Check if goal was received
ros2 topic echo /goal_pose --once

# Check costmaps in RViz - make sure path is clear
```

**"No path found" error:**
- Map might not be complete
- Goal might be in obstacle
- Try closer goal first

**Robot moves erratically:**
- Tune `nav2_params.yaml` velocities
- Check costmap inflation radius

---

## YOUR AUTONOMOUS SORTING MISSION

Combine:
1. ✅ **Camera** - Detect colored boxes
2. ✅ **Nav2** - Navigate to box/basket
3. ✅ **Arm** - Pick up box
4. ✅ **SLAM** - Build/use map

**Workflow:**
1. Detect red box with camera
2. Navigate to box position
3. Lower arm and grab
4. Navigate to red basket
5. Release box
6. Repeat for other colors!

---

## SUMMARY

**What You Had:**
- SLAM + manual control

**What You Added:**
- Nav2 for autonomous navigation

**How to Use:**
1. Launch complete_system
2. Build map (drive manually)
3. Launch nav2
4. Send goals (RViz or code)
5. Robot navigates autonomously! 🎉

---

## NEXT STEPS

Your uploaded files are now integrated. To use:

```bash
cd ~/leo_ws
colcon build
source install/setup.bash

# Terminal 1
ros2 launch leo_lidar_sim complete_system.launch.py

# Terminal 2 (after map built)
ros2 launch leo_lidar_sim nav2.launch.py

# Then send goals!
```
