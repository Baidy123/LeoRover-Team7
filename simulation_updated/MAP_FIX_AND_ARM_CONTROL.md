# FIX MAP + CONTROL ARM

## ISSUE 1: "No Map Received" - DIAGNOSIS & FIX

### Quick Diagnosis:

**In a new terminal, run these checks:**

```bash
source ~/leo_ws/install/setup.bash

# 1. Check if map topic exists
ros2 topic list | grep map
# Should show: /map

# 2. Check if map is publishing
ros2 topic hz /map
# Should show some rate (even slow like 0.5 Hz)

# 3. Check scan data
ros2 topic hz /scan
# Should be ~5-10 Hz

# 4. CRITICAL: Check TF chain
ros2 run tf2_ros tf2_echo map base_link
```

### If tf2_echo shows ERROR:

The TF chain is broken. Check which link is missing:

```bash
# Check each link:
ros2 run tf2_ros tf2_echo map odom
# Should work (SLAM publishes this)

ros2 run tf2_ros tf2_echo odom base_footprint
# If this FAILS - odom_to_tf.py not running!

ros2 run tf2_ros tf2_echo base_footprint base_link
# Should work (robot_state_publisher)
```

### FIX: If odom→base_footprint is missing:

**Option A: Check if odom_to_tf is running**

```bash
ros2 node list | grep odom_to_tf
```

If NOT there, it didn't start. Manually start it:

```bash
ros2 run leo_lidar_sim odom_to_tf.py --ros-args -p use_sim_time:=true
```

**Option B: Use static transform (temporary workaround)**

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint --ros-args -p use_sim_time:=true
```

**This will make the map work immediately!**

### After Fix:

1. **Drive the robot** with teleop for 10-15 seconds
2. **In RViz**, the map should start appearing
3. **Check**: `ros2 topic echo /map --once` - should show map data

### RViz Setup for Map:

If map still not visible in RViz:

1. Click **Add** → **Map**
2. Topic: `/map`
3. Color Scheme: `map`
4. Reliability Policy: `Reliable`
5. **Fixed Frame**: Must be `map` (not `odom`)

---

## ISSUE 2: CONTROL ARM MANUALLY

You can control the arm **right now** without any extra code!

### Method 1: Joint State Publisher (GUI - Easiest!)

```bash
# In a new terminal
source ~/leo_ws/install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

**What you'll see:**
- GUI window with sliders
- Sliders for `arm_segment1_joint` and `arm_segment2_joint`
- **Move sliders** → Arm moves in RViz!

**Note**: This controls the arm in RViz only, not in Gazebo (Gazebo needs joint controllers).

---

### Method 2: Command Line Control

**List available joints:**
```bash
ros2 topic echo /joint_states --once
```

**Publish joint positions:**
```bash
# Rotate base (joint 1)
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{name: ['arm_segment1_joint'], position: [1.57]}" --once

# Move shoulder (joint 2)  
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{name: ['arm_segment2_joint'], position: [0.5]}" --once

# Both at once
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{name: ['arm_segment1_joint', 'arm_segment2_joint'], position: [1.0, 0.8]}" --once
```

**Values:**
- `arm_segment1_joint`: -3.14 to 3.14 (full rotation)
- `arm_segment2_joint`: -1.5 to 1.5 (shoulder up/down)

---

### Method 3: Control Arm in Gazebo (Requires Setup)

To move arm in **Gazebo** (not just RViz), you need joint controllers.

**Quick test - Apply force to joint:**

In Gazebo GUI:
1. Right-click on arm → **Apply force/torque**
2. Select joint
3. Apply torque

**For programmatic control in Gazebo**, you need to add `ros2_control`:

I can create this if you want, but it requires:
- Adding `ros2_control` tags to URDF
- Joint controller configuration
- Controller manager

**Do you need Gazebo arm control?** Or is RViz + command line enough for now?

---

### Method 4: Create Simple Arm Control Script

**I can make you a script like:**

```python
#!/usr/bin/env python3
# arm_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
    
    def move_arm(self, joint1_angle, joint2_angle):
        msg = JointState()
        msg.name = ['arm_segment1_joint', 'arm_segment2_joint']
        msg.position = [joint1_angle, joint2_angle]
        self.pub.publish(msg)

# Usage:
# controller = ArmController()
# controller.move_arm(1.57, 0.5)  # 90 degrees rotation, 30 degrees up
```

---

## COMPLETE CHECKLIST:

### For Map:

```bash
# 1. Check TF chain
ros2 run tf2_ros tf2_echo map base_link
# Should show continuous updates

# 2. If broken, start odom_to_tf
ros2 run leo_lidar_sim odom_to_tf.py --ros-args -p use_sim_time:=true

# 3. Drive robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 4. Check map
ros2 topic hz /map
# Should update

# 5. In RViz, Fixed Frame = "map", Add Map display
```

### For Arm Control:

```bash
# Easiest - GUI with sliders
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Or command line
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"{name: ['arm_segment1_joint', 'arm_segment2_joint'], position: [0.0, 0.0]}" --once
```

---

## Quick Fix Summary:

**Map not working?**
```bash
# Run this in new terminal:
ros2 run leo_lidar_sim odom_to_tf.py --ros-args -p use_sim_time:=true
# Then drive robot - map will appear!
```

**Want to move arm?**
```bash
# Install if not installed:
sudo apt install ros-jazzy-joint-state-publisher-gui

# Run GUI:
ros2 run joint_state_publisher_gui joint_state_publisher_gui
# Move sliders → Arm moves!
```

Try these and let me know which works! 🎯
