# Leo Rover with Camera and myCobot Arm

## What's Added:

### 1. **Camera** (Front-facing Fisheye)
- Resolution: 640x480
- FOV: ~110 degrees (fisheye effect)
- Topic: `/camera/image_raw`
- Mounted on front of Leo rover

### 2. **myCobot 280 Arm** (Simplified)
- 3 DOF arm (base rotation, shoulder, elbow)
- 2-finger gripper
- Mounted on top center of rover
- Can grab small boxes

---

## Quick Test - Camera Only (Easiest)

If you just want to add the camera to your existing setup:

### Add Camera Bridge to Your Current Commands:

In **Terminal 5** (where you run bridges), add this line:

```bash
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image &
```

### View Camera Feed:

```bash
# Option 1: RQT Image View
ros2 run rqt_image_view rqt_image_view

# Select topic: /camera/image_raw

# Option 2: Command line
ros2 run image_view image_view --ros-args -r image:=/camera/image_raw
```

**BUT**: The official Leo description might not have a camera sensor configured in Gazebo. If you don't see images, you need to use the custom URDF.

---

## Full Setup - Camera + myCobot Arm

### Step 1: Update Your Robot Description

You need to use the custom URDF that includes camera and arm.

**Edit your Terminal 1 command** to use the custom URDF:

```bash
source ~/leo_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(xacro ~/leo_ws/src/leo_lidar_sim/urdf/leo_with_arm_camera.urdf.xacro)"
```

### Step 2: Use the Complete Launch File

**OR** use the automated launch file:

```bash
# Terminal 1 - Robot State with custom URDF
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(xacro ~/leo_ws/src/leo_lidar_sim/urdf/leo_with_arm_camera.urdf.xacro)"

# Terminal 2 - Everything else
cd ~/leo_ws
source install/setup.bash
ros2 launch leo_lidar_sim leo_with_camera.launch.py
```

### Step 3: View Camera Feed

```bash
# In a new terminal
ros2 run rqt_image_view rqt_image_view
# Select: /camera/image_raw
```

### Step 4: Control the Arm

```bash
# List arm joints
ros2 topic list | grep joint

# Control arm joint 1 (base rotation)
ros2 topic pub /arm_joint1_controller/command std_msgs/msg/Float64 "{data: 1.57}" --once

# Control arm joint 2 (shoulder)
ros2 topic pub /arm_joint2_controller/command std_msgs/msg/Float64 "{data: 0.5}" --once

# Open/close gripper
ros2 topic pub /gripper_left_joint_controller/command std_msgs/msg/Float64 "{data: 0.03}" --once
```

---

## Camera Topics Available:

```bash
# Raw image
/camera/image_raw

# Camera info (calibration)
/camera/camera_info
```

---

## Arm Joints:

- `arm_joint1` - Base rotation (360°)
- `arm_joint2` - Shoulder (-90° to 90°)
- `arm_joint3` - Elbow (-90° to 90°)
- `gripper_left_joint` - Left finger (0 to 0.03m)
- `gripper_right_joint` - Right finger (0 to 0.03m)

---

## Visualize in RViz:

Add these displays in RViz:

1. **Camera** display:
   - Add → Camera
   - Image Topic: `/camera/image_raw`
   - You'll see the fisheye view!

2. **Robot Model**:
   - Should show the arm on top of rover

3. **Interactive Markers** (if available):
   - To control arm visually

---

## Common Issues:

**"No camera topic":**
```bash
# Check if camera bridge is running
ros2 topic list | grep camera

# If not, manually add:
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
```

**"Arm not moving":**
- The arm joints need controllers. For now, use direct joint_state publishing or add ros2_control configuration.

**"Can't see arm in Gazebo":**
- Make sure you're using the custom URDF (`leo_with_arm_camera.urdf.xacro`)
- Check robot_state_publisher is using the right URDF

---

## Next Steps - Arm Control:

For proper arm control, you'd typically want to add:

1. **MoveIt2** for motion planning
2. **ros2_control** for joint controllers
3. **Grasping logic** for picking up boxes

But for testing, you can manually control joints with topic pub commands!

---

## Quick Camera Test (No Arm):

If you just want camera for now and arm later:

1. Keep using your current working setup
2. Just add camera bridge in Terminal 5
3. View with `rqt_image_view`

The camera should work even with the standard Leo description if Gazebo has the sensor configured!

Let me know what you want to focus on:
- Just camera? 
- Camera + arm?
- Full grasping system?
