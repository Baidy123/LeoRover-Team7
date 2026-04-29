# Fix Camera Info + Arm Falling Off

## ISSUE 1: Camera Info - SOLUTION

The camera_info_publisher needs to run. Make sure **Terminal 12** is running:

```bash
source ~/leo_ws/install/setup.bash
python3 ~/leo_ws/src/leo_lidar_sim/scripts/camera_info_publisher.py
```

Then in **RViz**:
1. Camera display → Image Topic: `/camera/image_raw`
2. **Uncheck and re-check** the Camera display to refresh
3. You should now see the camera view!

---

## ISSUE 2: Arm Falls Off - SOLUTION

The problem: Arm is spawned separately, not attached to Leo!

### **BEST FIX: Change Terminal 1 to Include Arm**

Instead of spawning arm separately, include it in Leo's URDF from the start.

**STOP Terminal 1** (Ctrl+C), then restart with this:

```bash
source ~/leo_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(xacro ~/leo_ws/src/leo_lidar_sim/urdf/leo_with_arm_camera.urdf.xacro)"
```

**IMPORTANT**: Now **DON'T run Terminal 14** (don't spawn arm separately). The arm is already part of the robot!

---

## Updated Terminal List:

### Original Terminals (Modified):

**Terminal 1** - Robot State (CHANGED - use arm URDF):
```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(xacro ~/leo_ws/src/leo_lidar_sim/urdf/leo_with_arm_camera.urdf.xacro)"
```

**Terminals 2-10** - Same as before (no changes)

### Camera Terminals:

**Terminal 11** - Camera Bridge:
```bash
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
```

**Terminal 12** - Camera Info:
```bash
python3 ~/leo_ws/src/leo_lidar_sim/scripts/camera_info_publisher.py
```

**Terminal 13** - Camera TF:
```bash
ros2 run tf2_ros static_transform_publisher 0.15 0 0.08 0 0.2 0 base_link camera_link
```

**~~Terminal 14~~** - ❌ DON'T SPAWN ARM SEPARATELY ANYMORE!

---

## What Changes:

- ✅ Arm is now **part of Leo** (built into URDF)
- ✅ Arm won't fall off when rover moves
- ✅ Camera is on front of rover
- ✅ Camera info published correctly

---

## Verify It Works:

```bash
# Check robot model includes arm
ros2 topic echo /robot_description --once | grep "arm_link"

# Should see arm links listed

# Check camera info
ros2 topic hz /camera/camera_info
# Should show ~30 Hz
```

---

## In Gazebo You Should See:

- Leo Rover at (1.8, -1.8)
- **Blue robotic arm** on top (attached, doesn't fall)
- LiDAR sensor above arm
- Camera on front

## In RViz Camera Display You Should See:

- First-person view from rover's front camera
- Forward-facing view of the room
- Can see boxes and walls

---

## If Arm Still Has Issues:

The arm joints might be "floppy". To stiffen them, you can publish joint positions:

```bash
# Keep arm upright
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
  "{name: ['arm_joint1', 'arm_joint2'], position: [0.0, 0.0]}" --once
```

But with the fixed URDF, the arm should stay attached to the rover now!

---

## Summary of Changes:

**Before**:
- Terminal 1: Standard Leo URDF
- Terminal 14: Spawn arm separately → ARM FALLS OFF ❌

**After**:
- Terminal 1: Leo + Arm combined URDF → ARM STAYS ON ✅
- No Terminal 14 needed

Try this and the arm should stay attached! 🦾
