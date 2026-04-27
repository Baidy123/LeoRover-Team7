# Troubleshooting Guide

## Issue 1: Arm Not Visible in Gazebo

### Check if arm is in URDF:
```bash
ros2 topic echo /robot_description --once | grep "arm_link"
```

If you see arm_link1, arm_link2, etc., the arm is in the URDF.

### Why it might not show in Gazebo:

**Problem**: The arm links don't have proper Gazebo materials/plugins.

**Solution**: The arm WILL show in RViz. To see it in Gazebo:

1. **Check RViz first** - Add RobotModel display, you should see the blue arm
2. **In Gazebo** - The arm might be transparent or same color as background

### Force Arm to Show:

Restart with this updated URDF (already fixed in latest package):
```bash
cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash
```

Then restart Terminal 1 (robot_state_publisher).

---

## Issue 2: TF Errors Fluctuating

### Symptoms:
- RViz shows transform errors appearing/disappearing
- "Transform timeout" messages
- Map jumps around

### Root Causes:

**1. Multiple TF publishers for same transform (CONFLICT!)**

Check for duplicates:
```bash
ros2 run tf2_tools tf2_monitor odom base_footprint

# Should show ONE publisher, not multiple!
```

If you see multiple publishers:
```bash
# Kill all TF publishers
pkill -f static_transform_publisher
pkill -f odom_to_tf

# Restart ONLY Terminal 6
bash ~/leo_ws/src/leo_lidar_sim/scripts/start_all_tf.sh
```

**2. use_sim_time mismatch**

```bash
# Check if all nodes are using sim time
ros2 param get /robot_state_publisher use_sim_time
ros2 param get /slam_toolbox use_sim_time
ros2 param get /rviz use_sim_time

# All should return: true
```

If any are false:
```bash
ros2 param set /NODE_NAME use_sim_time true
```

**3. Clock not publishing properly**

```bash
# Check clock
ros2 topic hz /clock

# Should show ~1000 Hz from Gazebo
```

If not, check Terminal 5 (bridges) - clock bridge should be running.

**4. Odom publishing rate issues**

```bash
# Check odom rate
ros2 topic hz /odom

# Should show ~10-50 Hz steady
```

If erratic, restart Terminal 5 (bridges).

---

## Issue 3: Map Not Building

### Check SLAM is getting data:

```bash
# 1. Is scan data arriving?
ros2 topic hz /scan
# Should show ~5-10 Hz

# 2. Is odometry arriving?
ros2 topic hz /odom  
# Should show ~10-50 Hz

# 3. Check TF chain
ros2 run tf2_ros tf2_echo map base_link
# Should show continuous updates
```

### If any fail:

**Scan not publishing**: Check Terminal 4 (LiDAR spawn) and Terminal 5 (scan bridge)

**Odom not publishing**: Check Terminal 5 (odom bridge)

**TF broken**: Check Terminal 6 (TF transforms)

---

## Issue 4: Robot Not Moving

```bash
# Test cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once

# Check in Gazebo - does robot move?
```

**If not moving**:
- Check Terminal 5 (cmd_vel bridge)
- Check robot spawned correctly (Terminal 3)
- Check Gazebo physics is running (not paused)

---

## Issue 5: Camera Not Showing

```bash
# Check camera topics
ros2 topic list | grep camera

# Should see:
# /camera/image_raw
# /camera/camera_info (if publisher running)
```

**If missing**:
- Leo's built-in camera might have different topic name
- Try: `ros2 topic list | grep image`
- Bridge the actual topic that exists

**In RViz**:
- Camera display needs BOTH `/camera/image_raw` AND `/camera/camera_info`
- If camera_info missing, run Terminal 11 (camera_info_publisher.py)

---

## Complete Reset Procedure

If everything is broken:

```bash
# 1. Kill ALL processes
pkill -9 gz
pkill -9 rviz
pkill -9 slam
pkill -f bridge
pkill -f transform
pkill -f robot_state

# 2. Wait 5 seconds
sleep 5

# 3. Clean workspace
cd ~/leo_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash

# 4. Start over from Terminal 1
```

---

## Verify Everything Works Checklist

After starting all terminals, verify:

```bash
# ✓ TF tree connected
ros2 run tf2_tools view_frames
# Check frames.pdf shows: map → odom → base_footprint → base_link → everything else

# ✓ Scan data flowing
ros2 topic hz /scan
# Should be ~5-10 Hz

# ✓ Odom data flowing  
ros2 topic hz /odom
# Should be ~10-50 Hz

# ✓ Map being published
ros2 topic hz /map
# Should update (even if slow like 0.2 Hz)

# ✓ Robot responds to commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once
# Robot should move in Gazebo

# ✓ Arm visible
# Check RViz RobotModel display - should see blue arm on top

# ✓ Camera working (if enabled)
ros2 topic hz /camera/image_raw
# Should be ~30 Hz
```

If ALL above pass, system is working correctly! 🎯

---

## Common Error Messages Explained

**"No transform from X to Y"**
→ Missing TF publisher - check Terminal 6

**"Transform timeout"**  
→ TF publisher stopped or use_sim_time mismatch

**"Could not find a connection"**
→ TF tree broken - regenerate with `view_frames`

**"Lookup would require extrapolation"**
→ Clock/timing issue - check /clock topic and use_sim_time

**"Frame doesn't exist"**
→ Typo in frame name OR frame not published yet (wait longer after spawn)
