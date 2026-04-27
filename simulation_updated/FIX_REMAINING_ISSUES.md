# FIX FOR REMAINING ISSUES

## Issue 1: RViz Not Launching - EXPLAINED

**This is BY DESIGN!** 

Terminal 1 (`robot_state.launch.py`) only launches the robot state publisher, NOT RViz.

RViz is launched **separately in Terminal 8** as shown in the guide.

### Why?
- Different terminals = easier to restart individual components
- RViz is heavy - you might not always want it
- Follows standard ROS2 practice

### If You Want RViz in Same Launch:
I can create that, but the current way is actually better for debugging.

---

## Issue 2: TF Transform Fluctuations - FIXED

The problem was `use_sim_time` not being set properly in odom_to_tf.py.

### The Fix:

1. **Rebuild the package** (required for odom_to_tf to work as ROS executable):

```bash
cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash
```

2. **Restart Terminal 6** with the updated script:

```bash
# Stop Terminal 6 (Ctrl+C)
# Then restart:
source ~/leo_ws/install/setup.bash
bash ~/leo_ws/src/leo_lidar_sim/scripts/simple_tf_start.sh
```

### What Changed:
- ✅ `odom_to_tf.py` now explicitly declares `use_sim_time=True`
- ✅ Script passes `--ros-args -p use_sim_time:=true` when running it
- ✅ Both static and dynamic TF publishers use sim_time consistently

### Verify TF is Stable:

```bash
# Check for transform fluctuations
ros2 run tf2_ros tf2_echo odom base_footprint

# Should show smooth, continuous updates with no errors
# Timestamps should match sim_time (not wall time)
```

### If Still Fluctuating:

**Check all nodes using sim_time:**

```bash
# Check each node
ros2 param get /robot_state_publisher use_sim_time  # Should be: true
ros2 param get /slam_toolbox use_sim_time           # Should be: true  
ros2 param get /odom_to_tf use_sim_time            # Should be: true
ros2 param get /rviz use_sim_time                  # Should be: true
```

**If any show false:**

```bash
ros2 param set /NODE_NAME use_sim_time true
```

**Check clock is publishing:**

```bash
ros2 topic hz /clock
# Should be ~1000 Hz (from Gazebo)
```

**Check for duplicate TF publishers:**

```bash
# Should show ONLY ONE publisher for odom->base_footprint
ros2 run tf2_tools tf2_monitor odom base_footprint
```

If multiple publishers, kill all and restart Terminal 6:

```bash
pkill -9 -f odom_to_tf
pkill -9 -f static_transform
sleep 2
# Then restart Terminal 6
```

---

## Complete Checklist After Rebuild:

```bash
# 1. Rebuild
cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash

# 2. Restart all terminals in order (1-9)

# 3. Verify TF stable
ros2 run tf2_ros tf2_echo odom base_footprint
# Should be smooth, no errors

# 4. Verify all use sim_time
ros2 param get /robot_state_publisher use_sim_time
ros2 param get /slam_toolbox use_sim_time
ros2 param get /odom_to_tf use_sim_time
# All should return: Boolean value is: True

# 5. Check TF tree
ros2 run tf2_tools view_frames
# Open frames.pdf - should show clean tree with no warnings
```

---

## Summary:

### Issue 1 (RViz): 
**Not an issue** - RViz is in Terminal 8, not Terminal 1. This is correct!

### Issue 2 (TF Fluctuations):
**Fixed** - Rebuild package, restart Terminal 6, TF will be stable.

The key was ensuring `use_sim_time=True` is set in ALL nodes including the dynamic TF publisher.
