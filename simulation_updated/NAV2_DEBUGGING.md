# NAV2 TROUBLESHOOTING - Robot Not Moving

## STEP-BY-STEP DEBUGGING

Run these commands in a NEW terminal while Nav2 is running:

### Step 1: Check Nav2 Nodes Are Running

```bash
source ~/leo_ws/install/setup.bash
ros2 node list | grep nav2
```

**You should see:**
- /bt_navigator
- /controller_server
- /planner_server
- /behavior_server
- /waypoint_follower
- /velocity_smoother

**If you DON'T see these**, Nav2 didn't start properly. Check the Nav2 terminal for errors.

---

### Step 2: Check TF Chain (CRITICAL!)

```bash
ros2 run tf2_ros tf2_echo map base_link
```

**Expected:** Should show continuous transforms with NO errors

**If ERROR:** Your TF chain is broken. This is THE most common issue!

**Common TF errors:**
- "map to base_link transform not found" → SLAM not publishing map→odom
- "Lookup would require extrapolation" → Timing issue

**Fix:**
```bash
# Check if SLAM is publishing map→odom transform
ros2 run tf2_ros tf2_echo map odom

# Check if odom_to_tf is running
ros2 node list | grep odom_to_tf

# If not running, restart Terminal 1 (complete_system)
```

---

### Step 3: Check Action Server

```bash
# Check if navigate_to_pose action is available
ros2 action list | grep navigate
```

**You should see:** `/navigate_to_pose`

**If NOT there:** Nav2 didn't start properly.

---

### Step 4: Send Test Goal via Command Line

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{
  pose: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 0.5, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}" --feedback
```

**Watch the feedback!** It should show:
- "Goal accepted"
- Distance remaining decreasing
- Robot moving

**If goal REJECTED:** Check error message

---

### Step 5: Check if Nav2 is Sending Commands

```bash
# Monitor velocity commands from Nav2
ros2 topic echo /cmd_vel
```

**Expected:** Should see Twist messages with linear/angular values when goal is active

**If EMPTY/NOTHING:**
- Nav2 controller isn't running
- Robot state not being published
- Costmaps have issues

---

### Step 6: Check Costmaps

```bash
# Check if costmaps are publishing
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
```

**Both should show ~1-5 Hz**

**In RViz:**
1. Add → Map → Topic: `/global_costmap/costmap`
2. Add → Map → Topic: `/local_costmap/costmap`

**You should see:**
- Red = obstacles
- Blue = inflation
- White/clear = free space

**If costmaps are ALL RED:** Robot thinks everything is an obstacle!

---

### Step 7: Check Robot Footprint

```bash
ros2 param get /controller_server robot_base_frame
ros2 param get /controller_server footprint
```

**Should return valid frame and footprint dimensions**

---

### Step 8: Verify Map Frame in RViz

**In RViz:**
- Top left: "Fixed Frame" should be `map` (NOT `odom`)
- If set to `odom`, change to `map`

---

## COMMON ISSUES & FIXES

### Issue 1: "No map→base_link transform"

**Cause:** TF chain broken

**Fix:**
```bash
# Check complete TF chain
ros2 run tf2_tools view_frames

# Open frames.pdf and verify:
# map → odom → base_footprint → base_link

# If broken, restart complete_system:
# Ctrl+C Terminal 1
ros2 launch leo_lidar_sim complete_system.launch.py
```

---

### Issue 2: Goal Accepted but Robot Doesn't Move

**Cause:** Controller not sending /cmd_vel OR costmaps blocking

**Check:**
```bash
# 1. Is controller running?
ros2 node info /controller_server

# 2. Check controller params
ros2 param list /controller_server | grep vel

# 3. Check if path is found
ros2 topic echo /plan --once
# Should show a valid path (list of poses)
```

**Fix:**
```bash
# Set higher max velocities
ros2 param set /controller_server FollowPath.max_vel_x 0.5
ros2 param set /controller_server FollowPath.max_vel_theta 1.0
```

---

### Issue 3: "No path could be found"

**Cause:** Goal is in obstacle OR map not loaded

**Check in RViz:**
- Is there a clear path from robot to goal?
- Is goal inside a wall/obstacle?
- Is costmap showing properly?

**Fix:**
- Choose closer, clear goal
- Make sure map is visible in RViz
- Check global costmap shows free space

---

### Issue 4: Nav2 Terminal Shows Errors

**Common errors:**

**"Failed to find valid control"**
→ Controller can't find safe velocity
→ Lower max velocities or increase tolerance

**"Aborting because a valid control could not be found"**
→ Robot is stuck or in obstacle
→ Send recovery behavior or choose new goal

**"TF timeout"**
→ TF chain broken or timing issues
→ Check use_sim_time is true everywhere

---

### Issue 5: Robot Spins in Place

**Cause:** Poor odometry or incorrect orientation

**Fix:**
```bash
# Check odometry
ros2 topic echo /odom --once

# Should show reasonable x, y position and orientation
```

---

## COMPLETE DIAGNOSTIC SCRIPT

Run this to check everything:

```bash
#!/bin/bash
echo "=== NAV2 DIAGNOSTICS ==="
echo ""

echo "1. Checking Nav2 nodes..."
ros2 node list | grep nav2
echo ""

echo "2. Checking TF chain..."
timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -5
echo ""

echo "3. Checking action server..."
ros2 action list | grep navigate
echo ""

echo "4. Checking cmd_vel topic..."
timeout 2 ros2 topic echo /cmd_vel --once 2>&1
echo ""

echo "5. Checking costmaps..."
echo "Global costmap:"
timeout 2 ros2 topic hz /global_costmap/costmap 2>&1 | head -3
echo "Local costmap:"
timeout 2 ros2 topic hz /local_costmap/costmap 2>&1 | head -3
echo ""

echo "6. Checking map..."
timeout 2 ros2 topic echo /map --once 2>&1 | head -10
echo ""

echo "=== END DIAGNOSTICS ==="
```

Save as `~/nav2_debug.sh`, make executable, and run:
```bash
chmod +x ~/nav2_debug.sh
~/nav2_debug.sh
```

---

## MOST LIKELY ISSUES (In Order):

### 1. TF Chain Broken (90% of cases)
**Symptom:** "Transform error" in Nav2 logs
**Fix:** Restart complete_system.launch.py

### 2. Nav2 Not Getting Map
**Symptom:** Empty costmaps in RViz
**Fix:** Make sure SLAM is running and publishing /map

### 3. Controller Parameters Wrong
**Symptom:** Goal accepted but no movement
**Fix:** Check max_vel_x is > 0 and reasonable

### 4. RViz Frame Wrong
**Symptom:** Can't see robot or goal position jumps
**Fix:** Set Fixed Frame to "map"

### 5. Goal in Obstacle
**Symptom:** "No valid path" error
**Fix:** Click goal in free space (white area in costmap)

---

## QUICK FIX - RESTART EVERYTHING IN ORDER

If nothing works, restart in this EXACT order:

```bash
# 1. Kill everything
pkill -9 gz
pkill -9 rviz
pkill -9 slam
pkill -f nav2

# 2. Wait 5 seconds
sleep 5

# 3. Terminal 1 - Complete system
ros2 launch leo_lidar_sim complete_system.launch.py

# 4. Wait 25 seconds for everything to start

# 5. Terminal 2 - Drive around to build map
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Drive for 2-3 minutes

# 6. Terminal 3 - Start Nav2
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=$(ros2 pkg prefix leo_lidar_sim)/share/leo_lidar_sim/config/nav2_params.yaml

# 7. Wait 10 seconds for Nav2 to initialize

# 8. In RViz:
# - Check Fixed Frame = "map"
# - Add global_costmap and local_costmap
# - Click 2D Goal Pose in FREE SPACE
```

---

## WHAT TO CHECK IN RVIZ

Before sending goal:

✅ Fixed Frame = "map"
✅ Map display showing (white/grey map)
✅ RobotModel showing robot
✅ LaserScan showing red points
✅ Global Costmap showing (obstacles in red)
✅ Local Costmap showing (around robot)
✅ Robot position makes sense on map

Then:
1. Click "2D Goal Pose"
2. Click in WHITE/FREE area (not red obstacle)
3. Watch for green path line
4. Robot should move!

---

## SEND ME THIS INFO IF STILL NOT WORKING

```bash
# Run these and send output:
ros2 node list
ros2 topic list | grep -E "map|odom|cmd_vel|nav"
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /cmd_vel --once
```

This will help identify the exact issue! 🔍
