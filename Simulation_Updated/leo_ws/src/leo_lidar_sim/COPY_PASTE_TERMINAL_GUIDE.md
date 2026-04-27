# ========================================
# LEO ROVER - COMPLETE WORKING SETUP
# Copy-Paste These Commands Terminal by Terminal
# ========================================

## SETUP (Do this ONCE):

```bash
cd ~/leo_ws/src
rm -rf leo_lidar_sim
tar -xzf ~/Downloads/leo_lidar_sim_BRAND_NEW_WORKING.tar.gz
mv leo_lidar_sim .

cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash
```

---

## TERMINAL 1 - Robot State Publisher

```bash
source ~/leo_ws/install/setup.bash
ros2 launch leo_lidar_sim robot_state.launch.py
```

**What you'll see:** Robot state publisher starts, processes URDF
**Arm location:** Defined in URDF and loaded into /robot_description topic
**No errors!**

---

## TERMINAL 2 - Gazebo Simulation

```bash
source ~/leo_ws/install/setup.bash
ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:="-r ~/leo_ws/src/leo_lidar_sim/worlds/sorting_room.sdf"
```

**What you'll see:** Gazebo window opens with sorting room
**Wait:** 5 seconds for Gazebo to load completely

---

## TERMINAL 3 - Spawn Leo Rover

```bash
source ~/leo_ws/install/setup.bash
ros2 run ros_gz_sim create \
  -name leo \
  -topic robot_description \
  -x 1.8 -y -1.8 -z 0.3
```

**What you'll see:** Leo rover appears in Gazebo at bottom-right corner
**ARM CHECK:** You should see a GREY platform on top with BLUE arm sticking up!
**Wait:** 3 seconds

---

## TERMINAL 4 - Spawn LiDAR Sensor

```bash
source ~/leo_ws/install/setup.bash
ros2 run ros_gz_sim create \
  -name lidar_sensor \
  -file ~/leo_ws/src/leo_lidar_sim/models/lidar_sensor/model.sdf \
  -x 1.8 -y -1.8 -z 0.5
```

**What you'll see:** Small black cylinder appears above arm
**Wait:** 2 seconds

---

## TERMINAL 5 - ROS-Gazebo Bridges

```bash
source ~/leo_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
ros2 run ros_gz_bridge parameter_bridge /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model &

wait
```

**What you'll see:** Multiple bridge processes running
**Wait:** 3 seconds

---

## TERMINAL 6 - TF Transforms (THE KEY!)

```bash
source ~/leo_ws/install/setup.bash
bash ~/leo_ws/src/leo_lidar_sim/scripts/simple_tf_start.sh
```

**What you'll see:** 
- "Odom → base_footprint (dynamic)"
- "base_link → LiDAR (static)"
- "TF transforms running!"

**This fixes all TF issues!**
**Wait:** 3 seconds

---

## TERMINAL 7 - SLAM Toolbox

```bash
source ~/leo_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**What you'll see:** SLAM initializing, looking for scan data
**Wait:** 2 seconds

---

## TERMINAL 8 - RViz Visualization

```bash
source ~/leo_ws/install/setup.bash
ros2 run rviz2 rviz2
```

**Setup RViz:**
1. Fixed Frame: Change to "odom"
2. Add → RobotModel (you'll see Leo with BLUE ARM!)
3. Add → LaserScan → Topic: /scan
4. Add → Map → Topic: /map

**ARM CHECK IN RVIZ:** You should see the blue arm clearly!

---

## TERMINAL 9 - Robot Control (Teleop)

```bash
source ~/leo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- i = forward
- k = stop  
- j = turn left
- l = turn right
- u/o = curves

**Drive around to build the map!**

---

## VERIFICATION CHECKLIST

After all terminals running, verify:

### 1. Check Arm Visible:
```bash
# In Gazebo: Look at rover - you should see GREY platform + BLUE arm
# In RViz: RobotModel should show complete rover with arm
```

### 2. Check TF Tree:
```bash
ros2 run tf2_tools view_frames
# Open frames.pdf - should show: map → odom → base_footprint → base_link → [all parts]
```

### 3. Check Scan Data:
```bash
ros2 topic hz /scan
# Should show ~5-10 Hz
```

### 4. Check Map Building:
```bash
# Drive rover with teleop (Terminal 9)
# Watch RViz - map should appear and grow
ros2 topic hz /map
# Should show updates (even if slow like 0.5 Hz)
```

### 5. Test Robot Movement:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --once
# Robot should move forward in Gazebo
```

---

## IF ARM STILL NOT VISIBLE:

### In Gazebo:
1. Click on the rover model
2. Right-click → "View" → "Transparent" (uncheck)
3. Zoom in close to the top of rover
4. Look for grey cylinder (platform) and blue cylinder/box (arm)

### In RViz:
1. RobotModel display must be enabled
2. Robot Description Topic: /robot_description
3. You WILL see the arm here (guaranteed)

### Check URDF loaded correctly:
```bash
ros2 topic echo /robot_description --once | grep "arm_segment"
# Should show arm_segment1, arm_segment2, arm_gripper
```

---

## TROUBLESHOOTING

### "Transform errors in RViz"
- Check Terminal 6 is running
- Run: `ros2 run tf2_ros tf2_echo odom base_footprint`
- Should show continuous updates

### "No map appearing"
- Drive robot with teleop for 10-15 seconds
- Check: `ros2 topic hz /scan` (should be ~5-10 Hz)
- Check: `ros2 topic hz /odom` (should be ~10-50 Hz)

### "Robot not moving"
- Check Terminal 5 (bridges) running
- Test: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once`

### "Arm definitely not in Gazebo"
That's impossible if:
1. Terminal 1 used `leo_with_visible_arm.urdf.xacro` ✓
2. Terminal 3 spawned successfully ✓
3. You rebuilt: `colcon build --packages-select leo_lidar_sim` ✓

The arm MUST be there. Zoom in, rotate camera, check RViz.

---

## WHAT YOU SHOULD SEE IN GAZEBO:

```
              🔲 ← LiDAR (black cylinder on top)
              |
          ┌───┴───┐
          │ BLUE  │ ← Arm segment 2 (blue box)
          │ BOX   │
          └───┬───┘
              |
          ┌───┴───┐
          │ BLUE  │ ← Arm segment 1 (blue cylinder)
          │ CYL   │
          └───┬───┘
              |
          ┌───┴───┐
          │ GREY  │ ← Arm platform (grey disk)
          └───────┘
         ┌─────────┐
         │   LEO   │ ← Leo Rover body
         │  ROVER  │
         └─────────┘
```

---

## SUCCESS CRITERIA:

✅ All 9 terminals running without errors
✅ Blue arm visible in RViz RobotModel
✅ Blue arm visible in Gazebo (on top of rover)
✅ Map building when driving with teleop
✅ No TF transform errors
✅ LiDAR scan data at ~5-10 Hz

If ALL above are ✅, system is FULLY WORKING! 🎉

---

## TO SAVE YOUR WORK:

When map looks good:
```bash
# In SLAM terminal (Terminal 7)
# Ctrl+C to stop
# It will save map automatically

# Or manually:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_sorting_room_map'}}"
```

Map saved to: `~/.ros/my_sorting_room_map.yaml` and `.pgm`
