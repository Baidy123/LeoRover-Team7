# SIMPLE 2-TERMINAL SETUP

## Install Package:

```bash
cd ~/leo_ws/src
rm -rf leo_lidar_sim
tar -xzf ~/Downloads/leo_lidar_sim_ALL_IN_ONE.tar.gz
mv leo_lidar_sim .

cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash
```

---

## TERMINAL 1 - Complete System (Everything!)

```bash
source ~/leo_ws/install/setup.bash
ros2 launch leo_lidar_sim complete_system.launch.py
```

**What this launches automatically:**
1. ✅ Robot State Publisher
2. ✅ Gazebo with sorting room
3. ✅ Leo Rover spawn
4. ✅ LiDAR sensor spawn
5. ✅ All ROS-Gazebo bridges
6. ✅ TF transforms (odom→base_footprint, base_link→lidar)
7. ✅ SLAM Toolbox
8. ✅ RViz with configuration

**Wait:** ~20 seconds for everything to start

**What you'll see:**
- Gazebo window with Leo Rover (with blue arm!)
- RViz window with robot model and laser scans
- Map will start building when you drive

---

## TERMINAL 2 - Robot Control (Teleop)

```bash
source ~/leo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` = forward
- `k` = stop
- `j` = left
- `l` = right
- `u`, `o` = curves
- `,` = backward

**Drive around to build the map!**

---

## That's It! Just 2 Terminals!

### What You Should See:

**In Gazebo:**
- Sorting room (5x5m with 4 walls)
- Colored boxes (red, green, blue)
- Colored baskets
- Leo Rover with **BLUE ARM** on top
- LiDAR sensor (black cylinder)

**In RViz:**
- Robot model with arm
- Red laser scan points
- Map building as you drive (white = free, black = obstacles, gray = unknown)

---

## Verify Everything Works:

```bash
# In a 3rd terminal (optional):

# Check TF tree
ros2 run tf2_tools view_frames
# Should show: map → odom → base_footprint → base_link → [all parts]

# Check scan data
ros2 topic hz /scan
# Should be ~5-10 Hz

# Check map
ros2 topic hz /map
# Should update (even if slow)

# Check arm in URDF
ros2 topic echo /robot_description --once | grep "arm_segment"
# Should show arm links
```

---

## Troubleshooting:

**"RViz not opening"**
- Wait 20 seconds - it starts last
- Check: `ps aux | grep rviz`
- If not there, RViz might have crashed - check logs

**"No arm visible"**
- In Gazebo: Look at top of rover - zoom in close
- In RViz: Check RobotModel display is enabled
- The arm IS there - grey platform + blue cylinders

**"Transform errors"**
- Wait 15 seconds for all TF publishers to start
- Check: `ros2 run tf2_ros tf2_echo odom base_footprint`

**"No map building"**
- Drive the robot with teleop (Terminal 2)
- Map only builds when robot moves
- Check scan data: `ros2 topic hz /scan`

---

## To Stop Everything:

**Ctrl+C in Terminal 1** - Stops everything cleanly

---

## Advantages of This Method:

- ✅ Only 2 terminals instead of 9!
- ✅ Automatic timing (no manual waiting)
- ✅ Everything starts in correct order
- ✅ All TF issues fixed
- ✅ Easy to restart: just Ctrl+C and relaunch

---

## If You Want Individual Control:

Use the old 9-terminal method from `COPY_PASTE_TERMINAL_GUIDE.md`

This gives you:
- Individual restart of components
- Easier debugging
- More control

But the 2-terminal method is **simpler for normal use**!

---

## Success Criteria:

After Terminal 1 starts (wait 20 seconds):

✅ Gazebo open with Leo Rover + blue arm
✅ RViz open showing robot
✅ Drive with teleop (Terminal 2)
✅ Map builds in RViz as you drive
✅ No transform errors

If all ✅, you're ready to work! 🎉
