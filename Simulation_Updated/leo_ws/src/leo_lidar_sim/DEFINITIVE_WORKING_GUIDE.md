# DEFINITIVE WORKING SETUP - Leo Rover with Arm & Camera

## THE PROBLEM YOU HAD:

Your TF tree shows ALL frames disconnected - no parent-child relationships!
This breaks SLAM, navigation, and everything.

## THE SOLUTION:

Follow these EXACT commands in order. Do NOT skip any step.

---

## TERMINAL COMMANDS (Exact Order):

### Terminal 1 - Robot State Publisher
```bash
source ~/leo_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(xacro ~/leo_ws/src/leo_lidar_sim/urdf/leo_with_arm_camera.urdf.xacro)"
```

### Terminal 2 - Gazebo
```bash
source ~/leo_ws/install/setup.bash
ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:="-r ~/leo_ws/src/leo_lidar_sim/worlds/sorting_room.sdf"
```

**WAIT 5 SECONDS for Gazebo to fully load**

### Terminal 3 - Spawn Leo Rover
```bash
source ~/leo_ws/install/setup.bash
ros2 run ros_gz_sim create \
  -name leo \
  -topic robot_description \
  -x 1.8 -y -1.8 -z 0.3
```

**WAIT 2 SECONDS for Leo to spawn**

### Terminal 4 - Spawn LiDAR
```bash
source ~/leo_ws/install/setup.bash
ros2 run ros_gz_sim create \
  -name lidar_sensor \
  -file ~/leo_ws/src/leo_lidar_sim/models/lidar_sensor/model.sdf \
  -x 1.8 -y -1.8 -z 0.5
```

**WAIT 2 SECONDS**

### Terminal 5 - ROS-Gazebo Bridges
```bash
source ~/leo_ws/install/setup.bash

ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist &
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry &
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &
ros2 run ros_gz_bridge parameter_bridge /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model &

wait
```

**WAIT 2 SECONDS for bridges to connect**

### Terminal 6 - ALL TF TRANSFORMS (THE FIX!)
```bash
source ~/leo_ws/install/setup.bash
bash ~/leo_ws/src/leo_lidar_sim/scripts/start_all_tf.sh
```

**This script starts ALL missing TF transforms! This is the KEY fix!**

**WAIT 3 SECONDS**

### Terminal 7 - SLAM
```bash
source ~/leo_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**WAIT 2 SECONDS**

### Terminal 8 - RViz
```bash
source ~/leo_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/leo_ws/src/leo_lidar_sim/rviz/leo_sim.rviz
```

### Terminal 9 - Teleop (Control Robot)
```bash
source ~/leo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## VERIFY IT WORKS:

### Check TF Tree:
```bash
ros2 run tf2_tools view_frames
```

**You should now see:**
```
map → odom → base_footprint → base_link → [all other links]
```

### Check Specific Transforms:
```bash
# Should work now:
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link lidar_sensor/lidar_link/lidar
```

### Check SLAM is Getting Data:
```bash
ros2 topic hz /scan    # Should show ~10 Hz
ros2 topic hz /map     # Should show updates
```

---

## FOR CAMERA (Optional - Add After Above Works):

### Terminal 10 - Camera Bridge
```bash
source ~/leo_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
```

### Terminal 11 - Camera Info
```bash
source ~/leo_ws/install/setup.bash
python3 ~/leo_ws/src/leo_lidar_sim/scripts/camera_info_publisher.py
```

Then in RViz:
- Add → Camera
- Image Topic: `/camera/image_raw`

---

## WHAT EACH TERMINAL DOES:

1. **Robot State** - Publishes robot structure and static TF
2. **Gazebo** - Runs physics simulation
3. **Spawn Leo** - Puts robot in world
4. **Spawn LiDAR** - Adds sensor on top
5. **Bridges** - Connects Gazebo ↔ ROS2
6. **TF Transforms** - **CRITICAL** - Links all frames together (THIS WAS MISSING!)
7. **SLAM** - Builds map
8. **RViz** - Visualization
9. **Teleop** - Drive the robot

---

## WHY YOUR TF TREE WAS BROKEN:

Looking at your PDF, you had:
- ❌ No `odom → base_footprint` connection
- ❌ No `base_link → lidar` connection  
- ❌ No `base_link → camera_link` connection

**All frames were islands!**

Terminal 6 (`start_all_tf.sh`) fixes ALL of these!

---

## TROUBLESHOOTING:

**If TF tree still broken:**
```bash
# Kill everything and restart
pkill -9 gz
pkill -9 slam
pkill -f static_transform
pkill -f odom_to_tf

# Wait 5 seconds
# Then start over from Terminal 1
```

**If script not found:**
```bash
# Make sure you have the latest package
cd ~/leo_ws
colcon build
source install/setup.bash
```

**If odom_to_tf.py fails:**
```bash
# Check if odometry is publishing
ros2 topic hz /odom

# If not, check odom bridge in Terminal 5
```

---

## THE KEY FIX:

The `start_all_tf.sh` script in Terminal 6 publishes:
1. `odom → base_footprint` (dynamic, from odometry)
2. `base_link → lidar_sensor/lidar_link/lidar` (static)
3. `base_link → camera_link` (static)

These were ALL missing in your setup, causing the disconnected TF tree!

---

This should work 100%. The TF tree will be properly connected and SLAM will work! 🎯
