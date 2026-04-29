# Leo Rover Simulation - WORKING LAUNCH GUIDE

## THE PROBLEM
The official Leo description doesn't include a LiDAR sensor, so we need to add it separately in Gazebo.

## GUARANTEED WORKING METHOD

### Setup (One Time Only)
```bash
# Set Gazebo model path
echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:~/leo_ws/src/leo_lidar_sim/models" >> ~/.bashrc
source ~/.bashrc
```

### OPTION 1: Manual Launch (5 Terminals - GUARANTEED TO WORK)

**Terminal 1 - Robot State Publisher:**
```bash
source ~/leo_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p use_sim_time:=true \
  -p robot_description:="$(xacro /opt/ros/jazzy/share/leo_description/urdf/leo_sim.urdf.xacro)"
```

**Terminal 2 - Gazebo:**
```bash
source ~/leo_ws/install/setup.bash
ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:="-r ~/leo_ws/src/leo_lidar_sim/worlds/leo_world.sdf"
```

Wait 5 seconds for Gazebo to fully load, then:

**Terminal 3 - Spawn Leo Rover:**
```bash
source ~/leo_ws/install/setup.bash
ros2 run ros_gz_sim create \
  -name leo \
  -topic robot_description \
  -x 0.0 -y 0.0 -z 0.3
```

Wait 2 seconds, then spawn LiDAR on top:

```bash
ros2 run ros_gz_sim create \
  -name lidar_sensor \
  -file ~/leo_ws/src/leo_lidar_sim/models/lidar_sensor/model.sdf \
  -x 0.0 -y 0.0 -z 0.5
```

**Terminal 4 - ROS-Gazebo Bridges:**
```bash
source ~/leo_ws/install/setup.bash

# Clock
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock &

# Cmd_vel
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist &

# Odometry  
ros2 run ros_gz_bridge parameter_bridge /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry &

# LiDAR (MOST IMPORTANT!)
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan &

# Joint states
ros2 run ros_gz_bridge parameter_bridge /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model &

wait
```

**Terminal 5 - SLAM:**
```bash
source ~/leo_ws/install/setup.bash
ros2 launch leo_lidar_sim slam.launch.py
```

**Terminal 6 - RViz:**
```bash
source ~/leo_ws/install/setup.bash
ros2 run rviz2 rviz2 -d ~/leo_ws/src/leo_lidar_sim/rviz/leo_sim.rviz
```

**Terminal 7 - Teleop:**
```bash
source ~/leo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### VERIFY IT'S WORKING

```bash
# Check LiDAR data is publishing
ros2 topic hz /scan
# Should show ~10 Hz

# Check scan data
ros2 topic echo /scan --once
# Should show ranges array with data

# Check map
ros2 topic hz /map
# Should show updates after driving around

# List all topics
ros2 topic list
```

### EXPECTED BEHAVIOR

1. **Gazebo**: You should see Leo Rover + small black cylinder on top (LiDAR)
2. **RViz**: 
   - Robot model visible
   - Red laser scan points around robot
   - Map starts appearing as you drive
3. **Teleop**: Robot moves when you press keys

### TROUBLESHOOTING

**No LiDAR data (`ros2 topic hz /scan` shows nothing):**
- Check if LiDAR sensor spawned: Look for black cylinder on top of robot in Gazebo
- Check bridge is running: `ros2 node list | grep bridge`
- Respawn the LiDAR sensor (Terminal 3 command again)

**No map appearing:**
- Drive the robot around slowly for 10-15 seconds
- Check SLAM is running: `ros2 node list | grep slam`
- Check scan data exists: `ros2 topic echo /scan --once`

**Robot not moving:**
- Check cmd_vel bridge: `ros2 topic echo /cmd_vel`
- Try publishing manually: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`

**TF errors in RViz:**
- Normal at startup, should resolve after a few seconds
- Change Fixed Frame to "odom" if "map" doesn't work initially

### OPTION 2: Attach LiDAR to Leo (Advanced)

If you want LiDAR permanently attached to Leo, you need to modify the URDF:

```bash
# Edit the official Leo URDF
sudo nano /opt/ros/jazzy/share/leo_description/urdf/leo_sim.urdf.xacro

# Add at the end before </robot>:
```

```xml
<!-- LiDAR Link -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.05"/>
    </geometry>
    <material name="black"/>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.04" length="0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" iyy="0.0001" izz="0.0001" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.1 0 0.15" rpy="0 0 0"/>
</joint>

<!-- LiDAR Gazebo Plugin -->
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <topic>scan</topic>
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.15</min>
        <max>10.0</max>
      </range>
    </lidar>
    <always_on>true</always_on>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

Then you won't need to spawn LiDAR separately!

---

## QUICK START SUMMARY

```bash
# Terminal 1
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:="$(xacro /opt/ros/jazzy/share/leo_description/urdf/leo_sim.urdf.xacro)"

# Terminal 2  
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r ~/leo_ws/src/leo_lidar_sim/worlds/leo_world.sdf"

# Wait 5 sec, then Terminal 3
ros2 run ros_gz_sim create -name leo -topic robot_description -x 0 -y 0 -z 0.3
ros2 run ros_gz_sim create -name lidar_sensor -file ~/leo_ws/src/leo_lidar_sim/models/lidar_sensor/model.sdf -x 0 -y 0 -z 0.5

# Terminal 4 (all bridges)
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model

# Terminal 5
ros2 launch leo_lidar_sim slam.launch.py

# Terminal 6
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
