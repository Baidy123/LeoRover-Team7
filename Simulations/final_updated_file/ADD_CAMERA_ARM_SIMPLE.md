# Add Camera + Arm WITHOUT Changing Your Working Commands

This guide shows how to ADD camera and myCobot arm to your existing setup **without changing any of your current terminal commands**.

## Your Current Terminal Commands (KEEP AS-IS):

✅ Terminal 1: Robot State Publisher  
✅ Terminal 2: Gazebo  
✅ Terminal 3: Spawn Leo  
✅ Terminal 4: Spawn LiDAR  
✅ Terminal 5: Bridges  
✅ Terminal 6: TF Transform  
✅ Terminal 7: Odom to TF  
✅ Terminal 8: SLAM  
✅ Terminal 9: RViz  
✅ Terminal 10: Teleop  

**Don't change any of these!**

---

## ADD Camera (3 New Terminals)

### NEW Terminal 11 - Camera Bridge:
```bash
source ~/leo_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
```

### NEW Terminal 12 - Camera Info Publisher:
```bash
source ~/leo_ws/install/setup.bash
python3 ~/leo_ws/src/leo_lidar_sim/scripts/camera_info_publisher.py
```

### NEW Terminal 13 - Camera Link TF:
```bash
source ~/leo_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0.15 0 0.08 0 0.2 0 base_link camera_link
```

---

## View Camera in RViz:

In your existing RViz window:

1. Click **"Add"** button (bottom left)
2. Select **"Camera"** from the list
3. In Camera display settings:
   - **Image Topic**: `/camera/image_raw`
   - **Transport Hint**: `raw`
   - Expand it to see the camera view!

You should now see the camera feed in RViz! 📷

---

## Add myCobot Arm (Optional)

To add the arm, you need to spawn it as a separate model in Gazebo.

### Create Arm Model File:

```bash
mkdir -p ~/leo_ws/src/leo_lidar_sim/models/mycobot_arm
```

Then create `~/leo_ws/src/leo_lidar_sim/models/mycobot_arm/model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="mycobot_arm">
    <pose>0 0 0.15 0 0 0</pose>
    <static>false</static>
    
    <!-- Arm base -->
    <link name="arm_base">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.08 0.08 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.5 0.5 0.5 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.08 0.08 0.05</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <!-- Arm link 1 -->
    <link name="arm_link1">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>0.3</mass>
        <inertia>
          <ixx>0.0005</ixx>
          <iyy>0.0005</iyy>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.3 0.8 1</ambient>
          <diffuse>0.2 0.3 0.8 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.03</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <joint name="arm_joint1" type="revolute">
      <parent>arm_base</parent>
      <child>arm_link1</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-3.14</lower>
          <upper>3.14</upper>
        </limit>
      </axis>
    </joint>
    
    <!-- Gripper -->
    <link name="gripper">
      <pose>0 0 0.2 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.04 0.06 0.03</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.04 0.06 0.03</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <joint name="gripper_joint" type="fixed">
      <parent>arm_link1</parent>
      <child>gripper</child>
    </joint>
    
  </model>
</sdf>
```

### NEW Terminal 14 - Spawn Arm (run AFTER Leo spawns):
```bash
source ~/leo_ws/install/setup.bash

# Wait until Leo is spawned in Gazebo, then run:
ros2 run ros_gz_sim create \
  -name mycobot_arm \
  -file ~/leo_ws/src/leo_lidar_sim/models/mycobot_arm/model.sdf \
  -x 1.8 \
  -y -1.8 \
  -z 0.45
```

This spawns a simple arm on top of the rover!

---

## Summary - What to Add:

### For Camera Only (3 terminals):
- Terminal 11: Camera bridge
- Terminal 12: Camera info publisher  
- Terminal 13: Camera TF

### For Arm (1 more terminal):
- Terminal 14: Spawn arm model

**All your existing commands stay exactly the same!**

---

## Verify Camera Works:

```bash
# Check camera topics
ros2 topic list | grep camera

# Should see:
# /camera/image_raw
# /camera/camera_info

# Check camera is publishing
ros2 topic hz /camera/image_raw
```

---

## Troubleshooting:

**"No image in RViz Camera display":**
- Make sure Terminal 11 (camera bridge) is running
- Make sure Terminal 12 (camera_info) is running
- In RViz Camera display, check Image Topic is `/camera/image_raw`

**"Camera link not in TF tree":**
- Make sure Terminal 13 (camera TF) is running
- Run: `ros2 run tf2_tools view_frames` to verify

**"Arm not visible in Gazebo":**
- Make sure you created the model file first
- Check coordinates match Leo's spawn position (1.8, -1.8)

---

## Install Required Tools (if needed):

```bash
# For camera_info_publisher
sudo apt install ros-jazzy-vision-msgs

# For image viewing
sudo apt install ros-jazzy-rqt-image-view
```

That's it! Your working setup stays unchanged, just add these extra terminals for camera and arm! 📷🦾
