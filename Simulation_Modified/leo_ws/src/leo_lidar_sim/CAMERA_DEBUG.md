# Camera Troubleshooting

## Step 1: Check what Gazebo is publishing

While your system is running, check:

```bash
# Check ALL topics
ros2 topic list

# Check Gazebo topics specifically
gz topic -l

# This will show Gazebo's internal topics
```

## Step 2: Find the actual camera topics

Look for topics like:
- `/world/sorting_room/model/leo/link/realsense_camera_link/sensor/realsense_color/image`
- Similar long Gazebo format

## Step 3: Bridge the ACTUAL topics

Once you find the real topic name, use:

```bash
# Example (use YOUR actual topic name from gz topic -l)
ros2 run ros_gz_bridge parameter_bridge \
  /world/WORLD_NAME/model/MODEL_NAME/link/LINK_NAME/sensor/SENSOR_NAME/image@sensor_msgs/msg/Image[gz.msgs.Image
```

---

## Quick Fix - Let me update the camera configuration

The issue is the Gazebo sensor topic naming. I'll fix it now.
