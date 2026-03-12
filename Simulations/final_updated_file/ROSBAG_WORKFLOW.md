# Leo Rover - Complete Rosbag Testing Workflow

This guide shows you how to test your complete pipeline before deploying to real hardware.

## Quick Start

### Method 1: Automatic Launcher (Easiest!)

```bash
cd ~/leo_ws/src/leo_lidar_sim
./auto_start.sh
```

This automatically launches everything in the correct order:
- Robot State Publisher
- Gazebo + Leo + LiDAR  
- All bridges
- TF transform
- SLAM
- RViz

Then in a new terminal, control the robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Workflow: Testing Before Real Hardware

### Step 1: Record Clean Simulation Data

**While simulation is running:**

```bash
# In a new terminal
cd ~/leo_ws/src/leo_lidar_sim/scripts
./record_rosbag.sh
```

Enter a name like `clean_data` and drive around for 2-3 minutes with teleop.

Press Ctrl+C when done. Your data is saved in `~/leo_ws/rosbag_data/clean_data/`

### Step 2: Playback and Verify

**Stop the simulation (Ctrl+C), then:**

```bash
# Play back your recorded data
cd ~/leo_ws/rosbag_data
ros2 bag play clean_data

# In another terminal, run SLAM on the playback
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# In another terminal, run RViz
ros2 run rviz2 rviz2
```

You should see the same map build from the recorded data!

### Step 3: Test with Corrupted Data (Realistic Testing)

**Create noisy data to test your filtering:**

```bash
# Terminal 1: Play clean rosbag
ros2 bag play clean_data

# Terminal 2: Add corruption
source ~/leo_ws/install/setup.bash
python3 ~/leo_ws/src/leo_lidar_sim/scripts/scan_corruptor.py

# Terminal 3: Filter unhealthy scans
python3 ~/leo_ws/src/leo_lidar_sim/scripts/scan_health_checker.py

# Terminal 4: Run SLAM on FILTERED data
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  --ros-args -r /scan:=/scan_healthy
```

**What this tests:**
- `/scan` → original clean data
- `/scan_corrupted` → data with noise/dropouts/outliers
- `/scan_healthy` → filtered clean data
- SLAM uses `/scan_healthy` to build map

This proves your filtering pipeline works!

### Step 4: Analyze Data Quality

**Check statistics while running:**

```bash
# Terminal running scan_health_checker.py will show:
# - Total scans processed
# - Healthy scan percentage
# - Recent health trend (last 100 scans)
```

**Visualize in RViz:**
Add multiple LaserScan displays:
- Topic: `/scan` (original)
- Topic: `/scan_corrupted` (noisy)
- Topic: `/scan_healthy` (filtered)

Compare them visually!

---

## Recording From Real Hardware

When you get your physical Leo Rover:

```bash
# On the Leo Rover, record all topics
ros2 bag record /scan /odom /tf /tf_static /cmd_vel /camera/image_raw -o real_leo_data

# Transfer bag file to your computer
scp leo@leo-rover:/path/to/real_leo_data.db3 ~/leo_ws/rosbag_data/

# Play back and process
ros2 bag play real_leo_data
# Run your same filtering pipeline!
```

---

## Customize Corruption Parameters

Edit `scripts/scan_corruptor.py` parameters:

```python
self.declare_parameter('noise_stddev', 0.05)      # Increase for more noise
self.declare_parameter('dropout_rate', 0.1)       # % of points to drop
self.declare_parameter('outlier_rate', 0.05)      # % of random outliers
self.declare_parameter('segment_dropout_prob', 0.1)  # Missing chunks
```

Run with custom params:
```bash
python3 scan_corruptor.py --ros-args \
  -p noise_stddev:=0.1 \
  -p dropout_rate:=0.2
```

## Customize Health Checker

Edit `scripts/scan_health_checker.py` parameters:

```python
self.declare_parameter('min_valid_percentage', 50.0)  # Min % valid points
self.declare_parameter('min_valid_points', 50)        # Min absolute count
self.declare_parameter('max_noise_stddev', 2.0)       # Max noise tolerance
self.declare_parameter('max_range_jump', 3.0)         # Max jump between scans
```

Make it stricter or more lenient based on your needs!

---

## Advanced: Record Multiple Scenarios

```bash
# Scenario 1: Straight corridor
./record_rosbag.sh
# Name: corridor_straight

# Scenario 2: Sharp turns
./record_rosbag.sh  
# Name: sharp_turns

# Scenario 3: Dynamic obstacles
./record_rosbag.sh
# Name: dynamic_obstacles

# Test your pipeline on all scenarios!
```

---

## Troubleshooting

**"No topics found in bag":**
```bash
# Check bag contents
ros2 bag info ~/leo_ws/rosbag_data/your_bag_name
```

**"Clock not publishing during playback":**
```bash
# Play with --clock option
ros2 bag play your_bag --clock
```

**"SLAM not building map from playback":**
```bash
# Make sure use_sim_time is true
ros2 param get /slam_toolbox use_sim_time

# Set it if needed
ros2 param set /slam_toolbox use_sim_time true
```

---

## What You Should See

### Clean Data:
- Smooth laser scans
- Consistent measurements
- Stable map building

### Corrupted Data:
- Scattered points
- Missing segments
- Noisy readings

### Filtered Data:
- Clean like original
- Bad scans rejected
- Good SLAM performance

This validates your code will work with real hardware! 🎯

---

## Next Steps

1. ✅ Record clean simulation data
2. ✅ Test SLAM with playback
3. ✅ Add corruption and test filtering
4. ✅ Tune filtering parameters
5. ✅ Document what "healthy" means for your use case
6. 🚀 Deploy to real Leo Rover!
