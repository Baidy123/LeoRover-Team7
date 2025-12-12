
# Project Overview

This project implements a complete mobile manipulation system capable of autonomously detecting a target object, navigating toward it, and transporting it to a designated drop-off box. The robot integrates perception, SLAM, path planning, navigation control, manipulation, and low-level actuation into a unified framework.

The following sections provide detailed documentation for each subsystem within the project, including the Perception System, SLAM module, Navigation module,Trajectory Path Planning module, Manipulator Control, and Motor Control—describing their roles, algorithms, and implementation details.


# Trajectory Path Planning
This repository contains the implementation of **Trajectory Planning** for the Leo Rover v1.8 using ROS2 Jazzy on a NUC and Raspberry Pi setup.

---
### Overview

- Global path planning using ROS2
- using controller for path following
- configurable planners and tunable gains
- intended to integrate with Navigation2 (NAV2) stack
- located in 'trajectory_planning'

---
### Requirements

- ROS2 Jazzy
- Nav2 stack
- Python 3.10+

---
### Run Trajectory Planner
```python
ros2 launch trajectory_planning trajectory_launch.py
```


---
# SLAM
This repository contains the implementation of **SLAM toolbox** for the Leo Rover v1.8 using ROS2 Jazzy on a NUC and Raspberry Pi setup.

---
### Overview

- using **slam_toolbox** to run 2D SLAM
- integrating **EKF** from robot_localization
- using slam_launch.py
- provides 'odom -> base_link -> laser' TF tree
- Publishing '/map' , '/odom', '/scan' and 'tf' frames
- integrated Lidar with the NUC
- located in 'slam/'
  
---
### Requirements

- ROS2 Jazzy
- slam_toolbox
- robot_localization
- Python 3.10+
- Nav2 stack


---
### Run SLAM
```python
ros2 launch slam_system slam_launch.py
```


---
# Installations

The SLAM and Navigation will be done based on the topics received from RPLiDAR A2M12.

Clone the lidar package: 
```python
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
```

Clone the Leo Rover ROS2 package
```python
git clone https://github.com/LeoRover/leo_robot-ros2.git
```


---
# File Structure

## leo_nav2

## launch folder: 

**ekf_launch.py ->** Used for robot localization by starting the Extended Kalman Filter
               
               
**slam_launch.py ->** Starts the slam toolbox node


**nav2_launch.py ->** Starts the entire Navigation2 stack

               
## config folder: 

**ekf.yaml ->** Defines the configuration for EKF


**slam_toolbox.yaml ->** Configure the file for slam_toolbox


**nav2_params.yaml ->** Configure the Nav2 System

---
# Important

---
## Building the Workspace
```bash
colcon build --symlink-install \
source install/setup.bash
```

---
## Launching the LIDAR with map
```python
ros2 launch rplidar_ros view_rplidar_a2m12_launch.py
```
---
## Launch EKF (Odometry Filter)
```python
ros2 launch leo_nav2 ekf_launch.py
```

---
## Launch SLAM Toolbox
```python
ros2 launch leo_nav2 slam_launch.py
```

---
## Launch Navigation
```python
ros2 launch leo_nav2 nav2_launch.py
```


---
# Perception System

This repository provides a Python module for real-time detection of colored blocks and the localization of a drop-off box center using an Intel RealSense depth camera and OpenCV. It serves as the perception front-end of a mobile manipulation system, providing 2D/3D target positions, simple orientation estimation, and drop-off box localization.

---

## Features

- Real-time color-based detection of blocks in HSV space  
- 3D position estimation using RealSense depth information  
- Block orientation estimation (0° / 90°) using bounding-box aspect ratio  
- Drop-off box detection using Canny edge and contour fitting  
- Interactive keyboard-based debug modes:
  - **Block mode** – color mask detection + depth extraction  
  - **Orientation mode** – block orientation classification  
  - **Edge mode** – drop-off box center detection  

---

## File Structure

- `detectionClass.py` – main module containing the `DetectionSystem` class and interactive debug interface  
- `colour_params.csv` – HSV lower/upper bounds for different colors

---

## Dependencies

- Python 3.8+  
- pyrealsense2  
- OpenCV  
- NumPy  

Install all dependencies:

```bash
pip install pyrealsense2 opencv-python numpy
```

Make sure Intel RealSense SDK & firmware are installed.

---

## HSV Parameter File

`colour_params.csv` contains HSV ranges for:

- Red  
- Green  
- Blue  
- Yellow  
- Purple  

These thresholds can be tuned without modifying the source code.

---

## How It Works (High-Level)

1. RealSense camera pipeline initialization  
2. Depth–color alignment  
3. HSV color masking for block detection  
4. Depth back-projection for 3D localization  
5. Canny edge detection for drop-off box center estimation  
6. OpenCV visualization for debug  

---

## Running the Detector

```bash
python detectionClass.py
```

This starts the system in **block mode** with keyboard-based debugging enabled.

---

## Keyboard Controls (Debug Only)

| Key | Function |
|-----|----------|
| `q` / `Esc` | Quit |
| `r` `g` `b` `p` `y` | Select color mask |
| `d` | Block detection mode |
| `o` | Orientation detection mode |
| `e` | Drop-off box detection mode |

### Debug Interface Notice  
The keyboard controls and the `run()` method are intended **only for debugging and visualization**.  
They allow inspection of detection results, masks, edges, and intermediate data.

### In deployment  
Higher-level components (Navigation / Manipulator Control) will **directly call the detection methods**, and the debug interface will **not** be used.

---

## Integration With ROS (Work in Progress)

Work is currently underway to integrate this perception module with a ROS node.

In the ROS workflow:

- The node will call methods such as  
  `detect_block()`, `detect_block_orientation()`, `detect_box_edge()` programmatically  
- Extracted information (target position, orientation, drop-off box center) will be converted into structured ROS messages  
- ROS topics will be published for:
  - **Navigation module** – target position  
  - **Manipulator Control module** – position + orientation; drop-off box center  

Once completed, the system will operate fully autonomously inside ROS, without any dependence on the interactive debug interface.

---

## DetectionSystem Class Overview

```python
class DetectionSystem:
    def __init__(self, color_params_file)
    def detect_all_blocks(self)
    def detect_block(self)
    def detect_block_orientation(self)
    def detect_box_edge(self)
    def run(self, mode='block')
    def stop(self)
```
# Arm Motion Control
This repository contains the Motion Planning and control for the MyCobot 280-pi planned on A NUC and executed on the arm using serial connection
## Overview
- Uses RViz2 to visualise mycobot 280-pi
- Uses Moveit2 for controllers and motion planning
- Plans motion on NUC
- Serial connection sync node ran on mycobot 280_pi
## Requirements
- moveit_config_utils
- ROS2 jazzy
- RViz2
- Python 3.8+
### Building the workspace
```python
colcon build
source install/setup.bash
```
## To Launch Motion Planning
### Run on external processor
```python
ros2 launch mycobot_moveit_config move_group.launch.py
```
## To Sync Motion Planning 
### Run on MyCobot 280-pi
```python
ros2 run mycobot_280_moveit2_control sync_plan.py
```

## File Structure
### NUC_ros2
**mycobot_description ->** Contains MyCobot 280-pi URDF description


### mycobot_ros2
**mycobot_280/mycobot_280_moveit2_control ->** Contains arm controller nodes
**mycobot_description ->** Contains MyCobot 280-pi URDF description
**mycobot_communication ->** Contains additional communication nodes for ROS2 topics and services


