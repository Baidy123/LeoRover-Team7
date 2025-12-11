
# Project Overview

This project implements a complete mobile manipulation system capable of autonomously detecting a target object, navigating toward it, and transporting it to a designated drop-off box. The robot integrates perception, SLAM, path planning, navigation control, manipulation, and low-level actuation into a unified framework.

The following sections provide detailed documentation for each subsystem within the project, including the Perception System, SLAM module, Navigation module,Trajectory Path Planning module, Manipulator Control, and Motor Control—describing their roles, algorithms, and implementation details.


# Trajectory Path Planning
TO BE DONE

---

# SLAM
TO BE DONE

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

