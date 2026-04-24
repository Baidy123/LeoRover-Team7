# Perception System

This repository provides a Python module for real-time detection of colored blocks and drop-off boxes using an Intel RealSense depth camera (or Gazebo simulation) and OpenCV. It serves as the perception front-end of a mobile manipulation system, providing 3D target positions, depth-based size classification, gripper orientation hints, and drop-off box localization.

---

## Features

- Real-time color-based detection in HSV space (red, green, blue, purple, yellow)
- 3D position estimation using RealSense depth or simulated depth
- **Depth-based real-world size computation** (pixel size + depth → metric size)
- **Automatic block vs. box classification** using size thresholds
- **Gripper orientation decision** (horizontal / vertical closing direction)
- **Graspability check** against gripper opening width
- Drop-off box center localization using Canny edge detection
- Dual-mode support: **real RealSense camera** and **Gazebo simulation**
- Interactive keyboard-based debug modes:
  - **Block mode** – single-color detection with size and type classification
  - **All mode** – multi-color detection across all five colors
  - **Orientation mode** – gripper direction and graspability
  - **Edge mode** – drop-off box center detection
  - **Target mode** – pick the best grasp target and print grasp command

---

## File Structure

- `detectionClass.py` – main module containing the `DetectionSystem` class and interactive debug interface
- `colour_params.csv` – HSV lower/upper bounds for each color (real camera only)

---

## Dependencies

- Python 3.8+
- pyrealsense2 (optional; only needed for real camera mode)
- OpenCV
- NumPy

Install all dependencies:

```bash
pip install pyrealsense2 opencv-python numpy
```

Make sure the Intel RealSense SDK and firmware are installed for real hardware use.

---

## HSV Parameter File

`colour_params.csv` contains HSV ranges for real-camera mode:

- Red
- Green
- Blue
- Yellow
- Purple

These thresholds can be tuned without modifying the source code.

Simulation mode uses a separate hard-coded set of HSV parameters (`sim_hsv_params`) because Gazebo colors differ from real camera output.

---

## How It Works (High-Level)

1. Camera pipeline initialization (RealSense for real, external image injection for sim)
2. Depth–color alignment
3. Depth range filtering (configurable min/max depth)
4. HSV color masking
5. Morphological closing to fill small holes inside masks
6. Contour extraction per color
7. Depth back-projection using camera intrinsics → 3D position
8. Pixel size × depth / focal length → **real-world size**
9. Size-based classification: block or box
10. Canny edge detection for drop-off box localization
11. OpenCV visualization for debug

---

## Size Computation

Real-world size is computed from pixel size and depth:

```
real_width  = pixel_width  × depth / fx
real_height = pixel_height × depth / fy
real_area   = pixel_area   × depth² / (fx × fy)
```

where `(fx, fy)` are camera focal lengths. This allows consistent size estimation regardless of distance, which is the basis for distinguishing small blocks (~2-3 cm) from large drop-off boxes (~30 cm).

---

## Running the Detector

```bash
python detectionClass.py
```

This starts the system in **block mode** (single-color detection) with keyboard-based debugging enabled.

---

## Keyboard Controls (Debug Only)

| Key | Function |
|-----|----------|
| `q` / `Esc` | Quit |
| `r` `g` `b` `p` `y` | Select color mask |
| `d` | Block detection mode (single color) |
| `a` | All-color detection mode |
| `o` | Orientation / graspability mode |
| `e` | Drop-off box edge detection mode |
| `t` | Trigger a target pick (prints grasp command) |

### Debug Interface Notice

The keyboard controls and the `run()` method are intended **only for debugging and visualization**. They allow inspection of detection results, masks, edges, and intermediate data.

---

## Configuration Parameters

The `DetectionSystem` constructor exposes the following parameters:

```python
DetectionSystem(
    color_params_file,
    sim_mode=False,              # True for Gazebo simulation
    min_depth=0.0,               # minimum detection distance (meters)
    max_depth=4.0,               # maximum detection distance (meters)
    box_edge_max_depth=0.35,     # shorter range for box edge detection
    use_morphology=True,         # apply closing to fill small mask holes
)
```

Additional runtime attributes that can be tuned:

- `size_threshold` (default `0.10` m) — cutoff between block and box classification
- `min_block_size` (default `20` px) — minimum pixel dimension for orientation mode

---

## Integration With ROS (Work in Progress)

Work is currently underway to integrate this perception module with a ROS node.

In the ROS workflow:

- The node will call methods such as
  `detect_block()`, `detect_all_blocks()`, `detect_block_orientation()`, `detect_box_edge()` programmatically
- Extracted information (target position, type, orientation, drop-off box center) will be converted into structured ROS messages
- ROS topics will be published for:
  - **Navigation module** – target position
  - **Manipulator Control module** – grasp position, gripper direction, drop-off box center

Once completed, the system will operate fully autonomously inside ROS, without any dependence on the interactive debug interface.

---

## DetectionSystem Class Overview

```python
class DetectionSystem:
    def __init__(self, color_params_file, sim_mode=False,
                 min_depth=0.0, max_depth=4.0, box_edge_max_depth=0.35,
                 use_morphology=True)

    # Single-color detection of the currently selected color.
    # Returns list of dicts with: centroid, position_3d, depth,
    # bounding_box, real_size, real_area, type ('block' | 'box').
    def detect_block(self)

    # Multi-color detection across all five colors.
    # Returns list of dicts with the same fields plus 'color'.
    def detect_all_blocks(self, classify_size=True)

    # Orientation and graspability check for the currently selected color.
    # Returns list of dicts with: long_edge, short_edge, grasp_dir
    # ('horizontal' | 'vertical'), can_grasp (bool).
    def detect_block_orientation(self, gripper_max_width=0.05)

    # Locate the drop-off box using Canny edge detection.
    # Returns a dict with center, area, box_points (or None).
    def detect_box_edge(self)

    # Debug entry point with interactive keyboard controls.
    def run(self, mode='block')

    # Release camera resources.
    def stop(self)
```

### Returned Data Example

```python
# detect_all_blocks()
[
    {
        'color': 'r',
        'type': 'block',
        'centroid': (320, 240),
        'position_3d': [0.042, -0.018, 0.523],
        'depth': 0.523,
        'bounding_box': (305, 225, 30, 30),
        'pixel_size': (30, 30),
        'real_size': (0.028, 0.029),
        'real_area': 0.000812,
    },
    ...
]

# detect_block_orientation()
[
    {
        'centroid': (320, 240),
        'position_3d': [0.042, -0.018, 0.523],
        'depth': 0.523,
        'bounding_box': (290, 230, 60, 20),
        'long_edge': 0.081,
        'short_edge': 0.029,
        'grasp_dir': 'vertical',
        'can_grasp': True,
    },
    ...
]
```
