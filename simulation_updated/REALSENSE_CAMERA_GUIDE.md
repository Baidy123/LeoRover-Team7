# Intel RealSense D435 Camera - Usage Guide

## What Was Added:

✅ **Intel RealSense D435** depth camera on front of Leo Rover
✅ **RGB Camera** - Color images for object detection
✅ **Depth Camera** - Distance/depth information
✅ **Automatic bridges** in complete_system.launch.py

---

## NO TERMINAL CHANGES NEEDED!

Just rebuild and launch as normal:

```bash
cd ~/leo_ws
colcon build --packages-select leo_lidar_sim
source install/setup.bash

# Same command as before!
ros2 launch leo_lidar_sim complete_system.launch.py
```

Everything starts automatically! Camera bridges are included.

---

## Available Camera Topics:

```bash
# RGB Color Image
/realsense/color/image            # sensor_msgs/Image (640x480, RGB)
/realsense/color/camera_info      # sensor_msgs/CameraInfo

# Depth Image
/realsense/depth/image            # sensor_msgs/Image (640x480, depth in meters)
/realsense/depth/camera_info      # sensor_msgs/CameraInfo
```

---

## View Camera in RViz:

### Color Camera:
1. In RViz, click **Add** → **Camera**
2. Image Topic: `/realsense/color/image`
3. You'll see RGB view!

### Depth Camera:
1. Click **Add** → **Image**
2. Image Topic: `/realsense/depth/image`
3. See depth visualization (closer = darker)

---

## Use Camera in Your Code:

### Subscribe to RGB:

```python
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MyDetector(Node):
    def __init__(self):
        super().__init__('detector')
        self.bridge = CvBridge()
        
        self.rgb_sub = self.create_subscription(
            Image,
            '/realsense/color/image',
            self.rgb_callback,
            10
        )
    
    def rgb_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Your color detection code here
        # Example: Detect red objects
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        
        # Find contours, etc.
```

### Subscribe to Depth:

```python
def __init__(self):
    super().__init__('detector')
    self.bridge = CvBridge()
    
    self.depth_sub = self.create_subscription(
        Image,
        '/realsense/depth/image',
        self.depth_callback,
        10
    )

def depth_callback(self, msg):
    # Convert to numpy array
    depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    
    # depth_image is in meters
    # Get distance at center pixel
    center_x = msg.width // 2
    center_y = msg.height // 2
    distance = depth_image[center_y, center_x]
    
    self.get_logger().info(f'Distance to object: {distance:.2f}m')
```

### Combined RGB + Depth:

```python
class RGBDDetector(Node):
    def __init__(self):
        super().__init__('rgbd_detector')
        self.bridge = CvBridge()
        
        self.rgb_image = None
        self.depth_image = None
        
        self.rgb_sub = self.create_subscription(
            Image, '/realsense/color/image', self.rgb_callback, 10
        )
        
        self.depth_sub = self.create_subscription(
            Image, '/realsense/depth/image', self.depth_callback, 10
        )
        
        self.timer = self.create_timer(0.1, self.process)
    
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
    
    def process(self):
        if self.rgb_image is None or self.depth_image is None:
            return
        
        # Find colored box in RGB
        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        
        # Find largest contour
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # Get distance from depth
                distance = self.depth_image[cy, cx]
                
                self.get_logger().info(f'Red box at ({cx}, {cy}), distance: {distance:.2f}m')
```

---

## Camera Specifications:

- **Position**: Front of rover (15cm forward, 8cm up)
- **Resolution**: 640x480
- **Frame Rate**: 30 Hz
- **FOV**: 69° horizontal
- **Depth Range**: 0.1m to 10m
- **Frame**: `realsense_camera_optical_frame`

---

## Test Camera:

```bash
# Check if topics are publishing
ros2 topic list | grep camera

# Should see:
# /realsense/color/image
# /camera/color/camera_info
# /realsense/depth/image
# /camera/depth/camera_info

# Check rates
ros2 topic hz /realsense/color/image
ros2 topic hz /realsense/depth/image
# Both should show ~30 Hz

# View an image
ros2 run rqt_image_view rqt_image_view
# Select /realsense/color/image
```

---

## Example: Color-Based Sorting with Distance

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorSorter(Node):
    def __init__(self):
        super().__init__('color_sorter')
        self.bridge = CvBridge()
        
        self.rgb_sub = self.create_subscription(Image, '/realsense/color/image', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/realsense/depth/image', self.depth_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.rgb_image = None
        self.depth_image = None
        
        # Color ranges (HSV)
        self.colors = {
            'red': ((0, 100, 100), (10, 255, 255)),
            'green': ((40, 50, 50), (80, 255, 255)),
            'blue': ((100, 100, 100), (130, 255, 255))
        }
        
        self.timer = self.create_timer(0.1, self.process)
    
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
    
    def detect_color(self, color_name):
        if self.rgb_image is None:
            return None, None
        
        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
        lower, upper = self.colors[color_name]
        mask = cv2.inRange(hsv, lower, upper)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, None
        
        # Largest contour
        largest = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M['m00'] < 100:  # Too small
            return None, None
        
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        # Get distance
        if self.depth_image is not None:
            distance = self.depth_image[cy, cx]
            return (cx, cy), distance
        
        return (cx, cy), None
    
    def process(self):
        # Try to find red box
        pos, dist = self.detect_color('red')
        if pos and dist and dist < 2.0:  # Within 2 meters
            self.get_logger().info(f'Red box detected at {dist:.2f}m')
            # Move toward it (simple control)
            cmd = Twist()
            
            # Center in image?
            error = pos[0] - 320  # Image center
            if abs(error) > 50:
                cmd.angular.z = -error * 0.001  # Turn
            else:
                cmd.linear.x = 0.1  # Move forward
            
            self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = ColorSorter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Troubleshooting:

**No camera topics?**
```bash
# Check bridges are running
ros2 node list | grep bridge

# Manually restart camera bridges
ros2 run ros_gz_bridge parameter_bridge /realsense/color/image@sensor_msgs/msg/Image[gz.msgs.Image
```

**Black depth image?**
- Normal if no objects in view
- Depth only shows 0.1m to 10m range

**Camera in wrong position?**
- Check in Gazebo - should be black box on front of rover
- In RViz, enable TF display to see `realsense_camera_link`

---

## Summary:

✅ **No terminal changes** - everything automatic
✅ **RGB + Depth** available
✅ **Ready for your code** - just subscribe to topics
✅ **Perfect for color sorting** task!

The camera is already working! Just rebuild and launch. 🎥
