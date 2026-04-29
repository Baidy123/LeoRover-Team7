#!/usr/bin/env python3
"""
box_detector.py
===============
Coloured Box Detector Node

Uses the Leo Rover's front camera to detect a coloured box in the scene.
Computes the box's 3D position using depth from LiDAR scan, then
transforms it into the map frame and publishes as PoseStamped.

Topics subscribed:
  /camera/image_raw   (sensor_msgs/Image)       - RGB image
  /scan               (sensor_msgs/LaserScan)    - LiDAR for depth

Topics published:
  /box_detection      (geometry_msgs/PoseStamped) - box pose in map frame
  /box_detection_image (sensor_msgs/Image)        - debug image with contour

TF used:
  camera_link → map  (via TF2 lookup)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header

import tf2_ros
import tf2_geometry_msgs

import cv2
import numpy as np
from cv_bridge import CvBridge
import math


# ── Color HSV ranges for detection ───────────────────────────────────────────
# Tune these in Gazebo by running the debug image topic
COLOR_RANGES = {
    'red': [
        (np.array([0,  120,  70]), np.array([10,  255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255])),
    ],
    'green': [
        (np.array([40,  60, 60]), np.array([80, 255, 255])),
    ],
    'blue': [
        (np.array([100, 80, 50]), np.array([130, 255, 255])),
    ],
}

# Which color to look for (can be made a ROS parameter)
TARGET_COLOR = 'red'

# Minimum contour area in pixels to filter noise
MIN_CONTOUR_AREA = 800


class BoxDetector(Node):

    def __init__(self):
        super().__init__('box_detector')

        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('target_color',     TARGET_COLOR)
        self.declare_parameter('min_contour_area', MIN_CONTOUR_AREA)
        self.declare_parameter('publish_rate_hz',  5.0)
        # Camera horizontal FOV in radians
        self.declare_parameter('camera_hfov',  1.047)  # ~60 degrees
        self.declare_parameter('camera_frame', 'camera_link')

        self.target_color     = self.get_parameter('target_color').value
        self.min_area         = self.get_parameter('min_contour_area').value
        self.camera_hfov      = self.get_parameter('camera_hfov').value
        self.camera_frame     = self.get_parameter('camera_frame').value

        # ── Utilities ────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── TF2 ─────────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── State ────────────────────────────────────────────────────────────
        self.latest_scan     = None
        self.detection_done  = False   # stop publishing once found

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw',
            self._image_cb, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            LaserScan, '/scan',
            self._scan_cb, 10,
            callback_group=self.cb_group)

        # ── Publishers ───────────────────────────────────────────────────────
        self.pose_pub  = self.create_publisher(PoseStamped, '/box_detection',       10)
        self.debug_pub = self.create_publisher(Image,       '/box_detection_image', 10)

        self.get_logger().info(
            f'BoxDetector ready. Looking for color: {self.target_color}')

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _image_cb(self, msg: Image):
        if self.detection_done:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        cx, cy, found = self._detect_color_centroid(cv_img)

        if not found:
            return

        # ── Estimate distance using LiDAR scan at the box's bearing ─────────
        distance = self._estimate_depth(cx, cv_img.shape[1])

        if distance is None or distance < 0.1 or distance > 8.0:
            return

        # ── Compute 3D point in camera frame ─────────────────────────────────
        # Horizontal angle from image centre
        img_w  = cv_img.shape[1]
        angle  = ((cx - img_w / 2.0) / img_w) * self.camera_hfov
        x_cam  = distance * math.cos(angle)
        y_cam  = -distance * math.sin(angle)   # right is negative y in camera
        z_cam  = 0.0                            # assume flat ground for coarse estimate

        # ── Transform to map frame ────────────────────────────────────────────
        pose_cam = PoseStamped()
        pose_cam.header.frame_id = self.camera_frame
        pose_cam.header.stamp    = self.get_clock().now().to_msg()
        pose_cam.pose.position.x = x_cam
        pose_cam.pose.position.y = y_cam
        pose_cam.pose.position.z = z_cam
        pose_cam.pose.orientation.w = 1.0

        try:
            pose_map = self.tf_buffer.transform(pose_cam, 'map', timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
            return

        self.get_logger().info(
            f'Box detected: distance={distance:.2f}m  '
            f'map=({pose_map.pose.position.x:.2f}, {pose_map.pose.position.y:.2f})')

        self.pose_pub.publish(pose_map)

        # ── Publish debug image ───────────────────────────────────────────────
        cv2.circle(cv_img, (int(cx), int(cy)), 15, (0, 255, 0), 3)
        cv2.putText(cv_img,
                    f'{self.target_color} box  {distance:.2f}m',
                    (int(cx) - 80, int(cy) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        self.debug_pub.publish(debug_msg)

    # ─────────────────────────────────────────────────────────────────────────
    # Detection helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _detect_color_centroid(self, bgr_img):
        """
        Returns (cx, cy, found) for the largest blob of target color.
        """
        hsv  = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        ranges = COLOR_RANGES.get(self.target_color, [])
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, lo, hi)

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return 0.0, 0.0, False

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return 0.0, 0.0, False

        M  = cv2.moments(largest)
        if M['m00'] == 0:
            return 0.0, 0.0, False

        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        return cx, cy, True

    def _estimate_depth(self, cx_pixel: float, img_width: int) -> float:
        """
        Use the laser scan to get depth at the horizontal angle
        corresponding to cx_pixel in the image.
        """
        if self.latest_scan is None:
            return None

        scan = self.latest_scan

        # Map pixel x → scan angle
        # Camera is centred; image x goes left to right, scan angle goes
        # from angle_min (right) to angle_max (left) for most LiDARs
        norm_x  = (cx_pixel / img_width) - 0.5           # −0.5 to +0.5
        bearing = -norm_x * self.camera_hfov              # right = negative angle

        # Find nearest scan index
        num_rays = len(scan.ranges)
        idx = int((bearing - scan.angle_min) / scan.angle_increment)
        idx = max(0, min(idx, num_rays - 1))

        # Average a small window for stability
        window  = 5
        lo      = max(0, idx - window)
        hi      = min(num_rays - 1, idx + window)
        samples = [r for r in scan.ranges[lo:hi+1]
                   if scan.range_min < r < scan.range_max and not math.isnan(r)]

        if not samples:
            return None

        return float(np.median(samples))


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('BoxDetector interrupted by user.')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
