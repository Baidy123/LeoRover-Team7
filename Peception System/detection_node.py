#!/usr/bin/env python3
"""
Detection node (real robot + sim).

Acts as a single-source RealSense provider for the rest of the ROS graph:
the internal DetectionSystem opens the camera directly via pyrealsense2,
runs colour/depth detection, and this node publishes both:

  1. The raw images on standard ROS topics — so other nodes (SLAM,
     manipulator, RViz, etc.) can subscribe instead of opening the
     camera themselves.
  2. The detection results (/detected_objects + /detected_boxes) for
     the mission_controller to consume.

Headless safety: cv2.imshow / waitKey / destroyAllWindows are stubbed
out at import time so the underlying DetectionSystem doesn't crash on
a NUC without an X server.

Publishes:
  /camera/color/image_raw                      (sensor_msgs/Image, bgr8)
  /camera/aligned_depth_to_color/image_raw     (sensor_msgs/Image, passthrough)
  /detected_objects                            (std_msgs/String, JSON list)
  /detected_boxes                              (visualization_msgs/MarkerArray)

Each /detected_objects entry contains:
  color: 'r'/'g'/'b'/'p'/'y'
  type:  'block'/'box'
  world: [x, y, z]   in world_frame
  depth: float       camera-frame z
  size:  [w, h]

Frames:
  DetectionSystem returns positions in the camera frame
  (x=right, y=down, z=forward). Each detection is transformed into
  world_frame via tf2 before being published.

Parameters:
  color_params_file
  camera_frame   real: 'camera_color_optical_frame'
                 sim:  'depth_camera_link'
  world_frame    real: 'map' (provided by slam_toolbox)
                 sim:  'odom'
  sim_mode       (bool, default False)
  only_blocks    (bool, default False — if True, boxes are filtered out)
  process_rate   (Hz, default 10.0)
  publish_images (bool, default True — turn off if no other node needs raw images)
"""

# Headless-safe stubs MUST be installed before DetectionSystem is imported,
# because that module also imports cv2.
import cv2
cv2.imshow = lambda *a, **kw: None
cv2.waitKey = lambda *a, **kw: -1
cv2.destroyAllWindows = lambda *a, **kw: None

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

import tf2_ros
from tf2_geometry_msgs import do_transform_point

import json
import os
import sys

try:
    from ament_index_python.packages import get_package_share_directory
    _SCRIPTS_DIR = os.path.join(
        get_package_share_directory('leo_lidar_sim'), 'scripts')
    if _SCRIPTS_DIR not in sys.path:
        sys.path.insert(0, _SCRIPTS_DIR)
except Exception:
    pass

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.append(_THIS_DIR)

from detectionClass import DetectionSystem


COLOR_INFO = {
    'r': {'name': 'red',    'rgba': (1.0, 0.0, 0.0, 1.0)},
    'g': {'name': 'green',  'rgba': (0.0, 1.0, 0.0, 1.0)},
    'b': {'name': 'blue',   'rgba': (0.0, 0.0, 1.0, 1.0)},
    'p': {'name': 'purple', 'rgba': (0.5, 0.0, 0.5, 1.0)},
    'y': {'name': 'yellow', 'rgba': (1.0, 1.0, 0.0, 1.0)},
}


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('color_params_file',
                               '/home/student15/rspd_venv/src/colour_params.csv')
        # Real RealSense optical frame: camera_color_optical_frame
        # Sim:                          depth_camera_link
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        # Real + slam_toolbox: map; sim: odom
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('only_blocks', False)
        self.declare_parameter('process_rate', 10.0)
        self.declare_parameter('publish_images', True)

        self.color_params_file = self.get_parameter('color_params_file').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        sim_mode = self.get_parameter('sim_mode').value
        self.only_blocks = self.get_parameter('only_blocks').value
        process_rate = float(self.get_parameter('process_rate').value)
        self.publish_images = self.get_parameter('publish_images').value

        # ── Detector ──────────────────────────────────────────────────────
        if not os.path.isfile(self.color_params_file):
            self.get_logger().error(
                f'Color params file not found: {self.color_params_file}')
            raise FileNotFoundError(self.color_params_file)

        # Default DetectionSystem behaviour:
        #   sim_mode=True  -> external feed via self.color_image / self.depth_image
        #                     (depth in metres). We push images in from Gazebo
        #                     image topics if running in sim.
        #   sim_mode=False -> opens its own RealSense pipeline (depth in mm).
        self.detector = DetectionSystem(self.color_params_file,
                                        sim_mode=sim_mode)
        self.sim_mode = sim_mode

        # In sim mode we still need to receive Gazebo image topics, since
        # DetectionSystem doesn't have a Gazebo backend. (Real mode reads
        # the camera directly inside DetectionSystem.)
        if self.sim_mode:
            self.declare_parameter('rgb_topic', '/depth_camera/image')
            self.declare_parameter('depth_topic', '/depth_camera/depth_image')
            rgb_topic = self.get_parameter('rgb_topic').value
            depth_topic = self.get_parameter('depth_topic').value
            self.create_subscription(Image, rgb_topic, self._rgb_cb, 10)
            self.create_subscription(Image, depth_topic, self._depth_cb, 10)

        # ── ROS plumbing ─────────────────────────────────────────────────
        self.bridge = CvBridge()

        self.objects_pub = self.create_publisher(
            String, '/detected_objects', 10)
        self.boxes_pub = self.create_publisher(
            MarkerArray, '/detected_boxes', 10)

        if self.publish_images:
            self.rgb_pub = self.create_publisher(
                Image, '/camera/color/image_raw', 10)
            self.depth_pub = self.create_publisher(
                Image,
                '/camera/aligned_depth_to_color/image_raw',
                10)
        else:
            self.rgb_pub = None
            self.depth_pub = None

        period = 1.0 / max(process_rate, 0.1)
        self.create_timer(period, self._process_tick)

        # ── TF ────────────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'detection_node started: world={self.world_frame}, '
            f'camera={self.camera_frame}, sim={sim_mode}, '
            f'only_blocks={self.only_blocks}, rate={process_rate} Hz, '
            f'publish_images={self.publish_images}')

    # ─────────────────────────────────────────────────────────────────────
    # Sim image callbacks (real mode doesn't use these — DetectionSystem
    # reads from RealSense directly).
    def _rgb_cb(self, msg):
        try:
            self.detector.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'rgb decode failed: {e}',
                                   throttle_duration_sec=2.0)

    def _depth_cb(self, msg):
        try:
            self.detector.depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'depth decode failed: {e}',
                                   throttle_duration_sec=2.0)

    # ─────────────────────────────────────────────────────────────────────
    def _process_tick(self):
        try:
            # detect_all_blocks() internally calls _update_frame() which,
            # in real mode, blocks for the next RealSense frame; in sim
            # mode it reuses whatever color_image/depth_image we set above.
            objs = self.detector.detect_all_blocks() or []
        except Exception as e:
            self.get_logger().error(f'Detection failed: {e}',
                                    throttle_duration_sec=2.0)
            return

        # Pull the images that were used for this detection round so we
        # can republish them in step.
        result = self.detector._current_result
        if result is None:
            return
        color_image = result.get('color_image')
        depth_image = result.get('depth_image')
        if color_image is None or depth_image is None:
            return

        stamp = self.get_clock().now().to_msg()

        # 1) Publish raw images on standard topics so other nodes can
        # subscribe instead of opening the camera themselves.
        if self.publish_images:
            self._publish_images(color_image, depth_image, stamp)

        if self.only_blocks:
            objs = [o for o in objs if o.get('type') == 'block']

        # 2) Transform each detection into world_frame
        published_objects = []
        for obj in objs:
            world = self.camera_to_world(obj['position_3d'], stamp)
            if world is None:
                continue
            # 'real_size' is the field name returned by DetectionSystem;
            # fall back gracefully if a different version returns 'size'.
            size = obj.get('real_size') or obj.get('size') or (0.0, 0.0)
            published_objects.append({
                'color': obj['color'],
                'type': obj.get('type'),
                'world': [float(world[0]), float(world[1]), float(world[2])],
                'depth': float(obj['depth']),
                'size': [float(size[0]), float(size[1])],
            })

        # 3) Publish object list (consumed by mission_controller)
        msg = String()
        msg.data = json.dumps(published_objects)
        self.objects_pub.publish(msg)

        # 4) Publish markers (RViz visualisation only)
        self.publish_detected_boxes(published_objects, stamp)

        self.get_logger().info(
            f'Detected {len(published_objects)} objects',
            throttle_duration_sec=2.0)

    # ─────────────────────────────────────────────────────────────────────
    def _publish_images(self, color_image, depth_image, stamp):
        try:
            rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = self.camera_frame
            self.rgb_pub.publish(rgb_msg)

            depth_msg = self.bridge.cv2_to_imgmsg(depth_image,
                                                  encoding='passthrough')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.camera_frame
            self.depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().warn(f'image publish failed: {e}',
                                   throttle_duration_sec=2.0)

    # ─────────────────────────────────────────────────────────────────────
    def camera_to_world(self, pos_cam, stamp):
        """Transform a camera-frame point to world_frame using tf2."""
        pt = PointStamped()
        pt.header.frame_id = self.camera_frame
        pt.header.stamp = stamp
        pt.point.x = float(pos_cam[0])
        pt.point.y = float(pos_cam[1])
        pt.point.z = float(pos_cam[2])

        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            out = do_transform_point(pt, tf)
            return (out.point.x, out.point.y, out.point.z)
        except Exception as e:
            self.get_logger().warn(
                f'TF {self.world_frame} <- {self.camera_frame} failed: {e}',
                throttle_duration_sec=2.0)
            return None

    # ─────────────────────────────────────────────────────────────────────
    def publish_detected_boxes(self, objs, stamp):
        marker_array = MarkerArray()

        # Clear previous markers
        clear = Marker()
        clear.header.frame_id = self.world_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        idx = 0
        for obj in objs:
            info = COLOR_INFO.get(obj['color'])
            if info is None:
                continue

            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = stamp
            m.ns = info['name']
            m.id = idx
            idx += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD

            m.pose.position.x = obj['world'][0]
            m.pose.position.y = obj['world'][1]
            m.pose.position.z = obj['world'][2]
            m.pose.orientation.w = 1.0

            # Blocks small, boxes (bins) larger
            scale = 0.05 if obj.get('type') == 'block' else 0.30
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale

            r, g, b, a = info['rgba']
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a

            m.lifetime.sec = 1
            marker_array.markers.append(m)

        self.boxes_pub.publish(marker_array)

    # ─────────────────────────────────────────────────────────────────────
    def shutdown(self):
        try:
            self.detector.stop()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
