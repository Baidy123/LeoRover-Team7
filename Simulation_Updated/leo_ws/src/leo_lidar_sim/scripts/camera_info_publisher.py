#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoPublisher(Node):
    """
    Publishes camera_info for RViz camera display.
    """
    
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Publishers for both color and depth
        self.color_info_pub = self.create_publisher(CameraInfo, '/realsense/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/realsense/depth/camera_info', 10)
        
        self.timer = self.create_timer(0.033, self.publish_camera_info)  # 30 Hz
        
        self.get_logger().info('RealSense Camera Info Publisher started')
    
    def publish_camera_info(self):
        msg = CameraInfo()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'realsense_camera_optical_frame'
        
        # Image dimensions
        msg.width = 640
        msg.height = 480
        
        # Distortion model
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Intrinsic camera matrix
        fx = fy = 386.0  # Approximate for RealSense D435
        cx = 320.0
        cy = 240.0
        
        msg.k = [fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0]
        
        # Rectification matrix
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        
        # Projection matrix
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        
        # Publish to both color and depth
        self.color_info_pub.publish(msg)
        self.depth_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
