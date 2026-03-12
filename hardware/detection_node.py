#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# Import the detection class
from leo_lidar_sim.detectionClass import Detection


class DetectionSimWrapper(Node):
    """
    ROS2 wrapper for color detection node
    Publishes detected object positions as navigation goals
    """
    
    def __init__(self):
        super().__init__('detection_sim_wrapper')
        
        # Parameter is set by launch file, no need to declare
        
        self.bridge = CvBridge()
        
        # Get package share directory for installed files
        pkg_dir = get_package_share_directory('leo_lidar_sim')
        csv_path = os.path.join(pkg_dir, 'config', 'colour_params.csv')
        
        self.get_logger().info(f'Loading colour parameters from: {csv_path}')
        
        # Initialize detection class with colour parameters
        self.detector = Detection(csv_path)
        
        # Subscribers
        self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        # Publisher for detected goal positions
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Store latest images
        self.latest_rgb = None
        self.latest_depth = None
        
        # Detection parameters
        self.current_color = None  # 'red', 'green', 'blue', etc.
        
        self.get_logger().info('Detection Sim Wrapper started!')
        self.get_logger().info('Waiting for camera images...')
        self.get_logger().info('Press r/g/b in OpenCV window to detect colors')
    
    def image_callback(self, msg):
        """Store latest RGB image"""
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')
    
    def depth_callback(self, msg):
        """Store latest depth image and run detection if RGB available"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Run detection if both images available
            if self.latest_rgb is not None:
                self.process_detection()
                
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
    
    def process_detection(self):
        """Run detection and publish goal if object detected"""
        if self.latest_rgb is None or self.latest_depth is None:
            return
        
        # Show image with color selection
        display_img = self.latest_rgb.copy()
        
        # Add instructions
        cv2.putText(display_img, 'Press: r=red, g=green, b=blue, p=purple, y=yellow, q=quit',
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # If color selected, run detection
        if self.current_color:
            # Run detection
            result = self.detector.detect(
                self.latest_rgb,
                self.latest_depth,
                self.current_color
            )
            
            if result and 'position' in result:
                # Extract 3D position
                x, y, z = result['position']
                
                self.get_logger().info(
                    f'Target {self.current_color.capitalize()} detected: '
                    f'x={x:.2f}, y={y:.2f}, z={z:.2f}'
                )
                
                # Create and publish goal pose
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = 'camera_depth_frame'
                
                # Position from detection
                goal.pose.position.x = float(z)  # Depth is forward (x in robot frame)
                goal.pose.position.y = float(-x)  # Image x is -y in robot frame
                goal.pose.position.z = float(-y)  # Image y is -z in robot frame
                
                # Orientation (facing the object)
                goal.pose.orientation.w = 1.0
                
                self.goal_pub.publish(goal)
                
                # Draw detection on image
                if 'bbox' in result:
                    x1, y1, x2, y2 = result['bbox']
                    cv2.rectangle(display_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(display_img, f'{self.current_color}: ({x:.2f}, {y:.2f}, {z:.2f})',
                               (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Display image
        cv2.imshow('Detection', display_img)
        key = cv2.waitKey(1) & 0xFF
        
        # Handle key presses
        if key == ord('r'):
            self.current_color = 'red'
            self.get_logger().info('Detecting RED objects')
        elif key == ord('g'):
            self.current_color = 'green'
            self.get_logger().info('Detecting GREEN objects')
        elif key == ord('b'):
            self.current_color = 'blue'
            self.get_logger().info('Detecting BLUE objects')
        elif key == ord('p'):
            self.current_color = 'purple'
            self.get_logger().info('Detecting PURPLE objects')
        elif key == ord('y'):
            self.current_color = 'yellow'
            self.get_logger().info('Detecting YELLOW objects')
        elif key == ord('q'):
            self.get_logger().info('Quitting detection')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = DetectionSimWrapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
