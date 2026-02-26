#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np
import sys

sys.path.append('/home/student31/leo_ws/src/leo_lidar_sim/scripts')
from detectionClass import DetectionSystem

class DetectionSimWrapper(Node):
    def __init__(self):
        super().__init__('detection_sim_wrapper')
        
        
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        
        self.bridge = CvBridge()
        
        self.detector = DetectionSystem('/home/student31/Downloads/colour_params.csv', sim_mode=True)
        
        image_sub = Subscriber(self, Image, '/depth_camera/image')
        depth_sub = Subscriber(self, Image, '/depth_camera/depth_image')
        
        self.sync = ApproximateTimeSynchronizer(
            [image_sub, depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.republish_timer = self.create_timer(0.5, self.republish_goal)
        
        self.last_goal = None
        self.no_detection_count = 0
        self.max_no_detection = 20  
        
        self.color_names = {
            0: 'Red',
            1: 'Green', 
            2: 'Blue',
            3: 'Purple',
            4: 'Yellow'
        }
        
        self.get_logger().info('Detection Sim Wrapper started (use_sim_time=True)')

    
    def sync_callback(self, image_msg, depth_msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
            self.detector.color_image = rgb
            self.detector.depth_image = depth
            
            blocks = self.detector.detect_block()
            
            if blocks:
                self.no_detection_count = 0
                
                img_cx, img_cy = 320, 240
                blocks.sort(key=lambda b: (b['centroid'][0] - img_cx)**2 + (b['centroid'][1] - img_cy)**2)
                pos_3d = blocks[0]['position_3d']
                
                if self.should_update_goal(pos_3d):
                    self.last_goal = pos_3d
                    current_color = self.color_names[self.detector.current_index]
                    self.get_logger().info(
                        f'Target {current_color} detected: '
                        f'x={pos_3d[0]:.2f}, y={pos_3d[1]:.2f}, z={pos_3d[2]:.2f}'
                    )
                self.publish_goal(self.last_goal)
            else:
                self.no_detection_count += 1
                if self.no_detection_count > self.max_no_detection:
                    if self.last_goal is not None:
                        self.get_logger().warn('Clean last goal')
                        self.last_goal = None
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info('Exit program')
                rclpy.shutdown()
            elif key == ord('r'):
                self.detector.current_index = 0
                self.last_goal = None  
                self.get_logger().info('Switch to red detection')
            elif key == ord('g'):
                self.detector.current_index = 1
                self.last_goal = None
                self.get_logger().info('Switch to green detection')
            elif key == ord('b'):
                self.detector.current_index = 2
                self.last_goal = None
                self.get_logger().info('Switch to blue detection')
            elif key == ord('p'):
                self.detector.current_index = 3
                self.last_goal = None
                self.get_logger().info('Switch to purple detection')
            elif key == ord('y'):
                self.detector.current_index = 4
                self.last_goal = None
                self.get_logger().info('Switch to yellow detection')
            
        except Exception as e:
            self.get_logger().error(f'Processing failed: {e}')
    
    def publish_goal(self, goal):
        if goal is None:
            return
        
        pose = PoseStamped()
        pose.header.frame_id = 'depth_camera_link'
        
        
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = float(goal[2])
        pose.pose.position.y = float(-goal[0])
        pose.pose.orientation.w = 1.0
        
        self.goal_pub.publish(pose)
    
    def republish_goal(self):
        if self.last_goal is not None:
            self.publish_goal(self.last_goal)
    
    def should_update_goal(self, new_goal):
        if self.last_goal is None:
            return True
        
        threshold = 0.05  # 5cm
        dy = new_goal[0] - self.last_goal[0]
        dx = new_goal[2] - self.last_goal[2]
        distance = np.sqrt(dx**2 + dy**2)
        
        return distance > threshold

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

if __name__ == '__main__':
    main()
