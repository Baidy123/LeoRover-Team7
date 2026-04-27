#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import sys

sys.path.append('/home/student15/leo_ws/src/leo_lidar_sim/scripts')
from detectionClass import DetectionSystem


class DetectionRealWrapper(Node):
    def __init__(self):
        super().__init__('detection_real_wrapper')
        
        # 真实相机模式，DetectionSystem 自己管理 RealSense pipeline
        self.detector = DetectionSystem(
            '/home/student15/rspd_venv/src/colour_params.csv',
            sim_mode=False
        )
        
        # 发布目标位置
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # 定时调用检测（30Hz，匹配相机帧率）
        self.detect_timer = self.create_timer(1.0 / 30.0, self.detect_callback)
        
        # 定时重发最后目标（0.5秒）
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
        
        self.get_logger().info('Detection Real Wrapper started')
        self.get_logger().info('Keyboard: r=Red, g=Green, b=Blue, p=Purple, y=Yellow, q=Quit')
    
    def detect_callback(self):
        """定时检测回调，直接从 RealSense 获取帧"""
        try:
            blocks = self.detector.detect_block()
            
            if blocks:
                self.no_detection_count = 0
                
                # 选离画面中心最近的 block
                img_cx, img_cy = 320, 240
                blocks.sort(key=lambda b: (b['centroid'][0] - img_cx)**2 + (b['centroid'][1] - img_cy)**2)
                pos_3d = blocks[0]['position_3d']
                
                if self.should_update_goal(pos_3d):
                    self.last_goal = pos_3d
                    current_color = self.color_names[self.detector.current_index]
                    self.get_logger().info(
                        f'Target {current_color} detected: '
                        f'x={pos_3d[0]:.3f}, y={pos_3d[1]:.3f}, z={pos_3d[2]:.3f}'
                    )
                self.publish_goal(self.last_goal)
            else:
                self.no_detection_count += 1
                if self.no_detection_count > self.max_no_detection:
                    if self.last_goal is not None:
                        self.get_logger().warn('No detection for a while, clearing goal')
                        self.last_goal = None
            
            # 键盘控制
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                self.get_logger().info('Exit program')
                rclpy.shutdown()
            elif key == ord('r'):
                self.detector.current_index = 0
                self.last_goal = None
                self.get_logger().info('Switch to Red')
            elif key == ord('g'):
                self.detector.current_index = 1
                self.last_goal = None
                self.get_logger().info('Switch to Green')
            elif key == ord('b'):
                self.detector.current_index = 2
                self.last_goal = None
                self.get_logger().info('Switch to Blue')
            elif key == ord('p'):
                self.detector.current_index = 3
                self.last_goal = None
                self.get_logger().info('Switch to Purple')
            elif key == ord('y'):
                self.detector.current_index = 4
                self.last_goal = None
                self.get_logger().info('Switch to Yellow')
        
        except Exception as e:
            self.get_logger().error(f'Detection failed: {e}')
    
    def publish_goal(self, goal):
        """发布目标位置"""
        if goal is None:
            return
        
        pose = PoseStamped()
        pose.header.frame_id = 'camera_frame'  # 真实相机的 TF frame
        pose.header.stamp = self.get_clock().now().to_msg()    
        
        # RealSense 坐标系: x右 y下 z前（和 ROS 不同）
        # 转换到 ROS: x前 y左 z上
        pose.pose.position.x = float(goal[2])   # 相机z(前) → ROS x
        pose.pose.position.y = float(-goal[0])  # 相机x(右) → ROS y(左)
        pose.pose.position.z = float(goal[1])  # 相机y(下) → ROS z(上)
        pose.pose.orientation.w = 1.0
        
        self.goal_pub.publish(pose)
    
    def republish_goal(self):
        """定时重新发布最后已知目标"""
        if self.last_goal is not None:
            self.publish_goal(self.last_goal)
    
    def should_update_goal(self, new_goal):
        """目标变化超过阈值才更新"""
        if self.last_goal is None:
            return True
        
        threshold = 0.05  # 5cm
        dx = new_goal[2] - self.last_goal[2]  # 前后（深度）
        dy = new_goal[0] - self.last_goal[0]  # 左右
        distance = np.sqrt(dx**2 + dy**2)
        
        return distance > threshold


def main(args=None):
    rclpy.init(args=args)
    node = DetectionRealWrapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.detector.stop()
        node.destroy_node()

if __name__ == '__main__':
    main()