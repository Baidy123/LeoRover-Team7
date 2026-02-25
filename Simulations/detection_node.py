#!/usr/bin/env python3


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import PoseStamped
# from cv_bridge import CvBridge
# from message_filters import ApproximateTimeSynchronizer, Subscriber
# import cv2
# import numpy as np
# import sys
# import time

# sys.path.append('/home/student15/leo_ws/src/leo_lidar_sim/scripts')
# from detectionClass import DetectionSystem
# # from detectionClass import DetectionSystem

# class DetectionSimWrapper(Node):
#     def __init__(self):
#         super().__init__('detection_sim_wrapper')
        
#         self.bridge = CvBridge()
        
#         # 初始化你的检测系统
#         self.detector = DetectionSystem('/home/student15/rspd_venv/src/colour_params.csv', sim_mode=True)
#         # 注意：不调用 detector.run()，因为我们从 ROS 获取图像
        
#         # 使用 message_filters 同步 RGB 和 Depth
#         image_sub = Subscriber(self, Image, '/depth_camera/image')
#         depth_sub = Subscriber(self, Image, '/depth_camera/depth_image')
        
#         # 时间同步器
#         self.sync = ApproximateTimeSynchronizer(
#             [image_sub, depth_sub],
#             queue_size=10,
#             slop=0.1  # 允许 0.1 秒时间差
#         )
#         self.sync.registerCallback(self.sync_callback)
        
#         # 发布目标位置
#         self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
#         self.last_goal = None
        
#                 # 颜色数组对应关系
#         self.color_names = {
#             0: 'Red',
#             1: 'Green', 
#             2: 'Blue',
#             3: 'Purple',
#             4: 'Yellow'
#         }
        
#         self.get_logger().info('Detection Sim Wrapper started')
#         self.get_logger().info('键盘控制: r=红色, g=绿色, b=蓝色, p=紫色, y=黄色, q=退出')
    
#     def sync_callback(self, image_msg, depth_msg):
#         """同步接收 RGB 和 Depth"""
#         try:
#             # 转换为 OpenCV 格式
#             rgb = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
#             depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
#             # 设置到检测器
#             self.detector.color_image = rgb
#             self.detector.depth_image = depth
            
#             # 调用检测方法
#             blocks = self.detector.detect_block()
#             # print(f"blocks: {blocks}")
            
#             if blocks:
#                 img_cx, img_cy = 320, 240
#                 blocks.sort(key=lambda b: (b['centroid'][0] - img_cx)**2 + (b['centroid'][1] - img_cy)**2)
#                 pos_3d = blocks[0]['position_3d']
                
#                 # 检查是否需要更新目标
#                 # if self.should_update_goal(pos_3d):
#                 if self.should_update_goal(pos_3d):
#                     # 发布目标相对于相机的位置
#                     print(f"Publishing goal: {pos_3d}")
#                     pose = PoseStamped()
#                     pose.header.frame_id = 'depth_camera_link'
#                     pose.header.stamp = self.get_clock().now().to_msg()
                    
#                     # 直接使用检测结果
#                     pose.pose.position.x = float(pos_3d[1])
#                     pose.pose.position.y = float(-pos_3d[0])
#                     # pose.pose.position.z = float(pos_3d[2])
#                     pose.pose.orientation.w = 1.0
                    
#                     self.goal_pub.publish(pose)
                    
#                     current_color = self.color_names[self.detector.current_index]
#                     self.get_logger().info(
#                         f'检测到 {current_color} 目标: '
#                         f'x={pos_3d[0]:.2f}, y={pos_3d[1]:.2f}, z={pos_3d[2]:.2f}'
#                     )
                    
#                     self.last_goal = pos_3d
#                 else:
#                     # time.sleep(0.5)
#                     # print("Goal not updated (too close to last)")
#                     print(f"Publishing goal: {self.last_goal}")
#                     pose = PoseStamped()
#                     pose.header.frame_id = 'depth_camera_link'
#                     pose.header.stamp = self.get_clock().now().to_msg()
                    
#                     # 直接使用检测结果
#                     pose.pose.position.x = float(self.last_goal[1])
#                     pose.pose.position.y = float(-self.last_goal[0])
#                     # pose.pose.position.z = float(pos_3d[2])
#                     pose.pose.orientation.w = 1.0
                    
#                     self.goal_pub.publish(pose)
            
#             # 可视化
#             # cv2.imshow('Detection', rgb)
#             else:
#                 print(f"Publishing goal: {self.last_goal}")
#                 pose = PoseStamped()
#                 pose.header.frame_id = 'depth_camera_link'
#                 pose.header.stamp = self.get_clock().now().to_msg()
                    
#                 # 直接使用检测结果
#                 pose.pose.position.x = float(self.last_goal[1])
#                 pose.pose.position.y = float(-self.last_goal[0])
#                 # pose.pose.position.z = float(pos_3d[2])
#                 pose.pose.orientation.w = 1.0
                    
#                 self.goal_pub.publish(pose)
#             # 键盘控制
#             key = cv2.waitKey(1) & 0xFF
#             if key == ord('q') or key == 27:  # q 或 ESC
#                 self.get_logger().info('退出程序')
#                 rclpy.shutdown()
#             elif key == ord('r'):
#                 self.detector.current_index = 0
#                 self.get_logger().info('切换到红色检测')
#             elif key == ord('g'):
#                 self.detector.current_index = 1
#                 self.get_logger().info('切换到绿色检测')
#             elif key == ord('b'):
#                 self.detector.current_index = 2
#                 self.get_logger().info('切换到蓝色检测')
#             elif key == ord('p'):
#                 self.detector.current_index = 3
#                 self.get_logger().info('切换到紫色检测')
#             elif key == ord('y'):
#                 self.detector.current_index = 4
#                 self.get_logger().info('切换到黄色检测')
            
#         except Exception as e:
#             self.get_logger().error(f'Processing failed: {e}')
    
#     def should_update_goal(self, new_goal):
#         """判断是否需要更新目标"""
#         if self.last_goal is None:
#             return True
        
#         threshold = 0.01  # 10cm
#         dx = new_goal[0] - self.last_goal[0]
#         dy = new_goal[1] - self.last_goal[1]
#         distance = np.sqrt(dx**2 + dy**2)
        
#         return distance > threshold

# def main(args=None):
#     rclpy.init(args=args)
#     node = DetectionSimWrapper()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()#

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import cv2
import numpy as np
import sys

sys.path.append('/home/student15/leo_ws/src/leo_lidar_sim/scripts')
from detectionClass import DetectionSystem

class DetectionSimWrapper(Node):
    def __init__(self):
        super().__init__('detection_sim_wrapper')
        
        self.bridge = CvBridge()
        
        self.detector = DetectionSystem('/home/student15/rspd_venv/src/colour_params.csv', sim_mode=True)
        
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
        
        self.get_logger().info('Detection Sim Wrapper started')

    
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

if __name__ == '__main__':
    main()