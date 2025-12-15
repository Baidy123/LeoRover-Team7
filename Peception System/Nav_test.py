from detectionClass import DetectionSystem
import cv2
import numpy as np
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import rclpy
import tf2_ros
import tf2_geometry_msgs



rclpy.init()
detector = DetectionSystem('/home/student15/rspd_venv/src/colour_params.csv')
# detector.run()
last_goal = None
node = rclpy.create_node('node1')
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer, node)
# node.current_index = 2
pub = node.create_publisher(
    msg_type=PoseStamped,
    topic='/goal_pose',
    qos_profile=1
    )


def goal_update_condition(new_goal):
    threshold = 0
    if last_goal is None:
        return True
        
    dx = new_goal[0] - last_goal[0]
    dz = new_goal[2] - last_goal[2]
    distance = np.sqrt(dx**2 + dz**2)
    return distance > threshold
while True:
    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        cv2.destroyAllWindows()
        break

    blocks = detector.detect_block()
    if blocks:
        if goal_update_condition(blocks[0]['position_3d']):
            pose_to_camera = PoseStamped()
            pose_to_camera.header.frame_id = 'map'
            pose_to_camera.header.stamp = node.get_clock().now().to_msg()
            pose_to_camera.pose.position.x = float(blocks[0]['position_3d'][0])
            pose_to_camera.pose.position.y = float(blocks[0]['position_3d'][1])
            pose_to_camera.pose.position.z = float(blocks[0]['position_3d'][2])
            pose_to_camera.pose.orientation.w = 1.0
            pub.publish(pose_to_camera)
            # pose_map = tf_buffer.transform(pose_to_camera, 'map')
            # pub.publish(pose_map)



            # pose_map = 

            
            # print(type(blocks[0]['position_3d']))
        last_goal =  blocks[0]['position_3d']

# print(detector.goal_point_3d)
