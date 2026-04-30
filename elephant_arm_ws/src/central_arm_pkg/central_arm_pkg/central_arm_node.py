# Main node for controlling operations on the arm
# Receives data from the depth camera
# Manages poses that the arm should go into based on received info
# Kind of want to have just one subscriber for poses/coorindates


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class CentralArmNode(Node):
    def __init__(self):
        super().__init__('central_arm_node')

        self.pose_sent = False

        self.depth_camera_coordinate_subscriber = self.create_subscription(
            msg_type = PoseStamped,
            topic = '/goal_pose',
            callback = self.depth_camera_coordinate_subscriber_callback,
            qos_profile=1
        )

        self.release_block_signal_subscriber  = self.create_subscription(
            msg_type=String,
            topic='/release_signal',
            callback = self.release_block_subscriber_callback,
            qos_profile=1
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/transformed_goal_pose',
            10
        )

        self.status_pub = self.create_subscription(
            String,
            '/manipulator_task_status',
            self.status_callback,
            10
        )
        
        self.release_pub = self.create_publisher(
            String,
            '/manipulator_release_block',
            10
        )

    # Subscriber to coordinates of the arm
    # Might want to do checks on reachability here but can be done in moveit node
    # Need to get rosbagged data and find out msg type
    # Need to publish to the moveit node based on data received here
    def depth_camera_coordinate_subscriber_callback(self, msg: PoseStamped):
        if (not self.pose_sent):
            self.pose_sent = True
            x = msg.pose.position.x
            y = msg.pose.position.y
            w = msg.pose.orientation.w

            self.get_logger().info(f"Coordinates of block are {x}, {y}, {w}")
            
            # Need to transform the camera
            # Hard code a z value based on the distance between the manipulator actuator and the block heights
            processed_pose = PoseStamped()
            processed_pose.header.frame_id = "world"
            processed_pose.header.stamp = self.get_clock().now().to_msg()

            # CHANGE THIS TO TRANSFORM TO MANIPULATOR
            processed_pose.pose.position.x = x
            processed_pose.pose.position.y = y
            processed_pose.pose.position.z = 0.025
            processed_pose.pose.orientation.w = 1.0

            self.goal_pub.publish(processed_pose)
        

    def status_callback(self,msg):
        # if msg.data == "OUT_OF_REACH":
        self.get_logger().error(msg.data)
            # Need to somehow get message that the arm needs to move closer

    def release_block_subscriber_callback(self, msg:String):
        msg = String()
        msg.data = "RELEASE"
        self.release_pub.publish(msg)
        self.pose_sent = False


def main(args=None):
    try:
        rclpy.init(args=args)
        node = CentralArmNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()