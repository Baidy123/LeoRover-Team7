# Main node for controlling operations on the arm
# Receives data from the depth camera
# Manages poses that the arm should go into based on received info
# Kind of want to have just one subscriber for poses/coorindates



import rclpy
from rclpy.node import Node

class CentralArmNode(Node):
    def __init__(self):
        super().__init__('central_arm_node')
        self.depth_camera_coordinate_subscriber = self.create_subscription(
            
        )

    # Subscriber to coordinates of the arm
    # Might want to do checks on reachabiliy here but can be done in moveit node
    # Need to get rosbagged data and find out msg type
    # Need to publish to the moveit node based on data received here
    def depth_camera_coordinate_subscriber_callback(self, msg):
        self.get_logger().info(f"Coordinates are")

def main(args=None):
    try:
        rclpy.init(args=args)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()