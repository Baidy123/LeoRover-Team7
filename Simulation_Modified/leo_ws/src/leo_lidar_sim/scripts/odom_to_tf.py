#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    """
    Publishes TF transform from odometry data.
    Converts /odom topic to odom->base_footprint TF transform.
    """
    
    def __init__(self):
        super().__init__('odom_to_tf')

        # use_sim_time is provided via the launch CLI (--ros-args -p
        # use_sim_time:=true). rclpy auto-declares it when passed that
        # way, so re-declaring here raises ParameterAlreadyDeclaredException
        # and kills the node before it broadcasts any TF.
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10  # QoS depth
        )
        
        self.last_stamp = None
        
        self.get_logger().info('Odom to TF broadcaster started (use_sim_time=True)')
    
    def odom_callback(self, msg):
        # Create transform message
        t = TransformStamped()
        
        # Header - use the EXACT timestamp from odometry
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Rotation
        t.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)
        
        # Log at startup only
        if self.last_stamp is None:
            self.get_logger().info(f'Publishing odom->base_footprint transform')
        
        self.last_stamp = msg.header.stamp


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
