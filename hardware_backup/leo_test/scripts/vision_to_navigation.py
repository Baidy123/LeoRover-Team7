#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs


class VisionToNavigation(Node):
    """
    Bridges vision detection to Nav2 autonomous navigation
    Subscribes to /goal_pose from vision, sends to Nav2
    """
    
    def __init__(self):
        super().__init__('vision_to_navigation')
        
        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to vision detections
        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Track current goal to avoid spam
        self.current_goal = None
        self.goal_active = False
        
        # Minimum distance to send new goal (meters)
        self.goal_threshold = 0.2
        
        self.get_logger().info('Vision-to-Navigation bridge started!')
        self.get_logger().info('Listening for detections on /goal_pose...')
    
    def goal_callback(self, detected_pose: PoseStamped):
        """
        Receives detected object position from vision node
        Transforms to map frame and sends to Nav2
        """
        
        # Don't send new goal if one is already executing
        if self.goal_active:
            # self.get_logger().debug('Goal already active, ignoring new detection')
            return
        
        try:
            # Transform from camera frame to map frame
            # Vision publishes in 'depth_camera_link' frame
            # Nav2 needs 'map' frame
            
            # Wait for transform to be available
            if not self.tf_buffer.can_transform(
                'map', 
                detected_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            ):
                self.get_logger().warn(
                    f'Cannot transform from {detected_pose.header.frame_id} to map'
                )
                return
            
            # Transform to map frame
            map_pose = self.tf_buffer.transform(
                detected_pose,
                'map',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Check if goal changed significantly
            if not self.should_send_goal(map_pose):
                return
            
            # Send to Nav2
            self.send_nav_goal(map_pose)
            
        except Exception as e:
            self.get_logger().error(f'Failed to process goal: {e}')
    
    def should_send_goal(self, new_pose: PoseStamped) -> bool:
        """Check if new goal is different enough from current goal"""
        
        if self.current_goal is None:
            return True
        
        # Calculate distance between goals
        dx = new_pose.pose.position.x - self.current_goal.pose.position.x
        dy = new_pose.pose.position.y - self.current_goal.pose.position.y
        distance = (dx**2 + dy**2)**0.5
        
        if distance < self.goal_threshold:
            # self.get_logger().debug(f'Goal too close to current ({distance:.2f}m)')
            return False
        
        return True
    
    def send_nav_goal(self, map_pose: PoseStamped):
        """Send navigation goal to Nav2"""
        
        # Wait for Nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('Nav2 action server not available!')
            return
        
        # Create Nav2 goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = map_pose
        
        self.get_logger().info(
            f'🎯 Sending navigation goal: '
            f'x={map_pose.pose.position.x:.2f}, '
            f'y={map_pose.pose.position.y:.2f}'
        )
        
        # Send goal
        self.goal_active = True
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        # Store current goal
        self.current_goal = map_pose
    
    def goal_response_callback(self, future):
        """Called when Nav2 accepts/rejects goal"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2!')
            self.goal_active = False
            return
        
        self.get_logger().info('✓ Goal accepted by Nav2')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Called periodically while robot is navigating"""
        # Can log progress here if needed
        pass
    
    def result_callback(self, future):
        """Called when navigation completes or fails"""
        result = future.result().result
        status = future.result().status
        
        self.goal_active = False
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('✓ Navigation succeeded!')
        elif status == 5:  # CANCELED
            self.get_logger().warn('Navigation was canceled')
        elif status == 6:  # ABORTED
            self.get_logger().error('Navigation aborted!')
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    
    node = VisionToNavigation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
