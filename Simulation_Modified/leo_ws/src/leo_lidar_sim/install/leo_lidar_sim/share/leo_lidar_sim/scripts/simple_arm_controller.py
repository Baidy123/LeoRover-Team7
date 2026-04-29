#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import termios
import tty

class SimpleArmController(Node):
    def __init__(self):
        super().__init__('simple_arm_controller')
        
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Current joint positions
        self.joint1_pos = 0.0  # Base rotation
        self.joint2_pos = 0.0  # Shoulder
        
        # Step size
        self.step = 0.1  # radians (~6 degrees)
        
        self.get_logger().info('Simple Arm Controller Started!')
        self.get_logger().info('Controls:')
        self.get_logger().info('  a/d - Rotate base left/right')
        self.get_logger().info('  w/s - Move shoulder up/down')
        self.get_logger().info('  r   - Reset to home position')
        self.get_logger().info('  q   - Quit')
        
        self.print_position()
    
    def get_key(self):
        """Get keyboard input"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    
    def publish_joint_state(self):
        """Publish current joint positions"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['arm_segment1_joint', 'arm_segment2_joint']
        msg.position = [self.joint1_pos, self.joint2_pos]
        self.publisher.publish(msg)
    
    def print_position(self):
        """Print current joint positions"""
        j1_deg = self.joint1_pos * 180 / 3.14159
        j2_deg = self.joint2_pos * 180 / 3.14159
        self.get_logger().info(f'Base: {j1_deg:.1f}°, Shoulder: {j2_deg:.1f}°')
    
    def run(self):
        """Main control loop"""
        while rclpy.ok():
            key = self.get_key()
            
            if key == 'a':  # Rotate left
                self.joint1_pos += self.step
                if self.joint1_pos > 3.14:
                    self.joint1_pos = 3.14
                self.publish_joint_state()
                self.print_position()
                
            elif key == 'd':  # Rotate right
                self.joint1_pos -= self.step
                if self.joint1_pos < -3.14:
                    self.joint1_pos = -3.14
                self.publish_joint_state()
                self.print_position()
                
            elif key == 'w':  # Shoulder up
                self.joint2_pos += self.step
                if self.joint2_pos > 1.5:
                    self.joint2_pos = 1.5
                self.publish_joint_state()
                self.print_position()
                
            elif key == 's':  # Shoulder down
                self.joint2_pos -= self.step
                if self.joint2_pos < -1.5:
                    self.joint2_pos = -1.5
                self.publish_joint_state()
                self.print_position()
                
            elif key == 'r':  # Reset
                self.joint1_pos = 0.0
                self.joint2_pos = 0.0
                self.publish_joint_state()
                self.get_logger().info('Reset to home position')
                self.print_position()
                
            elif key == 'q':  # Quit
                self.get_logger().info('Exiting...')
                break


def main(args=None):
    rclpy.init(args=args)
    controller = SimpleArmController()
    
    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
