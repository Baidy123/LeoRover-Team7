#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import random


class ScanCorruptor(Node):
    """
    Adds realistic corruption to LiDAR scan data for testing filtering algorithms.
    
    Corruption types:
    1. Gaussian noise
    2. Random dropouts (inf values)
    3. Outliers (random spikes)
    4. Missing segments
    """
    
    def __init__(self):
        super().__init__('scan_corruptor')
        
        # Parameters
        self.declare_parameter('noise_stddev', 0.05)  # Gaussian noise std dev
        self.declare_parameter('dropout_rate', 0.1)  # 10% dropout
        self.declare_parameter('outlier_rate', 0.05)  # 5% outliers
        self.declare_parameter('segment_dropout_prob', 0.1)  # 10% chance of missing segment
        
        self.noise_std = self.get_parameter('noise_stddev').value
        self.dropout_rate = self.get_parameter('dropout_rate').value
        self.outlier_rate = self.get_parameter('outlier_rate').value
        self.segment_prob = self.get_parameter('segment_dropout_prob').value
        
        # Subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.corrupted_scan_pub = self.create_publisher(
            LaserScan,
            '/scan_corrupted',
            10
        )
        
        self.get_logger().info('Scan Corruptor started')
        self.get_logger().info(f'Noise std dev: {self.noise_std}')
        self.get_logger().info(f'Dropout rate: {self.dropout_rate * 100}%')
        self.get_logger().info(f'Outlier rate: {self.outlier_rate * 100}%')
    
    def scan_callback(self, msg):
        corrupted_msg = LaserScan()
        corrupted_msg.header = msg.header
        corrupted_msg.angle_min = msg.angle_min
        corrupted_msg.angle_max = msg.angle_max
        corrupted_msg.angle_increment = msg.angle_increment
        corrupted_msg.time_increment = msg.time_increment
        corrupted_msg.scan_time = msg.scan_time
        corrupted_msg.range_min = msg.range_min
        corrupted_msg.range_max = msg.range_max
        
        ranges = np.array(msg.ranges)
        num_points = len(ranges)
        
        # 1. Add Gaussian noise
        noise = np.random.normal(0, self.noise_std, num_points)
        ranges = ranges + noise
        
        # 2. Add random dropouts
        dropout_mask = np.random.random(num_points) < self.dropout_rate
        ranges[dropout_mask] = float('inf')
        
        # 3. Add outliers
        outlier_mask = np.random.random(num_points) < self.outlier_rate
        ranges[outlier_mask] = np.random.uniform(msg.range_min, msg.range_max, np.sum(outlier_mask))
        
        # 4. Random segment dropout
        if random.random() < self.segment_prob:
            segment_start = random.randint(0, num_points - 50)
            segment_length = random.randint(20, 50)
            segment_end = min(segment_start + segment_length, num_points)
            ranges[segment_start:segment_end] = float('inf')
            self.get_logger().debug(
                f'Dropped segment: {segment_start} to {segment_end}',
                throttle_duration_sec=2.0
            )
        
        # Ensure ranges stay within valid bounds
        ranges = np.clip(ranges, msg.range_min, msg.range_max)
        
        corrupted_msg.ranges = ranges.tolist()
        corrupted_msg.intensities = msg.intensities
        
        self.corrupted_scan_pub.publish(corrupted_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanCorruptor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
