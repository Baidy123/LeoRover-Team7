#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from collections import deque
import math


class ScanHealthChecker(Node):
    """
    Checks LiDAR scan data quality and filters out unhealthy scans.
    
    Criteria for healthy scan:
    1. Valid range percentage > threshold (default 50%)
    2. No excessive noise (std dev check)
    3. Minimum number of valid points
    4. No sudden jumps between consecutive scans
    """
    
    def __init__(self):
        super().__init__('scan_health_checker')
        
        # Parameters
        self.declare_parameter('min_valid_percentage', 50.0)  # %
        self.declare_parameter('min_valid_points', 50)
        self.declare_parameter('max_noise_stddev', 2.0)  # meters
        self.declare_parameter('max_range_jump', 3.0)  # meters
        
        self.min_valid_pct = self.get_parameter('min_valid_percentage').value
        self.min_valid_points = self.get_parameter('min_valid_points').value
        self.max_noise = self.get_parameter('max_noise_stddev').value
        self.max_jump = self.get_parameter('max_range_jump').value
        
        # Subscribers and publishers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.healthy_scan_pub = self.create_publisher(
            LaserScan,
            '/scan_healthy',
            10
        )
        
        # Statistics
        self.total_scans = 0
        self.healthy_scans = 0
        self.last_scan_ranges = None
        self.recent_health = deque(maxlen=100)
        
        # Timer for statistics
        self.timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('Scan Health Checker started')
        self.get_logger().info(f'Min valid %: {self.min_valid_pct}')
        self.get_logger().info(f'Min valid points: {self.min_valid_points}')
    
    def scan_callback(self, msg):
        self.total_scans += 1
        
        # Extract valid ranges
        ranges = np.array(msg.ranges)
        valid_mask = ~(np.isnan(ranges) | np.isinf(ranges))
        valid_ranges = ranges[valid_mask]
        
        # Check 1: Valid percentage
        valid_pct = (len(valid_ranges) / len(ranges)) * 100
        if valid_pct < self.min_valid_pct:
            self.recent_health.append(False)
            self.get_logger().debug(
                f'Rejected: Low valid % ({valid_pct:.1f}%)',
                throttle_duration_sec=1.0
            )
            return
        
        # Check 2: Minimum valid points
        if len(valid_ranges) < self.min_valid_points:
            self.recent_health.append(False)
            self.get_logger().debug(
                f'Rejected: Too few points ({len(valid_ranges)})',
                throttle_duration_sec=1.0
            )
            return
        
        # Check 3: Noise level
        if len(valid_ranges) > 10:
            std_dev = np.std(valid_ranges)
            if std_dev > self.max_noise:
                self.recent_health.append(False)
                self.get_logger().debug(
                    f'Rejected: High noise (σ={std_dev:.2f})',
                    throttle_duration_sec=1.0
                )
                return
        
        # Check 4: Sudden jumps from previous scan
        if self.last_scan_ranges is not None:
            if len(self.last_scan_ranges) == len(valid_ranges):
                diffs = np.abs(valid_ranges - self.last_scan_ranges)
                max_diff = np.max(diffs)
                if max_diff > self.max_jump:
                    self.recent_health.append(False)
                    self.get_logger().debug(
                        f'Rejected: Large jump ({max_diff:.2f}m)',
                        throttle_duration_sec=1.0
                    )
                    return
        
        # Scan is healthy!
        self.healthy_scans += 1
        self.recent_health.append(True)
        self.last_scan_ranges = valid_ranges
        
        # Publish healthy scan
        self.healthy_scan_pub.publish(msg)
        
        self.get_logger().debug(
            f'Healthy scan: {valid_pct:.1f}% valid, {len(valid_ranges)} points',
            throttle_duration_sec=2.0
        )
    
    def print_stats(self):
        if self.total_scans == 0:
            return
        
        overall_health = (self.healthy_scans / self.total_scans) * 100
        recent_health = (sum(self.recent_health) / len(self.recent_health) * 100) if self.recent_health else 0
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Total scans:   {self.total_scans}')
        self.get_logger().info(f'Healthy scans: {self.healthy_scans} ({overall_health:.1f}%)')
        self.get_logger().info(f'Recent health: {recent_health:.1f}% (last 100 scans)')
        self.get_logger().info('=' * 50)


def main(args=None):
    rclpy.init(args=args)
    node = ScanHealthChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
