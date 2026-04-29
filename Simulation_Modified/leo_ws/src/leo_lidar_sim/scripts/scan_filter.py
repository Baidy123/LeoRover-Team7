#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class ScanNoiseFilter(Node):
    """
    Filters noisy LiDAR scans to clean up red/blue lines in RViz
    
    Techniques:
    1. Range filter - Remove readings outside valid range
    2. Intensity filter - Remove low-intensity points
    3. Jump filter - Remove sudden distance jumps
    4. Median filter - Smooth neighboring points
    """
    
    def __init__(self):
        super().__init__('scan_noise_filter')
        
        # Parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('min_range', 0.2)  # Minimum valid range (meters)
        self.declare_parameter('max_range', 9.5)  # Maximum valid range (meters)
        self.declare_parameter('jump_threshold', 0.5)  # Max allowed jump between points
        self.declare_parameter('median_window', 3)  # Window size for median filter
        
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.jump_threshold = self.get_parameter('jump_threshold').value
        self.median_window = self.get_parameter('median_window').value
        
        # Subscribe to raw scan
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publish filtered scan
        self.filtered_pub = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            10
        )
        
        self.get_logger().info('LiDAR noise filter started')
        self.get_logger().info(f'Range filter: {self.min_range}m - {self.max_range}m')
    
    def scan_callback(self, scan: LaserScan):
        """Filter noisy scan data"""
        
        filtered_scan = LaserScan()
        filtered_scan.header = scan.header
        filtered_scan.angle_min = scan.angle_min
        filtered_scan.angle_max = scan.angle_max
        filtered_scan.angle_increment = scan.angle_increment
        filtered_scan.time_increment = scan.time_increment
        filtered_scan.scan_time = scan.scan_time
        filtered_scan.range_min = self.min_range
        filtered_scan.range_max = self.max_range
        
        # Copy and filter ranges
        ranges = list(scan.ranges)
        filtered_ranges = self.filter_ranges(ranges)
        
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = scan.intensities
        
        self.filtered_pub.publish(filtered_scan)
    
    def filter_ranges(self, ranges):
        """Apply multiple filtering techniques"""
        
        filtered = []
        
        for i, r in enumerate(ranges):
            # 1. Range filter - Remove out-of-bounds readings
            if r < self.min_range or r > self.max_range or math.isinf(r) or math.isnan(r):
                filtered.append(float('inf'))  # Invalid reading
                continue
            
            # 2. Jump filter - Detect sudden distance changes (likely noise)
            if i > 0 and i < len(ranges) - 1:
                prev_r = ranges[i-1] if ranges[i-1] > 0 else r
                next_r = ranges[i+1] if ranges[i+1] > 0 else r
                
                # If current point is much closer/farther than neighbors, it's likely noise
                jump_prev = abs(r - prev_r)
                jump_next = abs(r - next_r)
                
                if jump_prev > self.jump_threshold and jump_next > self.jump_threshold:
                    # Isolated spike - likely noise
                    filtered.append(float('inf'))
                    continue
            
            # 3. Valid point
            filtered.append(r)
        
        # 4. Optional: Median filter for smoothing
        # filtered = self.median_filter(filtered)
        
        return filtered
    
    def median_filter(self, ranges):
        """Apply median filter to smooth readings"""
        
        smoothed = []
        half_window = self.median_window // 2
        
        for i in range(len(ranges)):
            # Get window of neighbors
            start = max(0, i - half_window)
            end = min(len(ranges), i + half_window + 1)
            window = [r for r in ranges[start:end] if not math.isinf(r) and not math.isnan(r)]
            
            if window:
                # Use median value
                window.sort()
                median = window[len(window) // 2]
                smoothed.append(median)
            else:
                smoothed.append(ranges[i])
        
        return smoothed


def main(args=None):
    rclpy.init(args=args)
    node = ScanNoiseFilter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
