#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math


class SimpleObjectDetector(Node):
    def __init__(self):
        super().__init__('simple_object_detector')
        
        # Parameters
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('max_cluster_size', 100)
        self.declare_parameter('cluster_tolerance', 0.3)
        self.declare_parameter('min_object_distance', 0.2)
        self.declare_parameter('max_object_distance', 5.0)
        
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        self.cluster_tolerance = self.get_parameter('cluster_tolerance').value
        self.min_distance = self.get_parameter('min_object_distance').value
        self.max_distance = self.get_parameter('max_object_distance').value
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publishers
        self.objects_pub = self.create_publisher(PoseArray, '/detected_objects', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/object_markers', 10)
        
        self.get_logger().info('Simple Object Detector started')
    
    def scan_callback(self, msg):
        # Convert LaserScan to 2D points
        points = []
        for i, range_val in enumerate(msg.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            if range_val < self.min_distance or range_val > self.max_distance:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            points.append([x, y])
        
        if not points:
            return
        
        points = np.array(points)
        
        # Simple clustering using distance threshold
        clusters = self.cluster_points(points)
        
        # Publish detected objects
        self.publish_objects(clusters, msg.header)
        self.publish_markers(clusters, msg.header)
        
        self.get_logger().info(f'Detected {len(clusters)} objects', throttle_duration_sec=1.0)
    
    def cluster_points(self, points):
        if len(points) == 0:
            return []
        
        clusters = []
        processed = np.zeros(len(points), dtype=bool)
        
        for i in range(len(points)):
            if processed[i]:
                continue
            
            cluster = [points[i]]
            processed[i] = True
            seed_queue = [i]
            
            while seed_queue:
                current_idx = seed_queue.pop()
                current_point = points[current_idx]
                
                # Find neighbors
                distances = np.linalg.norm(points - current_point, axis=1)
                neighbors = np.where((distances < self.cluster_tolerance) & (~processed))[0]
                
                for neighbor_idx in neighbors:
                    cluster.append(points[neighbor_idx])
                    seed_queue.append(neighbor_idx)
                    processed[neighbor_idx] = True
            
            # Filter by cluster size
            if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                cluster_array = np.array(cluster)
                centroid = np.mean(cluster_array, axis=0)
                clusters.append(centroid)
        
        return clusters
    
    def publish_objects(self, clusters, header):
        pose_array = PoseArray()
        pose_array.header = header
        pose_array.header.frame_id = 'lidar_link'
        
        for centroid in clusters:
            pose = Pose()
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        
        self.objects_pub.publish(pose_array)
    
    def publish_markers(self, clusters, header):
        marker_array = MarkerArray()
        
        # Delete old markers
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Create new markers
        for i, centroid in enumerate(clusters):
            marker = Marker()
            marker.header = header
            marker.header.frame_id = 'lidar_link'
            marker.ns = 'objects'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(centroid[0])
            marker.pose.position.y = float(centroid[1])
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            
            marker_array.markers.append(marker)
        
        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
