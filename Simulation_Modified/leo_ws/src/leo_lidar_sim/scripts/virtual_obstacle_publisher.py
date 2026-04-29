#!/usr/bin/env python3
"""
virtual_obstacle_publisher.py

Publishes a synthetic LaserScan on /virtual_scan that marks the known
pedestal positions as obstacles. The lidar mounted at world z=0.26 sits
above the 0.20 m pedestals, so empty pedestals (after pickup) are
invisible to the real scan and the planner drives through them. This
node injects each pedestal as a hit at the right angle/range relative
to the rover's current pose, so Nav2's obstacle_layer sees them
regardless of lidar geometry.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
import tf2_ros


# World coordinates of every pedestal (matches arm_controller / mission).
PEDESTALS = [
    (0.0,  0.0),    # red_box_1
    (1.5,  1.2),    # red_box_2
    (-0.8, -0.8),   # blue_box_1
    (-1.2, -1.3),   # blue_box_2
    (-1.3, 1.0),    # green_box_1
    (0.7,  1.5),    # green_box_2
    (0.6, -0.4),    # extra_obstacle_1
    (-0.4, 0.5),    # extra_obstacle_2
]
PEDESTAL_HALF = 0.15  # half-width — used to mark several rays per obstacle

# Rover spawn world offset (matches launch).
SPAWN_X = 1.8
SPAWN_Y = -1.8

# Scan settings.
N_SAMPLES = 360
ANGLE_MIN = -math.pi
ANGLE_MAX = math.pi
ANGLE_INC = (ANGLE_MAX - ANGLE_MIN) / N_SAMPLES
RANGE_MIN = 0.0
RANGE_MAX = 8.0


class VirtualObstaclePublisher(Node):
    def __init__(self):
        super().__init__('virtual_obstacle_publisher')
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._pub = self.create_publisher(LaserScan, '/virtual_scan', 10)
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'Virtual obstacle publisher started '
            f'({len(PEDESTALS)} pedestals).')

    def _rover_world(self):
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return None
        mx = t.transform.translation.x
        my = t.transform.translation.y
        q = t.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        return (mx + SPAWN_X, my + SPAWN_Y, yaw)

    def _tick(self):
        pose = self._rover_world()
        if pose is None:
            return
        rx, ry, ryaw = pose

        ranges = [float('inf')] * N_SAMPLES
        for px, py in PEDESTALS:
            dx = px - rx
            dy = py - ry
            # Express the pedestal in rover-body frame.
            bx = dx * math.cos(ryaw) + dy * math.sin(ryaw)
            by = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
            angle = math.atan2(by, bx)
            dist = math.hypot(bx, by) - PEDESTAL_HALF  # mark at the near edge
            if dist <= RANGE_MIN or dist >= RANGE_MAX:
                continue

            # Compute angular half-width subtended by the pedestal so we
            # mark a small wedge of rays, not just one — gives the
            # costmap a proper-sized obstacle.
            if dist > 0.05:
                half_angle = math.atan2(PEDESTAL_HALF, dist)
            else:
                half_angle = 0.05
            a_lo = angle - half_angle
            a_hi = angle + half_angle
            i_lo = int(round((a_lo - ANGLE_MIN) / ANGLE_INC))
            i_hi = int(round((a_hi - ANGLE_MIN) / ANGLE_INC))
            for i in range(i_lo, i_hi + 1):
                idx = i % N_SAMPLES
                if dist < ranges[idx]:
                    ranges[idx] = dist

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.angle_min = ANGLE_MIN
        msg.angle_max = ANGLE_MAX
        msg.angle_increment = ANGLE_INC
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualObstaclePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
