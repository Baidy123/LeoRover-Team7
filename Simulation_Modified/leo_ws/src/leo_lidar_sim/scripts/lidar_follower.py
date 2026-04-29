#!/usr/bin/env python3
"""
lidar_follower.py — keep the separate `lidar_sensor` Gazebo model glued to
the rover so its scans reflect the rover's current world pose.

Why: the lidar is spawned as an independent static model (so it doesn't
fall onto the rover or block the arm at startup). Without this node it
sits at its spawn pose forever and `/scan` is taken from the room corner.
This node listens to /odom (in odom frame) and writes the lidar's WORLD
pose every 50 ms via the world's `set_pose` service.
"""

import math
import subprocess

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

WORLD_NAME = 'sorting_room'
LIDAR_MODEL = 'lidar_sensor'
SPAWN_X = 1.8     # must match rover spawn in launch
SPAWN_Y = -1.8
LIDAR_DZ = 0.5    # height above world ground (matches static TF)


class LidarFollower(Node):
    def __init__(self):
        super().__init__('lidar_follower')
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.last_x = None
        self.last_y = None
        self.last_yaw = None
        self.create_timer(0.05, self._tick)
        self.get_logger().info('lidar_follower running.')

    def _odom_cb(self, msg: Odometry):
        self.last_x = msg.pose.pose.position.x + SPAWN_X
        self.last_y = msg.pose.pose.position.y + SPAWN_Y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.last_yaw = math.atan2(siny, cosy)

    def _tick(self):
        if self.last_x is None:
            return
        # Build the gz request. Quaternion = (0,0,sin(y/2),cos(y/2)).
        sz = math.sin(self.last_yaw / 2.0)
        cz = math.cos(self.last_yaw / 2.0)
        req = (f'name: "{LIDAR_MODEL}", '
               f'position: {{x: {self.last_x}, y: {self.last_y}, z: {LIDAR_DZ}}}, '
               f'orientation: {{x: 0, y: 0, z: {sz}, w: {cz}}}')
        try:
            subprocess.run(
                ['gz', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '300', '--req', req],
                capture_output=True, text=True, timeout=2.0)
        except Exception as e:
            self.get_logger().error(f'set_pose error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
