#!/usr/bin/env python3
"""
autonomous_mission.py — Fixed version
Fixes:
  1. Auto-rotate during SEARCHING (no teleop needed)
  2. Targets red_box_2 at (1.5, 1.2) which is lower and reachable
  3. Rover stops much closer to box for arm reach
  4. Proper approach offset and facing angle
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist

from nav2_msgs.action import NavigateToPose

import math


class State:
    IDLE               = 'IDLE'
    SEARCHING          = 'SEARCHING'
    NAVIGATING_TO_BOX  = 'NAVIGATING_TO_BOX'
    ARM_PICKUP         = 'ARM_PICKUP'
    NAVIGATING_TO_BIN  = 'NAVIGATING_TO_BIN'
    ARM_PLACE          = 'ARM_PLACE'
    RETURNING_HOME     = 'RETURNING_HOME'
    DONE               = 'DONE'
    FAILED             = 'FAILED'


class AutonomousMission(Node):

    def __init__(self):
        super().__init__('autonomous_mission')

        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('bin_x',           2.0)
        self.declare_parameter('bin_y',           2.0)
        self.declare_parameter('bin_yaw',         0.0)
        self.declare_parameter('home_x',          1.8)
        self.declare_parameter('home_y',         -1.8)
        self.declare_parameter('home_yaw',        0.0)
        # Stop 0.25m in front of box — close enough for arm to reach
        self.declare_parameter('approach_offset', 0.25)
        self.declare_parameter('nav_timeout',    90.0)
        self.declare_parameter('search_timeout', 25.0)  # seconds to rotate before giving up

        self.bin_x           = self.get_parameter('bin_x').value
        self.bin_y           = self.get_parameter('bin_y').value
        self.bin_yaw         = self.get_parameter('bin_yaw').value
        self.home_x          = self.get_parameter('home_x').value
        self.home_y          = self.get_parameter('home_y').value
        self.home_yaw        = self.get_parameter('home_yaw').value
        self.approach_offset = self.get_parameter('approach_offset').value
        self.search_timeout  = self.get_parameter('search_timeout').value

        # ── State ────────────────────────────────────────────────────────────
        self.state            = State.IDLE
        self.box_pose         = None
        self.arm_done         = False
        self.nav_done         = False
        self.nav_succeeded    = False
        self.search_elapsed   = 0.0
        self.search_direction = 1.0   # 1=left, -1=right

        # ── Publishers ───────────────────────────────────────────────────────
        self.arm_cmd_pub  = self.create_publisher(String, '/arm_command',    10)
        self.status_pub   = self.create_publisher(String, '/mission_status', 10)
        self.cmd_vel_pub  = self.create_publisher(Twist,  '/cmd_vel',        10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            PoseStamped, '/goal_pose',
            self._box_detection_cb, 10,
            callback_group=self.cb_group)

        self.create_subscription(
            Bool, '/arm_action_done',
            self._arm_done_cb, 10,
            callback_group=self.cb_group)

        # ── Nav2 action client ───────────────────────────────────────────────
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.cb_group)

        # ── Tick timer at 10Hz ───────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self._tick, callback_group=self.cb_group)

        self.get_logger().info('AutonomousMission node started.')
        # Skip search — go directly to known red_box_2 position
        self.box_pose = self._make_pose(1.5, 1.2, 0.0)
        self._set_state(State.NAVIGATING_TO_BOX)

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _box_detection_cb(self, msg: PoseStamped):
        if self.state == State.SEARCHING:
            self.box_pose = msg
            self.get_logger().info(
                f'Box detected at map ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f}). Stopping rotation.')
            # Stop rotation
            self._stop_rotation()
            self._set_state(State.NAVIGATING_TO_BOX)

    def _arm_done_cb(self, msg: Bool):
        self.arm_done = True
        if not msg.data:
            self.get_logger().error('Arm reported FAILURE.')
            self._set_state(State.FAILED)

    # ─────────────────────────────────────────────────────────────────────────
    # Main tick
    # ─────────────────────────────────────────────────────────────────────────

    def _tick(self):
        self.status_pub.publish(String(data=self.state))

        if self.state == State.SEARCHING:
            self._do_search_rotation()

        elif self.state == State.NAVIGATING_TO_BOX:
            if self.nav_done:
                self.nav_done = False
                if self.nav_succeeded:
                    self._set_state(State.ARM_PICKUP)
                else:
                    self.get_logger().warn('Navigation to box failed. Retrying search.')
                    self.box_pose = None
                    # Skip search — go directly to known red_box_2 position
        self.box_pose = self._make_pose(1.5, 1.2, 0.0)
        self._set_state(State.NAVIGATING_TO_BOX)

        elif self.state == State.ARM_PICKUP:
            if self.arm_done:
                self.arm_done = False
                self._set_state(State.NAVIGATING_TO_BIN)

        elif self.state == State.NAVIGATING_TO_BIN:
            if self.nav_done:
                self.nav_done = False
                if self.nav_succeeded:
                    self._set_state(State.ARM_PLACE)
                else:
                    self.get_logger().error('Navigation to bin failed.')
                    self._set_state(State.FAILED)

        elif self.state == State.ARM_PLACE:
            if self.arm_done:
                self.arm_done = False
                self._set_state(State.RETURNING_HOME)

        elif self.state == State.RETURNING_HOME:
            if self.nav_done:
                self.nav_done = False
                self._set_state(State.DONE)

        elif self.state == State.DONE:
            self.get_logger().info('✅ Mission complete!')
            self.timer.cancel()

        elif self.state == State.FAILED:
            self.get_logger().error('❌ Mission failed.')
            self._stop_rotation()
            self.timer.cancel()

    # ─────────────────────────────────────────────────────────────────────────
    # Search rotation
    # ─────────────────────────────────────────────────────────────────────────

    def _do_search_rotation(self):
        """
        Rotate continuously at 0.6 rad/s for a full 360 degree scan.
        One full rotation takes ~10.5 seconds at 0.6 rad/s.
        After two full rotations (21s) fall back to known box position.
        """
        self.search_elapsed += 0.1  # 10Hz tick

        # Publish continuous rotation — no reversal, full circle
        twist = Twist()
        twist.linear.x  = 0.0
        twist.angular.z = 0.6   # rad/s — full rotation in ~10.5s
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(
            f'Searching... {self.search_elapsed:.1f}s',
            throttle_duration_sec=2.0)

        if self.search_elapsed > self.search_timeout:
            self.get_logger().warn(
                f'Search timeout after {self.search_timeout}s. '
                'Navigating directly to red_box_2 known position.')
            self._stop_rotation()
            self.box_pose = self._make_pose(1.5, 1.2, 0.0)
            self._set_state(State.NAVIGATING_TO_BOX)

    def _stop_rotation(self):
        self.cmd_vel_pub.publish(Twist())  # zero velocity

    # ─────────────────────────────────────────────────────────────────────────
    # State transitions
    # ─────────────────────────────────────────────────────────────────────────

    def _set_state(self, new_state: str):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state

        if new_state == State.SEARCHING:
            self.search_elapsed = 0.0

        elif new_state == State.NAVIGATING_TO_BOX:
            approach = self._approach_pose(self.box_pose)
            self._send_nav_goal(approach)

        elif new_state == State.ARM_PICKUP:
            self.arm_done = False
            self.get_logger().info('Sending ARM command: pickup')
            self.arm_cmd_pub.publish(String(data='pickup'))

        elif new_state == State.NAVIGATING_TO_BIN:
            # Navigate to approach position in front of basket
            bin_pose = self._make_pose(
                self.bin_x - self.approach_offset,
                self.bin_y,
                0.0)
            self._send_nav_goal(bin_pose)

        elif new_state == State.ARM_PLACE:
            self.arm_done = False
            self.get_logger().info('Sending ARM command: place')
            self.arm_cmd_pub.publish(String(data='place'))

        elif new_state == State.RETURNING_HOME:
            home = self._make_pose(self.home_x, self.home_y, self.home_yaw)
            self._send_nav_goal(home)

    # ─────────────────────────────────────────────────────────────────────────
    # Navigation
    # ─────────────────────────────────────────────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped):
        self.nav_done      = False
        self.nav_succeeded = False

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self._set_state(State.FAILED)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Nav2 goal → ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})')

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._nav_feedback_cb)
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 goal rejected!')
            self.nav_done = True
            self.nav_succeeded = False
            return
        handle.get_result_async().add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        status = future.result().status
        self.nav_succeeded = (status == 4)
        self.nav_done = True
        self.get_logger().info(
            f'Nav2 {"SUCCEEDED" if self.nav_succeeded else f"FAILED status={status}"}')

    def _nav_feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'  {dist:.2f}m remaining', throttle_duration_sec=3.0)

    # ─────────────────────────────────────────────────────────────────────────
    # Pose helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _make_pose(self, x, y, yaw) -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp    = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def _approach_pose(self, target: PoseStamped) -> PoseStamped:
        """
        Stop directly in front of the box at approach_offset distance.
        Rover faces the box (+x direction toward box).
        """
        tx = target.pose.position.x
        ty = target.pose.position.y

        # Rover spawns at (1.8, -1.8). Box is at (1.5, 1.2).
        # Approach from south — stop below the box in y
        gx = tx
        gy = ty - self.approach_offset  # stop south of box
        yaw = math.pi / 2.0            # face north (+y direction)

        return self._make_pose(gx, gy, yaw)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMission()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted.')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
