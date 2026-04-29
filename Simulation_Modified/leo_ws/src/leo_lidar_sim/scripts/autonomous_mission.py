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
from nav_msgs.msg import Odometry

from nav2_msgs.action import NavigateToPose

import tf2_ros
from rclpy.duration import Duration

import math


# ── Box → world position (must match arm_controller.BOX_POSITIONS) ───────────
BOX_POSITIONS = {
    'red_box_1':   (0.0,  0.0),
    'red_box_2':   (1.5,  1.2),
    'blue_box_1':  (-0.8, -0.8),
    'blue_box_2':  (-1.2, -1.3),
    'green_box_1': (-1.3, 1.0),
    'green_box_2': (0.7,  1.5),
}

# ── Basket world positions (corners of the room) ─────────────────────────────
# Approach offsets so the rover stops in front of the basket facing it,
# rather than driving into the corner.
BASKET_POSITIONS = {
    'red':   (2.0,  2.0),
    'green': (-2.0, 2.0),
    'blue':  (-2.0, -2.0),
}


def color_of(box_name: str) -> str:
    """Extract 'red'/'green'/'blue' from a box name like 'red_box_1'."""
    return box_name.split('_', 1)[0]


class State:
    IDLE               = 'IDLE'
    SEARCHING          = 'SEARCHING'
    NAVIGATING_TO_BOX  = 'NAVIGATING_TO_BOX'
    ARM_PICKUP         = 'ARM_PICKUP'
    BACKUP_FROM_BOX    = 'BACKUP_FROM_BOX'
    NAVIGATING_TO_BIN  = 'NAVIGATING_TO_BIN'
    ARM_PLACE          = 'ARM_PLACE'
    BACKUP_FROM_BIN    = 'BACKUP_FROM_BIN'
    RETURNING_HOME     = 'RETURNING_HOME'
    DONE               = 'DONE'
    FAILED             = 'FAILED'


class AutonomousMission(Node):

    def __init__(self):
        super().__init__('autonomous_mission')

        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────────────────
        # target_boxes is a comma-separated list of boxes to pick up in
        # order. The mission runs the full pick-place cycle for each,
        # then returns home once. Override at launch with e.g.
        #   -p target_boxes:='red_box_2,red_box_1,blue_box_1,...'
        self.declare_parameter(
            'target_boxes',
            'red_box_2,red_box_1')
        # Single-box compatibility: if the user passes target_box (old
        # parameter name) we treat it as a list of one.
        self.declare_parameter('target_box',      '')
        self.declare_parameter('home_x',          1.8)
        self.declare_parameter('home_y',         -1.8)
        self.declare_parameter('home_yaw',        0.0)
        # Bin (basket) approach offset — basket is ~0.5m wide, stop 0.55m
        # away so the arm can reach over the rim.
        self.declare_parameter('approach_offset', 0.55)
        # BOX approach offset — much tighter. Arm shoulder is 0.35m above
        # box top, so horizontal reach is sqrt(0.59^2 - 0.35^2) ≈ 0.48m.
        # Rover must stop within that horizontal distance for IK to solve.
        self.declare_parameter('box_approach_offset', 0.40)
        self.declare_parameter('nav_timeout',    90.0)
        # Open-loop backup duration after pickup/place (seconds at -0.15 m/s).
        # 3.5s × 0.15 m/s = 0.52m back-out, enough to fully clear the
        # inflated zone (0.85m) of the box / basket so the global planner
        # has a free start cell and the rover faces clear space.
        self.declare_parameter('backup_duration', 3.5)
        # Pre-mission spin so SLAM can map the room before we send goals
        self.declare_parameter('search_timeout', 12.0)
        # Spawn pose in WORLD frame. SLAM's map frame is initialized at the
        # rover's start, so map_xy = world_xy - spawn_xy. The constants
        # in BOX_POSITIONS / BASKET_POSITIONS are in WORLD; we translate.
        self.declare_parameter('spawn_x',  1.8)
        self.declare_parameter('spawn_y', -1.8)

        self.home_x          = self.get_parameter('home_x').value
        self.home_y          = self.get_parameter('home_y').value
        self.home_yaw        = self.get_parameter('home_yaw').value
        self.approach_offset     = self.get_parameter('approach_offset').value
        self.box_approach_offset = self.get_parameter('box_approach_offset').value
        self.search_timeout  = self.get_parameter('search_timeout').value
        self.spawn_x         = self.get_parameter('spawn_x').value
        self.spawn_y         = self.get_parameter('spawn_y').value
        self._backup_duration = self.get_parameter('backup_duration').value
        self._backup_elapsed  = 0.0

        # ── State ────────────────────────────────────────────────────────────
        # NOTE: state vars MUST be initialized before _activate_current_target()
        # is called below, because that helper writes to self.box_pose.
        self.state             = State.IDLE
        self.box_pose          = None
        self.arm_done          = False
        self.nav_done          = False
        self.nav_succeeded     = False
        self.search_elapsed    = 0.0
        self.search_direction  = 1.0   # 1=left, -1=right
        # Timestamp (ns) when we last published an arm command. Used by
        # _arm_done_cb to ignore spurious /arm_action_done messages that
        # land within 1 s of the command (DDS race / leftover message).
        self._arm_cmd_sent_ns  = None
        # Latest rover world pose from /odom (odom origin == spawn).
        self._rover_x = 0.0
        self._rover_y = 0.0
        # Re-approach attempt counter so we don't loop forever.
        self._approach_attempts = 0

        # Build the queue of boxes to pick. Single-box override wins if set.
        # Strip ALL whitespace inside each token (not just edges) so values
        # mangled by terminal soft-wraps like "r ed_box_1" still parse.
        import re
        def _clean(tok: str) -> str:
            return re.sub(r'\s+', '', tok)
        single = _clean(self.get_parameter('target_box').value)
        if single:
            box_list = [single]
        else:
            box_list = [_clean(b) for b in
                        self.get_parameter('target_boxes').value.split(',')
                        if _clean(b)]

        for b in box_list:
            if b not in BOX_POSITIONS:
                self.get_logger().error(
                    f'Unknown box "{b}". Valid: {list(BOX_POSITIONS.keys())}')
                raise SystemExit(1)

        self._box_queue = list(box_list)
        self._box_index = 0
        self._activate_current_target()
        self.get_logger().info(
            f'Mission queue: {self._box_queue}')

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

        self.create_subscription(
            Odometry, '/odom',
            self._odom_cb, 10,
            callback_group=self.cb_group)

        # ── Nav2 action client ───────────────────────────────────────────────
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.cb_group)

        # ── TF — read the rover's MAP-frame pose the same way Nav2 does. ─────
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Tick timer at 10Hz ───────────────────────────────────────────────
        self.timer = self.create_timer(0.1, self._tick, callback_group=self.cb_group)

        self.get_logger().info('AutonomousMission node started.')
        # Box queue is already prepared by _activate_current_target() in
        # __init__; drive to the first target.
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

    def _odom_cb(self, msg: Odometry):
        # /odom origin is at the rover's spawn pose, so add the spawn offset
        # to get a world-frame position.
        self._rover_x = msg.pose.pose.position.x + self.spawn_x
        self._rover_y = msg.pose.pose.position.y + self.spawn_y

    def _arm_done_cb(self, msg: Bool):
        # Ignore Bools that arrive when we're not waiting on the arm.
        if self.state not in (State.ARM_PICKUP, State.ARM_PLACE):
            return
        # Ignore stale/spurious messages that land within the first second
        # of issuing the command. The arm needs ~6 s to physically execute
        # a pickup, so anything sub-1s is a leftover/race artifact.
        if self._arm_cmd_sent_ns is not None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._arm_cmd_sent_ns < 1_000_000_000:
                self.get_logger().warn(
                    f'Ignoring spurious /arm_action_done '
                    f'(data={msg.data}) received {(now_ns - self._arm_cmd_sent_ns)/1e6:.0f}ms after command.')
                return
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
                    # Read the rover's MAP-frame pose via TF (same view as
                    # Nav2). Convert to world by adding spawn offset.
                    rover_w = self._lookup_rover_world()
                    bx_w, by_w = BOX_POSITIONS[self.target_box]
                    if rover_w is None:
                        # No TF yet — trust Nav2 and proceed.
                        self.get_logger().warn(
                            'Could not read map→base_link TF; trusting Nav2.')
                        self._approach_attempts = 0
                        self._set_state(State.ARM_PICKUP)
                    else:
                        rx, ry = rover_w
                        dist_to_box = math.hypot(bx_w - rx, by_w - ry)
                        if dist_to_box <= 0.55:
                            self._approach_attempts = 0
                            self._set_state(State.ARM_PICKUP)
                        elif self._approach_attempts < 3:
                            self._approach_attempts += 1
                            self.get_logger().warn(
                                f'Rover too far from box ({dist_to_box:.2f}m, '
                                f'world={rx:.2f},{ry:.2f}). '
                                f'Retrying approach #{self._approach_attempts}.')
                            approach = self._approach_pose(self.box_pose)
                            self._send_nav_goal(approach)
                        else:
                            self.get_logger().error(
                                f'Failed to get within arm reach after 3 tries '
                                f'(still {dist_to_box:.2f}m). Aborting.')
                            self._set_state(State.FAILED)
                else:
                    self.get_logger().warn(
                        'Navigation to box failed. Retrying once with same goal.')
                    approach = self._approach_pose(self.box_pose)
                    self._send_nav_goal(approach)

        elif self.state == State.ARM_PICKUP:
            if self.arm_done:
                self.arm_done = False
                # Back away from the box before sending the next nav goal —
                # otherwise the planner's start cell is inside the inflated
                # zone and SmacPlanner/NavFn refuses to plan.
                self._set_state(State.BACKUP_FROM_BOX)

        elif self.state == State.BACKUP_FROM_BOX:
            self._do_open_loop_backup()
            if self._backup_elapsed >= self._backup_duration:
                self._stop_rotation()
                self._set_state(State.NAVIGATING_TO_BIN)

        elif self.state == State.NAVIGATING_TO_BIN:
            if self.nav_done:
                self.nav_done = False
                if self.nav_succeeded:
                    self._set_state(State.ARM_PLACE)
                else:
                    self.get_logger().warn(
                        'Navigation to bin failed. Backing up and retrying.')
                    self._set_state(State.BACKUP_FROM_BOX)

        elif self.state == State.ARM_PLACE:
            if self.arm_done:
                self.arm_done = False
                # Always back away from the basket before next nav. The arm
                # extension + basket geometry leaves the rover wedged in
                # the basket's inflation; planner can't plan from inside it.
                self._set_state(State.BACKUP_FROM_BIN)

        elif self.state == State.BACKUP_FROM_BIN:
            self._do_open_loop_backup()
            if self._backup_elapsed >= self._backup_duration:
                self._stop_rotation()
                # If there's another box queued, go pick it up; otherwise
                # return home as the final step.
                if self._box_index + 1 < len(self._box_queue):
                    self._box_index += 1
                    self._activate_current_target()
                    self._set_state(State.NAVIGATING_TO_BOX)
                else:
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
            self.get_logger().info(
                f'Search rotation done ({self.search_timeout}s). '
                f'Driving to {self.target_box}.')
            self._stop_rotation()
            bx_w, by_w = BOX_POSITIONS[self.target_box]
            bx_m, by_m = self._world_to_map(bx_w, by_w)
            self.box_pose = self._make_pose(bx_m, by_m, 0.0)
            self._set_state(State.NAVIGATING_TO_BOX)

    def _stop_rotation(self):
        self.cmd_vel_pub.publish(Twist())  # zero velocity

    def _do_open_loop_backup(self):
        """Drive backward at -0.15 m/s for self._backup_duration seconds.
        Used after arm pickup/place to clear the box / basket inflation
        zone so the global planner can start from a free cell."""
        self._backup_elapsed += 0.1  # 10Hz tick
        twist = Twist()
        twist.linear.x  = -0.15
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(
            f'Backing up... {self._backup_elapsed:.1f}/'
            f'{self._backup_duration:.1f}s',
            throttle_duration_sec=0.5)

    # ─────────────────────────────────────────────────────────────────────────
    # State transitions
    # ─────────────────────────────────────────────────────────────────────────

    def _set_state(self, new_state: str):
        self.get_logger().info(f'State: {self.state} → {new_state}')
        self.state = new_state

        if new_state == State.SEARCHING:
            self.search_elapsed = 0.0

        elif new_state in (State.BACKUP_FROM_BOX, State.BACKUP_FROM_BIN):
            self._backup_elapsed = 0.0

        elif new_state == State.NAVIGATING_TO_BOX:
            approach = self._approach_pose(self.box_pose)
            self._send_nav_goal(approach)

        elif new_state == State.ARM_PICKUP:
            self.arm_done = False
            cmd = f'pickup:{self.target_box}'
            self.get_logger().info(f'Sending ARM command: {cmd}')
            self.arm_cmd_pub.publish(String(data=cmd))
            self._arm_cmd_sent_ns = self.get_clock().now().nanoseconds

        elif new_state == State.NAVIGATING_TO_BIN:
            # Approach the corner basket from the room-center side. Basket is
            # at world (±2, ±2); offset toward the origin by approach_offset.
            sx = -1.0 if self.bin_x > 0 else 1.0
            sy = -1.0 if self.bin_y > 0 else 1.0
            gx_w = self.bin_x + sx * self.approach_offset
            gy_w = self.bin_y + sy * self.approach_offset
            yaw  = math.atan2(self.bin_y - gy_w, self.bin_x - gx_w)
            gx_m, gy_m = self._world_to_map(gx_w, gy_w)
            self._send_nav_goal(self._make_pose(gx_m, gy_m, yaw))

        elif new_state == State.ARM_PLACE:
            self.arm_done = False
            cmd = f'place:{self.target_color}'
            self.get_logger().info(f'Sending ARM command: {cmd}')
            self.arm_cmd_pub.publish(String(data=cmd))
            self._arm_cmd_sent_ns = self.get_clock().now().nanoseconds

        elif new_state == State.RETURNING_HOME:
            hx_m, hy_m = self._world_to_map(self.home_x, self.home_y)
            self._send_nav_goal(self._make_pose(hx_m, hy_m, self.home_yaw))

    # ─────────────────────────────────────────────────────────────────────────
    # Navigation
    # ─────────────────────────────────────────────────────────────────────────

    def _send_nav_goal(self, pose: PoseStamped):
        self.nav_done      = False
        self.nav_succeeded = False

        # Be patient — the user starts Nav2 in another terminal, and it can
        # take 30s+ to come up after Gazebo+SLAM. Wait up to 2 minutes,
        # logging progress every 5s, before declaring failure.
        deadline = 120.0
        elapsed  = 0.0
        step     = 5.0
        while not self.nav_client.wait_for_server(timeout_sec=step):
            elapsed += step
            if elapsed >= deadline:
                self.get_logger().error(
                    f'Nav2 action server still unavailable after {deadline}s. '
                    'Did you run `bash ~/start_nav2_minimal.sh` in another '
                    'terminal? Aborting mission.')
                self._set_state(State.FAILED)
                return
            self.get_logger().warn(
                f'Waiting for Nav2 action server... ({elapsed:.0f}s elapsed)')

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

    def _lookup_rover_world(self):
        """Return the rover's WORLD (x, y) by reading map→base_link from TF
        and adding the spawn offset. Returns None if TF isn't available."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except Exception:
            return None
        mx = t.transform.translation.x
        my = t.transform.translation.y
        # map origin is at the spawn pose, so world = map + spawn.
        return (mx + self.spawn_x, my + self.spawn_y)

    def _activate_current_target(self):
        """Set self.target_box / target_color / bin_xy from the current
        index in self._box_queue."""
        self.target_box = self._box_queue[self._box_index]
        self.target_color = color_of(self.target_box)
        self.bin_x, self.bin_y = BASKET_POSITIONS[self.target_color]
        bx, by = BOX_POSITIONS[self.target_box]
        bx_m, by_m = self._world_to_map(bx, by)
        self.box_pose = self._make_pose(bx_m, by_m, 0.0)
        self.get_logger().info(
            f'─── Target {self._box_index + 1}/{len(self._box_queue)}: '
            f'{self.target_box} (color={self.target_color}) '
            f'box@({bx},{by}) → basket@({self.bin_x},{self.bin_y}) ───')
        self._approach_attempts = 0

    def _world_to_map(self, wx: float, wy: float):
        """Translate WORLD-frame coords into the SLAM map frame, whose origin
        coincides with the rover's spawn pose."""
        return wx - self.spawn_x, wy - self.spawn_y

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

        # Approach from south at the BOX offset (tight, so arm reaches).
        gx = tx
        gy = ty - self.box_approach_offset
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
