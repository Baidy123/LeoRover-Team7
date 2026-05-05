"""
Team 7 Leo Rover - Detect + Approach (manual exploration version)

Minimal two-state FSM:
  WAIT_FOR_BLOCK  - Passively wait until detection_node reports a block
                    (operator drives the rover with PS4 to search). Once
                    any block is in view, lock onto the nearest one.
  APPROACH        - Compute a stand-off pose (stop_dist metres in front
                    of the block, on the rover->block ray, facing the
                    block) and send it to Nav2 via NavigateToPose.
                    Stream feedback while moving and read back the final
                    result (success / failed / cancelled).

Subscriptions:
  /detected_objects  (std_msgs/String, JSON list, from detection_node)

Action client:
  /navigate_to_pose  (nav2_msgs/action/NavigateToPose, provided by Nav2)

Parameters:
  world_frame   (default 'map'; use 'odom' in sim)
  base_frame    (default 'base_link')
  stop_dist     (default 0.5 m; how far to stop before the block so the
                 rover doesn't hit the obstacle the block sits on)
"""

import time
import enum
import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

import tf2_ros


class State(enum.Enum):
    WAIT_FOR_BLOCK = 'WAIT_FOR_BLOCK'
    APPROACH = 'APPROACH'
    DONE = 'DONE'
    RECOVERY = 'RECOVERY'


class DetectApproachNode(Node):

    NAV_TIMEOUT = 60.0
    POLL_PERIOD = 0.1
    FEEDBACK_LOG_PERIOD = 2.0       # how often to print Nav2 feedback

    def __init__(self):
        super().__init__('detect_approach')

        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('stop_dist', 0.5)   # m, stand-off from the block

        self.world_frame = self.get_parameter('world_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.stop_dist = float(self.get_parameter('stop_dist').value)

        self.cb_group = ReentrantCallbackGroup()

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- Subscriptions ----------
        self.create_subscription(
            String, '/detected_objects', self._objects_cb, 10,
            callback_group=self.cb_group)

        # ---------- Nav2 Action Client ----------
        self.nav_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.cb_group)

        # ---------- State variables ----------
        self.state = State.WAIT_FOR_BLOCK
        self.current_target = None       # currently selected block dict
        self._latest_objects = []        # most recent detection frame
        self._objects_lock = threading.Lock()

        self._nav_done = threading.Event()
        self._nav_result_status = None   # GoalStatus.STATUS_SUCCEEDED etc.
        self._nav_goal_handle = None     # active goal handle, used for cancel
        self._last_feedback_log = 0.0

        self.get_logger().info(
            f"Detect+Approach node ready (world_frame={self.world_frame})")
        self.get_logger().info(
            "Manual mode: drive the rover with PS4 to find a block")

    # ---------- Callbacks ----------

    def _objects_cb(self, msg):
        try:
            objs = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("bad /detected_objects JSON")
            return
        with self._objects_lock:
            self._latest_objects = objs

    # ---------- Nav2 action wrappers ----------

    def _cancel_current_nav_goal(self):
        """Cancel the currently running Nav2 goal. Nav2 will halt the rover
        on its own once the cancel propagates."""
        if self._nav_goal_handle is None:
            return
        self.get_logger().info("[NAV] cancelling current goal")
        self._nav_goal_handle.cancel_goal_async()
        # Don't block waiting; the result callback will be triggered with
        # status STATUS_CANCELED.

    def _send_nav_goal(self, pose_xy_yaw):
        """Send a NavigateToPose goal asynchronously. Blocks only long
        enough to confirm the action server is up.
        pose_xy_yaw: (x, y, yaw) in world_frame."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "[NAV] /navigate_to_pose action server not available")
            return False

        gx, gy, yaw = pose_xy_yaw
        qz, qw = self._yaw_to_quat(yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.world_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(gx)
        goal_msg.pose.pose.position.y = float(gy)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        self._nav_done.clear()
        self._nav_result_status = None
        self._last_feedback_log = 0.0

        self.get_logger().info(
            f"[NAV] sending goal -> ({gx:.2f}, {gy:.2f}, "
            f"yaw={math.degrees(yaw):.0f} deg) in {self.world_frame}")

        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)
        return True

    def _goal_response_cb(self, future):
        """Triggered when the action server accepts/rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[NAV] goal rejected by server")
            self._nav_result_status = GoalStatus.STATUS_ABORTED
            self._nav_done.set()
            return

        self.get_logger().info("[NAV] goal accepted")
        self._nav_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        """Periodic Nav2 feedback. NavigateToPose feedback fields:
            current_pose, navigation_time, estimated_time_remaining,
            number_of_recoveries, distance_remaining."""
        now = time.time()
        if now - self._last_feedback_log < self.FEEDBACK_LOG_PERIOD:
            return
        self._last_feedback_log = now

        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        eta = fb.estimated_time_remaining.sec
        recoveries = fb.number_of_recoveries
        self.get_logger().info(
            f"[NAV] {dist:.2f} m remaining, ETA ~{eta} s, "
            f"recoveries={recoveries}")

    def _result_cb(self, future):
        """Triggered when Nav2 finishes (success / failure / cancelled)."""
        result = future.result()
        status = result.status
        self._nav_result_status = status
        self._nav_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("[NAV] succeeded")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("[NAV] cancelled")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("[NAV] aborted")
        else:
            self.get_logger().warn(f"[NAV] ended with status {status}")

        self._nav_done.set()

    def _wait_nav_done(self, timeout):
        return self._nav_done.wait(timeout=timeout)

    def _get_robot_xy(self):
        """Look up the rover's current (x, y) in world_frame via TF.
        Returns None if TF not yet available."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
            return (tf.transform.translation.x,
                    tf.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(
                f"TF {self.world_frame} <- {self.base_frame} failed: {e}")
            return None

    def _compute_standoff_pose(self, block_xyz):
        """Given a block's world position, compute a goal pose that is
        stop_dist metres in front of the block, on the line from the
        rover's current position to the block. Yaw faces the block.

        Returns (gx, gy, yaw) or None if TF lookup fails or the rover
        is already inside the stand-off radius."""
        robot = self._get_robot_xy()
        if robot is None:
            return None
        rx, ry = robot
        bx, by = block_xyz[0], block_xyz[1]

        dx = bx - rx
        dy = by - ry
        d = math.hypot(dx, dy)

        if d < 1e-6:
            # rover is exactly on the block; nothing sensible to do
            return None

        # Yaw = direction from rover to block (so the rover ends up
        # facing the block when it stops).
        yaw = math.atan2(dy, dx)

        if d <= self.stop_dist:
            # We're already closer than the stand-off; don't move.
            self.get_logger().info(
                f"[NAV] already within stop_dist ({d:.2f} m <= "
                f"{self.stop_dist:.2f} m), staying put")
            return (rx, ry, yaw)

        # Goal = block position pulled back by stop_dist along the
        # rover->block ray.
        ux = dx / d
        uy = dy / d
        gx = bx - ux * self.stop_dist
        gy = by - uy * self.stop_dist
        return (gx, gy, yaw)

    @staticmethod
    def _yaw_to_quat(yaw):
        """2D yaw -> quaternion (z, w). Roll/pitch = 0."""
        return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    # ---------- Helpers ----------

    def _latest_blocks(self):
        with self._objects_lock:
            return [o for o in self._latest_objects
                    if o.get('type') == 'block']

    # ---------- State handlers ----------

    def _do_wait_for_block(self):
        self.get_logger().info("=== WAIT_FOR_BLOCK ===")
        self.get_logger().info(
            "Waiting for a block in view. Drive with PS4 to search.")

        target = None
        last_log = 0.0
        while rclpy.ok():
            blocks = self._latest_blocks()
            if blocks:
                # Pick the closest one by camera depth.
                # TODO: when there are several blocks, the distance from
                # base_link (or odom) is more meaningful than camera depth.
                # Switch to that once we add multi-target handling.
                target = min(blocks, key=lambda o: o['depth'])
                break

            now = time.time()
            if now - last_log > 5.0:
                self.get_logger().info("(still waiting...)")
                last_log = now

            time.sleep(self.POLL_PERIOD)

        self.current_target = target
        wx, wy, wz = target['world']
        self.get_logger().info(
            f"[FSM] target: {target['color']} block @ world "
            f"({wx:.2f}, {wy:.2f}, {wz:.2f}), depth {target['depth']:.2f} m")
        return State.APPROACH

    def _do_approach(self):
        self.get_logger().info("=== APPROACH ===")

        # TODO: target world coordinates are sampled once at lock time and
        # reused throughout the drive. If the block moves or the initial
        # detection is noisy, we won't correct mid-flight. Future work:
        # periodically refresh self.current_target['world'] from
        # /detected_objects and re-issue the goal if it drifts > threshold.

        # Compute a stand-off pose so we stop before hitting the block /
        # the obstacle it's resting on.
        goal_pose = self._compute_standoff_pose(self.current_target['world'])
        if goal_pose is None:
            self.get_logger().warn(
                "[NAV] could not compute stand-off pose (TF unavailable?)")
            return State.RECOVERY

        ok = self._send_nav_goal(goal_pose)
        if not ok:
            return State.RECOVERY

        ok = self._wait_nav_done(timeout=self.NAV_TIMEOUT)
        if not ok:
            self.get_logger().warn("[NAV] timeout, cancelling")
            self._cancel_current_nav_goal()
            return State.RECOVERY

        if self._nav_result_status != GoalStatus.STATUS_SUCCEEDED:
            return State.RECOVERY

        self.get_logger().info("[FSM] arrived at block, ready for PICKUP")
        return State.DONE

    def _do_recovery(self):
        self.get_logger().info("=== RECOVERY ===")
        self._cancel_current_nav_goal()
        return State.DONE

    # ---------- Main loop ----------

    def run(self):
        handlers = {
            State.WAIT_FOR_BLOCK: self._do_wait_for_block,
            State.APPROACH: self._do_approach,
            State.RECOVERY: self._do_recovery,
        }

        try:
            while rclpy.ok() and self.state != State.DONE:
                next_state = handlers[self.state]()
                if next_state != self.state:
                    self.get_logger().info(
                        f"[FSM] {self.state.value} -> {next_state.value}")
                self.state = next_state
        except KeyboardInterrupt:
            pass
        finally:
            self.get_logger().info("done")


def main():
    rclpy.init()
    node = DetectApproachNode()

    # Multi-threaded executor so ROS callbacks don't block the FSM thread
    # (and vice versa).
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
