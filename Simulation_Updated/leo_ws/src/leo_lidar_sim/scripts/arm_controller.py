#!/usr/bin/env python3
"""
arm_controller.py — IK-based automatic arm control
===================================================
Computes joint angles automatically from box world position
and rover current odom position.

Arm geometry (from URDF):
  base_link → arm_platform : z=0.11 (fixed)
  arm_platform → seg1      : z=0.05 (pan joint, z-axis)
  seg1 → seg2              : z=0.28 (shoulder, y-axis)
  seg2 → forearm           : z=0.28 (elbow, y-axis)
  forearm → gripper        : z=0.18 (wrist, y-axis)
  gripper → fingers        : z=0.04 + 0.05 offset

Total arm height from base_link when upright:
  0.11 + 0.05 + 0.28 + 0.28 + 0.18 + 0.09 = ~0.99m

3-joint planar IK (shoulder + elbow + wrist):
  L1 = 0.28 (seg2)
  L2 = 0.28 (forearm)  
  L3 = 0.27 (gripper+fingers tip)
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
import subprocess
import time
import math

# ── Arm DH parameters (link lengths in metres) ───────────────────────────────
ARM_BASE_Z   = 0.11 + 0.05   # height of seg1 joint above base_link
L1           = 0.28           # seg2 (shoulder to elbow)
L2           = 0.28           # forearm (elbow to wrist)
L3           = 0.27           # gripper + finger tip

# ── Joint topics ──────────────────────────────────────────────────────────────
TOPICS = {
    'seg1':  '/model/leo/joint/arm_segment1_joint/cmd_pos',
    'seg2':  '/model/leo/joint/arm_segment2_joint/cmd_pos',
    'fore':  '/model/leo/joint/arm_forearm_joint/cmd_pos',
    'wrist': '/model/leo/joint/arm_gripper_joint/cmd_pos',
    'fl':    '/model/leo/joint/finger_left_joint/cmd_pos',
    'fr':    '/model/leo/joint/finger_right_joint/cmd_pos',
}

# ── Known box world positions {name: (x, y, z)} ──────────────────────────────
BOX_POSITIONS = {
    'red_box_1':   (0.0,  0.0,  0.925),   # on obstacle_center
    'red_box_2':   (1.5,  1.2,  0.800),   # on obstacle_2
    'blue_box_1':  (-0.8, -0.8, 0.575),   # on obstacle_6
    'blue_box_2':  (-1.2, -1.3, 0.675),   # on obstacle_4
    'green_box_1': (-1.3, 1.0,  0.675),   # on obstacle_3
    'green_box_2': (0.7,  1.5,  0.675),   # on obstacle_7
}

# Names must match the <child_model> entries in the URDF DetachableJoint plugins
ALL_BOX_NAMES = list(BOX_POSITIONS.keys())

# ── Basket positions ──────────────────────────────────────────────────────────
BASKET_POSITIONS = {
    'red':   (2.0,  2.0,  0.5),
    'green': (-2.0, 2.0,  0.5),
    'blue':  (-2.0, -2.0, 0.5),
    'bin':   (2.0,  0.0,  0.2),
}

# ── Home pose ─────────────────────────────────────────────────────────────────
HOME_POSE = {'seg1':0.0,'seg2':0.0,'fore':0.0,'wrist':0.0,'fl':0.03,'fr':0.03}
HOME_DELAY = 3.0

# ── Carry pose (holding box while driving) ────────────────────────────────────
CARRY_POSE = {'seg1':0.0,'seg2':0.5,'fore':0.3,'wrist':0.0,'fl':-0.02,'fr':-0.02}
CARRY_DELAY = 2.5


class ArmController(Node):

    def __init__(self):
        super().__init__('arm_controller')
        self.cb_group = ReentrantCallbackGroup()

        self.done_pub = self.create_publisher(Bool, '/arm_action_done', 10)
        self.create_subscription(
            String, '/arm_command', self._command_cb, 10,
            callback_group=self.cb_group)

        # Track rover pose from odom
        self.rover_x   = 0.0
        self.rover_y   = 0.0
        self.rover_yaw = 0.0
        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10,
            callback_group=self.cb_group)

        self._busy        = False
        self._target_box  = None   # set by 'pickup:<box_name>' command
        self._target_drop = None   # set by 'place:<basket_name>' command
        self._held_box    = None   # name of currently grasped box, or None

        self.get_logger().info('ArmController IK ready. Commands:')
        self.get_logger().info('  pickup:<box_name>  e.g. pickup:red_box_1')
        self.get_logger().info('  place:<basket>     e.g. place:red')
        self.get_logger().info('  home')

        # CRITICAL: gz-sim-detachable-joint-system starts ATTACHED by default.
        # The launch file already broadcasts detach right after rover spawn
        # via ExecuteProcess. We re-broadcast IMMEDIATELY here as belt-and-
        # suspenders in case this node was launched standalone, and also at
        # 1s + 3s in case the gz subscriber wasn't ready on the first try.
        # Note: _initial_release_once is idempotent (guarded by a flag), but
        # the flag is reset before each scheduled call so all three rounds
        # actually publish.
        self._initial_release_done = False
        self._initial_release_once()
        self._schedule_repeated_release()

    # ── Odom ──────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.rover_yaw = math.atan2(siny, cosy)

    # ── Command handler ───────────────────────────────────────────────────────

    def _command_cb(self, msg: String):
        if self._busy:
            self.get_logger().warn('Arm busy — ignoring command')
            return
        self._busy = True
        try:
            cmd = msg.data.strip()
            if cmd.startswith('pickup:'):
                box_name = cmd.split(':', 1)[1]
                self._pickup(box_name)
            elif cmd.startswith('place:'):
                basket = cmd.split(':', 1)[1]
                self._place(basket)
            elif cmd == 'pickup':
                # Default to red_box_1
                self._pickup('red_box_1')
            elif cmd == 'place':
                self._place('red')
            elif cmd == 'home':
                self._go_home()
            else:
                self.get_logger().warn(f'Unknown command: {cmd}')
                self._publish_done(False)
        except Exception as e:
            self.get_logger().error(f'Arm error: {e}')
            self._publish_done(False)
        finally:
            self._busy = False

    # ── Pickup sequence ───────────────────────────────────────────────────────

    def _pickup(self, box_name: str):
        if box_name not in BOX_POSITIONS:
            self.get_logger().error(f'Unknown box: {box_name}')
            self._publish_done(False)
            return

        bx, by, bz = BOX_POSITIONS[box_name]
        self.get_logger().info(
            f'[PICKUP] Target: {box_name} at world ({bx},{by},{bz})')
        self.get_logger().info(
            f'[PICKUP] Rover at odom ({self.rover_x:.2f},{self.rover_y:.2f},'
            f' yaw={math.degrees(self.rover_yaw):.1f}°)')

        # Compute IK
        angles = self._compute_ik(bx, by, bz)
        if angles is None:
            self.get_logger().error('[PICKUP] IK failed — box out of reach')
            self._publish_done(False)
            return

        seg1, seg2, fore, wrist = angles
        self.get_logger().info(
            f'[PICKUP] IK → seg1={math.degrees(seg1):.1f}° '
            f'seg2={math.degrees(seg2):.1f}° '
            f'fore={math.degrees(fore):.1f}° '
            f'wrist={math.degrees(wrist):.1f}°')

        # Home first
        self._send_pose(HOME_POSE, HOME_DELAY)

        # Open fingers
        self._pub(TOPICS['fl'], 0.03)
        self._pub(TOPICS['fr'], 0.03)
        time.sleep(0.5)

        # Pre-grasp — pan to box, arm up
        self._pub(TOPICS['seg1'], seg1)
        self._pub(TOPICS['seg2'], seg2 * 0.6)
        self._pub(TOPICS['fore'], fore * 0.4)
        self._pub(TOPICS['wrist'], wrist * 0.4)
        time.sleep(2.5)

        # Full grasp position
        self._pub(TOPICS['seg2'], seg2)
        self._pub(TOPICS['fore'], fore)
        self._pub(TOPICS['wrist'], wrist)
        time.sleep(3.0)

        # Close fingers
        self._pub(TOPICS['fl'], -0.02)
        self._pub(TOPICS['fr'], -0.02)
        time.sleep(1.5)

        # Rigidly attach the box to arm_gripper (DetachableJoint plugin).
        # This is what actually makes the gripper "hold" the box in physics.
        self._attach_box(box_name)
        self._held_box = box_name
        time.sleep(0.5)

        # Lift
        self._pub(TOPICS['seg2'], seg2 * 0.5)
        self._pub(TOPICS['fore'], fore * 0.3)
        self._pub(TOPICS['wrist'], 0.0)
        time.sleep(2.5)

        # Carry pose — safe for driving
        self._send_pose(CARRY_POSE, CARRY_DELAY)

        self.get_logger().info(f'[PICKUP] ✅ Complete — holding {box_name}')
        self._publish_done(True)

    # ── Place sequence ────────────────────────────────────────────────────────

    def _place(self, basket: str):
        if basket not in BASKET_POSITIONS:
            self.get_logger().error(f'Unknown basket: {basket}')
            self._publish_done(False)
            return

        bx, by, bz = BASKET_POSITIONS[basket]
        self.get_logger().info(f'[PLACE] Target basket: {basket} at ({bx},{by},{bz})')

        angles = self._compute_ik(bx, by, bz + 0.1)  # slightly above basket
        if angles is None:
            self.get_logger().error('[PLACE] IK failed — basket out of reach')
            # Fallback: just open fingers to drop
            self._pub(TOPICS['fl'], 0.03)
            self._pub(TOPICS['fr'], 0.03)
            time.sleep(1.0)
            self._send_pose(HOME_POSE, HOME_DELAY)
            self._publish_done(True)
            return

        seg1, seg2, fore, wrist = angles

        # Move to basket position
        self._pub(TOPICS['seg1'], seg1)
        self._pub(TOPICS['seg2'], seg2)
        self._pub(TOPICS['fore'], fore)
        self._pub(TOPICS['wrist'], wrist)
        time.sleep(3.5)

        # Detach the held box BEFORE opening fingers — otherwise the rigid
        # gripper-finger constraint can flick the box across the room when
        # the prismatic joints suddenly move.
        if self._held_box is not None:
            self._detach_box(self._held_box)
            self.get_logger().info(f'[PLACE] Released {self._held_box}')
            self._held_box = None
            time.sleep(0.3)

        # Open fingers
        self._pub(TOPICS['fl'], 0.03)
        self._pub(TOPICS['fr'], 0.03)
        time.sleep(1.5)

        # Return home
        self._send_pose(HOME_POSE, HOME_DELAY)

        self.get_logger().info('[PLACE] ✅ Complete')
        self._publish_done(True)

    def _go_home(self):
        self._send_pose(HOME_POSE, HOME_DELAY)
        self._publish_done(True)

    # ── Inverse Kinematics ────────────────────────────────────────────────────

    def _compute_ik(self, world_x, world_y, world_z):
        """
        Compute joint angles to reach (world_x, world_y, world_z).

        Step 1: pan angle (seg1) = bearing from rover to target in rover frame
        Step 2: horizontal distance from arm base to target
        Step 3: vertical offset from arm shoulder to target
        Step 4: 3-joint planar IK (seg2, fore, wrist)
        """
        # Rover world position — odom approximates map for short distances
        rx = self.rover_x
        ry = self.rover_y
        ryaw = self.rover_yaw

        # Vector from rover to box in world frame
        dx_world = world_x - rx
        dy_world = world_y - ry

        # Transform to rover body frame
        dx_body =  dx_world * math.cos(ryaw) + dy_world * math.sin(ryaw)
        dy_body = -dx_world * math.sin(ryaw) + dy_world * math.cos(ryaw)

        # Pan angle — seg1 rotates around Z
        seg1 = math.atan2(dy_body, dx_body)
        # Clamp to joint limits
        seg1 = max(-3.14, min(3.14, seg1))

        # Horizontal reach needed (distance in body XY plane)
        h_reach = math.sqrt(dx_body**2 + dy_body**2)

        # Vertical: target z relative to shoulder joint height
        shoulder_z = ARM_BASE_Z  # height of seg1 joint above base_link
        dz = world_z - shoulder_z

        self.get_logger().info(
            f'IK: h_reach={h_reach:.3f}m  dz={dz:.3f}m  '
            f'seg1={math.degrees(seg1):.1f}°')

        # 3-joint planar IK for seg2 + fore + wrist
        # Target: reach h_reach horizontally, dz vertically
        # Using L1=seg2, L2=fore, L3=gripper_tip
        # Simplification: split reach between L1+L2 for main reach,
        # use wrist to fine-tune tip angle

        # Effective reach needed by L1+L2 (wrist handles last L3)
        # Target for L1+L2 tip:
        tx = h_reach - L3 * math.cos(0.3)  # approximate wrist contribution
        tz = dz      + L3 * math.sin(0.3)

        dist = math.sqrt(tx**2 + tz**2)

        # Check reachability
        if dist > (L1 + L2):
            self.get_logger().warn(
                f'IK: target dist {dist:.3f}m > max reach {L1+L2:.3f}m — '
                'clamping to max reach')
            scale = (L1 + L2 - 0.02) / dist
            tx *= scale
            tz *= scale
            dist = math.sqrt(tx**2 + tz**2)

        if dist < abs(L1 - L2):
            self.get_logger().warn('IK: target too close — clamping')
            dist = abs(L1 - L2) + 0.01

        # Law of cosines for elbow angle
        cos_elbow = (L1**2 + L2**2 - dist**2) / (2 * L1 * L2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_angle = math.acos(cos_elbow)  # 0=straight, pi=fully bent

        # Elbow down configuration
        fore = math.pi - elbow_angle

        # Shoulder angle
        alpha = math.atan2(tz, tx)
        beta  = math.acos(max(-1.0, min(1.0,
                    (L1**2 + dist**2 - L2**2) / (2 * L1 * dist))))
        seg2 = alpha + beta

        # Wrist — keep gripper roughly horizontal (pointing forward/down)
        wrist = -(seg2 - fore) * 0.5

        # Clamp all to joint limits
        seg2  = max(-2.0, min(2.0, seg2))
        fore  = max(-2.0, min(2.0, fore))
        wrist = max(-1.57, min(1.57, wrist))

        return seg1, seg2, fore, wrist

    # ── Low-level helpers ─────────────────────────────────────────────────────

    def _send_pose(self, pose: dict, delay: float):
        for joint, val in pose.items():
            self._pub(TOPICS[joint], val)
        time.sleep(delay)

    def _pub(self, topic: str, value: float):
        try:
            subprocess.run(
                ['gz', 'topic', '-t', topic,
                 '-m', 'gz.msgs.Double',
                 '-p', f'data: {value}'],
                capture_output=True, text=True, timeout=3.0)
        except Exception as e:
            self.get_logger().error(f'pub error: {e}')

    def _pub_empty(self, topic: str):
        """Publish an empty gz.msgs.Empty message — used for attach/detach."""
        try:
            subprocess.run(
                ['gz', 'topic', '-t', topic,
                 '-m', 'gz.msgs.Empty',
                 '-p', ''],
                capture_output=True, text=True, timeout=3.0)
        except Exception as e:
            self.get_logger().error(f'pub_empty error: {e}')

    # ── DetachableJoint control ───────────────────────────────────────────────

    def _attach_box(self, box_name: str):
        topic = f'/box/{box_name}/attach'
        self.get_logger().info(f'  -> ATTACH {topic}')
        self._pub_empty(topic)

    def _detach_box(self, box_name: str):
        topic = f'/box/{box_name}/detach'
        self.get_logger().info(f'  -> DETACH {topic}')
        self._pub_empty(topic)

    def _initial_release_once(self):
        """
        Publish detach to every box. Safe to call multiple times — the gz
        subscriber may not be ready on the first publish, so the launch file
        also fires this and we schedule repeats below.
        """
        self.get_logger().info(
            'Releasing all boxes from initial attached state...')
        for name in ALL_BOX_NAMES:
            self._detach_box(name)
        self.get_logger().info('Initial release complete.')

    def _schedule_repeated_release(self):
        """Re-broadcast detach at 1s and 3s in case the first attempt was
        too early for the gz subscriber to be ready."""
        self._t1 = self.create_timer(
            1.0, self._release_once_then_cancel(lambda: self._t1),
            callback_group=self.cb_group)
        self._t2 = self.create_timer(
            3.0, self._release_once_then_cancel(lambda: self._t2),
            callback_group=self.cb_group)

    def _release_once_then_cancel(self, get_timer):
        """Returns a callback that fires the release once, then cancels
        its own timer so it doesn't repeat forever."""
        def cb():
            self._initial_release_once()
            t = get_timer()
            if t is not None:
                t.cancel()
        return cb

    def _publish_done(self, success: bool):
        self.done_pub.publish(Bool(data=success))


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('ArmController interrupted.')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
