#!/usr/bin/env python3
"""
arm_controller.py — IK-based pick & place using "magnetic carry"
=================================================================
Strategy
--------
The DetachableJoint plugin is unreliable when many boxes start attached
(over-constrained startup yanks the rover and teleports boxes). Instead
we simulate grasp by directly setting the box pose every tick while held,
via the Gazebo Transport service `/world/<world>/set_pose`. This is rock
solid: when held, the box rides exactly where we put it; when released,
gravity takes over.

Commands (publish std_msgs/String to /arm_command):
  pickup:<box_name>      e.g. pickup:red_box_1
  place:<basket_name>    e.g. place:red    (red|green|blue)
  home

Status:
  publishes std_msgs/Bool on /arm_action_done (data=True on success).
"""

import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from std_msgs.msg import Bool, String

import tf2_ros

# In-process gz transport for BOTH arm joint commands and magnetic carry.
# Using subprocess for either path starves the other; in-process keeps
# both fast and contention-free.
try:
    from gz.transport13 import Node as _GzNode
    from gz.msgs10.pose_pb2 import Pose as _GzPose
    from gz.msgs10.boolean_pb2 import Boolean as _GzBoolean
    from gz.msgs10.double_pb2 import Double as _GzDouble
    _GZ_OK = True
except Exception as _e:
    _GZ_OK = False
    _GZ_IMPORT_ERR = str(_e)

# ── World name ────────────────────────────────────────────────────────────────
WORLD_NAME = 'sorting_room'

# ── Spawn pose (must match launch file) ──────────────────────────────────────
# Odometry initialises to (0,0,0) at the rover's spawn location, but the box
# and basket positions below are in WORLD frame. We add this offset to odom
# to compute the rover's world pose.
SPAWN_X = 1.8
SPAWN_Y = -1.8

# ── Arm geometry (must match URDF) ────────────────────────────────────────────
# Heights in z relative to base_link:
#   arm_platform top : 0.11 + 0.05      = 0.16
#   seg1 top         : 0.16 + 0.28      = 0.44   (seg1 is a vertical pillar)
#   seg2 origin      : same as seg1 top = 0.44
ARM_BASE_Z   = 0.11 + 0.05
SEG1_LEN     = 0.28      # vertical pillar (no pitch)
SEG2_ORIGIN_Z = ARM_BASE_Z + SEG1_LEN     # = 0.44 above base_link
L_SEG2 = 0.40            # shoulder-to-elbow
L_FORE = 0.30            # elbow-to-wrist
L_GRIP = 0.13            # wrist-to-tip (palm + fingers)
# Base_link sits ~0.16 m above the ground in leo_description
BASE_LINK_Z = 0.16

# Backward-compatibility aliases used by the FK helper
SEG2_LEN = L_SEG2
FORE_LEN = L_FORE
GRIP_LEN = L_GRIP

# ── Gazebo joint command topics ───────────────────────────────────────────────
TOPICS = {
    'seg1':  '/model/leo/joint/arm_segment1_joint/cmd_pos',
    'seg2':  '/model/leo/joint/arm_segment2_joint/cmd_pos',
    'fore':  '/model/leo/joint/arm_forearm_joint/cmd_pos',
    'wrist': '/model/leo/joint/arm_gripper_joint/cmd_pos',
    'fl':    '/model/leo/joint/finger_left_joint/cmd_pos',
    'fr':    '/model/leo/joint/finger_right_joint/cmd_pos',
}

# ── World positions of the boxes at startup ──────────────────────────────────
BOX_POSITIONS = {
    'red_box_1':   (0.0,  0.0,  0.25),
    'red_box_2':   (1.5,  1.2,  0.25),
    'blue_box_1':  (-0.8, -0.8, 0.25),
    'blue_box_2':  (-1.2, -1.3, 0.25),
    'green_box_1': (-1.3, 1.0,  0.25),
    'green_box_2': (0.7,  1.5,  0.25),
}
ALL_BOX_NAMES = list(BOX_POSITIONS.keys())

# ── Basket positions (corners of the room) ───────────────────────────────────
# These match sorting_room.sdf; the place height is the basket-rim+a bit so
# the box drops cleanly into the open top.
BASKET_POSITIONS = {
    'red':   (2.0,  2.0,  1.10),
    'green': (-2.0, 2.0,  1.10),
    'blue':  (-2.0, -2.0, 1.10),
}

# ── Poses ─────────────────────────────────────────────────────────────────────
HOME_POSE  = {'seg1': 0.0, 'seg2': 0.0, 'fore': 0.0, 'wrist': 0.0,
              'fl': 0.03, 'fr': 0.03}
CARRY_POSE = {'seg1': 0.0, 'seg2': 0.4, 'fore': -0.6, 'wrist': 0.2,
              'fl': -0.02, 'fr': -0.02}

# Link lengths used in forward kinematics (must match URDF).
SEG1_LEN  = 0.28   # arm_segment1 length (vertical pillar)
SEG2_LEN  = 0.28   # arm_segment2 (shoulder-elbow)
FORE_LEN  = 0.30   # arm_forearm (elbow-wrist) — matches URDF + IK
GRIP_LEN  = 0.13   # palm + finger tip


class ArmController(Node):

    def __init__(self):
        super().__init__('arm_controller')
        self.cb_group = ReentrantCallbackGroup()

        self.done_pub = self.create_publisher(Bool, '/arm_action_done', 10)
        self.create_subscription(
            String, '/arm_command', self._command_cb, 10,
            callback_group=self.cb_group)

        # Rover pose is read from TF (map → base_link) on demand at pickup
        # time. No /odom subscription — when we had one, an async odom_cb
        # could overwrite self.rover_* between the TF refresh and the IK
        # solve, causing IK to silently use stale data.
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_yaw = 0.0

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._busy = False
        self._held_box = None   # name of the box currently being carried

        # Track most recent commanded joint setpoints (kept for diagnostics).
        self._cmd = {'seg1': 0.0, 'seg2': 0.0, 'fore': 0.0, 'wrist': 0.0,
                     'fl': 0.03, 'fr': 0.03}

        # Magnetic-carry tick. While _held_box is set, the box is
        # teleported to follow the gripper tip via the world set_pose
        # service. 5 Hz is plenty for visual continuity and doesn't
        # starve nav2's controller_server with subprocess fork/execs
        # (20 Hz did — controller_server lost its 20 Hz target rate
        # and the rover stopped mid-path). Last-pose memo skips the
        # gz subprocess entirely when the gripper hasn't moved.
        # Subprocess gz CLI for joint commands (proven path). NO carry
        # timer — the DetachableJoint plugin in the URDF holds the box
        # rigidly to the gripper, and any periodic set_pose teleport
        # forks `gz service` subprocesses fast enough to stall the
        # rover's controller_server mid-drive.
        self._gz_node = None
        self._joint_pubs = {}
        self.get_logger().info(
            'Arm controller using subprocess gz CLI; carry tick disabled '
            '(DetachableJoint provides rigid attachment)')

        # Re-broadcast detach for every box on startup. The DetachableJoint
        # plugins start attached and must be released before they wreck
        # physics. The launch file already does this, but doing it here
        # too is cheap insurance.
        for name in ALL_BOX_NAMES:
            self._publish_detach(name)

        self.get_logger().info(
            'ArmController ready. Commands: pickup:<box>, place:<color>, home')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _refresh_pose_from_tf(self):
        """Read map→base_link from TF and convert to WORLD coords. This is
        the same pose Nav2/SLAM use; raw /odom can drift."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.3))
        except Exception as e:
            self.get_logger().warn(
                f'TF map→base_link unavailable; using /odom. ({e})')
            return
        self.rover_x = t.transform.translation.x + SPAWN_X
        self.rover_y = t.transform.translation.y + SPAWN_Y
        q = t.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.rover_yaw = math.atan2(siny, cosy)

    def _command_cb(self, msg: String):
        if self._busy:
            self.get_logger().warn('Arm busy — ignoring command')
            return
        self._busy = True
        try:
            cmd = msg.data.strip()
            if cmd.startswith('pickup:'):
                self._pickup(cmd.split(':', 1)[1])
            elif cmd.startswith('place:'):
                self._place(cmd.split(':', 1)[1])
            elif cmd == 'home':
                self._send_pose(HOME_POSE, 3.0)
                self._publish_done(True)
            else:
                self.get_logger().warn(f'Unknown command: {cmd}')
                self._publish_done(False)
        except Exception as e:
            self.get_logger().error(f'Arm error: {e}')
            self._publish_done(False)
        finally:
            self._busy = False

    # ── Pickup ───────────────────────────────────────────────────────────────

    def _pickup(self, box_name: str):
        if box_name not in BOX_POSITIONS:
            self.get_logger().error(f'Unknown box: {box_name}')
            self._publish_done(False)
            return

        # Refresh rover pose from TF and snapshot it into locals so no
        # background callback can change the values between the log line
        # and the IK solve.
        self._refresh_pose_from_tf()
        rx, ry, ryaw = self.rover_x, self.rover_y, self.rover_yaw

        bx, by, bz = BOX_POSITIONS[box_name]
        self.get_logger().info(
            f'[PICKUP] {box_name} at ({bx},{by},{bz}). '
            f'Rover at ({rx:.2f},{ry:.2f}, yaw={math.degrees(ryaw):.1f}°)')

        angles = self._compute_ik(bx, by, bz, rx, ry, ryaw)
        if angles is None:
            self.get_logger().error('[PICKUP] IK failed')
            self._publish_done(False)
            return
        seg1, seg2, fore, wrist = angles

        # Open fingers, go to a pre-grasp pose above the box.
        self._cmd_joint('fl', 0.03)
        self._cmd_joint('fr', 0.03)
        self._cmd_joint('seg1', seg1)
        self._cmd_joint('seg2', seg2 * 0.7)
        self._cmd_joint('fore', fore * 0.5)
        self._cmd_joint('wrist', wrist * 0.5)
        time.sleep(2.5)

        # Descend onto the box. Longer settle time so the joint position
        # controllers actually reach the IK target before the DetachableJoint
        # snapshot is taken — short settles left the gripper a few cm above
        # the box, locking that gap into the rigid joint.
        self._cmd_joint('seg2', seg2)
        self._cmd_joint('fore', fore)
        self._cmd_joint('wrist', wrist)
        time.sleep(4.0)

        # Close fingers (visual cue).
        self._cmd_joint('fl', -0.02)
        self._cmd_joint('fr', -0.02)
        time.sleep(1.2)

        # Rigidly attach the box to arm_gripper via the DetachableJoint
        # plugin.
        self._publish_attach(box_name)
        self._held_box = box_name
        time.sleep(0.5)

        # Lift to carry pose.
        self._send_pose(CARRY_POSE, 2.0)

        self.get_logger().info(f'[PICKUP] ✅ holding {box_name}')
        self._publish_done(True)

    # ── Place ────────────────────────────────────────────────────────────────

    def _place(self, basket: str):
        if basket not in BASKET_POSITIONS:
            self.get_logger().error(f'Unknown basket: {basket}')
            self._publish_done(False)
            return
        if self._held_box is None:
            self.get_logger().warn('[PLACE] no box held — opening fingers anyway')

        bx, by, bz = BASKET_POSITIONS[basket]
        self.get_logger().info(
            f'[PLACE] basket={basket} at ({bx},{by},{bz})')

        # Detach the box, then teleport it INTO the basket so it lands
        # cleanly inside (not on the floor near the basket).
        if self._held_box is not None:
            held = self._held_box
            self._publish_detach(held)
            self._held_box = None
            time.sleep(0.3)
            # Drop INSIDE the basket: at the bin's (x, y) and the bin
            # rim height so it falls cleanly into the open top.
            self._set_box_pose(held, bx, by, bz)
            time.sleep(0.3)

        # Open fingers + return to home.
        self._cmd_joint('fl', 0.03)
        self._cmd_joint('fr', 0.03)
        time.sleep(0.8)
        self._send_pose(HOME_POSE, 3.0)

        self.get_logger().info('[PLACE] ✅ done')
        self._publish_done(True)

    # ── DetachableJoint attach/detach ────────────────────────────────────────

    def _publish_attach(self, box_name: str):
        topic = f'/box/{box_name}/attach'
        self.get_logger().info(f'  -> ATTACH {topic}')
        self._publish_empty(topic)

    def _publish_detach(self, box_name: str):
        topic = f'/box/{box_name}/detach'
        self.get_logger().info(f'  -> DETACH {topic}')
        self._publish_empty(topic)

    def _publish_empty(self, topic: str):
        try:
            subprocess.run(
                ['gz', 'topic', '-t', topic,
                 '-m', 'gz.msgs.Empty', '-p', ''],
                capture_output=True, text=True, timeout=2.0)
        except Exception as e:
            self.get_logger().error(f'pub empty error: {e}')

    # ── Forward kinematics (kept for diagnostics; not used now) ─────────────

    def _gripper_world(self):
        """
        Return the world position of the gripper tip, computed from the
        most recent commanded joint setpoints (forward kinematics).

        Frame chain (matches URDF):
          base_link → arm_platform (+0.11 z)
                    → arm_segment1 (+0.05 z, then z-axis rotation by seg1)
                    → arm_segment2 (+0.28 z, then y-axis rotation by seg2)
                    → arm_forearm  (+0.28 z, then y-axis rotation by fore)
                    → arm_gripper  (+0.18 z, then y-axis rotation by wrist)
                    → tip          (+0.13 z, palm + fingers)
        """
        s1, s2, fr, wr = (self._cmd['seg1'], self._cmd['seg2'],
                          self._cmd['fore'], self._cmd['wrist'])

        # Cumulative pitch around y after each pitch joint
        p2 = s2
        p3 = s2 + fr
        p4 = s2 + fr + wr

        # Tip in the post-pan arm plane (x = horizontal forward, z = up).
        # arm_segment1 is vertical (only pans), so it contributes pure z.
        x = (math.sin(p2) * SEG2_LEN
             + math.sin(p3) * FORE_LEN
             + math.sin(p4) * GRIP_LEN)
        z = (SEG1_LEN
             + math.cos(p2) * SEG2_LEN
             + math.cos(p3) * FORE_LEN
             + math.cos(p4) * GRIP_LEN)

        # arm_platform sits at base_link + 0.11; seg1 at +0.05 above that.
        z += 0.11 + 0.05

        # base_link is ~0.16 m above ground (Leo URDF).
        z += 0.16

        # Pan around z by seg1 → body x/y; then transform body → world.
        body_x = x * math.cos(s1)
        body_y = x * math.sin(s1)
        c = math.cos(self.rover_yaw)
        s = math.sin(self.rover_yaw)
        wx = self.rover_x + c * body_x - s * body_y
        wy = self.rover_y + s * body_x + c * body_y
        wz = z
        return wx, wy, wz

    # ── Magnetic carry ───────────────────────────────────────────────────────

    def _carry_tick(self):
        """While a box is held, teleport it to follow the gripper tip.
        At 2 Hz the box would free-fall ~1.2 m between teleports and
        bounce. Sending a Pose message via gz set_pose normally resets
        velocity to zero, so the box should stay snapped to the gripper.
        We park it 4 cm below the tip so it visually sits in the
        fingers rather than overlapping the palm."""
        if self._held_box is None:
            return
        self._refresh_pose_from_tf()
        try:
            wx, wy, wz = self._gripper_world()
        except Exception as e:
            self.get_logger().warn(
                f'carry: _gripper_world failed: {e}',
                throttle_duration_sec=2.0)
            return
        bz = wz - 0.04
        self._set_box_pose(self._held_box, wx, wy, bz)

    # ── IK ───────────────────────────────────────────────────────────────────

    def _compute_ik(self, world_x, world_y, world_z, rx, ry, ryaw):
        """
        3-DOF planar IK with the gripper held vertical (pointing straight
        down) so it can grasp from above. Rover pose (rx, ry, ryaw) is
        passed in explicitly — never read from self — so background
        callbacks can't race with the solve.
        """
        dx = world_x - rx
        dy = world_y - ry
        dx_b =  dx * math.cos(ryaw) + dy * math.sin(ryaw)
        dy_b = -dx * math.sin(ryaw) + dy * math.cos(ryaw)

        # Pan (z-axis) angle to the box.
        seg1 = max(-3.14, min(3.14, math.atan2(dy_b, dx_b)))

        # Horizontal reach in the post-pan plane (forward = +x).
        h_reach = math.sqrt(dx_b * dx_b + dy_b * dy_b)

        # Vertical reach is from the seg2 origin (top of the seg1 pillar)
        # down to the box. seg2 origin world z = BASE_LINK_Z + SEG2_ORIGIN_Z.
        seg2_origin_z = BASE_LINK_Z + SEG2_ORIGIN_Z
        dz = world_z - seg2_origin_z   # negative because box is below

        # Force gripper to point STRAIGHT DOWN. Wrist target is L_GRIP
        # above the box, then pushed deeper by GRASP_DEPTH so the fingers
        # wrap *around* the box at attach time (no residual gap).
        GRASP_DEPTH = 0.05
        target_x = h_reach
        target_z = dz + L_GRIP - GRASP_DEPTH

        # 2-link planar IK to put the wrist at (target_x, target_z) using
        # links L_SEG2 and L_FORE.
        dist = math.hypot(target_x, target_z)
        max_reach = L_SEG2 + L_FORE
        if dist > max_reach:
            self.get_logger().warn(
                f'IK: target {dist:.3f}m beyond max reach {max_reach:.3f}m')
            return None
        if dist < abs(L_SEG2 - L_FORE) + 1e-3:
            dist = abs(L_SEG2 - L_FORE) + 1e-3

        # elbow_angle = angle between L_SEG2 and L_FORE links, 0 = straight.
        cos_elbow = (L_SEG2 ** 2 + L_FORE ** 2 - dist ** 2) / (2 * L_SEG2 * L_FORE)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_angle = math.acos(cos_elbow)
        # URDF convention: fore_urdf is the bend AT the elbow. fore_urdf > 0
        # bends toward +x in the post-pan plane (elbow-down configuration).
        fore = math.pi - elbow_angle

        # angle from x-axis to the wrist target (in the seg2 plane)
        phi = math.atan2(target_z, target_x)
        # angle between L_SEG2 direction and the (origin → target) line
        cos_alpha = (L_SEG2 ** 2 + dist ** 2 - L_FORE ** 2) / (2 * L_SEG2 * dist)
        cos_alpha = max(-1.0, min(1.0, cos_alpha))
        alpha = math.acos(cos_alpha)
        # seg2 absolute angle from x-axis. Elbow-DOWN config: seg2 is above
        # the line to the target by alpha (so the elbow swings down).
        seg2_from_x = phi + alpha
        # Convert to URDF convention (angle from vertical-up)
        seg2 = math.pi / 2.0 - seg2_from_x

        # Force gripper-down: A_grip = π = seg2 + fore + wrist.
        wrist = math.pi - seg2 - fore

        # Clamp to joint limits. seg2 ±2.0, fore ±2.0, wrist ±1.57.
        seg2  = max(-2.0, min(2.0, seg2))
        fore  = max(-2.0, min(2.0, fore))
        wrist = max(-1.57, min(1.57, wrist))

        self.get_logger().info(
            f'IK: h_reach={h_reach:.3f} dz={dz:.3f} '
            f'pan={math.degrees(seg1):.1f}° '
            f'seg2={math.degrees(seg2):.1f}° '
            f'fore={math.degrees(fore):.1f}° '
            f'wrist={math.degrees(wrist):.1f}°')

        return seg1, seg2, fore, wrist

    # ── Low-level helpers ────────────────────────────────────────────────────

    def _send_pose(self, pose: dict, delay: float):
        for joint, val in pose.items():
            self._cmd_joint(joint, val)
        time.sleep(delay)

    def _cmd_joint(self, joint_key: str, value: float):
        """Publish a joint setpoint AND record it in self._cmd so the FK
        in _gripper_world() reflects the latest command."""
        self._cmd[joint_key] = value
        if self._gz_node is not None:
            msg = _GzDouble()
            msg.data = float(value)
            try:
                self._joint_pubs[joint_key].publish(msg)
            except Exception as e:
                self.get_logger().warn(
                    f'in-process joint publish failed: {e}',
                    throttle_duration_sec=2.0)
            return
        # Subprocess fallback (slow).
        self._pub(TOPICS[joint_key], value)

    def _pub(self, topic: str, value: float):
        try:
            subprocess.run(
                ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double',
                 '-p', f'data: {value}'],
                capture_output=True, text=True, timeout=2.0)
        except Exception as e:
            self.get_logger().error(f'pub error: {e}')

    def _set_box_pose(self, name: str, x: float, y: float, z: float):
        """Teleport a model to (x,y,z). In-process gz transport when
        available; subprocess fallback otherwise."""
        if self._gz_node is not None:
            req = _GzPose()
            req.name = name
            req.position.x = float(x)
            req.position.y = float(y)
            req.position.z = float(z)
            req.orientation.w = 1.0
            try:
                ok, rep = self._gz_node.request(
                    self._set_pose_service, req, _GzPose, _GzBoolean, 200)
                if not ok:
                    self.get_logger().warn(
                        f'set_pose timed out',
                        throttle_duration_sec=3.0)
            except Exception as e:
                self.get_logger().warn(
                    f'gz set_pose threw: {e}', throttle_duration_sec=3.0)
            return

        # Subprocess fallback (slow; only used if gz.transport13 missing).
        req_str = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
                   f'orientation: {{x: 0, y: 0, z: 0, w: 1}}')
        try:
            subprocess.Popen(
                ['gz', 'service', '-s', f'/world/{WORLD_NAME}/set_pose',
                 '--reqtype', 'gz.msgs.Pose',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '300', '--req', req_str],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f'set_pose error: {e}')

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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
