#!/usr/bin/env python3
"""
arm_mission.py — Pick and place using DetachableJoint plugin
============================================================
The DetachableJoint plugin creates a rigid physics joint between
the gripper link and the box when attach topic is published.
The box physically moves with the rover — no teleporting.

Usage: python3 arm_mission.py [box_name] [basket_name]
  box_name:    red_box_1 | red_box_2 | blue_box_1 | blue_box_2 | green_box_1 | green_box_2
  basket_name: red | green | blue | bin

Drive rover next to the box first, then run this script.
"""
import sys
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# ── Config ────────────────────────────────────────────────────────────────────
BASKET_NAV = {
    'red':   (1.6,  1.6),
    'green': (-1.6, 1.6),
    'blue':  (-1.6, -1.6),
    'bin':   (1.6,  0.0),
}
HOME = (1.8, -1.8)

# Arm poses for pick sequence
# seg1=pan, seg2=shoulder, fore=elbow, wrist, fl=finger_left, fr=finger_right
POSES = {
    'home':     (0.0,  0.0,  0.0,  0.0,  0.03,  0.03),
    'pregrasp': (-0.3, 1.4,  1.6, -0.9,  0.03,  0.03),
    'grasp':    (-0.3, 1.5,  1.7, -1.0,  0.03,  0.03),
    'close':    (-0.3, 1.5,  1.7, -1.0, -0.02, -0.02),
    'lift':     (-0.3, 0.8,  0.5, -0.2, -0.02, -0.02),
    'carry':    (0.0,  0.4,  0.2,  0.0, -0.02, -0.02),
    'drop':     (0.0,  1.0,  0.8,  0.3, -0.02, -0.02),
    'release':  (0.0,  1.0,  0.8,  0.3,  0.03,  0.03),
}
DELAYS = {
    'home':3.0,'pregrasp':3.0,'grasp':2.5,'close':2.0,
    'lift':3.0,'carry':2.5,'drop':3.0,'release':2.0,
}

J = {
    'seg1':  '/model/leo/joint/arm_segment1_joint/cmd_pos',
    'seg2':  '/model/leo/joint/arm_segment2_joint/cmd_pos',
    'fore':  '/model/leo/joint/arm_forearm_joint/cmd_pos',
    'wrist': '/model/leo/joint/arm_gripper_joint/cmd_pos',
    'fl':    '/model/leo/joint/finger_left_joint/cmd_pos',
    'fr':    '/model/leo/joint/finger_right_joint/cmd_pos',
}

def gz_pub(topic, val):
    subprocess.run(
        ['gz','topic','-t',topic,'-m','gz.msgs.Double','-p',f'data: {val}'],
        capture_output=True, timeout=3.0)

def gz_empty(topic):
    """Publish empty message to attach/detach topic."""
    subprocess.run(
        ['gz','topic','-t',topic,'-m','gz.msgs.Empty','-p',''],
        capture_output=True, timeout=3.0)

def arm(pose_name):
    s1,s2,fo,wr,fl,fr = POSES[pose_name]
    print(f'  Arm → {pose_name}')
    gz_pub(J['seg1'], s1)
    gz_pub(J['seg2'], s2)
    gz_pub(J['fore'], fo)
    gz_pub(J['wrist'],wr)
    gz_pub(J['fl'],   fl)
    gz_pub(J['fr'],   fr)
    time.sleep(DELAYS[pose_name])

def attach(box_name):
    print(f'  Attaching {box_name} to gripper...')
    gz_empty(f'/box/{box_name}/attach')
    time.sleep(0.5)

def detach(box_name):
    print(f'  Detaching {box_name} from gripper...')
    gz_empty(f'/box/{box_name}/detach')
    time.sleep(0.5)


class Mission(Node):
    def __init__(self):
        super().__init__('arm_mission')
        self.cb_group = ReentrantCallbackGroup()
        self.nav = ActionClient(
            self, NavigateToPose, '/navigate_to_pose',
            callback_group=self.cb_group)

    def nav_to(self, x, y, label=''):
        if not self.nav.wait_for_server(timeout_sec=5.0):
            print('  ERROR: Nav2 unavailable')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        print(f'  Navigating to {label} ({x:.2f}, {y:.2f})...')
        f = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        h = f.result()
        if not h or not h.accepted:
            print('  Nav2 goal rejected!')
            return False
        r = h.get_result_async()
        rclpy.spin_until_future_complete(self, r, timeout_sec=120.0)
        print('  Arrived.')
        return True


def main():
    box_name    = sys.argv[1] if len(sys.argv) > 1 else 'red_box_1'
    basket_name = sys.argv[2] if len(sys.argv) > 2 else 'red'

    if basket_name not in BASKET_NAV:
        print(f'Unknown basket: {basket_name}. Use: red | green | blue | bin')
        return

    rclpy.init()
    node = Mission()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print('\n' + '='*52)
    print(f'  MISSION: Pick {box_name} → {basket_name} basket')
    print('='*52)
    print(f'\nDrive rover next to {box_name} using teleop.')
    input('Press Enter when rover is positioned...\n')

    # ── 1. Grasp sequence ──────────────────────────────────────────────────
    print('[1] Moving arm to grasp pose...')
    arm('home')
    arm('pregrasp')
    arm('grasp')

    # ── 2. Attach box via DetachableJoint ─────────────────────────────────
    print('[2] Closing fingers and attaching box...')
    arm('close')
    attach(box_name)

    # ── 3. Lift ────────────────────────────────────────────────────────────
    print('[3] Lifting box...')
    arm('lift')
    arm('carry')
    print('    Box is now physically attached to gripper.')

    # ── 4. Navigate to basket ─────────────────────────────────────────────
    print(f'[4] Navigating to {basket_name} basket...')
    bx, by = BASKET_NAV[basket_name]
    node.nav_to(bx, by, basket_name)

    # ── 5. Lower and release ──────────────────────────────────────────────
    print('[5] Lowering arm into basket...')
    arm('drop')
    detach(box_name)
    arm('release')
    print('    Box released into basket.')

    # ── 6. Arm home ────────────────────────────────────────────────────────
    print('[6] Arm returning to home...')
    arm('home')

    # ── 7. Rover home ─────────────────────────────────────────────────────
    print('[7] Rover returning home...')
    node.nav_to(HOME[0], HOME[1], 'home')

    print('\n' + '='*52)
    print('  ✅ MISSION COMPLETE!')
    print('='*52 + '\n')

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
