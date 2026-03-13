import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy 
from geometry_msgs.msg import PoseArray
import os
import time
import math
import pymycobot
from packaging import version

# min low version require
MIN_REQUIRE_VERSION = '3.6.1'

current_verison = pymycobot.__version__
print('current pymycobot library version: {}'.format(current_verison))
if version.parse(current_verison) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError('The version of pymycobot library must be greater than {} or higher. The current version is {}. Please upgrade the library version.'.format(MIN_REQUIRE_VERSION, current_verison))
else:
    print('pymycobot library version meets the requirements!')
    from pymycobot import MyCobot280

class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("control_sync_plan")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
                PoseArray,
                '/custom_poses',
                self.pose_listener_callback,
                3
        )
        self.subscription
        '''
        self.robot_m5 = os.popen("ls /dev/ttyUSB*").readline()[:-1]
        self.robot_wio = os.popen("ls /dev/ttyACM*").readline()[:-1]
        if self.robot_m5:
            port = self.robot_m5
        else:
            port = self.robot_wio
            '''
        self.get_logger().info("port:%s, baud:%d" % ("/dev/ttyAMA0", 1000000))
        self.mc = MyCobot280("/dev/ttyAMA0", 1000000)
        time.sleep(0.05)
        if self.mc.get_fresh_mode() == 0:
            self.mc.set_fresh_mode(1)
            time.sleep(0.05)

        # RViz中目标顺序
        """
        self.rviz_order = [
            'Joint1',
            'Joint2',
            'Joint3',
            'Joint4',
            'Joint5',
            'Joint6'
        ]
        """
        self.rviz_order = [
            'link1_to_link2',
            'link2_to_link3',
            'link3_to_link4',
            'link4_to_link5',
            'link5_to_link6',
            'link6_to_link6_flange',
        ]

        self.PS4_subscriber = self.create_subscription(
            msg_type=Joy,
            topic='/joy',
            callback=self.PS4_subscriber_callback,
            qos_profile=1)
    def PS4_subscriber_callback(self, msg: Joy):
        print(f"{msg.buttons}")
        if msg.buttons[0]:
            self.mc.jog_coord(1,1,1)
        else: 
            self.mc.stop()

    def listener_callback(self, msg):
        # 创建字典将关节名称与位置值关联
        joint_state_dict = {name: msg.position[i] for i, name in enumerate(msg.name)}
        # 根据 RViz 顺序重新排列关节角度
        data_list = []
        for joint in self.rviz_order:
            # 获取弧度并转为角度
            if joint in joint_state_dict:
                radians_to_angles = round(math.degrees(joint_state_dict[joint]), 3)
                data_list.append(radians_to_angles)
 
        # print('data_list: {}'.format(data_list))
        print('joint_state_dict: {}'.format(joint_state_dict))
        try:
            self.mc.send_angles(data_list, 35)
        except:
            print("No joint angles sent")

        try:
            if 'gripper_controller' in joint_state_dict:
                if joint_state_dict['gripper_controller'] >= -0.3:
                    self.mc.set_gripper_state(0,10)
                else: 
                    self.mc.set_gripper_state(1,10)
        except:
            print("Error sending gripper state")

    def pose_listener_callback(self, msg):
        custom_coords = msg.poses[0]
        custom_coords = [custom_coords.position.x, custom_coords.position.y, custom_coords.position.z, custom_coords.orientation.x, custom_coords.orientation.y, custom_coords.orientation.z]
        self.mc.sync_send_coords(custom_coords, 35, 1)
        print(f"custom coords:{custom_coords}")

def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
