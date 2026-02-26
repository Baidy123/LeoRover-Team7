<?xml version="1.0"?>
<sdf version="1.8">
  <model name="mycobot_arm">
    <static>false</static>
    
    <!-- Arm Base (mounted on rover) -->
    <link name="arm_base">
      <pose>0 0 0.025 0 0 0</pose>
      <inertial>
        <mass>0.5</mass>
        <inertia>
          <ixx>0.001</ixx>
          <iyy>0.001</iyy>
          <izz>0.001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.08 0.08 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.08 0.08 0.05</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <!-- Arm Link 1 (rotating column) -->
    <link name="arm_link1">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>0.3</mass>
        <inertia>
          <ixx>0.0005</ixx>
          <iyy>0.0005</iyy>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.025</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.4 0.8 1</ambient>
          <diffuse>0.2 0.4 0.8 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.025</radius>
            <length>0.1</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
    
    <joint name="arm_joint1" type="revolute">
      <parent>arm_base</parent>
      <child>arm_link1</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <lower>-3.14</lower>
          <upper>3.14</upper>
          <effort>10</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
    </joint>
    
    <!-- Arm Link 2 (shoulder) -->
    <link name="arm_link2">
      <pose>0 0 0.21 0 0 0</pose>
      <inertial>
        <mass>0.25</mass>
        <inertia>
          <ixx>0.0003</ixx>
          <iyy>0.0003</iyy>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.03 0.03 0.12</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.4 0.8 1</ambient>
          <diffuse>0.2 0.4 0.8 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.03 0.03 0.12</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <joint name="arm_joint2" type="revolute">
      <parent>arm_link1</parent>
      <child>arm_link2</child>
      <pose>0 0 -0.06 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>10</effort>
          <velocity>1.0</velocity>
        </limit>
      </axis>
    </joint>
    
    <!-- Gripper -->
    <link name="gripper">
      <pose>0 0 0.33 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.05 0.08 0.03</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.05 0.08 0.03</size>
          </box>
        </geometry>
      </collision>
    </link>
    
    <joint name="gripper_joint" type="fixed">
      <parent>arm_link2</parent>
      <child>gripper</child>
      <pose>0 0 -0.015 0 0 0</pose>
    </joint>
    
  </model>
</sdf>
