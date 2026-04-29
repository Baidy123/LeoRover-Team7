#!/bin/bash

# This script generates a combined Leo + Arm URDF on the fly

TEMP_URDF="/tmp/leo_with_arm.urdf"

cat > $TEMP_URDF << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="leo_complete">
  
  <!-- Include base Leo -->
  <xacro:include filename="$(find leo_description)/urdf/leo.urdf.xacro"/>
  
  <!-- Camera link -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.05 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.08" rpy="0 0.2 0"/>
  </joint>

  <!-- Gazebo camera sensor -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <topic>camera/image_raw</topic>
      <update_rate>30</update_rate>
      <camera>
        <horizontal_fov>1.91986</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.05</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <visualize>true</visualize>
    </sensor>
  </gazebo>

  <!-- Arm base -->
  <link name="arm_base_link">
    <visual>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.05"/>
      </geometry>
      <material name="grey">
        <color rgba="0.3 0.3 0.3 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="arm_base_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_base_link"/>
    <origin xyz="0.0 0 0.12" rpy="0 0 0"/>
  </joint>

  <!-- Arm link 1 -->
  <link name="arm_link1">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.4 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.025" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="arm_joint1" type="revolute">
    <parent link="arm_base_link"/>
    <child link="arm_link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1.0"/>
  </joint>

  <!-- Arm link 2 -->
  <link name="arm_link2">
    <visual>
      <origin xyz="0 0 0.06" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.12"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.06" rpy="0 0 0"/>
      <geometry>
        <box size="0.03 0.03 0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.25"/>
      <inertia ixx="0.0003" ixy="0" ixz="0" iyy="0.0003" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="arm_joint2" type="revolute">
    <parent link="arm_link1"/>
    <child link="arm_link2"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Gripper -->
  <link name="gripper">
    <visual>
      <origin xyz="0 0 0.03" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.08 0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.03" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.08 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="gripper_joint" type="fixed">
    <parent link="arm_link2"/>
    <child link="gripper"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/>
  </joint>

</robot>
EOF

# Process with xacro
xacro $TEMP_URDF
