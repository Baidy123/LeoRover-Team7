<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="leo_world">
    
    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Plugin for physics -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
    </plugin>
    
    <!-- Plugin for user commands -->
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands">
    </plugin>
    
    <!-- Plugin for scene -->
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    
    <!-- Plugin for sensors -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- Light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles for object detection -->
    
    <!-- Box 1 — target for pickup (non-static so arm can move it) -->
    <model name="box_1">
      <pose>3 2 0.25 0 0 0</pose>
      <static>false</static>
      <link name="link">
        <inertial>
          <mass>0.3</mass>
          <inertia><ixx>0.003</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.003</iyy><iyz>0</iyz><izz>0.003</izz></inertia>
        </inertial>
        <collision name="collision">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 0.5 0.5</size></box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Box 2 -->
    <model name="box_2">
      <pose>-2 3 0.3 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.6 0.6 0.6</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.6 0.6 0.6</size>
            </box>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Cylinder 1 -->
    <model name="cylinder_1">
      <pose>4 -3 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.0</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Wall -->
    <model name="wall">
      <pose>0 5 0.5 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>8 0.2 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>8 0.2 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Multiple small obstacles -->
    <model name="obstacle_1">
      <pose>1 1 0.15 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.3 0.3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.3 0.3</size>
            </box>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient>
            <diffuse>1 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="obstacle_2">
      <pose>-3 -2 0.15 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.3 0.3 0.3</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.3 0.3 0.3</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 1 1</ambient>
            <diffuse>1 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>


    <!-- BIN — drop target for the pick-and-place mission -->
    <model name="bin">
      <static>true</static>
      <pose>5 2 0 0 0 0</pose>
      <link name="floor">
        <pose>0 0 0.01 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.5 0.5 0.02</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>0.5 0.5 0.02</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.9 1</diffuse></material>
        </visual>
      </link>
      <link name="front_wall">
        <pose>0 -0.24 0.1 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.5 0.02 0.2</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>0.5 0.02 0.2</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.9 1</diffuse></material>
        </visual>
      </link>
      <link name="back_wall">
        <pose>0 0.24 0.1 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.5 0.02 0.2</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>0.5 0.02 0.2</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.9 1</diffuse></material>
        </visual>
      </link>
      <link name="left_wall">
        <pose>-0.24 0 0.1 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.02 0.5 0.2</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>0.02 0.5 0.2</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.9 1</diffuse></material>
        </visual>
      </link>
      <link name="right_wall">
        <pose>0.24 0 0.1 0 0 0</pose>
        <collision name="col"><geometry><box><size>0.02 0.5 0.2</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>0.02 0.5 0.2</size></box></geometry>
          <material><ambient>0.2 0.2 0.8 1</ambient><diffuse>0.2 0.2 0.9 1</diffuse></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
