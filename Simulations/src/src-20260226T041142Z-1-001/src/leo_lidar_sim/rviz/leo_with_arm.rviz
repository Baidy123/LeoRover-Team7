Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Color: 160; 160; 164
      Cell Size: 1
      
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Visual Enabled: true
      Collision Enabled: false
      
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        base_footprint:
          Value: true
        base_link:
          Value: true
        arm_platform:
          Value: true
        arm_segment1:
          Value: true
        arm_segment2:
          Value: true
        arm_gripper:
          Value: true
        lidar_sensor/lidar_link/lidar:
          Value: true
      Show Axes: true
      Show Names: true
      
    - Class: rviz_default_plugins/LaserScan
      Name: LaserScan
      Enabled: true
      Topic:
        Value: /scan
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
      Size (m): 0.05
      Color: 255; 0; 0
      
    - Class: rviz_default_plugins/Map
      Name: Map
      Enabled: true
      Topic:
        Value: /map
        Depth: 1
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
      Color Scheme: map
      
  Global Options:
    Fixed Frame: odom
    Background Color: 48; 48; 48
    
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5.0
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Pitch: 0.5
      Yaw: 0.5
