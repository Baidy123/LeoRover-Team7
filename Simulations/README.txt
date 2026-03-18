Final code terminal:

source ~/leo_ws/install/setup.bash
ros2 launch leo_lidar_sim complete_system.launch.py

(updated workspace with bars):  ros2 launch leo_lidar_sim complete_system_with_pillars.launch.py



control the robot:

source ~/leo_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard



map:

ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint










Testing codes:
Copy your Python files to /scripts (Put all codes there)


# Make them executable
chmod +x ~/leo_ws/src/leo_lidar_sim/scripts/*.py

Then update CMakeLists.txt and Add your files to the install(PROGRAMS ...) section

Rebuild and Run your code
