cmake_minimum_required(VERSION 3.8)
project(leo_lidar_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(include)

# C++ object detection node (DISABLED - use Python version instead)
# Uncomment if you need C++ version and have all dependencies
# add_executable(lidar_object_detector src/lidar_object_detector.cpp)
# ament_target_dependencies(lidar_object_detector
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
#   pcl_ros
#   pcl_conversions
# )

# Install C++ executables
# install(TARGETS
#   lidar_object_detector
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install Python scripts
install(PROGRAMS
  scripts/simple_object_detector.py
  scripts/odom_to_tf.py
  scripts/camera_info_publisher.py
  scripts/scan_health_checker.py
  scripts/scan_corruptor.py
  scripts/simple_arm_controller.py
  scripts/detection_node.py
  scripts/adapter.py
  scripts/vision_to_navigation.py
  scripts/autonomous_mission.py
  scripts/arm_controller.py
  scripts/arm_mission.py
  scripts/box_detector.py
  scripts/lidar_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  urdf
  worlds
  rviz
  maps
  models
  scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
