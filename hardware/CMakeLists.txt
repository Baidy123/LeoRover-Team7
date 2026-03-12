cmake_minimum_required(VERSION 3.8)
project(leo_lidar_sim)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install Python scripts and bash scripts
install(PROGRAMS
  scripts/detection_node.py
  scripts/vision_to_navigation.py
  scripts/scan_filter.py
  scripts/start_nav2.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz configs (optional)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
