# Install script for directory: /home/student31/leo_ws/src/leo_lidar_sim

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/student31/leo_ws/src/leo_lidar_sim/install/leo_lidar_sim")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/leo_lidar_sim" TYPE PROGRAM FILES
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts/simple_object_detector.py"
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts/odom_to_tf.py"
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts/camera_info_publisher.py"
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts/scan_health_checker.py"
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts/scan_corruptor.py"
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts/simple_arm_controller.py"
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts/detection_node.py"
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts/adapter.py"
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts/vision_to_navigation.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim" TYPE DIRECTORY FILES
    "/home/student31/leo_ws/src/leo_lidar_sim/launch"
    "/home/student31/leo_ws/src/leo_lidar_sim/config"
    "/home/student31/leo_ws/src/leo_lidar_sim/urdf"
    "/home/student31/leo_ws/src/leo_lidar_sim/worlds"
    "/home/student31/leo_ws/src/leo_lidar_sim/rviz"
    "/home/student31/leo_ws/src/leo_lidar_sim/maps"
    "/home/student31/leo_ws/src/leo_lidar_sim/models"
    "/home/student31/leo_ws/src/leo_lidar_sim/scripts"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/leo_lidar_sim")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/leo_lidar_sim")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim/environment" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim/environment" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_index/share/ament_index/resource_index/packages/leo_lidar_sim")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim/cmake" TYPE FILE FILES
    "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_core/leo_lidar_simConfig.cmake"
    "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/ament_cmake_core/leo_lidar_simConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/leo_lidar_sim" TYPE FILE FILES "/home/student31/leo_ws/src/leo_lidar_sim/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/student31/leo_ws/src/leo_lidar_sim/build/leo_lidar_sim/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
