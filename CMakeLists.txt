cmake_minimum_required(VERSION 3.8)
project(galax_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# DEPENDENCIES


# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(navigation2 REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(path_planning_package REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find_package(gazebo_ros REQUIRED)
# find_package(gazebo_ros_pkgs REQUIRED)
# find_package(gazebo_ros2_control REQUIRED)
# find_package(rviz2 REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# C++
# DWA Node
ament_auto_add_executable(DWA_node
  src/DWA_node.cpp
  src/DWA_class.cpp
)
# Observations Server Node
ament_auto_add_executable(observations_server_node
  src/observations_server_node.cpp
  src/observations_server_class.cpp
)


# Generate Parameter Library
# DWA parameters
generate_parameter_library(dwa_parameters_file # cmake target name for the parameter library
  config/dwa_parameters.yaml # path to input yaml file
)

# Parameters to the DWA node
target_link_libraries(DWA_node
  dwa_parameters_file
)
target_include_directories(DWA_node
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/dwa_parameters_file/include
)
install(TARGETS DWA_node
  DESTINATION lib/${PROJECT_NAME}
)

# Parameters to the observations server node
target_link_libraries(observations_server_node
  dwa_parameters_file
)
target_include_directories(observations_server_node
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/dwa_parameters_file/include
)
install(TARGETS observations_server_node
  DESTINATION lib/${PROJECT_NAME}
)


# PYTHON
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/initial_pose_publisher.py
  scripts/TD3_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# COPY A PARTICULAR FOLDER TO THE INSTALL DIRECTORY
# Install config dependencies
install(
  DIRECTORY
    agent
    config
    rviz
    map
  DESTINATION
    share/${PROJECT_NAME}
)


# LAUNCH
# Install launchfile
ament_auto_package(INSTALL_TO_SHARE launch)