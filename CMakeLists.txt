cmake_minimum_required(VERSION 3.8)
project(galax_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(galax_bringup REQUIRED)
find_package(path_planning_package REQUIRED)
# find_package(custom_interfaces REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()



# C++
# DWA parameters
generate_parameter_library(dwa_parameters_file # cmake target name for the parameter library
  config/dwa_parameters.yaml # path to input yaml file
)

# Generate Parameter Library
ament_auto_add_executable(DWA_node
  src/DWA_node.cpp
  src/DWA_class.cpp
  src/observations_server_class.cpp
)
target_link_libraries(DWA_node
  dwa_parameters_file
)
target_include_directories(DWA_node
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/dwa_parameters_file/include
)
install(TARGETS DWA_node
  DESTINATION lib/${PROJECT_NAME}
)


# COPY A PARTICULAR FOLDER TO THE INSTALL DIRECTORY
# Install config dependencies
install(
  DIRECTORY
    config
    rviz
    map
  DESTINATION
    share/${PROJECT_NAME}
)


# Install launchfile
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()