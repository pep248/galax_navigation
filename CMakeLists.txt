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
find_package(galax_bringup REQUIRED)
# find_package(custom_interfaces REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# # CPP
# # Add the action client executable
# ament_auto_add_executable(path_planner_server
#   src/path_planner_node.cpp
#   # src/path_planner_class.cpp
#   ) #dependency
# # Install the executable
# install(TARGETS path_planner_server
#   DESTINATION lib/${PROJECT_NAME}
# )

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