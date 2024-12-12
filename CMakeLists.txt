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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


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


# LAUNCH
# Install launchfile
ament_auto_package(INSTALL_TO_SHARE launch)