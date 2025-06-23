cmake_minimum_required(VERSION 3.22.1)
project(testing)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(polystar_msgs REQUIRED)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(
  polystar_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(serial_testing
  src/serial_testing.cpp
)

target_include_directories(serial_testing PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   ${ament_INCLUDE_DIRS}
)

ament_target_dependencies(serial_testing
  rclcpp
  polystar_msgs
)

target_link_libraries(serial_testing
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${serial_TARGETS}
)

ament_package()
