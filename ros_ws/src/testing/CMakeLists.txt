cmake_minimum_required(VERSION 3.22.1)
project(testing)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS serial
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

target_link_libraries(serial_testing PRIVATE
  rclcpp
  std_msgs
  sensor_msgs
  serial
)

add_dependencies(serial_testing ${catkin_EXPORTED_TARGETS})

ament_package()
