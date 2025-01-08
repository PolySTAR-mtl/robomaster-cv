cmake_minimum_required(VERSION 3.22.1)
project(locate)

set(CMAKE_CXX_STANDARD 17)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(
  nav_msgs
  sensor_msgs
  serial
  tf2
  tf2_ros
)

###########
## Build ##
###########

add_executable(locate
  src/locate.cpp
  src/odom.cpp
  src/imu.cpp
  src/turret.cpp
)

target_include_directories(locate PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  ${ament_INCLUDE_DIRS}
)

target_link_libraries(locate PRIVATE
  rclcpp:rclcpp
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${serial_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  Eigen3::Eigen
)

ament_package()