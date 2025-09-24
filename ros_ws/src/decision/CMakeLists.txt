cmake_minimum_required(VERSION 3.22.1)
project(decision)

add_compile_options(-std=c++17)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(detection REQUIRED)
find_package(polystar_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(
  detection
  polystar_msgs
  tf2
  tf2_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# Simple Tracker 

add_executable(simple_tracker
  src/simple_tracker.cpp
  src/tracker.cpp
)

target_link_libraries(simple_tracker PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${polystar_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${OpenCV_LIBRARIES}
)

target_include_directories(simple_tracker PRIVATE
  ${ament_INCLUDE_DIRS}
)

# Weighted tracker

add_executable(weighted_tracker
  src/weighted_tracker.cpp
  src/bounding_box.cpp
  src/tracker.cpp
)

target_include_directories(weighted_tracker PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
  ${ament_INCLUDE_DIRS}
  /usr/include/opencv4
)

target_link_libraries(weighted_tracker PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${polystar_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${OpenCV_LIBRARIES}
)

# Publisher 

add_executable(publisher 
  src/publisher.cpp
)

target_include_directories(publisher PRIVATE
  ${ament_INCLUDE_DIRS}
)

target_link_libraries(publisher PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${polystar_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)


install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  simple_tracker
  weighted_tracker
  publisher
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

ament_package()
