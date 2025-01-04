cmake_minimum_required(VERSION 3.22.1)
project(monitor)

add_compile_options(-std=c++17)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(detection REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS detection
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(monitor_video
  src/monitor_video_node.cpp
  src/video_monitor.cpp
)

target_include_directories(monitor_video PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  ${ament_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(monitor_video
  rclcpp
  std_msgs
  sensor_msgs
  detection
  cv_bridge
  ${OpenCV_LIBRARIES}
)

add_executable(monitor_logger
  src/monitor_logger.cpp
)

target_include_directories(monitor_logger PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  ${ament_INCLUDE_DIRS}
)

target_link_libraries(monitor_logger
  rclcpp
)

add_dependencies(monitor_video ${catkin_EXPORTED_TARGETS})

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