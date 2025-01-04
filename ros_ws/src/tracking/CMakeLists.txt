cmake_minimum_required(VERSION 3.22.1)
project(tracking)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_generate_interfaces REQUIRED)
find_package(detection REQUIRED)

ament_python_install_package()

## ----- Messages ----- ##

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Tracklet.msg
  Tracklets.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS detection
)

###########
## Build ##
###########

# This is a pure python node (so far), no build to do

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