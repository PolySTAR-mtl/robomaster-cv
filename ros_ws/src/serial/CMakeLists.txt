cmake_minimum_required(VERSION 3.22.1)
project(serial)

# Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_generate_interfaces REQUIRED)

## ----- Messages ----- ##

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  HP.msg
  Rune.msg
  SwitchOrder.msg
  Target.msg
  Movement.msg
  GameStatus.msg
  GameStage.msg
  TurretFeedback.msg
  PositionFeedback.msg
  Shoot.msg
)

generate_messages()


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
add_library(serial_spinner src/serial_spinner)

add_executable(serial_interface
  src/serial_interface.cpp
)

target_include_directories(serial_interface PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  ${ament_INCLUDE_DIRS}
)

target_link_libraries(serial_interface PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  rosidl_generate_interfaces::rosidl_generate_interfaces
  serial_spinner
)

add_dependencies(serial_spinner ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(serial_interface ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()


catkin_add_gtest(serialize test/serialize.cpp)
target_link_libraries(serialize serial_spinner ${catkin_LIBRARIES})

catkin_add_gtest(deserialize test/deserialize.cpp)
target_link_libraries(deserialize serial_spinner ${catkin_LIBRARIES})

ament_package()