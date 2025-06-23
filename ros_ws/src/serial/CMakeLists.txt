cmake_minimum_required(VERSION 3.22.1)
project(serial)

# Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(polystar_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(ament_cmake_gtest)


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

add_executable(serial_interface
  src/serial_spinner.cpp
  src/serial_interface.cpp
)

target_include_directories(serial_interface PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ament_INCLUDE_DIRS}
)

ament_target_dependencies(serial_interface 
  rclcpp
  polystar_msgs
)

ament_package()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()


# ament_add_gtest(serialize test/serialize.cpp)
# target_link_libraries(serialize PRIVATE 
  # serial_spinner 
  # ament_cmake_gtest::ament_cmake_gtest
# )

# ament_add_gtest(deserialize test/deserialize.cpp)
# target_link_libraries(deserialize PRIVATE
  # serial_spinner 
  # ament_cmake_gtest::ament_cmake_gtest
# )

# TODO: FIX TESTS
