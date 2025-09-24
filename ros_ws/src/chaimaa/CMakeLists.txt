cmake_minimum_required(VERSION 3.22.1)
project(chaimaa)

add_compile_options(-std=c++17)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
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

# Subscriber

add_executable(subscriber
  src/subscriber.cpp
)

target_link_libraries(subscriber PRIVATE
  rclcpp::rclcpp
  ${polystar_msgs_TARGETS}
)

target_include_directories(subscriber PRIVATE
  ${ament_INCLUDE_DIRS}
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
  ${polystar_msgs_TARGETS}
)

ament_package()
