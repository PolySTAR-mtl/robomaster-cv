cmake_minimum_required(VERSION 3.22.1)
project(detection)

set(CMAKE_CXX_STANDARD 17)

## Find macros and libraries
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(polystar_msgs REQUIRED)
find_package(OpenCV)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(
  polystar_msgs
)

if(DARKNET_CMAKE_PATH)
  set(CMAKE_CUDA_ARCHITECTURES 72)
  add_subdirectory(${CMAKE_SOURCE_DIR}/../${DARKNET_PATH} ${CMAKE_CURRENT_BINARY_DIR}/darknet)
  set(Darknet_LIBRARIES dark)
else()
  if(DARKNET_PATH)
    if(NOT IS_ABSOLUTE ${DARKNET_PATH})
      set(DARKNET_PATH ${CMAKE_SOURCE_DIR}/../${DARKNET_PATH})
    endif()
    set(Darknet_LIBRARIES ${DARKNET_PATH}/libdarknet.so)
    set(Darknet_INCLUDE_DIRS ${DARKNET_PATH}/include ${DARKNET_PATH}/src)
  else()
    if(NOT DEEPSTREAM)
      message("DARKNET_PATH not set, searching for darknet in the system")
      find_package(Darknet REQUIRED)
    endif()
  endif()
endif()

if(DEEPSTREAM)
  # Gstreamer
  find_package(PkgConfig) 
  pkg_search_module(GLIB REQUIRED glib-2.0) 
  pkg_search_module(GSTREAMER REQUIRED gstreamer-1.0)
  pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)
  pkg_search_module(GST_RTSP REQUIRED gstreamer-rtsp-1.0)
  pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)

  find_package(CUDA REQUIRED)
  find_package(X11 REQUIRED)

  
  set(Deepstream_PATH /opt/nvidia/deepstream/deepstream)
  file(GLOB Deepstream_common ${Deepstream_PATH}/sources/apps/apps-common/src/*.c)

  link_directories(
    /usr/local/cuda-10.2/lib64
    ${Deepstream_PATH}/lib
  )

  add_library(deepstream-runner
    src/deepstream-app/deepstream_app_config_parser.c
    src/deepstream-app/deepstream_app_main.c
    src/deepstream-app/deepstream_app.c
    ${Deepstream_common}
  )

  target_link_libraries(deepstream-runner PRIVATE
    nvdsgst_meta::nvdsgst_meta
    nvds_meta::nvds_meta
    nvdsgst_helper::nvdsgst_helper
    nvdsgst_smartrecord::nvdsgst_smartrecord
    nvds_utils::nvds_utils
    nvds_msgbroker::nvds_msgbroker
    nvbufsurftransform::nvbufsurftransform
    m::m
    gstrtspserver-1.0::gstrtspserver-1.0
    dl::dl
    ${GSTREAMER_LIBRARIES}
    ${GST_VIDEO_LIBRARIES}
    ${GST_RTSP_LIBRARIES}
    ${JSON_GLIB_LIBRARIES}
    ${X11_LIBRARIES}
    ${CUDA_LIBRARIES}
  ) # That's a lot

  set(Deepstream_INCLUDE_DIRS 
    src/deepstream-app
    ${Deepstream_PATH}/sources/apps/apps-common/includes
    ${Deepstream_PATH}/sources/includes
    ${GSTREAMER_INCLUDE_DIRS}
    ${JSON_GLIB_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    /usr/include/gstreamer-1.0
  )
endif()

## ----- Messages ----- ##

###################################
## ament specific configuration ##
###################################

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

if(DEEPSTREAM)

  add_executable(deepstream_detection
    src/deepstream_node.cpp
    src/deepstream_detector.cpp
  )

  target_include_directories(deepstream_detection PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    ${Deepstream_INCLUDE_DIRS}
    ${ament_INCLUDE_DIRS}
    ${Darknet_INCLUDE_DIRS}
  )

  target_link_libraries(deepstream_detection PRIVATE
    rclcpp::rclcpp
    ${detection_TARGETS}
    ${sensor_msgs_TARGETS}
    deepstream-runner
  )
  
  ament_target_dependencies(deepstream_detection
    rclcpp
    polystar_msgs
    sensor_msgs
  )

  add_dependencies(deepstream_detection ${${PROJECT_NAME}_EXPORTED_TARGETS})
else()

  add_executable(darknet_detection
    src/detection_node.cpp
    src/detector.cpp
  )

  target_include_directories(darknet_detection PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    ${Deepstream_INCLUDE_DIRS}
    ${ament_INCLUDE_DIRS}
    ${Darknet_INCLUDE_DIRS}
  )

  ament_target_dependencies(darknet_detection
    rclcpp
    polystar_msgs
    sensor_msgs
    cv_bridge
  )

  target_link_libraries(darknet_detection
    ${Darknet_LIBRARIES}
  )

  #add_dependencies(darknet_detection ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

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
