cmake_minimum_required(VERSION 3.22.1)
project(polystar_msgs)

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

###################################
## ament specific configuration ##
###################################

ament_export_dependencies(
  rosidl_default_runtime
)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/HP.msg
  msg/SwitchOrder.msg
  msg/Target.msg
  msg/Movement.msg
  msg/GameStatus.msg
  msg/GameStage.msg
  msg/TurretFeedback.msg
  msg/PositionFeedback.msg
  msg/Shoot.msg
  msg/Detection.msg
  msg/Detections.msg
  msg/Tracklet.msg
  msg/Tracklets.msg
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()
