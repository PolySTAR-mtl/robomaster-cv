// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from polystar_msgs:msg/TurretFeedback.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__TURRET_FEEDBACK__BUILDER_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__TURRET_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "polystar_msgs/msg/detail/turret_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace polystar_msgs
{

namespace msg
{

namespace builder
{

class Init_TurretFeedback_yaw
{
public:
  explicit Init_TurretFeedback_yaw(::polystar_msgs::msg::TurretFeedback & msg)
  : msg_(msg)
  {}
  ::polystar_msgs::msg::TurretFeedback yaw(::polystar_msgs::msg::TurretFeedback::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::polystar_msgs::msg::TurretFeedback msg_;
};

class Init_TurretFeedback_pitch
{
public:
  explicit Init_TurretFeedback_pitch(::polystar_msgs::msg::TurretFeedback & msg)
  : msg_(msg)
  {}
  Init_TurretFeedback_yaw pitch(::polystar_msgs::msg::TurretFeedback::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_TurretFeedback_yaw(msg_);
  }

private:
  ::polystar_msgs::msg::TurretFeedback msg_;
};

class Init_TurretFeedback_stamp
{
public:
  Init_TurretFeedback_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TurretFeedback_pitch stamp(::polystar_msgs::msg::TurretFeedback::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_TurretFeedback_pitch(msg_);
  }

private:
  ::polystar_msgs::msg::TurretFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::polystar_msgs::msg::TurretFeedback>()
{
  return polystar_msgs::msg::builder::Init_TurretFeedback_stamp();
}

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__TURRET_FEEDBACK__BUILDER_HPP_
