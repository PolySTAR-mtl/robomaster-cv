// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from polystar_msgs:msg/Movement.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__MOVEMENT__BUILDER_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__MOVEMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "polystar_msgs/msg/detail/movement__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace polystar_msgs
{

namespace msg
{

namespace builder
{

class Init_Movement_omega
{
public:
  explicit Init_Movement_omega(::polystar_msgs::msg::Movement & msg)
  : msg_(msg)
  {}
  ::polystar_msgs::msg::Movement omega(::polystar_msgs::msg::Movement::_omega_type arg)
  {
    msg_.omega = std::move(arg);
    return std::move(msg_);
  }

private:
  ::polystar_msgs::msg::Movement msg_;
};

class Init_Movement_v_y
{
public:
  explicit Init_Movement_v_y(::polystar_msgs::msg::Movement & msg)
  : msg_(msg)
  {}
  Init_Movement_omega v_y(::polystar_msgs::msg::Movement::_v_y_type arg)
  {
    msg_.v_y = std::move(arg);
    return Init_Movement_omega(msg_);
  }

private:
  ::polystar_msgs::msg::Movement msg_;
};

class Init_Movement_v_x
{
public:
  explicit Init_Movement_v_x(::polystar_msgs::msg::Movement & msg)
  : msg_(msg)
  {}
  Init_Movement_v_y v_x(::polystar_msgs::msg::Movement::_v_x_type arg)
  {
    msg_.v_x = std::move(arg);
    return Init_Movement_v_y(msg_);
  }

private:
  ::polystar_msgs::msg::Movement msg_;
};

class Init_Movement_stamp
{
public:
  Init_Movement_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Movement_v_x stamp(::polystar_msgs::msg::Movement::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Movement_v_x(msg_);
  }

private:
  ::polystar_msgs::msg::Movement msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::polystar_msgs::msg::Movement>()
{
  return polystar_msgs::msg::builder::Init_Movement_stamp();
}

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__MOVEMENT__BUILDER_HPP_
