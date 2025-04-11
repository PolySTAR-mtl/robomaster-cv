// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from polystar_msgs:msg/Shoot.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__SHOOT__BUILDER_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__SHOOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "polystar_msgs/msg/detail/shoot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace polystar_msgs
{

namespace msg
{

namespace builder
{

class Init_Shoot_shoot
{
public:
  explicit Init_Shoot_shoot(::polystar_msgs::msg::Shoot & msg)
  : msg_(msg)
  {}
  ::polystar_msgs::msg::Shoot shoot(::polystar_msgs::msg::Shoot::_shoot_type arg)
  {
    msg_.shoot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::polystar_msgs::msg::Shoot msg_;
};

class Init_Shoot_stamp
{
public:
  Init_Shoot_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Shoot_shoot stamp(::polystar_msgs::msg::Shoot::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Shoot_shoot(msg_);
  }

private:
  ::polystar_msgs::msg::Shoot msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::polystar_msgs::msg::Shoot>()
{
  return polystar_msgs::msg::builder::Init_Shoot_stamp();
}

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__SHOOT__BUILDER_HPP_
