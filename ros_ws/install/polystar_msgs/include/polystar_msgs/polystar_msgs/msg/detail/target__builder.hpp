// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from polystar_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__TARGET__BUILDER_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "polystar_msgs/msg/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace polystar_msgs
{

namespace msg
{

namespace builder
{

class Init_Target_dist
{
public:
  explicit Init_Target_dist(::polystar_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  ::polystar_msgs::msg::Target dist(::polystar_msgs::msg::Target::_dist_type arg)
  {
    msg_.dist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::polystar_msgs::msg::Target msg_;
};

class Init_Target_phi
{
public:
  explicit Init_Target_phi(::polystar_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_dist phi(::polystar_msgs::msg::Target::_phi_type arg)
  {
    msg_.phi = std::move(arg);
    return Init_Target_dist(msg_);
  }

private:
  ::polystar_msgs::msg::Target msg_;
};

class Init_Target_theta
{
public:
  explicit Init_Target_theta(::polystar_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_phi theta(::polystar_msgs::msg::Target::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_Target_phi(msg_);
  }

private:
  ::polystar_msgs::msg::Target msg_;
};

class Init_Target_distance_center
{
public:
  explicit Init_Target_distance_center(::polystar_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_theta distance_center(::polystar_msgs::msg::Target::_distance_center_type arg)
  {
    msg_.distance_center = std::move(arg);
    return Init_Target_theta(msg_);
  }

private:
  ::polystar_msgs::msg::Target msg_;
};

class Init_Target_located
{
public:
  explicit Init_Target_located(::polystar_msgs::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_distance_center located(::polystar_msgs::msg::Target::_located_type arg)
  {
    msg_.located = std::move(arg);
    return Init_Target_distance_center(msg_);
  }

private:
  ::polystar_msgs::msg::Target msg_;
};

class Init_Target_stamp
{
public:
  Init_Target_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_located stamp(::polystar_msgs::msg::Target::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_Target_located(msg_);
  }

private:
  ::polystar_msgs::msg::Target msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::polystar_msgs::msg::Target>()
{
  return polystar_msgs::msg::builder::Init_Target_stamp();
}

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__TARGET__BUILDER_HPP_
