// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from polystar_msgs:msg/SwitchOrder.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__SWITCH_ORDER__BUILDER_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__SWITCH_ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "polystar_msgs/msg/detail/switch_order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace polystar_msgs
{

namespace msg
{

namespace builder
{

class Init_SwitchOrder_order
{
public:
  explicit Init_SwitchOrder_order(::polystar_msgs::msg::SwitchOrder & msg)
  : msg_(msg)
  {}
  ::polystar_msgs::msg::SwitchOrder order(::polystar_msgs::msg::SwitchOrder::_order_type arg)
  {
    msg_.order = std::move(arg);
    return std::move(msg_);
  }

private:
  ::polystar_msgs::msg::SwitchOrder msg_;
};

class Init_SwitchOrder_stamp
{
public:
  Init_SwitchOrder_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SwitchOrder_order stamp(::polystar_msgs::msg::SwitchOrder::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_SwitchOrder_order(msg_);
  }

private:
  ::polystar_msgs::msg::SwitchOrder msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::polystar_msgs::msg::SwitchOrder>()
{
  return polystar_msgs::msg::builder::Init_SwitchOrder_stamp();
}

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__SWITCH_ORDER__BUILDER_HPP_
