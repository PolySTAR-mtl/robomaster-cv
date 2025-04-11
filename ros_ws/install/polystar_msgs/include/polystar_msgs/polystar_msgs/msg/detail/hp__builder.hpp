// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from polystar_msgs:msg/HP.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__HP__BUILDER_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__HP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "polystar_msgs/msg/detail/hp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace polystar_msgs
{

namespace msg
{

namespace builder
{

class Init_HP_ally_sentry
{
public:
  explicit Init_HP_ally_sentry(::polystar_msgs::msg::HP & msg)
  : msg_(msg)
  {}
  ::polystar_msgs::msg::HP ally_sentry(::polystar_msgs::msg::HP::_ally_sentry_type arg)
  {
    msg_.ally_sentry = std::move(arg);
    return std::move(msg_);
  }

private:
  ::polystar_msgs::msg::HP msg_;
};

class Init_HP_ally_standard2
{
public:
  explicit Init_HP_ally_standard2(::polystar_msgs::msg::HP & msg)
  : msg_(msg)
  {}
  Init_HP_ally_sentry ally_standard2(::polystar_msgs::msg::HP::_ally_standard2_type arg)
  {
    msg_.ally_standard2 = std::move(arg);
    return Init_HP_ally_sentry(msg_);
  }

private:
  ::polystar_msgs::msg::HP msg_;
};

class Init_HP_ally_standard1
{
public:
  explicit Init_HP_ally_standard1(::polystar_msgs::msg::HP & msg)
  : msg_(msg)
  {}
  Init_HP_ally_standard2 ally_standard1(::polystar_msgs::msg::HP::_ally_standard1_type arg)
  {
    msg_.ally_standard1 = std::move(arg);
    return Init_HP_ally_standard2(msg_);
  }

private:
  ::polystar_msgs::msg::HP msg_;
};

class Init_HP_ally_hero
{
public:
  explicit Init_HP_ally_hero(::polystar_msgs::msg::HP & msg)
  : msg_(msg)
  {}
  Init_HP_ally_standard1 ally_hero(::polystar_msgs::msg::HP::_ally_hero_type arg)
  {
    msg_.ally_hero = std::move(arg);
    return Init_HP_ally_standard1(msg_);
  }

private:
  ::polystar_msgs::msg::HP msg_;
};

class Init_HP_foe_sentry
{
public:
  explicit Init_HP_foe_sentry(::polystar_msgs::msg::HP & msg)
  : msg_(msg)
  {}
  Init_HP_ally_hero foe_sentry(::polystar_msgs::msg::HP::_foe_sentry_type arg)
  {
    msg_.foe_sentry = std::move(arg);
    return Init_HP_ally_hero(msg_);
  }

private:
  ::polystar_msgs::msg::HP msg_;
};

class Init_HP_foe_standard2
{
public:
  explicit Init_HP_foe_standard2(::polystar_msgs::msg::HP & msg)
  : msg_(msg)
  {}
  Init_HP_foe_sentry foe_standard2(::polystar_msgs::msg::HP::_foe_standard2_type arg)
  {
    msg_.foe_standard2 = std::move(arg);
    return Init_HP_foe_sentry(msg_);
  }

private:
  ::polystar_msgs::msg::HP msg_;
};

class Init_HP_foe_standard1
{
public:
  explicit Init_HP_foe_standard1(::polystar_msgs::msg::HP & msg)
  : msg_(msg)
  {}
  Init_HP_foe_standard2 foe_standard1(::polystar_msgs::msg::HP::_foe_standard1_type arg)
  {
    msg_.foe_standard1 = std::move(arg);
    return Init_HP_foe_standard2(msg_);
  }

private:
  ::polystar_msgs::msg::HP msg_;
};

class Init_HP_foe_hero
{
public:
  Init_HP_foe_hero()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HP_foe_standard1 foe_hero(::polystar_msgs::msg::HP::_foe_hero_type arg)
  {
    msg_.foe_hero = std::move(arg);
    return Init_HP_foe_standard1(msg_);
  }

private:
  ::polystar_msgs::msg::HP msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::polystar_msgs::msg::HP>()
{
  return polystar_msgs::msg::builder::Init_HP_foe_hero();
}

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__HP__BUILDER_HPP_
