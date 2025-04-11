// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from polystar_msgs:msg/GameStatus.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__BUILDER_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "polystar_msgs/msg/detail/game_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace polystar_msgs
{

namespace msg
{

namespace builder
{

class Init_GameStatus_mode
{
public:
  explicit Init_GameStatus_mode(::polystar_msgs::msg::GameStatus & msg)
  : msg_(msg)
  {}
  ::polystar_msgs::msg::GameStatus mode(::polystar_msgs::msg::GameStatus::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::polystar_msgs::msg::GameStatus msg_;
};

class Init_GameStatus_blu_sty_hp
{
public:
  explicit Init_GameStatus_blu_sty_hp(::polystar_msgs::msg::GameStatus & msg)
  : msg_(msg)
  {}
  Init_GameStatus_mode blu_sty_hp(::polystar_msgs::msg::GameStatus::_blu_sty_hp_type arg)
  {
    msg_.blu_sty_hp = std::move(arg);
    return Init_GameStatus_mode(msg_);
  }

private:
  ::polystar_msgs::msg::GameStatus msg_;
};

class Init_GameStatus_blu_hro_hp
{
public:
  explicit Init_GameStatus_blu_hro_hp(::polystar_msgs::msg::GameStatus & msg)
  : msg_(msg)
  {}
  Init_GameStatus_blu_sty_hp blu_hro_hp(::polystar_msgs::msg::GameStatus::_blu_hro_hp_type arg)
  {
    msg_.blu_hro_hp = std::move(arg);
    return Init_GameStatus_blu_sty_hp(msg_);
  }

private:
  ::polystar_msgs::msg::GameStatus msg_;
};

class Init_GameStatus_blu_std_hp
{
public:
  explicit Init_GameStatus_blu_std_hp(::polystar_msgs::msg::GameStatus & msg)
  : msg_(msg)
  {}
  Init_GameStatus_blu_hro_hp blu_std_hp(::polystar_msgs::msg::GameStatus::_blu_std_hp_type arg)
  {
    msg_.blu_std_hp = std::move(arg);
    return Init_GameStatus_blu_hro_hp(msg_);
  }

private:
  ::polystar_msgs::msg::GameStatus msg_;
};

class Init_GameStatus_red_sty_hp
{
public:
  explicit Init_GameStatus_red_sty_hp(::polystar_msgs::msg::GameStatus & msg)
  : msg_(msg)
  {}
  Init_GameStatus_blu_std_hp red_sty_hp(::polystar_msgs::msg::GameStatus::_red_sty_hp_type arg)
  {
    msg_.red_sty_hp = std::move(arg);
    return Init_GameStatus_blu_std_hp(msg_);
  }

private:
  ::polystar_msgs::msg::GameStatus msg_;
};

class Init_GameStatus_red_hro_hp
{
public:
  explicit Init_GameStatus_red_hro_hp(::polystar_msgs::msg::GameStatus & msg)
  : msg_(msg)
  {}
  Init_GameStatus_red_sty_hp red_hro_hp(::polystar_msgs::msg::GameStatus::_red_hro_hp_type arg)
  {
    msg_.red_hro_hp = std::move(arg);
    return Init_GameStatus_red_sty_hp(msg_);
  }

private:
  ::polystar_msgs::msg::GameStatus msg_;
};

class Init_GameStatus_red_std_hp
{
public:
  explicit Init_GameStatus_red_std_hp(::polystar_msgs::msg::GameStatus & msg)
  : msg_(msg)
  {}
  Init_GameStatus_red_hro_hp red_std_hp(::polystar_msgs::msg::GameStatus::_red_std_hp_type arg)
  {
    msg_.red_std_hp = std::move(arg);
    return Init_GameStatus_red_hro_hp(msg_);
  }

private:
  ::polystar_msgs::msg::GameStatus msg_;
};

class Init_GameStatus_robot_type
{
public:
  explicit Init_GameStatus_robot_type(::polystar_msgs::msg::GameStatus & msg)
  : msg_(msg)
  {}
  Init_GameStatus_red_std_hp robot_type(::polystar_msgs::msg::GameStatus::_robot_type_type arg)
  {
    msg_.robot_type = std::move(arg);
    return Init_GameStatus_red_std_hp(msg_);
  }

private:
  ::polystar_msgs::msg::GameStatus msg_;
};

class Init_GameStatus_stamp
{
public:
  Init_GameStatus_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GameStatus_robot_type stamp(::polystar_msgs::msg::GameStatus::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_GameStatus_robot_type(msg_);
  }

private:
  ::polystar_msgs::msg::GameStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::polystar_msgs::msg::GameStatus>()
{
  return polystar_msgs::msg::builder::Init_GameStatus_stamp();
}

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__BUILDER_HPP_
