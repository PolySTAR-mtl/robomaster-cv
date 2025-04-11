// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from polystar_msgs:msg/GameStage.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__GAME_STAGE__BUILDER_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__GAME_STAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "polystar_msgs/msg/detail/game_stage__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace polystar_msgs
{

namespace msg
{

namespace builder
{

class Init_GameStage_gamestage
{
public:
  explicit Init_GameStage_gamestage(::polystar_msgs::msg::GameStage & msg)
  : msg_(msg)
  {}
  ::polystar_msgs::msg::GameStage gamestage(::polystar_msgs::msg::GameStage::_gamestage_type arg)
  {
    msg_.gamestage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::polystar_msgs::msg::GameStage msg_;
};

class Init_GameStage_stamp
{
public:
  Init_GameStage_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GameStage_gamestage stamp(::polystar_msgs::msg::GameStage::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_GameStage_gamestage(msg_);
  }

private:
  ::polystar_msgs::msg::GameStage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::polystar_msgs::msg::GameStage>()
{
  return polystar_msgs::msg::builder::Init_GameStage_stamp();
}

}  // namespace polystar_msgs

#endif  // POLYSTAR_MSGS__MSG__DETAIL__GAME_STAGE__BUILDER_HPP_
