// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from polystar_msgs:msg/GameStatus.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__TRAITS_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "polystar_msgs/msg/detail/game_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace polystar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GameStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: robot_type
  {
    out << "robot_type: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_type, out);
    out << ", ";
  }

  // member: red_std_hp
  {
    out << "red_std_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.red_std_hp, out);
    out << ", ";
  }

  // member: red_hro_hp
  {
    out << "red_hro_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.red_hro_hp, out);
    out << ", ";
  }

  // member: red_sty_hp
  {
    out << "red_sty_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.red_sty_hp, out);
    out << ", ";
  }

  // member: blu_std_hp
  {
    out << "blu_std_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.blu_std_hp, out);
    out << ", ";
  }

  // member: blu_hro_hp
  {
    out << "blu_hro_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.blu_hro_hp, out);
    out << ", ";
  }

  // member: blu_sty_hp
  {
    out << "blu_sty_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.blu_sty_hp, out);
    out << ", ";
  }

  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GameStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: robot_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_type: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_type, out);
    out << "\n";
  }

  // member: red_std_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "red_std_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.red_std_hp, out);
    out << "\n";
  }

  // member: red_hro_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "red_hro_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.red_hro_hp, out);
    out << "\n";
  }

  // member: red_sty_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "red_sty_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.red_sty_hp, out);
    out << "\n";
  }

  // member: blu_std_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "blu_std_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.blu_std_hp, out);
    out << "\n";
  }

  // member: blu_hro_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "blu_hro_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.blu_hro_hp, out);
    out << "\n";
  }

  // member: blu_sty_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "blu_sty_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.blu_sty_hp, out);
    out << "\n";
  }

  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GameStatus & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace polystar_msgs

namespace rosidl_generator_traits
{

[[deprecated("use polystar_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const polystar_msgs::msg::GameStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  polystar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use polystar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const polystar_msgs::msg::GameStatus & msg)
{
  return polystar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<polystar_msgs::msg::GameStatus>()
{
  return "polystar_msgs::msg::GameStatus";
}

template<>
inline const char * name<polystar_msgs::msg::GameStatus>()
{
  return "polystar_msgs/msg/GameStatus";
}

template<>
struct has_fixed_size<polystar_msgs::msg::GameStatus>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<polystar_msgs::msg::GameStatus>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<polystar_msgs::msg::GameStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__TRAITS_HPP_
