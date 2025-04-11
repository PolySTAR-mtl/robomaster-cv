// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from polystar_msgs:msg/HP.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__HP__TRAITS_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__HP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "polystar_msgs/msg/detail/hp__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace polystar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HP & msg,
  std::ostream & out)
{
  out << "{";
  // member: foe_hero
  {
    out << "foe_hero: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_hero, out);
    out << ", ";
  }

  // member: foe_standard1
  {
    out << "foe_standard1: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_standard1, out);
    out << ", ";
  }

  // member: foe_standard2
  {
    out << "foe_standard2: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_standard2, out);
    out << ", ";
  }

  // member: foe_sentry
  {
    out << "foe_sentry: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_sentry, out);
    out << ", ";
  }

  // member: ally_hero
  {
    out << "ally_hero: ";
    rosidl_generator_traits::value_to_yaml(msg.ally_hero, out);
    out << ", ";
  }

  // member: ally_standard1
  {
    out << "ally_standard1: ";
    rosidl_generator_traits::value_to_yaml(msg.ally_standard1, out);
    out << ", ";
  }

  // member: ally_standard2
  {
    out << "ally_standard2: ";
    rosidl_generator_traits::value_to_yaml(msg.ally_standard2, out);
    out << ", ";
  }

  // member: ally_sentry
  {
    out << "ally_sentry: ";
    rosidl_generator_traits::value_to_yaml(msg.ally_sentry, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HP & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: foe_hero
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "foe_hero: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_hero, out);
    out << "\n";
  }

  // member: foe_standard1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "foe_standard1: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_standard1, out);
    out << "\n";
  }

  // member: foe_standard2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "foe_standard2: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_standard2, out);
    out << "\n";
  }

  // member: foe_sentry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "foe_sentry: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_sentry, out);
    out << "\n";
  }

  // member: ally_hero
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ally_hero: ";
    rosidl_generator_traits::value_to_yaml(msg.ally_hero, out);
    out << "\n";
  }

  // member: ally_standard1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ally_standard1: ";
    rosidl_generator_traits::value_to_yaml(msg.ally_standard1, out);
    out << "\n";
  }

  // member: ally_standard2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ally_standard2: ";
    rosidl_generator_traits::value_to_yaml(msg.ally_standard2, out);
    out << "\n";
  }

  // member: ally_sentry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ally_sentry: ";
    rosidl_generator_traits::value_to_yaml(msg.ally_sentry, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HP & msg, bool use_flow_style = false)
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
  const polystar_msgs::msg::HP & msg,
  std::ostream & out, size_t indentation = 0)
{
  polystar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use polystar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const polystar_msgs::msg::HP & msg)
{
  return polystar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<polystar_msgs::msg::HP>()
{
  return "polystar_msgs::msg::HP";
}

template<>
inline const char * name<polystar_msgs::msg::HP>()
{
  return "polystar_msgs/msg/HP";
}

template<>
struct has_fixed_size<polystar_msgs::msg::HP>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<polystar_msgs::msg::HP>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<polystar_msgs::msg::HP>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // POLYSTAR_MSGS__MSG__DETAIL__HP__TRAITS_HPP_
