// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from polystar_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__TARGET__TRAITS_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "polystar_msgs/msg/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace polystar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Target & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: located
  {
    out << "located: ";
    rosidl_generator_traits::value_to_yaml(msg.located, out);
    out << ", ";
  }

  // member: distance_center
  {
    out << "distance_center: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_center, out);
    out << ", ";
  }

  // member: theta
  {
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << ", ";
  }

  // member: phi
  {
    out << "phi: ";
    rosidl_generator_traits::value_to_yaml(msg.phi, out);
    out << ", ";
  }

  // member: dist
  {
    out << "dist: ";
    rosidl_generator_traits::value_to_yaml(msg.dist, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Target & msg,
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

  // member: located
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "located: ";
    rosidl_generator_traits::value_to_yaml(msg.located, out);
    out << "\n";
  }

  // member: distance_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_center: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_center, out);
    out << "\n";
  }

  // member: theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "theta: ";
    rosidl_generator_traits::value_to_yaml(msg.theta, out);
    out << "\n";
  }

  // member: phi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "phi: ";
    rosidl_generator_traits::value_to_yaml(msg.phi, out);
    out << "\n";
  }

  // member: dist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dist: ";
    rosidl_generator_traits::value_to_yaml(msg.dist, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Target & msg, bool use_flow_style = false)
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
  const polystar_msgs::msg::Target & msg,
  std::ostream & out, size_t indentation = 0)
{
  polystar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use polystar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const polystar_msgs::msg::Target & msg)
{
  return polystar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<polystar_msgs::msg::Target>()
{
  return "polystar_msgs::msg::Target";
}

template<>
inline const char * name<polystar_msgs::msg::Target>()
{
  return "polystar_msgs/msg/Target";
}

template<>
struct has_fixed_size<polystar_msgs::msg::Target>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<polystar_msgs::msg::Target>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<polystar_msgs::msg::Target>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // POLYSTAR_MSGS__MSG__DETAIL__TARGET__TRAITS_HPP_
