// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from polystar_msgs:msg/PositionFeedback.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__TRAITS_HPP_
#define POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "polystar_msgs/msg/detail/position_feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace polystar_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PositionFeedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: imu_ax
  {
    out << "imu_ax: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_ax, out);
    out << ", ";
  }

  // member: imu_ay
  {
    out << "imu_ay: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_ay, out);
    out << ", ";
  }

  // member: imu_az
  {
    out << "imu_az: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_az, out);
    out << ", ";
  }

  // member: imu_gx
  {
    out << "imu_gx: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_gx, out);
    out << ", ";
  }

  // member: imu_gy
  {
    out << "imu_gy: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_gy, out);
    out << ", ";
  }

  // member: imu_gz
  {
    out << "imu_gz: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_gz, out);
    out << ", ";
  }

  // member: imu_rx
  {
    out << "imu_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_rx, out);
    out << ", ";
  }

  // member: imu_ry
  {
    out << "imu_ry: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_ry, out);
    out << ", ";
  }

  // member: imu_rz
  {
    out << "imu_rz: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_rz, out);
    out << ", ";
  }

  // member: enc_1
  {
    out << "enc_1: ";
    rosidl_generator_traits::value_to_yaml(msg.enc_1, out);
    out << ", ";
  }

  // member: enc_2
  {
    out << "enc_2: ";
    rosidl_generator_traits::value_to_yaml(msg.enc_2, out);
    out << ", ";
  }

  // member: enc_3
  {
    out << "enc_3: ";
    rosidl_generator_traits::value_to_yaml(msg.enc_3, out);
    out << ", ";
  }

  // member: enc_4
  {
    out << "enc_4: ";
    rosidl_generator_traits::value_to_yaml(msg.enc_4, out);
    out << ", ";
  }

  // member: v_enc_1
  {
    out << "v_enc_1: ";
    rosidl_generator_traits::value_to_yaml(msg.v_enc_1, out);
    out << ", ";
  }

  // member: v_enc_2
  {
    out << "v_enc_2: ";
    rosidl_generator_traits::value_to_yaml(msg.v_enc_2, out);
    out << ", ";
  }

  // member: v_enc_3
  {
    out << "v_enc_3: ";
    rosidl_generator_traits::value_to_yaml(msg.v_enc_3, out);
    out << ", ";
  }

  // member: v_enc_4
  {
    out << "v_enc_4: ";
    rosidl_generator_traits::value_to_yaml(msg.v_enc_4, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PositionFeedback & msg,
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

  // member: imu_ax
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_ax: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_ax, out);
    out << "\n";
  }

  // member: imu_ay
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_ay: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_ay, out);
    out << "\n";
  }

  // member: imu_az
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_az: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_az, out);
    out << "\n";
  }

  // member: imu_gx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_gx: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_gx, out);
    out << "\n";
  }

  // member: imu_gy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_gy: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_gy, out);
    out << "\n";
  }

  // member: imu_gz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_gz: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_gz, out);
    out << "\n";
  }

  // member: imu_rx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_rx, out);
    out << "\n";
  }

  // member: imu_ry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_ry: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_ry, out);
    out << "\n";
  }

  // member: imu_rz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_rz: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_rz, out);
    out << "\n";
  }

  // member: enc_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enc_1: ";
    rosidl_generator_traits::value_to_yaml(msg.enc_1, out);
    out << "\n";
  }

  // member: enc_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enc_2: ";
    rosidl_generator_traits::value_to_yaml(msg.enc_2, out);
    out << "\n";
  }

  // member: enc_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enc_3: ";
    rosidl_generator_traits::value_to_yaml(msg.enc_3, out);
    out << "\n";
  }

  // member: enc_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enc_4: ";
    rosidl_generator_traits::value_to_yaml(msg.enc_4, out);
    out << "\n";
  }

  // member: v_enc_1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_enc_1: ";
    rosidl_generator_traits::value_to_yaml(msg.v_enc_1, out);
    out << "\n";
  }

  // member: v_enc_2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_enc_2: ";
    rosidl_generator_traits::value_to_yaml(msg.v_enc_2, out);
    out << "\n";
  }

  // member: v_enc_3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_enc_3: ";
    rosidl_generator_traits::value_to_yaml(msg.v_enc_3, out);
    out << "\n";
  }

  // member: v_enc_4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_enc_4: ";
    rosidl_generator_traits::value_to_yaml(msg.v_enc_4, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PositionFeedback & msg, bool use_flow_style = false)
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
  const polystar_msgs::msg::PositionFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  polystar_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use polystar_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const polystar_msgs::msg::PositionFeedback & msg)
{
  return polystar_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<polystar_msgs::msg::PositionFeedback>()
{
  return "polystar_msgs::msg::PositionFeedback";
}

template<>
inline const char * name<polystar_msgs::msg::PositionFeedback>()
{
  return "polystar_msgs/msg/PositionFeedback";
}

template<>
struct has_fixed_size<polystar_msgs::msg::PositionFeedback>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<polystar_msgs::msg::PositionFeedback>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<polystar_msgs::msg::PositionFeedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__TRAITS_HPP_
