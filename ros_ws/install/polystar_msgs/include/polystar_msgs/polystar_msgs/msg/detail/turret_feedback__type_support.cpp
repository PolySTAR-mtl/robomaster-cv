// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from polystar_msgs:msg/TurretFeedback.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "polystar_msgs/msg/detail/turret_feedback__functions.h"
#include "polystar_msgs/msg/detail/turret_feedback__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace polystar_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TurretFeedback_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) polystar_msgs::msg::TurretFeedback(_init);
}

void TurretFeedback_fini_function(void * message_memory)
{
  auto typed_message = static_cast<polystar_msgs::msg::TurretFeedback *>(message_memory);
  typed_message->~TurretFeedback();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TurretFeedback_message_member_array[3] = {
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(polystar_msgs::msg::TurretFeedback, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pitch",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(polystar_msgs::msg::TurretFeedback, pitch),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(polystar_msgs::msg::TurretFeedback, yaw),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TurretFeedback_message_members = {
  "polystar_msgs::msg",  // message namespace
  "TurretFeedback",  // message name
  3,  // number of fields
  sizeof(polystar_msgs::msg::TurretFeedback),
  TurretFeedback_message_member_array,  // message members
  TurretFeedback_init_function,  // function to initialize message memory (memory has to be allocated)
  TurretFeedback_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TurretFeedback_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TurretFeedback_message_members,
  get_message_typesupport_handle_function,
  &polystar_msgs__msg__TurretFeedback__get_type_hash,
  &polystar_msgs__msg__TurretFeedback__get_type_description,
  &polystar_msgs__msg__TurretFeedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace polystar_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<polystar_msgs::msg::TurretFeedback>()
{
  return &::polystar_msgs::msg::rosidl_typesupport_introspection_cpp::TurretFeedback_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, polystar_msgs, msg, TurretFeedback)() {
  return &::polystar_msgs::msg::rosidl_typesupport_introspection_cpp::TurretFeedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
