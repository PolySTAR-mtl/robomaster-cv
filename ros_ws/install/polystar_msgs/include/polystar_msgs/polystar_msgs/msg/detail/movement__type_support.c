// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from polystar_msgs:msg/Movement.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "polystar_msgs/msg/detail/movement__rosidl_typesupport_introspection_c.h"
#include "polystar_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "polystar_msgs/msg/detail/movement__functions.h"
#include "polystar_msgs/msg/detail/movement__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  polystar_msgs__msg__Movement__init(message_memory);
}

void polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_fini_function(void * message_memory)
{
  polystar_msgs__msg__Movement__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_message_member_array[4] = {
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(polystar_msgs__msg__Movement, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "v_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(polystar_msgs__msg__Movement, v_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "v_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(polystar_msgs__msg__Movement, v_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "omega",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(polystar_msgs__msg__Movement, omega),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_message_members = {
  "polystar_msgs__msg",  // message namespace
  "Movement",  // message name
  4,  // number of fields
  sizeof(polystar_msgs__msg__Movement),
  polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_message_member_array,  // message members
  polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_init_function,  // function to initialize message memory (memory has to be allocated)
  polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_message_type_support_handle = {
  0,
  &polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_message_members,
  get_message_typesupport_handle_function,
  &polystar_msgs__msg__Movement__get_type_hash,
  &polystar_msgs__msg__Movement__get_type_description,
  &polystar_msgs__msg__Movement__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_polystar_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, polystar_msgs, msg, Movement)() {
  polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_message_type_support_handle.typesupport_identifier) {
    polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &polystar_msgs__msg__Movement__rosidl_typesupport_introspection_c__Movement_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
