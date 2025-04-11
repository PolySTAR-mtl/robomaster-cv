// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from polystar_msgs:msg/SwitchOrder.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__SWITCH_ORDER__STRUCT_H_
#define POLYSTAR_MSGS__MSG__DETAIL__SWITCH_ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'ORDER_NOTHING'.
/**
  * Constants
 */
enum
{
  polystar_msgs__msg__SwitchOrder__ORDER_NOTHING = 0
};

/// Constant 'ORDER_NEXT'.
enum
{
  polystar_msgs__msg__SwitchOrder__ORDER_NEXT = 1
};

/// Constant 'ORDER_RIGHT'.
enum
{
  polystar_msgs__msg__SwitchOrder__ORDER_RIGHT = 2
};

/// Constant 'ORDER_LEFT'.
enum
{
  polystar_msgs__msg__SwitchOrder__ORDER_LEFT = 3
};

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/SwitchOrder in the package polystar_msgs.
/**
  * SwitchOrder.msg
  * From CS to CV : switch to a different target
 */
typedef struct polystar_msgs__msg__SwitchOrder
{
  /// Message
  builtin_interfaces__msg__Time stamp;
  uint8_t order;
} polystar_msgs__msg__SwitchOrder;

// Struct for a sequence of polystar_msgs__msg__SwitchOrder.
typedef struct polystar_msgs__msg__SwitchOrder__Sequence
{
  polystar_msgs__msg__SwitchOrder * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} polystar_msgs__msg__SwitchOrder__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POLYSTAR_MSGS__MSG__DETAIL__SWITCH_ORDER__STRUCT_H_
