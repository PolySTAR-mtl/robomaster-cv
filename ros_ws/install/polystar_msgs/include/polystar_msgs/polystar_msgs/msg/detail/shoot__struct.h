// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from polystar_msgs:msg/Shoot.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__SHOOT__STRUCT_H_
#define POLYSTAR_MSGS__MSG__DETAIL__SHOOT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/Shoot in the package polystar_msgs.
/**
  * Shoot.msg
  * Shoot order
 */
typedef struct polystar_msgs__msg__Shoot
{
  builtin_interfaces__msg__Time stamp;
  bool shoot;
} polystar_msgs__msg__Shoot;

// Struct for a sequence of polystar_msgs__msg__Shoot.
typedef struct polystar_msgs__msg__Shoot__Sequence
{
  polystar_msgs__msg__Shoot * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} polystar_msgs__msg__Shoot__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POLYSTAR_MSGS__MSG__DETAIL__SHOOT__STRUCT_H_
