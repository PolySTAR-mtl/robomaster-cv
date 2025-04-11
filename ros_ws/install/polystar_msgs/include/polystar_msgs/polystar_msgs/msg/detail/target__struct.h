// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from polystar_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__TARGET__STRUCT_H_
#define POLYSTAR_MSGS__MSG__DETAIL__TARGET__STRUCT_H_

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

/// Struct defined in msg/Target in the package polystar_msgs.
/**
  * Target.msg
  * From CV to CS : Coordinates of the current target
 */
typedef struct polystar_msgs__msg__Target
{
  /// Message
  builtin_interfaces__msg__Time stamp;
  bool located;
  float distance_center;
  /// milli-ad
  int16_t theta;
  /// millirad
  int16_t phi;
  /// mm
  uint16_t dist;
} polystar_msgs__msg__Target;

// Struct for a sequence of polystar_msgs__msg__Target.
typedef struct polystar_msgs__msg__Target__Sequence
{
  polystar_msgs__msg__Target * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} polystar_msgs__msg__Target__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POLYSTAR_MSGS__MSG__DETAIL__TARGET__STRUCT_H_
