// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from polystar_msgs:msg/TurretFeedback.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__TURRET_FEEDBACK__STRUCT_H_
#define POLYSTAR_MSGS__MSG__DETAIL__TURRET_FEEDBACK__STRUCT_H_

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

/// Struct defined in msg/TurretFeedback in the package polystar_msgs.
/**
  * TurretFeedback.msg
  * Turret positional feedback, in radians
 */
typedef struct polystar_msgs__msg__TurretFeedback
{
  builtin_interfaces__msg__Time stamp;
  float pitch;
  float yaw;
} polystar_msgs__msg__TurretFeedback;

// Struct for a sequence of polystar_msgs__msg__TurretFeedback.
typedef struct polystar_msgs__msg__TurretFeedback__Sequence
{
  polystar_msgs__msg__TurretFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} polystar_msgs__msg__TurretFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POLYSTAR_MSGS__MSG__DETAIL__TURRET_FEEDBACK__STRUCT_H_
