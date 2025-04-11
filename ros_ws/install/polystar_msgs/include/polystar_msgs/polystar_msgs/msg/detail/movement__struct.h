// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from polystar_msgs:msg/Movement.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__MOVEMENT__STRUCT_H_
#define POLYSTAR_MSGS__MSG__DETAIL__MOVEMENT__STRUCT_H_

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

/// Struct defined in msg/Movement in the package polystar_msgs.
/**
  * Movement.msg
  * Describes the desired velocity of the robot. Linear speed is in m/s, angular in rad/s
 */
typedef struct polystar_msgs__msg__Movement
{
  builtin_interfaces__msg__Time stamp;
  float v_x;
  float v_y;
  float omega;
} polystar_msgs__msg__Movement;

// Struct for a sequence of polystar_msgs__msg__Movement.
typedef struct polystar_msgs__msg__Movement__Sequence
{
  polystar_msgs__msg__Movement * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} polystar_msgs__msg__Movement__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POLYSTAR_MSGS__MSG__DETAIL__MOVEMENT__STRUCT_H_
