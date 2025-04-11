// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from polystar_msgs:msg/PositionFeedback.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__STRUCT_H_
#define POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__STRUCT_H_

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

/// Struct defined in msg/PositionFeedback in the package polystar_msgs.
/**
  * PositionFeedback.msg
  * Positional feedback from sensors. In m/s², rad/s² and seconds
 */
typedef struct polystar_msgs__msg__PositionFeedback
{
  builtin_interfaces__msg__Time stamp;
  float imu_ax;
  float imu_ay;
  float imu_az;
  float imu_gx;
  float imu_gy;
  float imu_gz;
  float imu_rx;
  float imu_ry;
  float imu_rz;
  int64_t enc_1;
  int64_t enc_2;
  int64_t enc_3;
  int64_t enc_4;
  float v_enc_1;
  float v_enc_2;
  float v_enc_3;
  float v_enc_4;
} polystar_msgs__msg__PositionFeedback;

// Struct for a sequence of polystar_msgs__msg__PositionFeedback.
typedef struct polystar_msgs__msg__PositionFeedback__Sequence
{
  polystar_msgs__msg__PositionFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} polystar_msgs__msg__PositionFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POLYSTAR_MSGS__MSG__DETAIL__POSITION_FEEDBACK__STRUCT_H_
