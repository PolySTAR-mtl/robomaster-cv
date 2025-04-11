// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from polystar_msgs:msg/HP.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__HP__STRUCT_H_
#define POLYSTAR_MSGS__MSG__DETAIL__HP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/HP in the package polystar_msgs.
/**
  * Rune.msg
  * From CS to CV : Health points of all robots
 */
typedef struct polystar_msgs__msg__HP
{
  /// Message
  uint16_t foe_hero;
  uint16_t foe_standard1;
  uint16_t foe_standard2;
  uint16_t foe_sentry;
  uint16_t ally_hero;
  uint16_t ally_standard1;
  uint16_t ally_standard2;
  uint16_t ally_sentry;
} polystar_msgs__msg__HP;

// Struct for a sequence of polystar_msgs__msg__HP.
typedef struct polystar_msgs__msg__HP__Sequence
{
  polystar_msgs__msg__HP * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} polystar_msgs__msg__HP__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POLYSTAR_MSGS__MSG__DETAIL__HP__STRUCT_H_
