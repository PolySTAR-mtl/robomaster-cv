// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from polystar_msgs:msg/GameStage.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__GAME_STAGE__STRUCT_H_
#define POLYSTAR_MSGS__MSG__DETAIL__GAME_STAGE__STRUCT_H_

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

/// Struct defined in msg/GameStage in the package polystar_msgs.
/**
  * GameStage.msg
  * Current game stage (begin & end)
 */
typedef struct polystar_msgs__msg__GameStage
{
  builtin_interfaces__msg__Time stamp;
  uint16_t gamestage;
} polystar_msgs__msg__GameStage;

// Struct for a sequence of polystar_msgs__msg__GameStage.
typedef struct polystar_msgs__msg__GameStage__Sequence
{
  polystar_msgs__msg__GameStage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} polystar_msgs__msg__GameStage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POLYSTAR_MSGS__MSG__DETAIL__GAME_STAGE__STRUCT_H_
