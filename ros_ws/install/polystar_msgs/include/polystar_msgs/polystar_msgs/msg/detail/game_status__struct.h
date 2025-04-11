// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from polystar_msgs:msg/GameStatus.idl
// generated code does not contain a copyright notice

#ifndef POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__STRUCT_H_
#define POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__STRUCT_H_

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

/// Struct defined in msg/GameStatus in the package polystar_msgs.
/**
  * GameStatus.msg
  * Game status : Robot type, hp, mode
 */
typedef struct polystar_msgs__msg__GameStatus
{
  builtin_interfaces__msg__Time stamp;
  uint8_t robot_type;
  uint16_t red_std_hp;
  uint16_t red_hro_hp;
  uint16_t red_sty_hp;
  uint16_t blu_std_hp;
  uint16_t blu_hro_hp;
  uint16_t blu_sty_hp;
  uint8_t mode;
} polystar_msgs__msg__GameStatus;

// Struct for a sequence of polystar_msgs__msg__GameStatus.
typedef struct polystar_msgs__msg__GameStatus__Sequence
{
  polystar_msgs__msg__GameStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} polystar_msgs__msg__GameStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POLYSTAR_MSGS__MSG__DETAIL__GAME_STATUS__STRUCT_H_
