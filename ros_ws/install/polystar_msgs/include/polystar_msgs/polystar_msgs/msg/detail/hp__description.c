// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from polystar_msgs:msg/HP.idl
// generated code does not contain a copyright notice

#include "polystar_msgs/msg/detail/hp__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_polystar_msgs
const rosidl_type_hash_t *
polystar_msgs__msg__HP__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4b, 0x28, 0xdf, 0x8d, 0x41, 0x19, 0x59, 0xc9,
      0x5c, 0xae, 0x13, 0x4f, 0x1b, 0x75, 0x1f, 0x4c,
      0xf1, 0xec, 0x00, 0x03, 0x49, 0x8a, 0x8a, 0xea,
      0x7d, 0xf6, 0x27, 0x1d, 0xcc, 0x46, 0xc3, 0x88,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char polystar_msgs__msg__HP__TYPE_NAME[] = "polystar_msgs/msg/HP";

// Define type names, field names, and default values
static char polystar_msgs__msg__HP__FIELD_NAME__foe_hero[] = "foe_hero";
static char polystar_msgs__msg__HP__FIELD_NAME__foe_standard1[] = "foe_standard1";
static char polystar_msgs__msg__HP__FIELD_NAME__foe_standard2[] = "foe_standard2";
static char polystar_msgs__msg__HP__FIELD_NAME__foe_sentry[] = "foe_sentry";
static char polystar_msgs__msg__HP__FIELD_NAME__ally_hero[] = "ally_hero";
static char polystar_msgs__msg__HP__FIELD_NAME__ally_standard1[] = "ally_standard1";
static char polystar_msgs__msg__HP__FIELD_NAME__ally_standard2[] = "ally_standard2";
static char polystar_msgs__msg__HP__FIELD_NAME__ally_sentry[] = "ally_sentry";

static rosidl_runtime_c__type_description__Field polystar_msgs__msg__HP__FIELDS[] = {
  {
    {polystar_msgs__msg__HP__FIELD_NAME__foe_hero, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__HP__FIELD_NAME__foe_standard1, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__HP__FIELD_NAME__foe_standard2, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__HP__FIELD_NAME__foe_sentry, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__HP__FIELD_NAME__ally_hero, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__HP__FIELD_NAME__ally_standard1, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__HP__FIELD_NAME__ally_standard2, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__HP__FIELD_NAME__ally_sentry, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
polystar_msgs__msg__HP__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {polystar_msgs__msg__HP__TYPE_NAME, 20, 20},
      {polystar_msgs__msg__HP__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Rune.msg\n"
  "## From CS to CV : Health points of all robots\n"
  "\n"
  "# Message\n"
  "\n"
  "uint16 foe_hero\n"
  "uint16 foe_standard1\n"
  "uint16 foe_standard2\n"
  "uint16 foe_sentry\n"
  "\n"
  "uint16 ally_hero\n"
  "uint16 ally_standard1\n"
  "uint16 ally_standard2\n"
  "uint16 ally_sentry";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
polystar_msgs__msg__HP__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {polystar_msgs__msg__HP__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 227, 227},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
polystar_msgs__msg__HP__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *polystar_msgs__msg__HP__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
