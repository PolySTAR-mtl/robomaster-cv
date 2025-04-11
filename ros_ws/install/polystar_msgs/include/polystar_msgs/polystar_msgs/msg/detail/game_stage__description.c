// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from polystar_msgs:msg/GameStage.idl
// generated code does not contain a copyright notice

#include "polystar_msgs/msg/detail/game_stage__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_polystar_msgs
const rosidl_type_hash_t *
polystar_msgs__msg__GameStage__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x63, 0x40, 0x4b, 0xfa, 0xab, 0x7a, 0x71, 0xa7,
      0x03, 0xc6, 0xf9, 0x94, 0xce, 0xea, 0x89, 0xaa,
      0x15, 0xcf, 0x75, 0x1e, 0x33, 0xd2, 0xec, 0x73,
      0xe6, 0x03, 0x7c, 0xf2, 0xfa, 0x53, 0x4f, 0x8e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char polystar_msgs__msg__GameStage__TYPE_NAME[] = "polystar_msgs/msg/GameStage";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char polystar_msgs__msg__GameStage__FIELD_NAME__stamp[] = "stamp";
static char polystar_msgs__msg__GameStage__FIELD_NAME__gamestage[] = "gamestage";

static rosidl_runtime_c__type_description__Field polystar_msgs__msg__GameStage__FIELDS[] = {
  {
    {polystar_msgs__msg__GameStage__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__GameStage__FIELD_NAME__gamestage, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription polystar_msgs__msg__GameStage__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
polystar_msgs__msg__GameStage__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {polystar_msgs__msg__GameStage__TYPE_NAME, 27, 27},
      {polystar_msgs__msg__GameStage__FIELDS, 2, 2},
    },
    {polystar_msgs__msg__GameStage__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GameStage.msg\n"
  "## Current game stage (begin & end)\n"
  "\n"
  "builtin_interfaces/Time stamp\n"
  "uint16 gamestage";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
polystar_msgs__msg__GameStage__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {polystar_msgs__msg__GameStage__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 100, 100},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
polystar_msgs__msg__GameStage__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *polystar_msgs__msg__GameStage__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
