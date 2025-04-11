// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from polystar_msgs:msg/GameStatus.idl
// generated code does not contain a copyright notice

#include "polystar_msgs/msg/detail/game_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_polystar_msgs
const rosidl_type_hash_t *
polystar_msgs__msg__GameStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1c, 0xcc, 0x7d, 0xc5, 0xcf, 0xca, 0xce, 0x63,
      0x6d, 0x78, 0x6e, 0x01, 0x5b, 0x76, 0x46, 0xc6,
      0x80, 0x38, 0xe1, 0x16, 0x06, 0x5c, 0x08, 0x0f,
      0x00, 0xc6, 0x96, 0xf8, 0xcf, 0x21, 0x42, 0xd6,
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

static char polystar_msgs__msg__GameStatus__TYPE_NAME[] = "polystar_msgs/msg/GameStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char polystar_msgs__msg__GameStatus__FIELD_NAME__stamp[] = "stamp";
static char polystar_msgs__msg__GameStatus__FIELD_NAME__robot_type[] = "robot_type";
static char polystar_msgs__msg__GameStatus__FIELD_NAME__red_std_hp[] = "red_std_hp";
static char polystar_msgs__msg__GameStatus__FIELD_NAME__red_hro_hp[] = "red_hro_hp";
static char polystar_msgs__msg__GameStatus__FIELD_NAME__red_sty_hp[] = "red_sty_hp";
static char polystar_msgs__msg__GameStatus__FIELD_NAME__blu_std_hp[] = "blu_std_hp";
static char polystar_msgs__msg__GameStatus__FIELD_NAME__blu_hro_hp[] = "blu_hro_hp";
static char polystar_msgs__msg__GameStatus__FIELD_NAME__blu_sty_hp[] = "blu_sty_hp";
static char polystar_msgs__msg__GameStatus__FIELD_NAME__mode[] = "mode";

static rosidl_runtime_c__type_description__Field polystar_msgs__msg__GameStatus__FIELDS[] = {
  {
    {polystar_msgs__msg__GameStatus__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__GameStatus__FIELD_NAME__robot_type, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__GameStatus__FIELD_NAME__red_std_hp, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__GameStatus__FIELD_NAME__red_hro_hp, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__GameStatus__FIELD_NAME__red_sty_hp, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__GameStatus__FIELD_NAME__blu_std_hp, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__GameStatus__FIELD_NAME__blu_hro_hp, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__GameStatus__FIELD_NAME__blu_sty_hp, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__GameStatus__FIELD_NAME__mode, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription polystar_msgs__msg__GameStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
polystar_msgs__msg__GameStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {polystar_msgs__msg__GameStatus__TYPE_NAME, 28, 28},
      {polystar_msgs__msg__GameStatus__FIELDS, 9, 9},
    },
    {polystar_msgs__msg__GameStatus__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GameStatus.msg\n"
  "## Game status : Robot type, hp, mode\n"
  "\n"
  "builtin_interfaces/Time stamp\n"
  "\n"
  "uint8 robot_type\n"
  "\n"
  "uint16 red_std_hp\n"
  "uint16 red_hro_hp\n"
  "uint16 red_sty_hp\n"
  "uint16 blu_std_hp\n"
  "uint16 blu_hro_hp\n"
  "uint16 blu_sty_hp\n"
  "\n"
  "uint8 mode";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
polystar_msgs__msg__GameStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {polystar_msgs__msg__GameStatus__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 225, 225},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
polystar_msgs__msg__GameStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *polystar_msgs__msg__GameStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
