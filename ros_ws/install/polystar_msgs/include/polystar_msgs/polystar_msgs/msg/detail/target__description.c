// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from polystar_msgs:msg/Target.idl
// generated code does not contain a copyright notice

#include "polystar_msgs/msg/detail/target__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_polystar_msgs
const rosidl_type_hash_t *
polystar_msgs__msg__Target__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd1, 0x02, 0xae, 0xd1, 0x07, 0x33, 0x9d, 0x90,
      0x8c, 0x12, 0x86, 0x1e, 0x93, 0x1b, 0x29, 0x26,
      0x7d, 0x21, 0xd1, 0xae, 0xbe, 0x9d, 0x0f, 0xf4,
      0x31, 0xd4, 0x7d, 0x08, 0x72, 0x52, 0xd2, 0x89,
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

static char polystar_msgs__msg__Target__TYPE_NAME[] = "polystar_msgs/msg/Target";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char polystar_msgs__msg__Target__FIELD_NAME__stamp[] = "stamp";
static char polystar_msgs__msg__Target__FIELD_NAME__located[] = "located";
static char polystar_msgs__msg__Target__FIELD_NAME__distance_center[] = "distance_center";
static char polystar_msgs__msg__Target__FIELD_NAME__theta[] = "theta";
static char polystar_msgs__msg__Target__FIELD_NAME__phi[] = "phi";
static char polystar_msgs__msg__Target__FIELD_NAME__dist[] = "dist";

static rosidl_runtime_c__type_description__Field polystar_msgs__msg__Target__FIELDS[] = {
  {
    {polystar_msgs__msg__Target__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__Target__FIELD_NAME__located, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__Target__FIELD_NAME__distance_center, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__Target__FIELD_NAME__theta, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__Target__FIELD_NAME__phi, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__Target__FIELD_NAME__dist, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription polystar_msgs__msg__Target__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
polystar_msgs__msg__Target__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {polystar_msgs__msg__Target__TYPE_NAME, 24, 24},
      {polystar_msgs__msg__Target__FIELDS, 6, 6},
    },
    {polystar_msgs__msg__Target__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Target.msg\n"
  "## From CV to CS : Coordinates of the current target\n"
  "\n"
  "# Message\n"
  "\n"
  "builtin_interfaces/Time stamp\n"
  "bool located\n"
  "\n"
  "float32 distance_center\n"
  "\n"
  "int16 theta    # milli-ad\n"
  "int16 phi       # millirad\n"
  "uint16 dist     # mm";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
polystar_msgs__msg__Target__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {polystar_msgs__msg__Target__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 221, 221},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
polystar_msgs__msg__Target__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *polystar_msgs__msg__Target__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
