// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from polystar_msgs:msg/PositionFeedback.idl
// generated code does not contain a copyright notice

#include "polystar_msgs/msg/detail/position_feedback__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_polystar_msgs
const rosidl_type_hash_t *
polystar_msgs__msg__PositionFeedback__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x03, 0xe9, 0xee, 0xe9, 0x2b, 0xcf, 0x18, 0xf4,
      0xdb, 0xc1, 0xed, 0x2d, 0x97, 0x8e, 0x3d, 0xe1,
      0x59, 0x3e, 0xaa, 0xf7, 0x30, 0x03, 0x5c, 0x09,
      0x46, 0x75, 0xa6, 0xc4, 0xa8, 0x56, 0xf3, 0xb3,
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

static char polystar_msgs__msg__PositionFeedback__TYPE_NAME[] = "polystar_msgs/msg/PositionFeedback";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__stamp[] = "stamp";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_ax[] = "imu_ax";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_ay[] = "imu_ay";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_az[] = "imu_az";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_gx[] = "imu_gx";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_gy[] = "imu_gy";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_gz[] = "imu_gz";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_rx[] = "imu_rx";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_ry[] = "imu_ry";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_rz[] = "imu_rz";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__enc_1[] = "enc_1";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__enc_2[] = "enc_2";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__enc_3[] = "enc_3";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__enc_4[] = "enc_4";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__v_enc_1[] = "v_enc_1";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__v_enc_2[] = "v_enc_2";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__v_enc_3[] = "v_enc_3";
static char polystar_msgs__msg__PositionFeedback__FIELD_NAME__v_enc_4[] = "v_enc_4";

static rosidl_runtime_c__type_description__Field polystar_msgs__msg__PositionFeedback__FIELDS[] = {
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_ax, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_ay, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_az, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_gx, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_gy, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_gz, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_rx, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_ry, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__imu_rz, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__enc_1, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__enc_2, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__enc_3, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__enc_4, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__v_enc_1, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__v_enc_2, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__v_enc_3, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {polystar_msgs__msg__PositionFeedback__FIELD_NAME__v_enc_4, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription polystar_msgs__msg__PositionFeedback__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
polystar_msgs__msg__PositionFeedback__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {polystar_msgs__msg__PositionFeedback__TYPE_NAME, 34, 34},
      {polystar_msgs__msg__PositionFeedback__FIELDS, 18, 18},
    },
    {polystar_msgs__msg__PositionFeedback__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# PositionFeedback.msg\n"
  "## Positional feedback from sensors. In m/s\\xc2\\xb2, rad/s\\xc2\\xb2 and seconds\n"
  "\n"
  "builtin_interfaces/Time stamp\n"
  "\n"
  "float32 imu_ax\n"
  "float32 imu_ay\n"
  "float32 imu_az\n"
  "float32 imu_gx\n"
  "float32 imu_gy\n"
  "float32 imu_gz\n"
  "float32 imu_rx\n"
  "float32 imu_ry\n"
  "float32 imu_rz\n"
  "int64 enc_1\n"
  "int64 enc_2\n"
  "int64 enc_3\n"
  "int64 enc_4\n"
  "float32 v_enc_1\n"
  "float32 v_enc_2\n"
  "float32 v_enc_3\n"
  "float32 v_enc_4";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
polystar_msgs__msg__PositionFeedback__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {polystar_msgs__msg__PositionFeedback__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 367, 367},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
polystar_msgs__msg__PositionFeedback__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *polystar_msgs__msg__PositionFeedback__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
