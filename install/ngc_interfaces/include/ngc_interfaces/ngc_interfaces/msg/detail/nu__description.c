// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ngc_interfaces:msg/Nu.idl
// generated code does not contain a copyright notice

#include "ngc_interfaces/msg/detail/nu__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_type_hash_t *
ngc_interfaces__msg__Nu__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1a, 0x94, 0x72, 0xd4, 0x80, 0x8f, 0x64, 0x0a,
      0x56, 0xad, 0x41, 0x8d, 0x33, 0xd2, 0xd0, 0xcb,
      0xde, 0x70, 0x15, 0xb2, 0xf4, 0x7a, 0x2b, 0xc5,
      0x20, 0x2d, 0x46, 0xa3, 0x7a, 0xc6, 0x4f, 0xfc,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ngc_interfaces__msg__Nu__TYPE_NAME[] = "ngc_interfaces/msg/Nu";

// Define type names, field names, and default values
static char ngc_interfaces__msg__Nu__FIELD_NAME__u[] = "u";
static char ngc_interfaces__msg__Nu__FIELD_NAME__v[] = "v";
static char ngc_interfaces__msg__Nu__FIELD_NAME__w[] = "w";
static char ngc_interfaces__msg__Nu__FIELD_NAME__p[] = "p";
static char ngc_interfaces__msg__Nu__FIELD_NAME__q[] = "q";
static char ngc_interfaces__msg__Nu__FIELD_NAME__r[] = "r";

static rosidl_runtime_c__type_description__Field ngc_interfaces__msg__Nu__FIELDS[] = {
  {
    {ngc_interfaces__msg__Nu__FIELD_NAME__u, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Nu__FIELD_NAME__v, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Nu__FIELD_NAME__w, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Nu__FIELD_NAME__p, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Nu__FIELD_NAME__q, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Nu__FIELD_NAME__r, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ngc_interfaces__msg__Nu__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ngc_interfaces__msg__Nu__TYPE_NAME, 21, 21},
      {ngc_interfaces__msg__Nu__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#Linear and angular velocity in according with SNAME notation\n"
  "float32 u   # meters pr. sec\n"
  "float32 v   # meters pr. sec\n"
  "float32 w   # meters pr. sec\n"
  "float32 p   # radians pr. sec\n"
  "float32 q   # radians pr. sec\n"
  "float32 r   # radians pr. sec";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ngc_interfaces__msg__Nu__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ngc_interfaces__msg__Nu__TYPE_NAME, 21, 21},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 238, 238},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ngc_interfaces__msg__Nu__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ngc_interfaces__msg__Nu__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
