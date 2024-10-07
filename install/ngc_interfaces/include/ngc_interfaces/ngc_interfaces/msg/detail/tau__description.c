// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ngc_interfaces:msg/Tau.idl
// generated code does not contain a copyright notice

#include "ngc_interfaces/msg/detail/tau__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_type_hash_t *
ngc_interfaces__msg__Tau__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8c, 0xe4, 0x9e, 0x8a, 0x5d, 0xa2, 0x10, 0xad,
      0x35, 0xd7, 0x29, 0x5a, 0xef, 0x47, 0x34, 0xc8,
      0xc0, 0x99, 0xf9, 0xc6, 0xb7, 0x4e, 0x8a, 0xdc,
      0xee, 0xb2, 0xb6, 0xd4, 0xbf, 0xc8, 0xdb, 0xd7,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ngc_interfaces__msg__Tau__TYPE_NAME[] = "ngc_interfaces/msg/Tau";

// Define type names, field names, and default values
static char ngc_interfaces__msg__Tau__FIELD_NAME__surge_x[] = "surge_x";
static char ngc_interfaces__msg__Tau__FIELD_NAME__sway_y[] = "sway_y";
static char ngc_interfaces__msg__Tau__FIELD_NAME__heave_z[] = "heave_z";
static char ngc_interfaces__msg__Tau__FIELD_NAME__roll_k[] = "roll_k";
static char ngc_interfaces__msg__Tau__FIELD_NAME__pitch_m[] = "pitch_m";
static char ngc_interfaces__msg__Tau__FIELD_NAME__yaw_n[] = "yaw_n";

static rosidl_runtime_c__type_description__Field ngc_interfaces__msg__Tau__FIELDS[] = {
  {
    {ngc_interfaces__msg__Tau__FIELD_NAME__surge_x, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Tau__FIELD_NAME__sway_y, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Tau__FIELD_NAME__heave_z, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Tau__FIELD_NAME__roll_k, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Tau__FIELD_NAME__pitch_m, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Tau__FIELD_NAME__yaw_n, 5, 5},
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
ngc_interfaces__msg__Tau__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ngc_interfaces__msg__Tau__TYPE_NAME, 22, 22},
      {ngc_interfaces__msg__Tau__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#Forces and moments in according with SNAME notation\n"
  "float32 surge_x\n"
  "float32 sway_y\n"
  "float32 heave_z\n"
  "float32 roll_k\n"
  "float32 pitch_m\n"
  "float32 yaw_n";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ngc_interfaces__msg__Tau__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ngc_interfaces__msg__Tau__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 144, 144},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ngc_interfaces__msg__Tau__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ngc_interfaces__msg__Tau__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
