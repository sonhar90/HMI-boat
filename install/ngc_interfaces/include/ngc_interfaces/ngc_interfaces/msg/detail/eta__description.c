// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ngc_interfaces:msg/Eta.idl
// generated code does not contain a copyright notice

#include "ngc_interfaces/msg/detail/eta__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_type_hash_t *
ngc_interfaces__msg__Eta__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb8, 0x71, 0x4a, 0x3e, 0x98, 0xc8, 0x32, 0xf1,
      0x89, 0xfc, 0x10, 0xda, 0x2a, 0xd0, 0xae, 0x76,
      0xff, 0x88, 0x66, 0x30, 0xe8, 0x89, 0x97, 0x68,
      0x4d, 0x75, 0xdd, 0xad, 0x35, 0x9d, 0x86, 0x42,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ngc_interfaces__msg__Eta__TYPE_NAME[] = "ngc_interfaces/msg/Eta";

// Define type names, field names, and default values
static char ngc_interfaces__msg__Eta__FIELD_NAME__lat[] = "lat";
static char ngc_interfaces__msg__Eta__FIELD_NAME__lon[] = "lon";
static char ngc_interfaces__msg__Eta__FIELD_NAME__z[] = "z";
static char ngc_interfaces__msg__Eta__FIELD_NAME__phi[] = "phi";
static char ngc_interfaces__msg__Eta__FIELD_NAME__theta[] = "theta";
static char ngc_interfaces__msg__Eta__FIELD_NAME__psi[] = "psi";

static rosidl_runtime_c__type_description__Field ngc_interfaces__msg__Eta__FIELDS[] = {
  {
    {ngc_interfaces__msg__Eta__FIELD_NAME__lat, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Eta__FIELD_NAME__lon, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Eta__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Eta__FIELD_NAME__phi, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Eta__FIELD_NAME__theta, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Eta__FIELD_NAME__psi, 3, 3},
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
ngc_interfaces__msg__Eta__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ngc_interfaces__msg__Eta__TYPE_NAME, 22, 22},
      {ngc_interfaces__msg__Eta__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#Position in NED, in according with SNAME notation\n"
  "float32 lat     # in decimal degrees\n"
  "float32 lon     # in decimal degrees\n"
  "float32 z       # in meters\n"
  "float32 phi     # in radians\n"
  "float32 theta   # in radians\n"
  "float32 psi     # in radians";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ngc_interfaces__msg__Eta__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ngc_interfaces__msg__Eta__TYPE_NAME, 22, 22},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 239, 239},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ngc_interfaces__msg__Eta__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ngc_interfaces__msg__Eta__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
