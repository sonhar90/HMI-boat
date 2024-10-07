// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ngc_interfaces:msg/Wind.idl
// generated code does not contain a copyright notice

#include "ngc_interfaces/msg/detail/wind__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_type_hash_t *
ngc_interfaces__msg__Wind__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x2d, 0xdd, 0xde, 0x6c, 0x53, 0xcf, 0x8f, 0x76,
      0x95, 0x8e, 0x73, 0xf5, 0x05, 0xf7, 0xb4, 0xa3,
      0x0b, 0x0d, 0xab, 0xc0, 0xd9, 0xb9, 0xd7, 0xfa,
      0x00, 0x66, 0x3f, 0x27, 0x00, 0xeb, 0xba, 0x29,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ngc_interfaces__msg__Wind__TYPE_NAME[] = "ngc_interfaces/msg/Wind";

// Define type names, field names, and default values
static char ngc_interfaces__msg__Wind__FIELD_NAME__direction_relative_deg[] = "direction_relative_deg";
static char ngc_interfaces__msg__Wind__FIELD_NAME__magnitude_ms[] = "magnitude_ms";

static rosidl_runtime_c__type_description__Field ngc_interfaces__msg__Wind__FIELDS[] = {
  {
    {ngc_interfaces__msg__Wind__FIELD_NAME__direction_relative_deg, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__Wind__FIELD_NAME__magnitude_ms, 12, 12},
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
ngc_interfaces__msg__Wind__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ngc_interfaces__msg__Wind__TYPE_NAME, 23, 23},
      {ngc_interfaces__msg__Wind__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 direction_relative_deg      # wind direction in degrees\n"
  "float32 magnitude_ms                # wind speed in meters pr. second";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ngc_interfaces__msg__Wind__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ngc_interfaces__msg__Wind__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 133, 133},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ngc_interfaces__msg__Wind__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ngc_interfaces__msg__Wind__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
