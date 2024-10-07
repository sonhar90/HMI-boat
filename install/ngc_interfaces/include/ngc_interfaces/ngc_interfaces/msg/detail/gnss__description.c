// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ngc_interfaces:msg/GNSS.idl
// generated code does not contain a copyright notice

#include "ngc_interfaces/msg/detail/gnss__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_type_hash_t *
ngc_interfaces__msg__GNSS__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x04, 0x8b, 0x93, 0xb1, 0x22, 0x6d, 0x13, 0x4a,
      0xf7, 0xfa, 0xe7, 0x43, 0xf1, 0xdd, 0xbf, 0x1e,
      0xad, 0xe0, 0x8b, 0xd9, 0x76, 0x31, 0xf9, 0x29,
      0x2a, 0x83, 0x56, 0xa0, 0x5e, 0x23, 0x3e, 0x8f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ngc_interfaces__msg__GNSS__TYPE_NAME[] = "ngc_interfaces/msg/GNSS";

// Define type names, field names, and default values
static char ngc_interfaces__msg__GNSS__FIELD_NAME__lat[] = "lat";
static char ngc_interfaces__msg__GNSS__FIELD_NAME__lon[] = "lon";
static char ngc_interfaces__msg__GNSS__FIELD_NAME__valid_signal[] = "valid_signal";

static rosidl_runtime_c__type_description__Field ngc_interfaces__msg__GNSS__FIELDS[] = {
  {
    {ngc_interfaces__msg__GNSS__FIELD_NAME__lat, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__GNSS__FIELD_NAME__lon, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__GNSS__FIELD_NAME__valid_signal, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ngc_interfaces__msg__GNSS__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ngc_interfaces__msg__GNSS__TYPE_NAME, 23, 23},
      {ngc_interfaces__msg__GNSS__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GNSS output \n"
  "float32 lat             # in decimal degrees\n"
  "float32 lon             # in decimal degrees\n"
  "bool    valid_signal";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ngc_interfaces__msg__GNSS__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ngc_interfaces__msg__GNSS__TYPE_NAME, 23, 23},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 125, 125},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ngc_interfaces__msg__GNSS__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ngc_interfaces__msg__GNSS__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
