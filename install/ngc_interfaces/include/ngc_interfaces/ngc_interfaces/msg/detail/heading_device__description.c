// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ngc_interfaces:msg/HeadingDevice.idl
// generated code does not contain a copyright notice

#include "ngc_interfaces/msg/detail/heading_device__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_type_hash_t *
ngc_interfaces__msg__HeadingDevice__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x74, 0xe7, 0x35, 0xad, 0x25, 0x4d, 0xf2, 0x63,
      0x3d, 0x02, 0xaa, 0xd1, 0xf9, 0xdf, 0xe2, 0x9b,
      0xc6, 0x71, 0xd5, 0x65, 0x57, 0x47, 0x63, 0x8a,
      0xf4, 0x77, 0xc8, 0x55, 0x60, 0x83, 0x76, 0xbf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ngc_interfaces__msg__HeadingDevice__TYPE_NAME[] = "ngc_interfaces/msg/HeadingDevice";

// Define type names, field names, and default values
static char ngc_interfaces__msg__HeadingDevice__FIELD_NAME__heading[] = "heading";
static char ngc_interfaces__msg__HeadingDevice__FIELD_NAME__rot[] = "rot";
static char ngc_interfaces__msg__HeadingDevice__FIELD_NAME__valid_signal[] = "valid_signal";

static rosidl_runtime_c__type_description__Field ngc_interfaces__msg__HeadingDevice__FIELDS[] = {
  {
    {ngc_interfaces__msg__HeadingDevice__FIELD_NAME__heading, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__HeadingDevice__FIELD_NAME__rot, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__HeadingDevice__FIELD_NAME__valid_signal, 12, 12},
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
ngc_interfaces__msg__HeadingDevice__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ngc_interfaces__msg__HeadingDevice__TYPE_NAME, 32, 32},
      {ngc_interfaces__msg__HeadingDevice__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# A heading device (gyro, gnss, compass) output \n"
  "float32 heading             # in radians \n"
  "float32 rot                 # in radians pr. sec\n"
  "bool    valid_signal";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ngc_interfaces__msg__HeadingDevice__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ngc_interfaces__msg__HeadingDevice__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 160, 160},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ngc_interfaces__msg__HeadingDevice__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ngc_interfaces__msg__HeadingDevice__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
