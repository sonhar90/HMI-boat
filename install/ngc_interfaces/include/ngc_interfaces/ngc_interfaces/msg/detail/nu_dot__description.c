// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ngc_interfaces:msg/NuDot.idl
// generated code does not contain a copyright notice

#include "ngc_interfaces/msg/detail/nu_dot__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_type_hash_t *
ngc_interfaces__msg__NuDot__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9c, 0x05, 0x26, 0x11, 0xf6, 0x8e, 0x71, 0xda,
      0x38, 0x47, 0x53, 0xd0, 0xfc, 0xef, 0x53, 0xdc,
      0xfa, 0xce, 0x51, 0x00, 0xd3, 0xfd, 0x81, 0xaf,
      0x98, 0x3e, 0x51, 0x5e, 0xa7, 0x88, 0x5c, 0x47,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ngc_interfaces__msg__NuDot__TYPE_NAME[] = "ngc_interfaces/msg/NuDot";

// Define type names, field names, and default values
static char ngc_interfaces__msg__NuDot__FIELD_NAME__u_dot[] = "u_dot";
static char ngc_interfaces__msg__NuDot__FIELD_NAME__v_dot[] = "v_dot";
static char ngc_interfaces__msg__NuDot__FIELD_NAME__w_dot[] = "w_dot";
static char ngc_interfaces__msg__NuDot__FIELD_NAME__p_dot[] = "p_dot";
static char ngc_interfaces__msg__NuDot__FIELD_NAME__q_dot[] = "q_dot";
static char ngc_interfaces__msg__NuDot__FIELD_NAME__r_dot[] = "r_dot";

static rosidl_runtime_c__type_description__Field ngc_interfaces__msg__NuDot__FIELDS[] = {
  {
    {ngc_interfaces__msg__NuDot__FIELD_NAME__u_dot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__NuDot__FIELD_NAME__v_dot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__NuDot__FIELD_NAME__w_dot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__NuDot__FIELD_NAME__p_dot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__NuDot__FIELD_NAME__q_dot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__NuDot__FIELD_NAME__r_dot, 5, 5},
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
ngc_interfaces__msg__NuDot__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ngc_interfaces__msg__NuDot__TYPE_NAME, 24, 24},
      {ngc_interfaces__msg__NuDot__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#Linear and angular accelleration in according with SNAME notation\n"
  "float32 u_dot\n"
  "float32 v_dot\n"
  "float32 w_dot\n"
  "float32 p_dot\n"
  "float32 q_dot\n"
  "float32 r_dot";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ngc_interfaces__msg__NuDot__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ngc_interfaces__msg__NuDot__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 150, 150},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ngc_interfaces__msg__NuDot__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ngc_interfaces__msg__NuDot__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
