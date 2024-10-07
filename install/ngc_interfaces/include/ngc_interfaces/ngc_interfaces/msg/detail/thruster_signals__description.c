// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ngc_interfaces:msg/ThrusterSignals.idl
// generated code does not contain a copyright notice

#include "ngc_interfaces/msg/detail/thruster_signals__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ngc_interfaces
const rosidl_type_hash_t *
ngc_interfaces__msg__ThrusterSignals__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf5, 0x92, 0x2d, 0x22, 0xd8, 0xa1, 0x1a, 0xa7,
      0x1b, 0x51, 0x87, 0xa0, 0x51, 0x1f, 0x1a, 0xed,
      0xef, 0xa0, 0x9a, 0x8f, 0xb0, 0x91, 0x24, 0xea,
      0xef, 0x64, 0xd5, 0xe5, 0xfc, 0xea, 0xa9, 0x8e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ngc_interfaces__msg__ThrusterSignals__TYPE_NAME[] = "ngc_interfaces/msg/ThrusterSignals";

// Define type names, field names, and default values
static char ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__thruster_id[] = "thruster_id";
static char ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__rps[] = "rps";
static char ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__pitch[] = "pitch";
static char ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__azimuth_deg[] = "azimuth_deg";
static char ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__active[] = "active";
static char ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__error[] = "error";

static rosidl_runtime_c__type_description__Field ngc_interfaces__msg__ThrusterSignals__FIELDS[] = {
  {
    {ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__thruster_id, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__rps, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__azimuth_deg, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__active, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ngc_interfaces__msg__ThrusterSignals__FIELD_NAME__error, 5, 5},
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
ngc_interfaces__msg__ThrusterSignals__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ngc_interfaces__msg__ThrusterSignals__TYPE_NAME, 34, 34},
      {ngc_interfaces__msg__ThrusterSignals__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 thruster_id       # Identifier for the thruster\n"
  "float64 rps             # Desired rps of the thruster\n"
  "float64 pitch           # Desired pitch for controllable pitch propellers \n"
  "float64 azimuth_deg     # Desired azimuth angle for azimuth thrusters (degrees)\n"
  "bool active             # Command to activate or deactivate the thruster \n"
  "bool error              # Unit has a failure ";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ngc_interfaces__msg__ThrusterSignals__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ngc_interfaces__msg__ThrusterSignals__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 382, 382},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ngc_interfaces__msg__ThrusterSignals__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ngc_interfaces__msg__ThrusterSignals__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
