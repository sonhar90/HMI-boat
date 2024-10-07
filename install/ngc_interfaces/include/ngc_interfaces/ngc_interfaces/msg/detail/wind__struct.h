// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ngc_interfaces:msg/Wind.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/wind.h"


#ifndef NGC_INTERFACES__MSG__DETAIL__WIND__STRUCT_H_
#define NGC_INTERFACES__MSG__DETAIL__WIND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Wind in the package ngc_interfaces.
typedef struct ngc_interfaces__msg__Wind
{
  /// wind direction in degrees
  float direction_relative_deg;
  /// wind speed in meters pr. second
  float magnitude_ms;
} ngc_interfaces__msg__Wind;

// Struct for a sequence of ngc_interfaces__msg__Wind.
typedef struct ngc_interfaces__msg__Wind__Sequence
{
  ngc_interfaces__msg__Wind * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ngc_interfaces__msg__Wind__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NGC_INTERFACES__MSG__DETAIL__WIND__STRUCT_H_
