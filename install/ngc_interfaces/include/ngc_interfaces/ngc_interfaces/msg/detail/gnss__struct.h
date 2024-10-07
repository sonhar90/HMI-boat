// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ngc_interfaces:msg/GNSS.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/gnss.h"


#ifndef NGC_INTERFACES__MSG__DETAIL__GNSS__STRUCT_H_
#define NGC_INTERFACES__MSG__DETAIL__GNSS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/GNSS in the package ngc_interfaces.
/**
  * GNSS output 
 */
typedef struct ngc_interfaces__msg__GNSS
{
  /// in decimal degrees
  float lat;
  /// in decimal degrees
  float lon;
  bool valid_signal;
} ngc_interfaces__msg__GNSS;

// Struct for a sequence of ngc_interfaces__msg__GNSS.
typedef struct ngc_interfaces__msg__GNSS__Sequence
{
  ngc_interfaces__msg__GNSS * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ngc_interfaces__msg__GNSS__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NGC_INTERFACES__MSG__DETAIL__GNSS__STRUCT_H_
