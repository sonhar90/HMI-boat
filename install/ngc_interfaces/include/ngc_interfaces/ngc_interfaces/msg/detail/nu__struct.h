// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ngc_interfaces:msg/Nu.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__NU__STRUCT_H_
#define NGC_INTERFACES__MSG__DETAIL__NU__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Nu in the package ngc_interfaces.
/**
  * Linear and angular velocity in according with SNAME notation
 */
typedef struct ngc_interfaces__msg__Nu
{
  /// meters pr. sec
  float u;
  /// meters pr. sec
  float v;
  /// meters pr. sec
  float w;
  /// radians pr. sec
  float p;
  /// radians pr. sec
  float q;
  /// radians pr. sec
  float r;
} ngc_interfaces__msg__Nu;

// Struct for a sequence of ngc_interfaces__msg__Nu.
typedef struct ngc_interfaces__msg__Nu__Sequence
{
  ngc_interfaces__msg__Nu * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ngc_interfaces__msg__Nu__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NGC_INTERFACES__MSG__DETAIL__NU__STRUCT_H_
