// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ngc_interfaces:msg/Ned.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__NED__STRUCT_H_
#define NGC_INTERFACES__MSG__DETAIL__NED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Ned in the package ngc_interfaces.
/**
  * Position in NED, in according with SNAME notation
 */
typedef struct ngc_interfaces__msg__Ned
{
  float x;
  float y;
  float z;
  float phi;
  float theta;
  float psi;
} ngc_interfaces__msg__Ned;

// Struct for a sequence of ngc_interfaces__msg__Ned.
typedef struct ngc_interfaces__msg__Ned__Sequence
{
  ngc_interfaces__msg__Ned * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ngc_interfaces__msg__Ned__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NGC_INTERFACES__MSG__DETAIL__NED__STRUCT_H_
