// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ngc_interfaces:msg/NuDot.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__NU_DOT__STRUCT_H_
#define NGC_INTERFACES__MSG__DETAIL__NU_DOT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/NuDot in the package ngc_interfaces.
/**
  * Linear and angular accelleration in according with SNAME notation
 */
typedef struct ngc_interfaces__msg__NuDot
{
  float u_dot;
  float v_dot;
  float w_dot;
  float p_dot;
  float q_dot;
  float r_dot;
} ngc_interfaces__msg__NuDot;

// Struct for a sequence of ngc_interfaces__msg__NuDot.
typedef struct ngc_interfaces__msg__NuDot__Sequence
{
  ngc_interfaces__msg__NuDot * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ngc_interfaces__msg__NuDot__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NGC_INTERFACES__MSG__DETAIL__NU_DOT__STRUCT_H_
