// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ngc_interfaces:msg/Tau.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__TAU__STRUCT_H_
#define NGC_INTERFACES__MSG__DETAIL__TAU__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Tau in the package ngc_interfaces.
/**
  * Forces and moments in according with SNAME notation
 */
typedef struct ngc_interfaces__msg__Tau
{
  float surge_x;
  float sway_y;
  float heave_z;
  float roll_k;
  float pitch_m;
  float yaw_n;
} ngc_interfaces__msg__Tau;

// Struct for a sequence of ngc_interfaces__msg__Tau.
typedef struct ngc_interfaces__msg__Tau__Sequence
{
  ngc_interfaces__msg__Tau * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ngc_interfaces__msg__Tau__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NGC_INTERFACES__MSG__DETAIL__TAU__STRUCT_H_
