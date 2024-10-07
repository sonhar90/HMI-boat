// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ngc_interfaces:msg/ThrusterSignals.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/thruster_signals.h"


#ifndef NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__STRUCT_H_
#define NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/ThrusterSignals in the package ngc_interfaces.
typedef struct ngc_interfaces__msg__ThrusterSignals
{
  /// Identifier for the thruster
  int32_t thruster_id;
  /// Desired rps of the thruster
  double rps;
  /// Desired pitch for controllable pitch propellers
  double pitch;
  /// Desired azimuth angle for azimuth thrusters (degrees)
  double azimuth_deg;
  /// Command to activate or deactivate the thruster
  bool active;
  /// Unit has a failure
  bool error;
} ngc_interfaces__msg__ThrusterSignals;

// Struct for a sequence of ngc_interfaces__msg__ThrusterSignals.
typedef struct ngc_interfaces__msg__ThrusterSignals__Sequence
{
  ngc_interfaces__msg__ThrusterSignals * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ngc_interfaces__msg__ThrusterSignals__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__STRUCT_H_
