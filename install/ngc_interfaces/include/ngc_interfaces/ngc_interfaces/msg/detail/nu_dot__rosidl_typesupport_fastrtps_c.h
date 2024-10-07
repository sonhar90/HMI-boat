// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from ngc_interfaces:msg/NuDot.idl
// generated code does not contain a copyright notice
#ifndef NGC_INTERFACES__MSG__DETAIL__NU_DOT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define NGC_INTERFACES__MSG__DETAIL__NU_DOT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "ngc_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ngc_interfaces/msg/detail/nu_dot__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ngc_interfaces
bool cdr_serialize_ngc_interfaces__msg__NuDot(
  const ngc_interfaces__msg__NuDot * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ngc_interfaces
bool cdr_deserialize_ngc_interfaces__msg__NuDot(
  eprosima::fastcdr::Cdr &,
  ngc_interfaces__msg__NuDot * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ngc_interfaces
size_t get_serialized_size_ngc_interfaces__msg__NuDot(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ngc_interfaces
size_t max_serialized_size_ngc_interfaces__msg__NuDot(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ngc_interfaces
bool cdr_serialize_key_ngc_interfaces__msg__NuDot(
  const ngc_interfaces__msg__NuDot * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ngc_interfaces
size_t get_serialized_size_key_ngc_interfaces__msg__NuDot(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ngc_interfaces
size_t max_serialized_size_key_ngc_interfaces__msg__NuDot(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ngc_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ngc_interfaces, msg, NuDot)();

#ifdef __cplusplus
}
#endif

#endif  // NGC_INTERFACES__MSG__DETAIL__NU_DOT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_