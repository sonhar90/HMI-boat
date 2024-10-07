// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ngc_interfaces:msg/Eta.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ngc_interfaces/msg/detail/eta__rosidl_typesupport_introspection_c.h"
#include "ngc_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ngc_interfaces/msg/detail/eta__functions.h"
#include "ngc_interfaces/msg/detail/eta__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ngc_interfaces__msg__Eta__init(message_memory);
}

void ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_fini_function(void * message_memory)
{
  ngc_interfaces__msg__Eta__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_message_member_array[6] = {
  {
    "lat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ngc_interfaces__msg__Eta, lat),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lon",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ngc_interfaces__msg__Eta, lon),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ngc_interfaces__msg__Eta, z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "phi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ngc_interfaces__msg__Eta, phi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "theta",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ngc_interfaces__msg__Eta, theta),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "psi",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ngc_interfaces__msg__Eta, psi),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_message_members = {
  "ngc_interfaces__msg",  // message namespace
  "Eta",  // message name
  6,  // number of fields
  sizeof(ngc_interfaces__msg__Eta),
  false,  // has_any_key_member_
  ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_message_member_array,  // message members
  ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_init_function,  // function to initialize message memory (memory has to be allocated)
  ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_message_type_support_handle = {
  0,
  &ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_message_members,
  get_message_typesupport_handle_function,
  &ngc_interfaces__msg__Eta__get_type_hash,
  &ngc_interfaces__msg__Eta__get_type_description,
  &ngc_interfaces__msg__Eta__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ngc_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ngc_interfaces, msg, Eta)() {
  if (!ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_message_type_support_handle.typesupport_identifier) {
    ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ngc_interfaces__msg__Eta__rosidl_typesupport_introspection_c__Eta_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
