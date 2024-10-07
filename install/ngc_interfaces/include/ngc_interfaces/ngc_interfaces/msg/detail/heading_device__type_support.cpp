// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ngc_interfaces:msg/HeadingDevice.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ngc_interfaces/msg/detail/heading_device__functions.h"
#include "ngc_interfaces/msg/detail/heading_device__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ngc_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void HeadingDevice_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ngc_interfaces::msg::HeadingDevice(_init);
}

void HeadingDevice_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ngc_interfaces::msg::HeadingDevice *>(message_memory);
  typed_message->~HeadingDevice();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember HeadingDevice_message_member_array[3] = {
  {
    "heading",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ngc_interfaces::msg::HeadingDevice, heading),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rot",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ngc_interfaces::msg::HeadingDevice, rot),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "valid_signal",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ngc_interfaces::msg::HeadingDevice, valid_signal),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers HeadingDevice_message_members = {
  "ngc_interfaces::msg",  // message namespace
  "HeadingDevice",  // message name
  3,  // number of fields
  sizeof(ngc_interfaces::msg::HeadingDevice),
  false,  // has_any_key_member_
  HeadingDevice_message_member_array,  // message members
  HeadingDevice_init_function,  // function to initialize message memory (memory has to be allocated)
  HeadingDevice_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t HeadingDevice_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &HeadingDevice_message_members,
  get_message_typesupport_handle_function,
  &ngc_interfaces__msg__HeadingDevice__get_type_hash,
  &ngc_interfaces__msg__HeadingDevice__get_type_description,
  &ngc_interfaces__msg__HeadingDevice__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ngc_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ngc_interfaces::msg::HeadingDevice>()
{
  return &::ngc_interfaces::msg::rosidl_typesupport_introspection_cpp::HeadingDevice_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ngc_interfaces, msg, HeadingDevice)() {
  return &::ngc_interfaces::msg::rosidl_typesupport_introspection_cpp::HeadingDevice_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
