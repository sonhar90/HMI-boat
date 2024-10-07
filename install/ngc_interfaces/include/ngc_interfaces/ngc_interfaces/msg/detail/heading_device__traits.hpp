// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ngc_interfaces:msg/HeadingDevice.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/heading_device.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__HEADING_DEVICE__TRAITS_HPP_
#define NGC_INTERFACES__MSG__DETAIL__HEADING_DEVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ngc_interfaces/msg/detail/heading_device__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ngc_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const HeadingDevice & msg,
  std::ostream & out)
{
  out << "{";
  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: rot
  {
    out << "rot: ";
    rosidl_generator_traits::value_to_yaml(msg.rot, out);
    out << ", ";
  }

  // member: valid_signal
  {
    out << "valid_signal: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_signal, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const HeadingDevice & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: rot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rot: ";
    rosidl_generator_traits::value_to_yaml(msg.rot, out);
    out << "\n";
  }

  // member: valid_signal
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid_signal: ";
    rosidl_generator_traits::value_to_yaml(msg.valid_signal, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const HeadingDevice & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ngc_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ngc_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ngc_interfaces::msg::HeadingDevice & msg,
  std::ostream & out, size_t indentation = 0)
{
  ngc_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ngc_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ngc_interfaces::msg::HeadingDevice & msg)
{
  return ngc_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ngc_interfaces::msg::HeadingDevice>()
{
  return "ngc_interfaces::msg::HeadingDevice";
}

template<>
inline const char * name<ngc_interfaces::msg::HeadingDevice>()
{
  return "ngc_interfaces/msg/HeadingDevice";
}

template<>
struct has_fixed_size<ngc_interfaces::msg::HeadingDevice>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ngc_interfaces::msg::HeadingDevice>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ngc_interfaces::msg::HeadingDevice>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NGC_INTERFACES__MSG__DETAIL__HEADING_DEVICE__TRAITS_HPP_
