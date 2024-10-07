// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ngc_interfaces:msg/Wind.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/wind.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__WIND__TRAITS_HPP_
#define NGC_INTERFACES__MSG__DETAIL__WIND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ngc_interfaces/msg/detail/wind__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ngc_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Wind & msg,
  std::ostream & out)
{
  out << "{";
  // member: direction_relative_deg
  {
    out << "direction_relative_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.direction_relative_deg, out);
    out << ", ";
  }

  // member: magnitude_ms
  {
    out << "magnitude_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.magnitude_ms, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Wind & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: direction_relative_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction_relative_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.direction_relative_deg, out);
    out << "\n";
  }

  // member: magnitude_ms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnitude_ms: ";
    rosidl_generator_traits::value_to_yaml(msg.magnitude_ms, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Wind & msg, bool use_flow_style = false)
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
  const ngc_interfaces::msg::Wind & msg,
  std::ostream & out, size_t indentation = 0)
{
  ngc_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ngc_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ngc_interfaces::msg::Wind & msg)
{
  return ngc_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ngc_interfaces::msg::Wind>()
{
  return "ngc_interfaces::msg::Wind";
}

template<>
inline const char * name<ngc_interfaces::msg::Wind>()
{
  return "ngc_interfaces/msg/Wind";
}

template<>
struct has_fixed_size<ngc_interfaces::msg::Wind>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ngc_interfaces::msg::Wind>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ngc_interfaces::msg::Wind>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NGC_INTERFACES__MSG__DETAIL__WIND__TRAITS_HPP_
