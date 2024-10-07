// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ngc_interfaces:msg/Tau.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/tau.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__TAU__TRAITS_HPP_
#define NGC_INTERFACES__MSG__DETAIL__TAU__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ngc_interfaces/msg/detail/tau__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ngc_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Tau & msg,
  std::ostream & out)
{
  out << "{";
  // member: surge_x
  {
    out << "surge_x: ";
    rosidl_generator_traits::value_to_yaml(msg.surge_x, out);
    out << ", ";
  }

  // member: sway_y
  {
    out << "sway_y: ";
    rosidl_generator_traits::value_to_yaml(msg.sway_y, out);
    out << ", ";
  }

  // member: heave_z
  {
    out << "heave_z: ";
    rosidl_generator_traits::value_to_yaml(msg.heave_z, out);
    out << ", ";
  }

  // member: roll_k
  {
    out << "roll_k: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_k, out);
    out << ", ";
  }

  // member: pitch_m
  {
    out << "pitch_m: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_m, out);
    out << ", ";
  }

  // member: yaw_n
  {
    out << "yaw_n: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_n, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Tau & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: surge_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "surge_x: ";
    rosidl_generator_traits::value_to_yaml(msg.surge_x, out);
    out << "\n";
  }

  // member: sway_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sway_y: ";
    rosidl_generator_traits::value_to_yaml(msg.sway_y, out);
    out << "\n";
  }

  // member: heave_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heave_z: ";
    rosidl_generator_traits::value_to_yaml(msg.heave_z, out);
    out << "\n";
  }

  // member: roll_k
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_k: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_k, out);
    out << "\n";
  }

  // member: pitch_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_m: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_m, out);
    out << "\n";
  }

  // member: yaw_n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_n: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_n, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Tau & msg, bool use_flow_style = false)
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
  const ngc_interfaces::msg::Tau & msg,
  std::ostream & out, size_t indentation = 0)
{
  ngc_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ngc_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ngc_interfaces::msg::Tau & msg)
{
  return ngc_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ngc_interfaces::msg::Tau>()
{
  return "ngc_interfaces::msg::Tau";
}

template<>
inline const char * name<ngc_interfaces::msg::Tau>()
{
  return "ngc_interfaces/msg/Tau";
}

template<>
struct has_fixed_size<ngc_interfaces::msg::Tau>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ngc_interfaces::msg::Tau>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ngc_interfaces::msg::Tau>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NGC_INTERFACES__MSG__DETAIL__TAU__TRAITS_HPP_
