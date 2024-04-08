// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ngc_interfaces:msg/NuDot.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__NU_DOT__TRAITS_HPP_
#define NGC_INTERFACES__MSG__DETAIL__NU_DOT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ngc_interfaces/msg/detail/nu_dot__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ngc_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const NuDot & msg,
  std::ostream & out)
{
  out << "{";
  // member: u_dot
  {
    out << "u_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.u_dot, out);
    out << ", ";
  }

  // member: v_dot
  {
    out << "v_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.v_dot, out);
    out << ", ";
  }

  // member: w_dot
  {
    out << "w_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.w_dot, out);
    out << ", ";
  }

  // member: p_dot
  {
    out << "p_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.p_dot, out);
    out << ", ";
  }

  // member: q_dot
  {
    out << "q_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q_dot, out);
    out << ", ";
  }

  // member: r_dot
  {
    out << "r_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.r_dot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NuDot & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: u_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "u_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.u_dot, out);
    out << "\n";
  }

  // member: v_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.v_dot, out);
    out << "\n";
  }

  // member: w_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "w_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.w_dot, out);
    out << "\n";
  }

  // member: p_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "p_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.p_dot, out);
    out << "\n";
  }

  // member: q_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "q_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.q_dot, out);
    out << "\n";
  }

  // member: r_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.r_dot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NuDot & msg, bool use_flow_style = false)
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
  const ngc_interfaces::msg::NuDot & msg,
  std::ostream & out, size_t indentation = 0)
{
  ngc_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ngc_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ngc_interfaces::msg::NuDot & msg)
{
  return ngc_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ngc_interfaces::msg::NuDot>()
{
  return "ngc_interfaces::msg::NuDot";
}

template<>
inline const char * name<ngc_interfaces::msg::NuDot>()
{
  return "ngc_interfaces/msg/NuDot";
}

template<>
struct has_fixed_size<ngc_interfaces::msg::NuDot>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ngc_interfaces::msg::NuDot>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ngc_interfaces::msg::NuDot>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NGC_INTERFACES__MSG__DETAIL__NU_DOT__TRAITS_HPP_
