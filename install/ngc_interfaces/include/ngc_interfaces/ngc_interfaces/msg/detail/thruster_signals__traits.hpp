// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ngc_interfaces:msg/ThrusterSignals.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/thruster_signals.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__TRAITS_HPP_
#define NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ngc_interfaces/msg/detail/thruster_signals__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ngc_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ThrusterSignals & msg,
  std::ostream & out)
{
  out << "{";
  // member: thruster_id
  {
    out << "thruster_id: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_id, out);
    out << ", ";
  }

  // member: rps
  {
    out << "rps: ";
    rosidl_generator_traits::value_to_yaml(msg.rps, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: azimuth_deg
  {
    out << "azimuth_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.azimuth_deg, out);
    out << ", ";
  }

  // member: active
  {
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << ", ";
  }

  // member: error
  {
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ThrusterSignals & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: thruster_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thruster_id: ";
    rosidl_generator_traits::value_to_yaml(msg.thruster_id, out);
    out << "\n";
  }

  // member: rps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rps: ";
    rosidl_generator_traits::value_to_yaml(msg.rps, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: azimuth_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "azimuth_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.azimuth_deg, out);
    out << "\n";
  }

  // member: active
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active: ";
    rosidl_generator_traits::value_to_yaml(msg.active, out);
    out << "\n";
  }

  // member: error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "error: ";
    rosidl_generator_traits::value_to_yaml(msg.error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ThrusterSignals & msg, bool use_flow_style = false)
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
  const ngc_interfaces::msg::ThrusterSignals & msg,
  std::ostream & out, size_t indentation = 0)
{
  ngc_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ngc_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ngc_interfaces::msg::ThrusterSignals & msg)
{
  return ngc_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ngc_interfaces::msg::ThrusterSignals>()
{
  return "ngc_interfaces::msg::ThrusterSignals";
}

template<>
inline const char * name<ngc_interfaces::msg::ThrusterSignals>()
{
  return "ngc_interfaces/msg/ThrusterSignals";
}

template<>
struct has_fixed_size<ngc_interfaces::msg::ThrusterSignals>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ngc_interfaces::msg::ThrusterSignals>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ngc_interfaces::msg::ThrusterSignals>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__TRAITS_HPP_
