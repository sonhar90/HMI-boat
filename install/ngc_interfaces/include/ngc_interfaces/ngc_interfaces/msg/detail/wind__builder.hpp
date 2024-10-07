// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ngc_interfaces:msg/Wind.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/wind.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__WIND__BUILDER_HPP_
#define NGC_INTERFACES__MSG__DETAIL__WIND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ngc_interfaces/msg/detail/wind__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ngc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Wind_magnitude_ms
{
public:
  explicit Init_Wind_magnitude_ms(::ngc_interfaces::msg::Wind & msg)
  : msg_(msg)
  {}
  ::ngc_interfaces::msg::Wind magnitude_ms(::ngc_interfaces::msg::Wind::_magnitude_ms_type arg)
  {
    msg_.magnitude_ms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ngc_interfaces::msg::Wind msg_;
};

class Init_Wind_direction_relative_deg
{
public:
  Init_Wind_direction_relative_deg()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wind_magnitude_ms direction_relative_deg(::ngc_interfaces::msg::Wind::_direction_relative_deg_type arg)
  {
    msg_.direction_relative_deg = std::move(arg);
    return Init_Wind_magnitude_ms(msg_);
  }

private:
  ::ngc_interfaces::msg::Wind msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ngc_interfaces::msg::Wind>()
{
  return ngc_interfaces::msg::builder::Init_Wind_direction_relative_deg();
}

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__WIND__BUILDER_HPP_
