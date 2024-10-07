// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ngc_interfaces:msg/HeadingDevice.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/heading_device.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__HEADING_DEVICE__BUILDER_HPP_
#define NGC_INTERFACES__MSG__DETAIL__HEADING_DEVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ngc_interfaces/msg/detail/heading_device__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ngc_interfaces
{

namespace msg
{

namespace builder
{

class Init_HeadingDevice_valid_signal
{
public:
  explicit Init_HeadingDevice_valid_signal(::ngc_interfaces::msg::HeadingDevice & msg)
  : msg_(msg)
  {}
  ::ngc_interfaces::msg::HeadingDevice valid_signal(::ngc_interfaces::msg::HeadingDevice::_valid_signal_type arg)
  {
    msg_.valid_signal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ngc_interfaces::msg::HeadingDevice msg_;
};

class Init_HeadingDevice_rot
{
public:
  explicit Init_HeadingDevice_rot(::ngc_interfaces::msg::HeadingDevice & msg)
  : msg_(msg)
  {}
  Init_HeadingDevice_valid_signal rot(::ngc_interfaces::msg::HeadingDevice::_rot_type arg)
  {
    msg_.rot = std::move(arg);
    return Init_HeadingDevice_valid_signal(msg_);
  }

private:
  ::ngc_interfaces::msg::HeadingDevice msg_;
};

class Init_HeadingDevice_heading
{
public:
  Init_HeadingDevice_heading()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeadingDevice_rot heading(::ngc_interfaces::msg::HeadingDevice::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_HeadingDevice_rot(msg_);
  }

private:
  ::ngc_interfaces::msg::HeadingDevice msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ngc_interfaces::msg::HeadingDevice>()
{
  return ngc_interfaces::msg::builder::Init_HeadingDevice_heading();
}

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__HEADING_DEVICE__BUILDER_HPP_
