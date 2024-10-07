// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ngc_interfaces:msg/GNSS.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/gnss.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__GNSS__BUILDER_HPP_
#define NGC_INTERFACES__MSG__DETAIL__GNSS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ngc_interfaces/msg/detail/gnss__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ngc_interfaces
{

namespace msg
{

namespace builder
{

class Init_GNSS_valid_signal
{
public:
  explicit Init_GNSS_valid_signal(::ngc_interfaces::msg::GNSS & msg)
  : msg_(msg)
  {}
  ::ngc_interfaces::msg::GNSS valid_signal(::ngc_interfaces::msg::GNSS::_valid_signal_type arg)
  {
    msg_.valid_signal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ngc_interfaces::msg::GNSS msg_;
};

class Init_GNSS_lon
{
public:
  explicit Init_GNSS_lon(::ngc_interfaces::msg::GNSS & msg)
  : msg_(msg)
  {}
  Init_GNSS_valid_signal lon(::ngc_interfaces::msg::GNSS::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_GNSS_valid_signal(msg_);
  }

private:
  ::ngc_interfaces::msg::GNSS msg_;
};

class Init_GNSS_lat
{
public:
  Init_GNSS_lat()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GNSS_lon lat(::ngc_interfaces::msg::GNSS::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_GNSS_lon(msg_);
  }

private:
  ::ngc_interfaces::msg::GNSS msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ngc_interfaces::msg::GNSS>()
{
  return ngc_interfaces::msg::builder::Init_GNSS_lat();
}

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__GNSS__BUILDER_HPP_
