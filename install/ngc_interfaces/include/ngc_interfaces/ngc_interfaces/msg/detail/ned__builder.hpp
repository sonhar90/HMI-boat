// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ngc_interfaces:msg/Ned.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__NED__BUILDER_HPP_
#define NGC_INTERFACES__MSG__DETAIL__NED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ngc_interfaces/msg/detail/ned__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ngc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Ned_psi
{
public:
  explicit Init_Ned_psi(::ngc_interfaces::msg::Ned & msg)
  : msg_(msg)
  {}
  ::ngc_interfaces::msg::Ned psi(::ngc_interfaces::msg::Ned::_psi_type arg)
  {
    msg_.psi = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ngc_interfaces::msg::Ned msg_;
};

class Init_Ned_theta
{
public:
  explicit Init_Ned_theta(::ngc_interfaces::msg::Ned & msg)
  : msg_(msg)
  {}
  Init_Ned_psi theta(::ngc_interfaces::msg::Ned::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_Ned_psi(msg_);
  }

private:
  ::ngc_interfaces::msg::Ned msg_;
};

class Init_Ned_phi
{
public:
  explicit Init_Ned_phi(::ngc_interfaces::msg::Ned & msg)
  : msg_(msg)
  {}
  Init_Ned_theta phi(::ngc_interfaces::msg::Ned::_phi_type arg)
  {
    msg_.phi = std::move(arg);
    return Init_Ned_theta(msg_);
  }

private:
  ::ngc_interfaces::msg::Ned msg_;
};

class Init_Ned_z
{
public:
  explicit Init_Ned_z(::ngc_interfaces::msg::Ned & msg)
  : msg_(msg)
  {}
  Init_Ned_phi z(::ngc_interfaces::msg::Ned::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Ned_phi(msg_);
  }

private:
  ::ngc_interfaces::msg::Ned msg_;
};

class Init_Ned_y
{
public:
  explicit Init_Ned_y(::ngc_interfaces::msg::Ned & msg)
  : msg_(msg)
  {}
  Init_Ned_z y(::ngc_interfaces::msg::Ned::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Ned_z(msg_);
  }

private:
  ::ngc_interfaces::msg::Ned msg_;
};

class Init_Ned_x
{
public:
  Init_Ned_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Ned_y x(::ngc_interfaces::msg::Ned::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Ned_y(msg_);
  }

private:
  ::ngc_interfaces::msg::Ned msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ngc_interfaces::msg::Ned>()
{
  return ngc_interfaces::msg::builder::Init_Ned_x();
}

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__NED__BUILDER_HPP_
