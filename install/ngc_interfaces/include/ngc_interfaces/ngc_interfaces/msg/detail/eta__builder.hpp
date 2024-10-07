// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ngc_interfaces:msg/Eta.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/eta.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__ETA__BUILDER_HPP_
#define NGC_INTERFACES__MSG__DETAIL__ETA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ngc_interfaces/msg/detail/eta__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ngc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Eta_psi
{
public:
  explicit Init_Eta_psi(::ngc_interfaces::msg::Eta & msg)
  : msg_(msg)
  {}
  ::ngc_interfaces::msg::Eta psi(::ngc_interfaces::msg::Eta::_psi_type arg)
  {
    msg_.psi = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ngc_interfaces::msg::Eta msg_;
};

class Init_Eta_theta
{
public:
  explicit Init_Eta_theta(::ngc_interfaces::msg::Eta & msg)
  : msg_(msg)
  {}
  Init_Eta_psi theta(::ngc_interfaces::msg::Eta::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_Eta_psi(msg_);
  }

private:
  ::ngc_interfaces::msg::Eta msg_;
};

class Init_Eta_phi
{
public:
  explicit Init_Eta_phi(::ngc_interfaces::msg::Eta & msg)
  : msg_(msg)
  {}
  Init_Eta_theta phi(::ngc_interfaces::msg::Eta::_phi_type arg)
  {
    msg_.phi = std::move(arg);
    return Init_Eta_theta(msg_);
  }

private:
  ::ngc_interfaces::msg::Eta msg_;
};

class Init_Eta_z
{
public:
  explicit Init_Eta_z(::ngc_interfaces::msg::Eta & msg)
  : msg_(msg)
  {}
  Init_Eta_phi z(::ngc_interfaces::msg::Eta::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Eta_phi(msg_);
  }

private:
  ::ngc_interfaces::msg::Eta msg_;
};

class Init_Eta_lon
{
public:
  explicit Init_Eta_lon(::ngc_interfaces::msg::Eta & msg)
  : msg_(msg)
  {}
  Init_Eta_z lon(::ngc_interfaces::msg::Eta::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_Eta_z(msg_);
  }

private:
  ::ngc_interfaces::msg::Eta msg_;
};

class Init_Eta_lat
{
public:
  Init_Eta_lat()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Eta_lon lat(::ngc_interfaces::msg::Eta::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_Eta_lon(msg_);
  }

private:
  ::ngc_interfaces::msg::Eta msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ngc_interfaces::msg::Eta>()
{
  return ngc_interfaces::msg::builder::Init_Eta_lat();
}

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__ETA__BUILDER_HPP_
