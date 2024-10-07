// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ngc_interfaces:msg/Tau.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/tau.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__TAU__BUILDER_HPP_
#define NGC_INTERFACES__MSG__DETAIL__TAU__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ngc_interfaces/msg/detail/tau__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ngc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Tau_yaw_n
{
public:
  explicit Init_Tau_yaw_n(::ngc_interfaces::msg::Tau & msg)
  : msg_(msg)
  {}
  ::ngc_interfaces::msg::Tau yaw_n(::ngc_interfaces::msg::Tau::_yaw_n_type arg)
  {
    msg_.yaw_n = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ngc_interfaces::msg::Tau msg_;
};

class Init_Tau_pitch_m
{
public:
  explicit Init_Tau_pitch_m(::ngc_interfaces::msg::Tau & msg)
  : msg_(msg)
  {}
  Init_Tau_yaw_n pitch_m(::ngc_interfaces::msg::Tau::_pitch_m_type arg)
  {
    msg_.pitch_m = std::move(arg);
    return Init_Tau_yaw_n(msg_);
  }

private:
  ::ngc_interfaces::msg::Tau msg_;
};

class Init_Tau_roll_k
{
public:
  explicit Init_Tau_roll_k(::ngc_interfaces::msg::Tau & msg)
  : msg_(msg)
  {}
  Init_Tau_pitch_m roll_k(::ngc_interfaces::msg::Tau::_roll_k_type arg)
  {
    msg_.roll_k = std::move(arg);
    return Init_Tau_pitch_m(msg_);
  }

private:
  ::ngc_interfaces::msg::Tau msg_;
};

class Init_Tau_heave_z
{
public:
  explicit Init_Tau_heave_z(::ngc_interfaces::msg::Tau & msg)
  : msg_(msg)
  {}
  Init_Tau_roll_k heave_z(::ngc_interfaces::msg::Tau::_heave_z_type arg)
  {
    msg_.heave_z = std::move(arg);
    return Init_Tau_roll_k(msg_);
  }

private:
  ::ngc_interfaces::msg::Tau msg_;
};

class Init_Tau_sway_y
{
public:
  explicit Init_Tau_sway_y(::ngc_interfaces::msg::Tau & msg)
  : msg_(msg)
  {}
  Init_Tau_heave_z sway_y(::ngc_interfaces::msg::Tau::_sway_y_type arg)
  {
    msg_.sway_y = std::move(arg);
    return Init_Tau_heave_z(msg_);
  }

private:
  ::ngc_interfaces::msg::Tau msg_;
};

class Init_Tau_surge_x
{
public:
  Init_Tau_surge_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Tau_sway_y surge_x(::ngc_interfaces::msg::Tau::_surge_x_type arg)
  {
    msg_.surge_x = std::move(arg);
    return Init_Tau_sway_y(msg_);
  }

private:
  ::ngc_interfaces::msg::Tau msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ngc_interfaces::msg::Tau>()
{
  return ngc_interfaces::msg::builder::Init_Tau_surge_x();
}

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__TAU__BUILDER_HPP_
