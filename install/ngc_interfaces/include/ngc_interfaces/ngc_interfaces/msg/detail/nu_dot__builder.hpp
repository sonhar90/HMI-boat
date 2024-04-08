// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ngc_interfaces:msg/NuDot.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__NU_DOT__BUILDER_HPP_
#define NGC_INTERFACES__MSG__DETAIL__NU_DOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ngc_interfaces/msg/detail/nu_dot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ngc_interfaces
{

namespace msg
{

namespace builder
{

class Init_NuDot_r_dot
{
public:
  explicit Init_NuDot_r_dot(::ngc_interfaces::msg::NuDot & msg)
  : msg_(msg)
  {}
  ::ngc_interfaces::msg::NuDot r_dot(::ngc_interfaces::msg::NuDot::_r_dot_type arg)
  {
    msg_.r_dot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ngc_interfaces::msg::NuDot msg_;
};

class Init_NuDot_q_dot
{
public:
  explicit Init_NuDot_q_dot(::ngc_interfaces::msg::NuDot & msg)
  : msg_(msg)
  {}
  Init_NuDot_r_dot q_dot(::ngc_interfaces::msg::NuDot::_q_dot_type arg)
  {
    msg_.q_dot = std::move(arg);
    return Init_NuDot_r_dot(msg_);
  }

private:
  ::ngc_interfaces::msg::NuDot msg_;
};

class Init_NuDot_p_dot
{
public:
  explicit Init_NuDot_p_dot(::ngc_interfaces::msg::NuDot & msg)
  : msg_(msg)
  {}
  Init_NuDot_q_dot p_dot(::ngc_interfaces::msg::NuDot::_p_dot_type arg)
  {
    msg_.p_dot = std::move(arg);
    return Init_NuDot_q_dot(msg_);
  }

private:
  ::ngc_interfaces::msg::NuDot msg_;
};

class Init_NuDot_w_dot
{
public:
  explicit Init_NuDot_w_dot(::ngc_interfaces::msg::NuDot & msg)
  : msg_(msg)
  {}
  Init_NuDot_p_dot w_dot(::ngc_interfaces::msg::NuDot::_w_dot_type arg)
  {
    msg_.w_dot = std::move(arg);
    return Init_NuDot_p_dot(msg_);
  }

private:
  ::ngc_interfaces::msg::NuDot msg_;
};

class Init_NuDot_v_dot
{
public:
  explicit Init_NuDot_v_dot(::ngc_interfaces::msg::NuDot & msg)
  : msg_(msg)
  {}
  Init_NuDot_w_dot v_dot(::ngc_interfaces::msg::NuDot::_v_dot_type arg)
  {
    msg_.v_dot = std::move(arg);
    return Init_NuDot_w_dot(msg_);
  }

private:
  ::ngc_interfaces::msg::NuDot msg_;
};

class Init_NuDot_u_dot
{
public:
  Init_NuDot_u_dot()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NuDot_v_dot u_dot(::ngc_interfaces::msg::NuDot::_u_dot_type arg)
  {
    msg_.u_dot = std::move(arg);
    return Init_NuDot_v_dot(msg_);
  }

private:
  ::ngc_interfaces::msg::NuDot msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ngc_interfaces::msg::NuDot>()
{
  return ngc_interfaces::msg::builder::Init_NuDot_u_dot();
}

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__NU_DOT__BUILDER_HPP_
