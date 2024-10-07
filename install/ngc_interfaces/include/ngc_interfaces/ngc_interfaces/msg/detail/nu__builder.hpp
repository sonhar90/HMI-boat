// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ngc_interfaces:msg/Nu.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/nu.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__NU__BUILDER_HPP_
#define NGC_INTERFACES__MSG__DETAIL__NU__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ngc_interfaces/msg/detail/nu__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ngc_interfaces
{

namespace msg
{

namespace builder
{

class Init_Nu_r
{
public:
  explicit Init_Nu_r(::ngc_interfaces::msg::Nu & msg)
  : msg_(msg)
  {}
  ::ngc_interfaces::msg::Nu r(::ngc_interfaces::msg::Nu::_r_type arg)
  {
    msg_.r = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ngc_interfaces::msg::Nu msg_;
};

class Init_Nu_q
{
public:
  explicit Init_Nu_q(::ngc_interfaces::msg::Nu & msg)
  : msg_(msg)
  {}
  Init_Nu_r q(::ngc_interfaces::msg::Nu::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_Nu_r(msg_);
  }

private:
  ::ngc_interfaces::msg::Nu msg_;
};

class Init_Nu_p
{
public:
  explicit Init_Nu_p(::ngc_interfaces::msg::Nu & msg)
  : msg_(msg)
  {}
  Init_Nu_q p(::ngc_interfaces::msg::Nu::_p_type arg)
  {
    msg_.p = std::move(arg);
    return Init_Nu_q(msg_);
  }

private:
  ::ngc_interfaces::msg::Nu msg_;
};

class Init_Nu_w
{
public:
  explicit Init_Nu_w(::ngc_interfaces::msg::Nu & msg)
  : msg_(msg)
  {}
  Init_Nu_p w(::ngc_interfaces::msg::Nu::_w_type arg)
  {
    msg_.w = std::move(arg);
    return Init_Nu_p(msg_);
  }

private:
  ::ngc_interfaces::msg::Nu msg_;
};

class Init_Nu_v
{
public:
  explicit Init_Nu_v(::ngc_interfaces::msg::Nu & msg)
  : msg_(msg)
  {}
  Init_Nu_w v(::ngc_interfaces::msg::Nu::_v_type arg)
  {
    msg_.v = std::move(arg);
    return Init_Nu_w(msg_);
  }

private:
  ::ngc_interfaces::msg::Nu msg_;
};

class Init_Nu_u
{
public:
  Init_Nu_u()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Nu_v u(::ngc_interfaces::msg::Nu::_u_type arg)
  {
    msg_.u = std::move(arg);
    return Init_Nu_v(msg_);
  }

private:
  ::ngc_interfaces::msg::Nu msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ngc_interfaces::msg::Nu>()
{
  return ngc_interfaces::msg::builder::Init_Nu_u();
}

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__NU__BUILDER_HPP_
