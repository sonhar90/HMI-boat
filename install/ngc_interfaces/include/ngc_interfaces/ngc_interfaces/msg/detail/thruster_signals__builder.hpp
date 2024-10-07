// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ngc_interfaces:msg/ThrusterSignals.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/thruster_signals.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__BUILDER_HPP_
#define NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ngc_interfaces/msg/detail/thruster_signals__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ngc_interfaces
{

namespace msg
{

namespace builder
{

class Init_ThrusterSignals_error
{
public:
  explicit Init_ThrusterSignals_error(::ngc_interfaces::msg::ThrusterSignals & msg)
  : msg_(msg)
  {}
  ::ngc_interfaces::msg::ThrusterSignals error(::ngc_interfaces::msg::ThrusterSignals::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ngc_interfaces::msg::ThrusterSignals msg_;
};

class Init_ThrusterSignals_active
{
public:
  explicit Init_ThrusterSignals_active(::ngc_interfaces::msg::ThrusterSignals & msg)
  : msg_(msg)
  {}
  Init_ThrusterSignals_error active(::ngc_interfaces::msg::ThrusterSignals::_active_type arg)
  {
    msg_.active = std::move(arg);
    return Init_ThrusterSignals_error(msg_);
  }

private:
  ::ngc_interfaces::msg::ThrusterSignals msg_;
};

class Init_ThrusterSignals_azimuth_deg
{
public:
  explicit Init_ThrusterSignals_azimuth_deg(::ngc_interfaces::msg::ThrusterSignals & msg)
  : msg_(msg)
  {}
  Init_ThrusterSignals_active azimuth_deg(::ngc_interfaces::msg::ThrusterSignals::_azimuth_deg_type arg)
  {
    msg_.azimuth_deg = std::move(arg);
    return Init_ThrusterSignals_active(msg_);
  }

private:
  ::ngc_interfaces::msg::ThrusterSignals msg_;
};

class Init_ThrusterSignals_pitch
{
public:
  explicit Init_ThrusterSignals_pitch(::ngc_interfaces::msg::ThrusterSignals & msg)
  : msg_(msg)
  {}
  Init_ThrusterSignals_azimuth_deg pitch(::ngc_interfaces::msg::ThrusterSignals::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_ThrusterSignals_azimuth_deg(msg_);
  }

private:
  ::ngc_interfaces::msg::ThrusterSignals msg_;
};

class Init_ThrusterSignals_rps
{
public:
  explicit Init_ThrusterSignals_rps(::ngc_interfaces::msg::ThrusterSignals & msg)
  : msg_(msg)
  {}
  Init_ThrusterSignals_pitch rps(::ngc_interfaces::msg::ThrusterSignals::_rps_type arg)
  {
    msg_.rps = std::move(arg);
    return Init_ThrusterSignals_pitch(msg_);
  }

private:
  ::ngc_interfaces::msg::ThrusterSignals msg_;
};

class Init_ThrusterSignals_thruster_id
{
public:
  Init_ThrusterSignals_thruster_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ThrusterSignals_rps thruster_id(::ngc_interfaces::msg::ThrusterSignals::_thruster_id_type arg)
  {
    msg_.thruster_id = std::move(arg);
    return Init_ThrusterSignals_rps(msg_);
  }

private:
  ::ngc_interfaces::msg::ThrusterSignals msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ngc_interfaces::msg::ThrusterSignals>()
{
  return ngc_interfaces::msg::builder::Init_ThrusterSignals_thruster_id();
}

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__BUILDER_HPP_
