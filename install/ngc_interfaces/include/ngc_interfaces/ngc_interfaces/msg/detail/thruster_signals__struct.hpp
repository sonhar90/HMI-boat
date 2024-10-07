// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ngc_interfaces:msg/ThrusterSignals.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/thruster_signals.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__STRUCT_HPP_
#define NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ngc_interfaces__msg__ThrusterSignals __attribute__((deprecated))
#else
# define DEPRECATED__ngc_interfaces__msg__ThrusterSignals __declspec(deprecated)
#endif

namespace ngc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ThrusterSignals_
{
  using Type = ThrusterSignals_<ContainerAllocator>;

  explicit ThrusterSignals_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->thruster_id = 0l;
      this->rps = 0.0;
      this->pitch = 0.0;
      this->azimuth_deg = 0.0;
      this->active = false;
      this->error = false;
    }
  }

  explicit ThrusterSignals_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->thruster_id = 0l;
      this->rps = 0.0;
      this->pitch = 0.0;
      this->azimuth_deg = 0.0;
      this->active = false;
      this->error = false;
    }
  }

  // field types and members
  using _thruster_id_type =
    int32_t;
  _thruster_id_type thruster_id;
  using _rps_type =
    double;
  _rps_type rps;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _azimuth_deg_type =
    double;
  _azimuth_deg_type azimuth_deg;
  using _active_type =
    bool;
  _active_type active;
  using _error_type =
    bool;
  _error_type error;

  // setters for named parameter idiom
  Type & set__thruster_id(
    const int32_t & _arg)
  {
    this->thruster_id = _arg;
    return *this;
  }
  Type & set__rps(
    const double & _arg)
  {
    this->rps = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__azimuth_deg(
    const double & _arg)
  {
    this->azimuth_deg = _arg;
    return *this;
  }
  Type & set__active(
    const bool & _arg)
  {
    this->active = _arg;
    return *this;
  }
  Type & set__error(
    const bool & _arg)
  {
    this->error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator> *;
  using ConstRawPtr =
    const ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ngc_interfaces__msg__ThrusterSignals
    std::shared_ptr<ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ngc_interfaces__msg__ThrusterSignals
    std::shared_ptr<ngc_interfaces::msg::ThrusterSignals_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ThrusterSignals_ & other) const
  {
    if (this->thruster_id != other.thruster_id) {
      return false;
    }
    if (this->rps != other.rps) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->azimuth_deg != other.azimuth_deg) {
      return false;
    }
    if (this->active != other.active) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    return true;
  }
  bool operator!=(const ThrusterSignals_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ThrusterSignals_

// alias to use template instance with default allocator
using ThrusterSignals =
  ngc_interfaces::msg::ThrusterSignals_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__THRUSTER_SIGNALS__STRUCT_HPP_
