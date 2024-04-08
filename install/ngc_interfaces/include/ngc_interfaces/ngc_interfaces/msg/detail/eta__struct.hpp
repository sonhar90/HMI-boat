// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ngc_interfaces:msg/Eta.idl
// generated code does not contain a copyright notice

#ifndef NGC_INTERFACES__MSG__DETAIL__ETA__STRUCT_HPP_
#define NGC_INTERFACES__MSG__DETAIL__ETA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ngc_interfaces__msg__Eta __attribute__((deprecated))
#else
# define DEPRECATED__ngc_interfaces__msg__Eta __declspec(deprecated)
#endif

namespace ngc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Eta_
{
  using Type = Eta_<ContainerAllocator>;

  explicit Eta_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lat = 0.0f;
      this->lon = 0.0f;
      this->z = 0.0f;
      this->phi = 0.0f;
      this->theta = 0.0f;
      this->psi = 0.0f;
    }
  }

  explicit Eta_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lat = 0.0f;
      this->lon = 0.0f;
      this->z = 0.0f;
      this->phi = 0.0f;
      this->theta = 0.0f;
      this->psi = 0.0f;
    }
  }

  // field types and members
  using _lat_type =
    float;
  _lat_type lat;
  using _lon_type =
    float;
  _lon_type lon;
  using _z_type =
    float;
  _z_type z;
  using _phi_type =
    float;
  _phi_type phi;
  using _theta_type =
    float;
  _theta_type theta;
  using _psi_type =
    float;
  _psi_type psi;

  // setters for named parameter idiom
  Type & set__lat(
    const float & _arg)
  {
    this->lat = _arg;
    return *this;
  }
  Type & set__lon(
    const float & _arg)
  {
    this->lon = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__phi(
    const float & _arg)
  {
    this->phi = _arg;
    return *this;
  }
  Type & set__theta(
    const float & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__psi(
    const float & _arg)
  {
    this->psi = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ngc_interfaces::msg::Eta_<ContainerAllocator> *;
  using ConstRawPtr =
    const ngc_interfaces::msg::Eta_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ngc_interfaces::msg::Eta_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ngc_interfaces::msg::Eta_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::Eta_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::Eta_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::Eta_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::Eta_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ngc_interfaces::msg::Eta_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ngc_interfaces::msg::Eta_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ngc_interfaces__msg__Eta
    std::shared_ptr<ngc_interfaces::msg::Eta_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ngc_interfaces__msg__Eta
    std::shared_ptr<ngc_interfaces::msg::Eta_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Eta_ & other) const
  {
    if (this->lat != other.lat) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->phi != other.phi) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->psi != other.psi) {
      return false;
    }
    return true;
  }
  bool operator!=(const Eta_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Eta_

// alias to use template instance with default allocator
using Eta =
  ngc_interfaces::msg::Eta_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__ETA__STRUCT_HPP_
