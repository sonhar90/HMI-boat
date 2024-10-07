// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ngc_interfaces:msg/GNSS.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/gnss.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__GNSS__STRUCT_HPP_
#define NGC_INTERFACES__MSG__DETAIL__GNSS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ngc_interfaces__msg__GNSS __attribute__((deprecated))
#else
# define DEPRECATED__ngc_interfaces__msg__GNSS __declspec(deprecated)
#endif

namespace ngc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GNSS_
{
  using Type = GNSS_<ContainerAllocator>;

  explicit GNSS_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lat = 0.0f;
      this->lon = 0.0f;
      this->valid_signal = false;
    }
  }

  explicit GNSS_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->lat = 0.0f;
      this->lon = 0.0f;
      this->valid_signal = false;
    }
  }

  // field types and members
  using _lat_type =
    float;
  _lat_type lat;
  using _lon_type =
    float;
  _lon_type lon;
  using _valid_signal_type =
    bool;
  _valid_signal_type valid_signal;

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
  Type & set__valid_signal(
    const bool & _arg)
  {
    this->valid_signal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ngc_interfaces::msg::GNSS_<ContainerAllocator> *;
  using ConstRawPtr =
    const ngc_interfaces::msg::GNSS_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ngc_interfaces::msg::GNSS_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ngc_interfaces::msg::GNSS_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::GNSS_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::GNSS_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::GNSS_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::GNSS_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ngc_interfaces::msg::GNSS_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ngc_interfaces::msg::GNSS_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ngc_interfaces__msg__GNSS
    std::shared_ptr<ngc_interfaces::msg::GNSS_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ngc_interfaces__msg__GNSS
    std::shared_ptr<ngc_interfaces::msg::GNSS_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GNSS_ & other) const
  {
    if (this->lat != other.lat) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    if (this->valid_signal != other.valid_signal) {
      return false;
    }
    return true;
  }
  bool operator!=(const GNSS_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GNSS_

// alias to use template instance with default allocator
using GNSS =
  ngc_interfaces::msg::GNSS_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__GNSS__STRUCT_HPP_
