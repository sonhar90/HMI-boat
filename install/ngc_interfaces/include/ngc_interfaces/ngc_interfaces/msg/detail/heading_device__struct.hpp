// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ngc_interfaces:msg/HeadingDevice.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "ngc_interfaces/msg/heading_device.hpp"


#ifndef NGC_INTERFACES__MSG__DETAIL__HEADING_DEVICE__STRUCT_HPP_
#define NGC_INTERFACES__MSG__DETAIL__HEADING_DEVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ngc_interfaces__msg__HeadingDevice __attribute__((deprecated))
#else
# define DEPRECATED__ngc_interfaces__msg__HeadingDevice __declspec(deprecated)
#endif

namespace ngc_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct HeadingDevice_
{
  using Type = HeadingDevice_<ContainerAllocator>;

  explicit HeadingDevice_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->heading = 0.0f;
      this->rot = 0.0f;
      this->valid_signal = false;
    }
  }

  explicit HeadingDevice_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->heading = 0.0f;
      this->rot = 0.0f;
      this->valid_signal = false;
    }
  }

  // field types and members
  using _heading_type =
    float;
  _heading_type heading;
  using _rot_type =
    float;
  _rot_type rot;
  using _valid_signal_type =
    bool;
  _valid_signal_type valid_signal;

  // setters for named parameter idiom
  Type & set__heading(
    const float & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__rot(
    const float & _arg)
  {
    this->rot = _arg;
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
    ngc_interfaces::msg::HeadingDevice_<ContainerAllocator> *;
  using ConstRawPtr =
    const ngc_interfaces::msg::HeadingDevice_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ngc_interfaces::msg::HeadingDevice_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ngc_interfaces::msg::HeadingDevice_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::HeadingDevice_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::HeadingDevice_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ngc_interfaces::msg::HeadingDevice_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ngc_interfaces::msg::HeadingDevice_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ngc_interfaces::msg::HeadingDevice_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ngc_interfaces::msg::HeadingDevice_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ngc_interfaces__msg__HeadingDevice
    std::shared_ptr<ngc_interfaces::msg::HeadingDevice_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ngc_interfaces__msg__HeadingDevice
    std::shared_ptr<ngc_interfaces::msg::HeadingDevice_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const HeadingDevice_ & other) const
  {
    if (this->heading != other.heading) {
      return false;
    }
    if (this->rot != other.rot) {
      return false;
    }
    if (this->valid_signal != other.valid_signal) {
      return false;
    }
    return true;
  }
  bool operator!=(const HeadingDevice_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct HeadingDevice_

// alias to use template instance with default allocator
using HeadingDevice =
  ngc_interfaces::msg::HeadingDevice_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ngc_interfaces

#endif  // NGC_INTERFACES__MSG__DETAIL__HEADING_DEVICE__STRUCT_HPP_
